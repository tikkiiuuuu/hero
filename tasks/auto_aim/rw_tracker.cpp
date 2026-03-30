#include "rw_tracker.hpp"
#include "armor.hpp"
#include "tools/extended_kalman_filter.hpp"
#include <list>
#include <vector>

namespace auto_aim {
/************************************************************************************************************************
 * @brief RW跟踪构造函数
 * 主要是调用函数，动态构造了观测矩阵和观测雅可比
 * 另外不用传入畸变矩阵，在误差范围内
***********************************************************************************************************************/
auto_aim::RWTracker::RWTracker(const std::string& config_path, Solver& solver):
    solver(solver),
    tracker_state(TrackState::LOST),
    tracked_id(ArmorName::not_armor),
    target_state(Eigen::VectorXd::Zero(11)),
    tracked_armors_num(4),
    measure_dimension(0),
    show_target_interface_(*this)
    {
    auto yaml = YAML::LoadFile(config_path);
    match_thre_distance = yaml["match_thre_distance"].as<float>();
    match_thre_angle = yaml["match_thre_angle"].as<float>();
    match_thre_distance_light = yaml["match_thre_distance_light"].as<float>();
    match_thre_angle_light = yaml["match_thre_angle_light"].as<float>();
    match_thre_length_light = yaml["match_thre_length_light"].as<float>();
    s2_normal = {
        .q_xyz = yaml["tracker"]["ekf"]["sigma2_q_xyz"].as<double>(),
        .q_yaw = yaml["tracker"]["ekf"]["sigma2_q_yaw"].as<double>(),
        .q_r = yaml["tracker"]["ekf"]["sigma2_q_r"].as<double>(),
        .r_u = yaml["tracker"]["ekf"]["sigam2_r_u"].as<double>(),
        .r_v = yaml["tracker"]["ekf"]["sigam2_r_v"].as<double>()
    };
    s2_outpost = {
        .q_xyz = yaml["tracker"]["ekf_outpost"]["sigma2_q_xyz"].as<double>(),
        .q_yaw = yaml["tracker"]["ekf_outpost"]["sigma2_q_yaw"].as<double>(),
        .q_r = yaml["tracker"]["ekf_outpost"]["sigma2_q_r"].as<double>(),
        .r_u = yaml["tracker"]["ekf_outpost"]["sigam2_r_u"].as<double>(),
        .r_v = yaml["tracker"]["ekf_outpost"]["sigam2_r_v"].as<double>()
    };
    max_armor_distance_ = yaml["tracker"]["max_armor_distance"].as<double>();
    max_match_error = yaml["tracker"]["max_match_error"].as<double>();
    track_thres = yaml["tracker"]["track_thres"].as<double>();
    lost_thres = yaml["tracker"]["lost_thres"].as<double>();
    self_height = yaml["tracker"]["self_height"].as<float>();
    enemy_color_ = (yaml["enemy_color"].as<std::string>() == "red") ? Color::red : Color::blue;
    // 初始化相机内参矩阵
    auto camera_matrix_data = yaml["camera_matrix"].as<std::vector<double>>();
    Eigen::Matrix<double, 3, 3, Eigen::RowMajor> camera_matrix(camera_matrix_data.data());
    cam_info_ = camera_matrix;

    // 初始化 odom_camera_matrix 和 odom2camera_tvec（虽然会在update中更新，但先初始化避免未定义行为）
    odom2camera_tvec = solver.odom2camera_tvec();
    odom_camera_matrix = solver.odom_camera_matrix();

    /*************************************************
     *                 图像点直接观测EKF               *
     ************************************************/

    // Todo: 大小装甲板判断、前哨站判断

    // EKF
    // xa = x_armor, xc = x_robot_center
    // state:       [xc, v_xc, yc, v_yc, za_1, v_za, theta, omega, r1 , za_2 ,r2]
    // measurement: [u_p1, v_p1, u_p2, v_p2, u_p3, v_p3, u_p4, v_p4]

    /* 过程矩阵 F */
    auto f = [this](const Eigen::VectorXd& x) {
        Eigen::VectorXd x_predict = x;
        x_predict(0) += x(1) * dt_;
        x_predict(2) += x(3) * dt_;
        x_predict(4) += x(5) * dt_;
        x_predict(6) += x(7) * dt_;
        x_predict(9) += x(5) * dt_;
        return x_predict;
    };

    /* 过程矩阵 F 雅可比 */
    auto j_f = [this](const Eigen::VectorXd&) {
        Eigen::MatrixXd f(11, 11);
        // clang-format off
        f << 1,   dt_, 0,   0,   0,   0,   0,   0,   0,   0,   0,
             0,   1,   0,   0,   0,   0,   0,   0,   0,   0,   0,
             0,   0,   1,   dt_, 0,   0,   0,   0,   0,   0,   0,
             0,   0,   0,   1,   0,   0,   0,   0,   0,   0,   0,
             0,   0,   0,   0,   1,   dt_, 0,   0,   0,   0,   0,
             0,   0,   0,   0,   0,   1,   0,   0,   0,   0,   0,
             0,   0,   0,   0,   0,   0,   1,   dt_, 0,   0,   0,
             0,   0,   0,   0,   0,   0,   0,   1,   0,   0,   0,
             0,   0,   0,   0,   0,   0,   0,   0,   1,   0,   0,
             0,   0,   0,   0,   0,   dt_, 0,   0,   0,   1,   0,
             0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   1;
        // clang-format on
        return f;
    };

    /* 观测矩阵 H */
    auto h = [this](const Eigen::VectorXd& x) {
        Eigen::VectorXd z(this->measure_dimension);
        int index = 0;
        // 获取每个装甲板z向量，拼接
        for (int i = 0; i < this->tracked_armors_num; i++) {
            if (this->match_armors[i].match_type != MatchType::NONE) {
                Eigen::VectorXd z_singal_armor(
                    Singal_Armor_h(x, i, this->match_armors[i].match_type)
                );
                z.segment(index, z_singal_armor.size()) = z_singal_armor;
                index += z_singal_armor.size();
            }
        }
        // std::cout << "z: " << z << std::endl;
        return z;
    };

    /* 观测矩阵 H 雅可比 */
    auto j_h = [this](const Eigen::VectorXd& x) {
        Eigen::MatrixXd h(this->measure_dimension, 11);
        int current_row = 0;
        // 获取每个装甲h矩阵，拼接
        for (int i = 0; i < this->tracked_armors_num; i++) {
            if (this->match_armors[i].match_type != MatchType::NONE) {
                Eigen::MatrixXd h_singal_armor(
                    Singal_Armor_jh(x, i, this->match_armors[i].match_type)
                );
                h.middleRows(current_row, h_singal_armor.rows()) = h_singal_armor;
                current_row += h_singal_armor.rows();
            }
        }
        // std::cout << "h: " << h << std::endl;
        return h;
    };

    /* 过程噪声协方差矩阵 Q */
    auto u_q = [this]() {
        Eigen::MatrixXd q(11, 11);
        double t = dt_;
        double x, y, r;
        // 前哨站/普通装甲板方差区别
        if (this->tracked_id == ArmorName::outpost) {
            x = this->s2_outpost.q_xyz;
            y = this->s2_outpost.q_yaw;
            r = this->s2_outpost.q_r;
        } else {
            x = this->s2_normal.q_xyz;
            y = this->s2_normal.q_yaw;
            r = this->s2_normal.q_r;
        }
        double q_x_x = pow(t, 4) / 4 * x, q_x_vx = pow(t, 3) / 2 * x, q_vx_vx = pow(t, 2) * x;
        double q_y_y = pow(t, 4) / 4 * y, q_y_vy = pow(t, 3) / 2 * y, q_vy_vy = pow(t, 2) * y;
        //? 这里之前科写的q_y_vy是pow(t, 3) / 2 * x，给改成pow(t, 3) / 2 * y了
        double q_r = pow(t, 4) / 4 * r;
        // clang-format off
        //   xc      v_xc    yc      v_yc    za_1    v_za    yaw     v_yaw   r_1   za_2  r_2
        q << q_x_x,  q_x_vx, 0,      0,      0,      0,      0,      0,      0,    0,     0,
             q_x_vx, q_vx_vx,0,      0,      0,      0,      0,      0,      0,    0,     0,
             0,      0,      q_x_x,  q_x_vx, 0,      0,      0,      0,      0,    0,     0,
             0,      0,      q_x_vx, q_vx_vx,0,      0,      0,      0,      0,    0,     0,
             0,      0,      0,      0,      q_x_x,  q_x_vx, 0,      0,      0,    q_x_x, 0,
             0,      0,      0,      0,      q_x_vx, q_vx_vx,0,      0,      0,    q_x_vx,0,
             0,      0,      0,      0,      0,      0,      q_y_y,  q_y_vy, 0,    0,     0,
             0,      0,      0,      0,      0,      0,      q_y_vy, q_vy_vy,0,    0,     0,
             0,      0,      0,      0,      0,      0,      0,      0,      q_r,  0,     0,
             0,      0,      0,      0,      q_x_x,  q_x_vx, 0,      0,      0,    q_x_x, 0,
             0,      0,      0,      0,      0,      0,      0,      0,      0,    0,     q_r;
        // clang-format on
        return q;
    };

    /* 测量噪声协方差矩阵 R */
    auto u_r = [this]() {
        Eigen::VectorXd r_vec(this->measure_dimension);
        Eigen::Vector2d r_point(this->s2_normal.r_u, this->s2_normal.r_v);
        int index = 0;
        for (int i = 0; i < this->measure_dimension / 2; i++) {
            r_vec.segment(index, r_point.size()) = r_point;
            index += r_point.size();
        }
        Eigen::MatrixXd r = r_vec.asDiagonal();
        // std::cout << "r" << r << std::endl;
        return r;
    };

    /* 状态协方差矩阵 P */
    Eigen::DiagonalMatrix<double, 11> p0;
    p0.setIdentity();

    /* 创建 EKF 对象 */
    ekf = tools::RWExtendedKalmanFilter(f, h, j_f, j_h, u_q, u_r, p0);
}

/************************************************************************************************************************
 * @brief   跟踪器初始化函数
 * @param   armors_msg      识别到的装甲板队列
 ***********************************************************************************************************************/
void auto_aim::RWTracker::init(const std::list<Armor>& armors) {
    /* 无装甲板，直接返回 */
    // 过滤非敌方颜色装甲板
    std::list<Armor> filtered_armors = armors;
    filtered_armors.remove_if([this](const Armor& a) { return a.color != enemy_color_; });

    if (filtered_armors.empty()) {
        return;
    }

    /* 简单选择距离图像中心近装甲板为跟踪对象 */
    double min_distance = DBL_MAX;
    tracked_armor = filtered_armors.front();
    for (const auto& armor: filtered_armors) {
        auto distance = solver.calculateDistanceToCenter(armor.center);
        if (distance < min_distance) {
            min_distance = distance;
            tracked_armor = armor;
        }
    }

    /* 初始化 EKF */
    initEKF(*tracked_armor);
    // RCLCPP_DEBUG(rclcpp::get_logger("armor_observer_node"), "Init EKF!");

    /* 填充信息 */
    this->matched = true;
    this->tracked_id = tracked_armor->name;

    this->tracker_state = DETECTING;
    updateTrackedArmorsNum(*tracked_armor);
    this->match_armors.resize(this->tracked_armors_num);
    std::fill(this->match_armors.begin(), this->match_armors.end(), this->MATCH_ARMOR_INIT);
}

/************************************************************************************************************************
 * @brief RW跟踪预测、筛选与更新
 * 对于所有装甲板和识别的装甲板以外的灯条通过预测，在范围内的进行匹配，匹配成功后进行EKF更新
 * @param armors_msg 
 * @param light_array 
***********************************************************************************************************************/
void auto_aim::RWTracker::update(const std::list<Armor>& armors, std::list<Light> light_array) {
    // auto match_thre_distance = this->max_match_error;  // 装甲板匹配误差阈值（距离）都是经验参数
    // auto match_thre_angle = 2.0f;                      // 装甲板匹配误差阈值（角度）
    // auto match_thre_distance_light = 60.0f;            // 灯条匹配误差阈值（距离）
    // auto match_thre_angle_light = 0.1f;                // 灯条匹配误差阈值（角度）
    // auto match_thre_length_light = 0.1f;  // 灯条匹配误差阈值（百分比长度）

    std::list<Armor> filtered_armors = armors;
    filtered_armors.remove_if([this](const Armor& a) { return a.color != enemy_color_; });

    this->matched = false; // 当前帧是否有匹配的装甲板、灯条
    std::vector<Armor> same_id_armors; // 与当前跟踪相同id的装甲板
    same_id_armors.reserve(this->tracked_armors_num);//* 也就是看到了可用来观测的
    this->match_armors.resize(this->tracked_armors_num);//* 整车观测，记录所有状态
    std::fill(this->match_armors.begin(), this->match_armors.end(), this->MATCH_ARMOR_INIT);

    /* EKF 预测 */
    ekf_prediction = ekf.predict();
    target_state = ekf_prediction;
    double x_c = ekf_prediction(0), y_c = ekf_prediction(2), theta = ekf_prediction(6);
    // double z_a1 = ekf_prediction(4), z_a2 = ekf_prediction(9), r_1 = ekf_prediction(8), r_2 = ekf_prediction(10);
    // RCLCPP_DEBUG(rclcpp::get_logger("armor_observer_node"), "EKF predict");

    /* 寻找相同id装甲板 */
    for (const auto& armor: filtered_armors) {
        if (armor.name == this->tracked_id) {
            same_id_armors.emplace_back(armor);
        }
    }
  

    if (!same_id_armors.empty()) {
        /* 根据预测筛选可匹配装甲板     TODO: 装甲板数量不同时阈值修改 */
        //! 这里的坐标变换是新增的
        this->odom2camera_tvec = this->solver.odom2camera_tvec();
        this->odom_camera_matrix = this->solver.odom_camera_matrix();
        //! 还没细看solver对时间戳的依赖，这里假定每一帧下各种坐标不会改变

        double camera_enemy_theta =
            atan2(y_c - this->odom_camera_matrix.block<3, 1>(0, 3)[1], x_c - this->odom_camera_matrix.block<3, 1>(0, 3)[0]);
        // std::cout << "camera_enemy_theta: " << camera_enemy_theta << std::endl;

        //* 首先根据状态转移预测点
        double theta_threshold_max = normalize_angle(camera_enemy_theta + M_PI / 2);
        double theta_threshold_min = normalize_angle(camera_enemy_theta - M_PI / 2);
        for (int i = 0; i < this->tracked_armors_num; i++) {
            double original_za1 = ekf_prediction(static_cast<int>(State::ZA1));
            //! 这里对前哨站中心临时变化以适应高低差，之后注意要变回去
            if (this->tracked_id == ArmorName::outpost) {
                ekf_prediction(static_cast<int>(State::ZA1)) += 0.1*circle_type[(circle_index+i)%3];
            }
            double predict_armor_theta = normalize_angle(
                theta + 2 * M_PI / (static_cast<int>(this->tracked_armors_num)) * i
            );
            // 处理[-pi,pi)区间跳跃
            if (theta_threshold_max > theta_threshold_min) {
                if (predict_armor_theta < theta_threshold_min
                    || predict_armor_theta > theta_threshold_max) {
                    // 当前预测装甲板落入观测区间，允许匹配
                    match_armors[i].need_match = true;
                    match_armors[i].predict_points =
                        Singal_Armor_h(ekf_prediction, i, MatchType::ARMOR);
                } else {
                    // 当前预测装甲板不在观测区间，拒绝匹配
                    match_armors[i].need_match = false;
                }
            } else {
                if (predict_armor_theta < theta_threshold_min
                    && predict_armor_theta > theta_threshold_max) {
                    // 当前预测装甲板落入观测区间，允许匹配
                    match_armors[i].need_match = true;
                    match_armors[i].predict_points =
                        Singal_Armor_h(ekf_prediction, i, MatchType::ARMOR);
                } else {
                    // 当前预测装甲板不在观测区间，拒绝匹配
                    match_armors[i].need_match = false;
                }
            }
            //! 这里状态量变回去
            if(this->tracked_id == ArmorName::outpost) {
                ekf_prediction(static_cast<int>(State::ZA1)) -= 0.1*circle_type[(circle_index+i)%3];
            }
            // Eigen::VectorXd predict_points = Singal_Armor_h(ekf_prediction, i, MatchType::ARMOR);
            // // std::cout << "[DEBUG] After Singal_Armor_h - predict_points=" << predict_points.transpose() << std::endl;
        }

        /* 进行装甲板匹配 */
        int match_armor_num = 0;
        int match_armor_id = -1;
        for (const auto& armor: same_id_armors) {
            int min_error_id = -1;
            double min_error = DBL_MAX;
            Eigen::VectorXd measure_points(8);
            //clang-format off
            measure_points << armor.left.top.x, armor.left.top.y, armor.right.top.x,
                armor.right.top.y, armor.left.bottom.x, armor.left.bottom.y, armor.right.bottom.x,
                armor.right.bottom.y;
            //clang-format on
            for (int i = 0; i < this->tracked_armors_num; i++) {
                if (match_armors[i].need_match == true
                    && match_armors[i].match_type == MatchType::NONE) {
                    // 检查 predict_points 是否有效
                    if (match_armors[i].predict_points.size() == 0) {
                        continue;  // 跳过未初始化的预测点
                    }
                    // 寻找距离误差最小的装甲板
                    auto error_distance =
                        getError_Distance(match_armors[i].predict_points, measure_points);

                    if (error_distance < min_error) {
                        min_error = error_distance;
                        min_error_id = i;
                    }
                }
            }
            if (min_error_id >= 0 && match_armors[min_error_id].predict_points.size() > 0) {
                // 判断匹配的装甲板距离误差、角度误差是否满足要求
                auto error_angle =
                    getError_Angle(match_armors[min_error_id].predict_points, measure_points);
                if (min_error < match_thre_distance && error_angle < match_thre_angle) {
                    match_armor_num += 1;
                    match_armor_id = min_error_id;
                    match_armors[min_error_id].error_distance = min_error;
                    match_armors[min_error_id].error_angle = error_angle;
                    match_armors[min_error_id].match_type = MatchType::ARMOR;
                    match_armors[min_error_id].measure_points = measure_points;

                    if(armor.name == ArmorName::outpost){
                        if(outpost_errcount == 1){
                            circle_index = (circle_index - 1 + 3) % 3;
                        } else if(outpost_errcount == 2){
                            circle_index = (circle_index - 2 + 3) % 3;
                        }
                        outpost_errcount = 0;
                    }
                } else {
                    if(armor.name == ArmorName::outpost){
                        outpost_errcount++;
                    }
                    // 添加调试信息：显示为什么匹配失败
                    std::cout << "Match failed for armor: min_error=" << min_error 
                              << " (thre=" << match_thre_distance << "), error_angle=" << error_angle 
                              << " (thre=" << match_thre_angle << ")" << std::endl;
                }
            }
        }

        /* 进行灯条匹配 */
        if (match_armor_num == 1) {
            // 填充左右相邻装甲板预测点集位置
            int armor_left_id, armor_right_id;
            if (this->tracked_id == ArmorName::outpost) {
                armor_left_id = (match_armor_id + 2) % 3;
                armor_right_id = (match_armor_id + 1) % 3;
            } else {
                armor_left_id = (match_armor_id + 3) % 4;
                armor_right_id = (match_armor_id + 1) % 4;
            }
            MatchArmor& armor_left = match_armors[armor_left_id];
            MatchArmor& armor_right = match_armors[armor_right_id];
            armor_left.predict_points =
                Singal_Armor_h(ekf_prediction, armor_left_id, MatchType::LIGHT_RIGHT);
            armor_right.predict_points =
                Singal_Armor_h(ekf_prediction, armor_right_id, MatchType::LIGHT_LEFT);
            // 获取匹配装甲板灯条平均长度
            Eigen::Vector2d point[4];
            for (int i = 0; i < 4; i++) {
                point[i] = match_armors[match_armor_id].measure_points.segment(i * 2, 2);
            }
            double average_length =
                ((point[0] - point[2]).norm() + (point[1] - point[3]).norm()) / 2;

            Light min_error_light_left;
            Light min_error_light_right;
            double min_error_left = DBL_MAX;
            double min_error_right = DBL_MAX;
            double min_error_left_angle;
            double min_error_right_angle;
            for (const auto& light: light_array) {
                // 找到长度满足要求的灯条
                double length_error = abs(light.length / average_length - 1);
                if (length_error < match_thre_length_light) {
                    double error_left = DBL_MAX;
                    double error_right = DBL_MAX;
                    Eigen::VectorXd measure(4);
                    measure << light.top.x, light.top.y, light.bottom.x, light.bottom.y;

                    // 寻找角度满足要求，距离误差最小灯条
                    auto left_angle_error = getError_Angle(armor_left.predict_points, measure);
                    auto right_angle_error = getError_Angle(armor_right.predict_points, measure);
                    if (left_angle_error < match_thre_angle_light) {
                        error_left = getError_Distance(armor_left.predict_points, measure);
                    }
                    if (right_angle_error < match_thre_angle_light) {
                        error_right = getError_Distance(armor_right.predict_points, measure);
                    }
                    if (error_right > error_left) {
                        if (error_left < min_error_left) {
                            min_error_left = error_left;
                            min_error_light_left = light;
                            min_error_left_angle = left_angle_error;
                        }
                    } else {
                        if (error_right < min_error_right) {
                            min_error_right = error_right;
                            min_error_light_right = light;
                            min_error_right_angle = right_angle_error;
                        }
                    }
                }
            }
            if (min_error_left < match_thre_distance_light) {
                armor_left.error_distance = min_error_left;
                armor_left.error_angle = min_error_left_angle;
                armor_left.match_type = MatchType::LIGHT_RIGHT;
                armor_left.measure_points.resize(4);
                armor_left.measure_points << min_error_light_left.top.x, min_error_light_left.top.y,
                    min_error_light_left.bottom.x, min_error_light_left.bottom.y;
            }
            if (min_error_right < match_thre_distance_light) {
                armor_right.error_distance = min_error_right;
                armor_right.error_angle = min_error_right_angle;
                armor_right.match_type = MatchType::LIGHT_LEFT;
                armor_right.measure_points.resize(4);
                armor_right.measure_points << min_error_light_right.top.x,
                    min_error_light_right.top.y, min_error_light_right.bottom.x,
                    min_error_light_right.bottom.y;
            }
        }

        /* 计算本次更新测量量维度 */
        this->measure_dimension = 0;
        for (int i = 0; i < this->tracked_armors_num; i++) {
            if (match_armors[i].match_type == MatchType::ARMOR) {
                this->measure_dimension += 8;
            } else if (match_armors[i].match_type == MatchType::LIGHT_LEFT || match_armors[i].match_type == MatchType::LIGHT_RIGHT)
            {
                this->measure_dimension += 4;
            }
        }

        /* 判断是否有匹配测量，有则进行 EKF 更新 */
        if (this->measure_dimension != 0) {
            this->matched = true;
            // 拼接测量向量z
            Eigen::VectorXd measurement(this->measure_dimension);
            int index = 0;
            for (int i = 0; i < this->tracked_armors_num; i++) {
                if (this->match_armors[i].match_type != MatchType::NONE) {
                    measurement.segment(index, match_armors[i].measure_points.size()) =
                        match_armors[i].measure_points;
                    index += match_armors[i].measure_points.size();
                }
            }
            // EKF 更新
            target_state = ekf.update(measurement);
        }
    }

    /* 相关信息打印 */
    // for (int i = 0; i < this->tracked_armors_num; i++) {
    //     std::cout << "armor[" << i << "]: need_match " << match_armors[i].need_match
    //               << ", error_distance " << match_armors[i].error_distance << ", error_angle "
    //               << match_armors[i].error_angle << ", match_type " << match_armors[i].match_type
    //               << std::endl;
    // }
    std::cout << "measure_dimension: " << measure_dimension << std::endl;

    /* 状态量限幅，防止观测器发散 */
    if (target_state(8) < 0.12) {
        target_state(8) = 0.12;
        ekf.setState(target_state);
    } else if (target_state(8) > 0.4) {
        target_state(8) = 0.4;
        ekf.setState(target_state);
    }
    if (target_state(10) < 0.12) {
        target_state(10) = 0.12;
        ekf.setState(target_state);
    } else if (target_state(10) > 0.4) {
        target_state(10) = 0.4;
        ekf.setState(target_state);
    }

    /* 跟踪状态状态机更新 */
    if (this->tracker_state == TrackState::DETECTING) {
        if (this->matched) {
            this->track_time_count += this->dt_;
            if (this->track_time_count > this->track_thres) {
                this->track_time_count = 0;
                this->tracker_state = TrackState::TRACKING;
            }
            std::cout << "TRACKINGstate " << this->tracker_state << ", " << this->track_time_count << ", " << this->dt_
                      << std::endl;
        } else {
            this->track_time_count = 0;
            this->tracker_state = TrackState::LOST;
        }
    } else if (this->tracker_state == TrackState::TRACKING) {
        if (!this->matched) {
            this->lost_time_count += this->dt_;
            this->tracker_state = TrackState::TEMP_LOST;
        }
        std::cout << "TRACKINGstate " << this->tracker_state << ", " << this->track_time_count << ", " << this->dt_
                      << std::endl;
    } else if (this->tracker_state == TrackState::TEMP_LOST) {
        if (!this->matched) {
            this->lost_time_count += this->dt_;
            if (this->lost_time_count > this->lost_thres) {
                this->lost_time_count = 0;
                this->tracker_state = TrackState::LOST;
            }
        } else {
            this->lost_time_count = 0;
            this->tracker_state = TrackState::TRACKING;
        }
        std::cout << "TRACKINGstate " << this->tracker_state << ", " << this->track_time_count << ", " << this->dt_
                      << std::endl;
    }

}

void auto_aim::RWTracker::set_enemy_color(uint8_t enemycolor){
    if(enemycolor==1){
        enemy_color_=Color::blue;
    }
    if(enemycolor==101){
        enemy_color_==Color::red;
    }
}

/************************************************************************************************************************
 * @brief   跟踪器初始化 EKF
 * @param   armor   需要跟踪的装甲板
 ***********************************************************************************************************************/
void auto_aim::RWTracker::initEKF(const Armor& armor) {

    double xa = armor.xyz_in_world[0];
    double ya = armor.xyz_in_world[1];
    double za = armor.xyz_in_world[2];
    double yaw = normalize_angle(armor.ypr_in_world[0]+M_PI);

    target_state = Eigen::VectorXd::Zero(11);
    double r = 0.26;//!这里先给了个先验值
    double xc = xa - r * cos(yaw);
    double yc = ya - r * sin(yaw);
    if(armor.name == ArmorName::outpost) {
        if(std::pow(za - (1.316-self_height),2) < std::pow(za - (1.416-self_height),2)) {
            outpost_height[0] = 1.316f - self_height;
            outpost_height[1] = 1.216f - self_height;
            outpost_height[2] = 1.116f - self_height;
            double za1 = 1.216f - self_height;
        } else {
            outpost_height[0] = 1.616f - self_height;
            outpost_height[1] = 1.516f - self_height;
            outpost_height[2] = 1.416f - self_height;
            double za1 = 1.516f - self_height;
        }
        auto *closest = std::min_element(std::begin(outpost_height), std::end(outpost_height),
            [za](double a, double b) {
                return std::abs(za - a) < std::abs(za - b);
            });
        circle_index = static_cast<int>(closest - std::begin(outpost_height));
    }
    target_state << xc, 0, yc, 0, za, 0, yaw, 0, r, za, r;

    std::cout << "init state: " << xc << ", " << yc << ", " << za << ", " << yaw << ", " << r
              << std::endl;

    ekf.reinitP();
    ekf.setState(target_state);
    ekf_prediction = target_state;
}

/************************************************************************************************************************
 * @brief   更新装甲板数字
 * @param   armor   当前跟踪的装甲板
 ***********************************************************************************************************************/
void auto_aim::RWTracker::updateTrackedArmorsNum(const Armor& armor) {
    if (armor.type == ArmorType::big
        && (tracked_id == ArmorName::three || tracked_id == ArmorName::four
            || tracked_id == ArmorName::five))
    {
        tracked_armors_num = 2;
    } else if (tracked_id == ArmorName::outpost) {
        tracked_armors_num = 3;
    } else {
        tracked_armors_num = 4;
    }
}

/************************************************************************************************************************
 * @brief   获取测量误差（平均距离）
 * @param   predict_points  预测点集（两点：灯条，四点：装甲板）
 * @param   measure_points  测量点集（两点：灯条，四点：装甲板）
 * @param   match_type      匹配类型
 * @return  double          平均距离误差 点的欧氏距离的平均
 ***********************************************************************************************************************/
double auto_aim::RWTracker::getError_Distance(
    const Eigen::VectorXd& predict_points,
    const Eigen::VectorXd& measure_points
) {
    double measure_error = 0;
    auto point_num = predict_points.size() / 2;
    auto error = predict_points - measure_points;
    for (int i = 0; i < point_num; i++) {
        Eigen::Vector2d point = error.segment(i * 2, 2);
        measure_error += point.norm();
    }
    return measure_error / point_num;
}

/************************************************************************************************************************
  * @brief   获取测量误差（轮廓角度平均平方误差）
  * @param   predict_points  预测点集（两点：灯条，四点：装甲板）
  * @param   measure_points  测量点集（两点：灯条，四点：装甲板）
  * @return  double      轮廓角度平均平方误差 和竖直方向夹角（弧度）的平方的平均
  ***********************************************************************************************************************/
double auto_aim::RWTracker::getError_Angle(
    const Eigen::VectorXd& predict_points,
    const Eigen::VectorXd& measure_points
) {
    double measure_error = 0;
    auto point_num = predict_points.size() / 2;
    if (point_num == 4) {
        int point_encode[4] = { 0, 1, 3, 2 };
        //* 原始数据是左下，左上，右上，右下，现在改为灯条索引
        for (int i = 0; i < 4; i++) {
            double target_angle, measure_angle;
            //* 这里乘二是访问像素坐标,加一是从u到v的转换
            target_angle = atan2(
                predict_points(2 * point_encode[(i + 1) % 4] + 1)
                    - predict_points(2 * point_encode[i] + 1),
                predict_points(2 * point_encode[(i + 1) % 4]) - predict_points(2 * point_encode[i])
            );
            measure_angle = atan2(
                measure_points(2 * point_encode[(i + 1) % 4] + 1)
                    - measure_points(2 * point_encode[i] + 1),
                measure_points(2 * point_encode[(i + 1) % 4]) - measure_points(2 * point_encode[i])
            );
            measure_error += pow((normalize_angle(target_angle - measure_angle)), 2);
        }
        measure_error /= 4;
    } else if (point_num == 2) {
        double target_angle, measure_angle;
        target_angle =
            atan2(predict_points(3) - predict_points(1), predict_points(2) - predict_points(0));
        measure_angle =
            atan2(measure_points(3) - measure_points(1), measure_points(2) - measure_points(0));
        measure_error = pow((normalize_angle(target_angle - measure_angle)), 2);
    }
    return measure_error;
}

/************************************************************************************************************************
  * @brief   获取测量误差（面积百分比误差）
  * @param   predict_points  预测点集（四点：装甲板）
  * @param   measure_points  测量点集（四点：装甲板）
  * @return  double          面积百分比误差 装甲板4点闭合四边形的面积比
  ***********************************************************************************************************************/
double auto_aim::RWTracker::getError_Area(
    const Eigen::VectorXd& predict_points,
    const Eigen::VectorXd& measure_points
) {
    double measure_error = 0;
    std::vector<cv::Point> armor_predict(4);
    std::vector<cv::Point> armor_measure(4);
    int point_encode[4] = { 0, 1, 3, 2 };
    for (int i = 0; i < 4; i++) {
        armor_predict[i].x = predict_points(2 * point_encode[i]);
        armor_predict[i].y = predict_points(2 * point_encode[i] + 1);
        armor_measure[i].x = measure_points(2 * point_encode[i]);
        armor_measure[i].y = measure_points(2 * point_encode[i] + 1);
    }
    auto area_predict = cv::contourArea(armor_predict);
    auto area_measure = cv::contourArea(armor_measure);
    measure_error = abs(1 - area_predict / area_measure);
    return measure_error;
}

/************************************************************************************************************************
 * @brief   单装甲板观测函数 H
 * @param   x           观测器状态量
 * @param   armor_id    目标装甲板编号
 * @param   match_type  匹配类型
 * @return  VectorXd    测量值向量
 ***********************************************************************************************************************/
Eigen::VectorXd
auto_aim::RWTracker::Singal_Armor_h(const Eigen::VectorXd& x, int armor_id, MatchType match_type) {
    double x_c = x(0), y_c = x(2), z_a, theta, r;
    double theta_pa;
    if (this->tracked_id == ArmorName::outpost) {
        theta_pa = -this->ARMOR_PITCH;
    } else {
        theta_pa = this->ARMOR_PITCH;
    }

    double L_a = this->SMALL_ARMOR_LENGTH;
    double W_a = this->SMALL_ARMOR_WIDTH;
    if (this->tracked_id == ArmorName::one) {
        L_a = this->LARGE_ARMOR_LENGTH;
        W_a = this->LARGE_ARMOR_WIDTH;
    }
    //clang-format off
    // f_x  0    c_x
    // 0    f_y  c_y
    // 0    0    1
    //clang-format on
    double f_x = this->cam_info_(0, 0), f_y = this->cam_info_(1, 1);
    double c_x = this->cam_info_(0, 2), c_y = this->cam_info_(1, 2);
    double delta_r = W_a * sin(theta_pa);
    double delta_z = W_a * cos(theta_pa);

    /* 依据编号赋值当前装甲板 theta、r、z */
    theta = x(6) + 2 * M_PI / (static_cast<int>(this->tracked_armors_num)) * armor_id;
    if (this->tracked_armors_num == 4 && armor_id % 2 == 1) {
        r = x(10); //当前装甲板半径 r2
        z_a = x(9); //当前装甲板高度 za2
    } else {
        r = x(8); //当前装甲板半径 r1
        z_a = x(4); //当前装甲板高度 za1
    }
    if (this->tracked_id == ArmorName::outpost) {
        z_a += 0.1 * circle_type[(circle_index + armor_id) % 3];;
    }
    double r_1t = r - delta_r / 2;
    double r_1b = r + delta_r / 2;

    /* 依据不同匹配类型计算 z */
    Eigen::Vector4d p_1w, p_2w, p_3w, p_4w;
    Eigen::Vector3d p_1c, p_2c, p_3c, p_4c;
    double u_1, v_1, u_2, v_2, u_3, v_3, u_4, v_4;
    if (match_type == MatchType::ARMOR || match_type == MatchType::LIGHT_LEFT) {
        // 计算世界系下装甲板关键点坐标（齐次坐标）
        p_1w << cos(theta) * r_1t + sin(theta) * L_a / 2 + x_c,
            sin(theta) * r_1t - cos(theta) * L_a / 2 + y_c, z_a + delta_z / 2, 1;
        p_3w << cos(theta) * r_1b + sin(theta) * L_a / 2 + x_c,
            sin(theta) * r_1b - cos(theta) * L_a / 2 + y_c, z_a - delta_z / 2, 1;
        // 变换到相机坐标系
        p_1c = (this->odom_camera_matrix * p_1w).head<3>();
        p_3c = (this->odom_camera_matrix * p_3w).head<3>();

        // 变换到图像坐标系
        u_1 = f_x * (p_1c.x() / p_1c.z()) + c_x;
        v_1 = f_y * (p_1c.y() / p_1c.z()) + c_y;
        u_3 = f_x * (p_3c.x() / p_3c.z()) + c_x;
        v_3 = f_y * (p_3c.y() / p_3c.z()) + c_y;

    }
    if (match_type == MatchType::ARMOR || match_type == MatchType::LIGHT_RIGHT) {
        // 计算世界系下装甲板关键点坐标（齐次坐标）
        p_2w << cos(theta) * r_1t - sin(theta) * L_a / 2 + x_c,
            sin(theta) * r_1t + cos(theta) * L_a / 2 + y_c, z_a + delta_z / 2, 1;
        p_4w << cos(theta) * r_1b - sin(theta) * L_a / 2 + x_c,
            sin(theta) * r_1b + cos(theta) * L_a / 2 + y_c, z_a - delta_z / 2, 1;
        // 变换到相机坐标系
        p_2c = (this->odom_camera_matrix * p_2w).head<3>();
        p_4c = (this->odom_camera_matrix * p_4w).head<3>();
        // 变换到图像坐标系
        u_2 = f_x * (p_2c.x() / p_2c.z()) + c_x;
        v_2 = f_y * (p_2c.y() / p_2c.z()) + c_y;
        u_4 = f_x * (p_4c.x() / p_4c.z()) + c_x;
        v_4 = f_y * (p_4c.y() / p_4c.z()) + c_y;
    }

    /* 依据不同匹配类型填充 z */
    Eigen::VectorXd z;
    if (match_type == MatchType::ARMOR) {
        z.resize(8);
        z << u_1, v_1, u_2, v_2, u_3, v_3, u_4, v_4;
    } else if (match_type == MatchType::LIGHT_LEFT) {
        z.resize(4);
        z << u_1, v_1, u_3, v_3;
    } else if (match_type == MatchType::LIGHT_RIGHT) {
        z.resize(4);
        z << u_2, v_2, u_4, v_4;
    }
    return z;
}

/************************************************************************************************************************
 * @brief   单装甲板观测函数 H 雅可比
 * @param   x           观测器状态量
 * @param   armor_id    目标装甲板编号
 * @param   match_type  匹配类型
 * @return  MatrixXd    H 雅可比矩阵
 ***********************************************************************************************************************/
 Eigen::MatrixXd auto_aim::RWTracker::Singal_Armor_jh(const Eigen::VectorXd & x, int armor_id, MatchType match_type)
 {
     double x_c = x(0), y_c = x(2), z_a, theta, r;
     double theta_pa;
     if (this->tracked_id == ArmorName::outpost)
     {
         theta_pa = -this->ARMOR_PITCH;
     }
     else
     {
         theta_pa = this->ARMOR_PITCH;
     }
 
     double L_a = this->SMALL_ARMOR_LENGTH;
     double W_a = this->SMALL_ARMOR_WIDTH;
     if (this->tracked_id == ArmorName::one){
         L_a = this->LARGE_ARMOR_LENGTH;
         W_a = this->LARGE_ARMOR_WIDTH;
     }
     
     double f_x = this->cam_info_(0, 0), f_y = this->cam_info_(1, 1);
     //* 标定后得到的像素焦距
     double t_11 = this->odom_camera_matrix(0, 0), t_12 = this->odom_camera_matrix(0, 1), t_13 = this->odom_camera_matrix(0, 2);
     double t_21 = this->odom_camera_matrix(1, 0), t_22 = this->odom_camera_matrix(1, 1), t_23 = this->odom_camera_matrix(1, 2);
     double t_31 = this->odom_camera_matrix(2, 0), t_32 = this->odom_camera_matrix(2, 1), t_33 = this->odom_camera_matrix(2, 2);
     double delta_r = W_a * sin(theta_pa); //* 半径方向的 
     double delta_z = W_a * cos(theta_pa); //* 高度方向的 
     //* pitch倾角
 
     // 依据编号赋值当前装甲板 theta、r、z
     theta = x(6) + 2*M_PI/(static_cast<int>(this->tracked_armors_num))*armor_id;
     //* 据前面误差阈值的逻辑动态赋值
     
     if (this->tracked_armors_num == 4 && armor_id % 2 == 1)
     {
         r = x(10);  //当前装甲板半径 r2
         z_a = x(9); //当前装甲板高度 za2
     }
     else
     {
         r = x(8);   //当前装甲板半径 r1
         z_a = x(4); //当前装甲板高度 za1
     }
     double r_1t = r - delta_r/2;
     double r_1b = r + delta_r/2;
 
     /* 依据不同匹配类型计算 z */
     Eigen::Vector4d p_1w, p_2w, p_3w, p_4w; //* 世界坐标系下4角点
     Eigen::Vector3d p_1c, p_2c, p_3c, p_4c; //* 相机坐标系下4角点
     double D_u1_D_xc, D_u1_D_yc, D_u1_D_za1, D_u1_D_theta, D_u1_D_r1; //* u1, v1 雅可比
     double D_v1_D_xc, D_v1_D_yc, D_v1_D_za1, D_v1_D_theta, D_v1_D_r1; //* u1, v1 雅可比
     double D_u2_D_xc, D_u2_D_yc, D_u2_D_za1, D_u2_D_theta, D_u2_D_r1; //* u2, v2 雅可比
     double D_v2_D_xc, D_v2_D_yc, D_v2_D_za1, D_v2_D_theta, D_v2_D_r1; //* u2, v2 雅可比
     double D_u3_D_xc, D_u3_D_yc, D_u3_D_za1, D_u3_D_theta, D_u3_D_r1; //* u3, v3 雅可比
     double D_v3_D_xc, D_v3_D_yc, D_v3_D_za1, D_v3_D_theta, D_v3_D_r1; //* u3, v3 雅可比
     double D_u4_D_xc, D_u4_D_yc, D_u4_D_za1, D_u4_D_theta, D_u4_D_r1; //* u4, v4 雅可比
     double D_v4_D_xc, D_v4_D_yc, D_v4_D_za1, D_v4_D_theta, D_v4_D_r1; //* u4, v4 雅可比
     if (match_type == MatchType::ARMOR || match_type == MatchType::LIGHT_LEFT)
     {
         // 计算世界系下装甲板关键点坐标（齐次坐标）
         p_1w << cos(theta)*r_1t + sin(theta)*L_a/2 + x_c,   sin(theta)*r_1t - cos(theta)*L_a/2 + y_c,   z_a + delta_z/2,   1;
         p_3w << cos(theta)*r_1b + sin(theta)*L_a/2 + x_c,   sin(theta)*r_1b - cos(theta)*L_a/2 + y_c,   z_a - delta_z/2,   1;
         // 变换到相机坐标系
         p_1c = (this->odom_camera_matrix * p_1w).head<3>();
         p_3c = (this->odom_camera_matrix * p_3w).head<3>();
         // u1, v1 雅可比计算
         double D_u1_D_x_p1c = f_x/p_1c.z();
         double D_u1_D_z_p1c = -f_x*p_1c.x()/pow(p_1c.z(), 2);
         double D_v1_D_y_p1c = f_y/p_1c.z();
         double D_v1_D_z_p1c = -f_y*p_1c.y()/pow(p_1c.z(), 2);
 
         double D_x_p1w_D_theta = cos(theta)*L_a/2 - r_1t*sin(theta);
         double D_y_p1w_D_theta = sin(theta)*L_a/2 + r_1t*cos(theta);
 
         double D_x_p1c_D_theta = t_11*D_x_p1w_D_theta + t_12*D_y_p1w_D_theta;
         double D_y_p1c_D_theta = t_21*D_x_p1w_D_theta + t_22*D_y_p1w_D_theta;
         double D_z_p1c_D_theta = t_31*D_x_p1w_D_theta + t_32*D_y_p1w_D_theta;
 
         double D_x_p1c_D_r1 = t_11*cos(theta) + t_12*sin(theta);
         double D_y_p1c_D_r1 = t_21*cos(theta) + t_22*sin(theta);
         double D_z_p1c_D_r1 = t_31*cos(theta) + t_32*sin(theta);
 
         D_u1_D_xc    = D_u1_D_x_p1c * t_11               + D_u1_D_z_p1c * t_31;
         D_u1_D_yc    = D_u1_D_x_p1c * t_12               + D_u1_D_z_p1c * t_32;
         D_u1_D_za1   = D_u1_D_x_p1c * t_13               + D_u1_D_z_p1c * t_33;
         D_u1_D_theta = D_u1_D_x_p1c * D_x_p1c_D_theta    + D_u1_D_z_p1c * D_z_p1c_D_theta;
         D_u1_D_r1    = D_u1_D_x_p1c * D_x_p1c_D_r1       + D_u1_D_z_p1c * D_z_p1c_D_r1;
 
         D_v1_D_xc    = D_v1_D_y_p1c * t_21               + D_v1_D_z_p1c * t_31;
         D_v1_D_yc    = D_v1_D_y_p1c * t_22               + D_v1_D_z_p1c * t_32;
         D_v1_D_za1   = D_v1_D_y_p1c * t_23               + D_v1_D_z_p1c * t_33;
         D_v1_D_theta = D_v1_D_y_p1c * D_y_p1c_D_theta    + D_v1_D_z_p1c * D_z_p1c_D_theta;
         D_v1_D_r1    = D_v1_D_y_p1c * D_y_p1c_D_r1       + D_v1_D_z_p1c * D_z_p1c_D_r1;
         // u3, v3 雅可比计算
         double D_u3_D_x_p3c = f_x/p_3c.z();
         double D_u3_D_z_p3c = -f_x*p_3c.x()/pow(p_3c.z(), 2);
         double D_v3_D_y_p3c = f_y/p_3c.z();
         double D_v3_D_z_p3c = -f_y*p_3c.y()/pow(p_3c.z(), 2);
 
         double D_x_p3w_D_theta = cos(theta)*L_a/2 - r_1b*sin(theta);
         double D_y_p3w_D_theta = sin(theta)*L_a/2 + r_1b*cos(theta);
         
         double D_x_p3c_D_theta = t_11*D_x_p3w_D_theta + t_12*D_y_p3w_D_theta;
         double D_y_p3c_D_theta = t_21*D_x_p3w_D_theta + t_22*D_y_p3w_D_theta;
         double D_z_p3c_D_theta = t_31*D_x_p3w_D_theta + t_32*D_y_p3w_D_theta;
 
         double D_x_p3c_D_r1 = t_11*cos(theta) + t_12*sin(theta);
         double D_y_p3c_D_r1 = t_21*cos(theta) + t_22*sin(theta);
         double D_z_p3c_D_r1 = t_31*cos(theta) + t_32*sin(theta);
 
         D_u3_D_xc    = D_u3_D_x_p3c * t_11               + D_u3_D_z_p3c * t_31;
         D_u3_D_yc    = D_u3_D_x_p3c * t_12               + D_u3_D_z_p3c * t_32;
         D_u3_D_za1   = D_u3_D_x_p3c * t_13               + D_u3_D_z_p3c * t_33;
         D_u3_D_theta = D_u3_D_x_p3c * D_x_p3c_D_theta    + D_u3_D_z_p3c * D_z_p3c_D_theta;
         D_u3_D_r1    = D_u3_D_x_p3c * D_x_p3c_D_r1       + D_u3_D_z_p3c * D_z_p3c_D_r1;
 
         D_v3_D_xc    = D_v3_D_y_p3c * t_21               + D_v3_D_z_p3c * t_31;
         D_v3_D_yc    = D_v3_D_y_p3c * t_22               + D_v3_D_z_p3c * t_32;
         D_v3_D_za1   = D_v3_D_y_p3c * t_23               + D_v3_D_z_p3c * t_33;
         D_v3_D_theta = D_v3_D_y_p3c * D_y_p3c_D_theta    + D_v3_D_z_p3c * D_z_p3c_D_theta;
         D_v3_D_r1    = D_v3_D_y_p3c * D_y_p3c_D_r1       + D_v3_D_z_p3c * D_z_p3c_D_r1;
     }
     if (match_type == MatchType::ARMOR || match_type == MatchType::LIGHT_RIGHT)
     {
         // 计算世界系下装甲板关键点坐标（齐次坐标）
         p_2w << cos(theta)*r_1t - sin(theta)*L_a/2 + x_c,   sin(theta)*r_1t + cos(theta)*L_a/2 + y_c,   z_a + delta_z/2,   1;
         p_4w << cos(theta)*r_1b - sin(theta)*L_a/2 + x_c,   sin(theta)*r_1b + cos(theta)*L_a/2 + y_c,   z_a - delta_z/2,   1;
         // 变换到相机坐标系
         p_2c = (this->odom_camera_matrix * p_2w).head<3>();
         p_4c = (this->odom_camera_matrix * p_4w).head<3>();
         // u2, v2 雅可比计算
         double D_u2_D_x_p2c = f_x/p_2c.z();
         double D_u2_D_z_p2c = -f_x*p_2c.x()/pow(p_2c.z(), 2);
         double D_v2_D_y_p2c = f_y/p_2c.z();
         double D_v2_D_z_p2c = -f_y*p_2c.y()/pow(p_2c.z(), 2);
 
         double D_x_p2w_D_theta = -cos(theta)*L_a/2 - r_1t*sin(theta);
         double D_y_p2w_D_theta = -sin(theta)*L_a/2 + r_1t*cos(theta);
         
         double D_x_p2c_D_theta = t_11*D_x_p2w_D_theta + t_12*D_y_p2w_D_theta;
         double D_y_p2c_D_theta = t_21*D_x_p2w_D_theta + t_22*D_y_p2w_D_theta;
         double D_z_p2c_D_theta = t_31*D_x_p2w_D_theta + t_32*D_y_p2w_D_theta;
 
         double D_x_p2c_D_r1 = t_11*cos(theta) + t_12*sin(theta);
         double D_y_p2c_D_r1 = t_21*cos(theta) + t_22*sin(theta);
         double D_z_p2c_D_r1 = t_31*cos(theta) + t_32*sin(theta);
 
         D_u2_D_xc    = D_u2_D_x_p2c * t_11               + D_u2_D_z_p2c * t_31;
         D_u2_D_yc    = D_u2_D_x_p2c * t_12               + D_u2_D_z_p2c * t_32;
         D_u2_D_za1   = D_u2_D_x_p2c * t_13               + D_u2_D_z_p2c * t_33;
         D_u2_D_theta = D_u2_D_x_p2c * D_x_p2c_D_theta    + D_u2_D_z_p2c * D_z_p2c_D_theta;
         D_u2_D_r1    = D_u2_D_x_p2c * D_x_p2c_D_r1       + D_u2_D_z_p2c * D_z_p2c_D_r1;
 
         D_v2_D_xc    = D_v2_D_y_p2c * t_21               + D_v2_D_z_p2c * t_31;
         D_v2_D_yc    = D_v2_D_y_p2c * t_22               + D_v2_D_z_p2c * t_32;
         D_v2_D_za1   = D_v2_D_y_p2c * t_23               + D_v2_D_z_p2c * t_33;
         D_v2_D_theta = D_v2_D_y_p2c * D_y_p2c_D_theta    + D_v2_D_z_p2c * D_z_p2c_D_theta;
         D_v2_D_r1    = D_v2_D_y_p2c * D_y_p2c_D_r1       + D_v2_D_z_p2c * D_z_p2c_D_r1;
         // u4, v4 雅可比计算
         double D_u4_D_x_p4c = f_x/p_4c.z();
         double D_u4_D_z_p4c = -f_x*p_4c.x()/pow(p_4c.z(), 2);
         double D_v4_D_y_p4c = f_y/p_4c.z();
         double D_v4_D_z_p4c = -f_y*p_4c.y()/pow(p_4c.z(), 2);
 
         double D_x_p4w_D_theta = -cos(theta)*L_a/2 - r_1b*sin(theta);
         double D_y_p4w_D_theta = -sin(theta)*L_a/2 + r_1b*cos(theta);
         
         double D_x_p4c_D_theta = t_11*D_x_p4w_D_theta + t_12*D_y_p4w_D_theta;
         double D_y_p4c_D_theta = t_21*D_x_p4w_D_theta + t_22*D_y_p4w_D_theta;
         double D_z_p4c_D_theta = t_31*D_x_p4w_D_theta + t_32*D_y_p4w_D_theta;
 
         double D_x_p4c_D_r1 = t_11*cos(theta) + t_12*sin(theta);
         double D_y_p4c_D_r1 = t_21*cos(theta) + t_22*sin(theta);
         double D_z_p4c_D_r1 = t_31*cos(theta) + t_32*sin(theta);
 
         D_u4_D_xc    = D_u4_D_x_p4c * t_11               + D_u4_D_z_p4c * t_31;
         D_u4_D_yc    = D_u4_D_x_p4c * t_12               + D_u4_D_z_p4c * t_32;
         D_u4_D_za1   = D_u4_D_x_p4c * t_13               + D_u4_D_z_p4c * t_33;
         D_u4_D_theta = D_u4_D_x_p4c * D_x_p4c_D_theta    + D_u4_D_z_p4c * D_z_p4c_D_theta;
         D_u4_D_r1    = D_u4_D_x_p4c * D_x_p4c_D_r1       + D_u4_D_z_p4c * D_z_p4c_D_r1;
 
         D_v4_D_xc    = D_v4_D_y_p4c * t_21               + D_v4_D_z_p4c * t_31;
         D_v4_D_yc    = D_v4_D_y_p4c * t_22               + D_v4_D_z_p4c * t_32;
         D_v4_D_za1   = D_v4_D_y_p4c * t_23               + D_v4_D_z_p4c * t_33;
         D_v4_D_theta = D_v4_D_y_p4c * D_y_p4c_D_theta    + D_v4_D_z_p4c * D_z_p4c_D_theta;
         D_v4_D_r1    = D_v4_D_y_p4c * D_y_p4c_D_r1       + D_v4_D_z_p4c * D_z_p4c_D_r1;
     }
 
     Eigen::MatrixXd h;
     if (this->tracked_armors_num == 4 && armor_id % 2 == 1)
     {
         if (match_type == MatchType::ARMOR)
         {
             h.resize(8, 11);
             // clang-format off
             //      x_c             v_xc,   y_c         v_yc,   z_a1        v_za,   theta,          omega,  r_1,        z_a2,       r_2
             h <<    D_u1_D_xc,      0,      D_u1_D_yc,  0,      0,          0,      D_u1_D_theta,   0,      0,          D_u1_D_za1, D_u1_D_r1,
                     D_v1_D_xc,      0,      D_v1_D_yc,  0,      0,          0,      D_v1_D_theta,   0,      0,          D_v1_D_za1, D_v1_D_r1,
                     D_u2_D_xc,      0,      D_u2_D_yc,  0,      0,          0,      D_u2_D_theta,   0,      0,          D_u2_D_za1, D_u2_D_r1,
                     D_v2_D_xc,      0,      D_v2_D_yc,  0,      0,          0,      D_v2_D_theta,   0,      0,          D_v2_D_za1, D_v2_D_r1,
                     D_u3_D_xc,      0,      D_u3_D_yc,  0,      0,          0,      D_u3_D_theta,   0,      0,          D_u3_D_za1, D_u3_D_r1,
                     D_v3_D_xc,      0,      D_v3_D_yc,  0,      0,          0,      D_v3_D_theta,   0,      0,          D_v3_D_za1, D_v3_D_r1,
                     D_u4_D_xc,      0,      D_u4_D_yc,  0,      0,          0,      D_u4_D_theta,   0,      0,          D_u4_D_za1, D_u4_D_r1,
                     D_v4_D_xc,      0,      D_v4_D_yc,  0,      0,          0,      D_v4_D_theta,   0,      0,          D_v4_D_za1, D_v4_D_r1;
             // clang-format on
         }
         else if (match_type == MatchType::LIGHT_LEFT)
         {
             h.resize(4, 11);
             // clang-format off
             //      x_c             v_xc,   y_c         v_yc,   z_a1        v_za,   theta,          omega,  r_1,        z_a2,       r_2
             h <<    D_u1_D_xc,      0,      D_u1_D_yc,  0,      0,          0,      D_u1_D_theta,   0,      0,          D_u1_D_za1, D_u1_D_r1,
                     D_v1_D_xc,      0,      D_v1_D_yc,  0,      0,          0,      D_v1_D_theta,   0,      0,          D_v1_D_za1, D_v1_D_r1,
                     D_u3_D_xc,      0,      D_u3_D_yc,  0,      0,          0,      D_u3_D_theta,   0,      0,          D_u3_D_za1, D_u3_D_r1,
                     D_v3_D_xc,      0,      D_v3_D_yc,  0,      0,          0,      D_v3_D_theta,   0,      0,          D_v3_D_za1, D_v3_D_r1;
             // clang-format on
         }
         else if (match_type == MatchType::LIGHT_RIGHT)
         {
             h.resize(4, 11);
             // clang-format off
             //      x_c             v_xc,   y_c         v_yc,   z_a1        v_za,   theta,          omega,  r_1,        z_a2,       r_2
             h <<    D_u2_D_xc,      0,      D_u2_D_yc,  0,      0,          0,      D_u2_D_theta,   0,      0,          D_u2_D_za1, D_u2_D_r1,
                     D_v2_D_xc,      0,      D_v2_D_yc,  0,      0,          0,      D_v2_D_theta,   0,      0,          D_v2_D_za1, D_v2_D_r1,
                     D_u4_D_xc,      0,      D_u4_D_yc,  0,      0,          0,      D_u4_D_theta,   0,      0,          D_u4_D_za1, D_u4_D_r1,
                     D_v4_D_xc,      0,      D_v4_D_yc,  0,      0,          0,      D_v4_D_theta,   0,      0,          D_v4_D_za1, D_v4_D_r1;
             // clang-format on
         }
     }
     else
     {
         if (match_type == MatchType::ARMOR)
         {
             h.resize(8, 11);
             // clang-format off
             //      x_c             v_xc,   y_c         v_yc,   z_a1        v_za,   theta,          omega,  r_1,        z_a2,       r_2
             h <<    D_u1_D_xc,      0,      D_u1_D_yc,  0,      D_u1_D_za1, 0,      D_u1_D_theta,   0,      D_u1_D_r1,  0,          0,
                     D_v1_D_xc,      0,      D_v1_D_yc,  0,      D_v1_D_za1, 0,      D_v1_D_theta,   0,      D_v1_D_r1,  0,          0,
                     D_u2_D_xc,      0,      D_u2_D_yc,  0,      D_u2_D_za1, 0,      D_u2_D_theta,   0,      D_u2_D_r1,  0,          0,
                     D_v2_D_xc,      0,      D_v2_D_yc,  0,      D_v2_D_za1, 0,      D_v2_D_theta,   0,      D_v2_D_r1,  0,          0,
                     D_u3_D_xc,      0,      D_u3_D_yc,  0,      D_u3_D_za1, 0,      D_u3_D_theta,   0,      D_u3_D_r1,  0,          0,
                     D_v3_D_xc,      0,      D_v3_D_yc,  0,      D_v3_D_za1, 0,      D_v3_D_theta,   0,      D_v3_D_r1,  0,          0,
                     D_u4_D_xc,      0,      D_u4_D_yc,  0,      D_u4_D_za1, 0,      D_u4_D_theta,   0,      D_u4_D_r1,  0,          0,
                     D_v4_D_xc,      0,      D_v4_D_yc,  0,      D_v4_D_za1, 0,      D_v4_D_theta,   0,      D_v4_D_r1,  0,          0;
             // clang-format on
         }
         else if (match_type == MatchType::LIGHT_LEFT)
         {
             h.resize(4, 11);
             // clang-format off
             //      x_c             v_xc,   y_c         v_yc,   z_a1        v_za,   theta,          omega,  r_1,        z_a2,       r_2
             h <<    D_u1_D_xc,      0,      D_u1_D_yc,  0,      D_u1_D_za1, 0,      D_u1_D_theta,   0,      D_u1_D_r1,  0,          0,
                     D_v1_D_xc,      0,      D_v1_D_yc,  0,      D_v1_D_za1, 0,      D_v1_D_theta,   0,      D_v1_D_r1,  0,          0,
                     D_u3_D_xc,      0,      D_u3_D_yc,  0,      D_u3_D_za1, 0,      D_u3_D_theta,   0,      D_u3_D_r1,  0,          0,
                     D_v3_D_xc,      0,      D_v3_D_yc,  0,      D_v3_D_za1, 0,      D_v3_D_theta,   0,      D_v3_D_r1,  0,          0;
             // clang-format on
         }
         else if (match_type == MatchType::LIGHT_RIGHT)
         {
             h.resize(4, 11);
             // clang-format off
             //      x_c             v_xc,   y_c         v_yc,   z_a1        v_za,   theta,          omega,  r_1,        z_a2,       r_2
             h <<    D_u2_D_xc,      0,      D_u2_D_yc,  0,      D_u2_D_za1, 0,      D_u2_D_theta,   0,      D_u2_D_r1,  0,          0,
                     D_v2_D_xc,      0,      D_v2_D_yc,  0,      D_v2_D_za1, 0,      D_v2_D_theta,   0,      D_v2_D_r1,  0,          0,
                     D_u4_D_xc,      0,      D_u4_D_yc,  0,      D_u4_D_za1, 0,      D_u4_D_theta,   0,      D_u4_D_r1,  0,          0,
                     D_v4_D_xc,      0,      D_v4_D_yc,  0,      D_v4_D_za1, 0,      D_v4_D_theta,   0,      D_v4_D_r1,  0,          0;
             // clang-format on
         }
     }
     return h;
 }
 
double auto_aim::RWTracker::normalize_angle(double angle) {
    const double result = fmod(angle + M_PI, 2.0 * M_PI);
    if (result <= 0.0)
        return result + M_PI;
    return result - M_PI;
}

/************************************************************************************************************************
 * @brief   跟踪结果绘制函数
 * @param   img     待绘制图片
 ***********************************************************************************************************************/
 void auto_aim::RWTracker::drawResults(cv::Mat & img)
 {
     if (this->match_armors.size() != this->tracked_armors_num)
     {
         return;
     }
 
     /* 绘制本次预测装甲板 */
     std::cout<<"debug: drawResults"<<std::endl;
     for (int i = 0; i < this->tracked_armors_num; i++)
     {
         cv::Scalar cv_color = cv::Scalar(255, 0, 0);
         std::vector<cv::Point2f> armor_corner(4);
         // 获取装甲板角点
         auto target_armor = Singal_Armor_h(this->ekf_prediction, i, MatchType::ARMOR);
         for (int j = 0; j < 4; j++)
         {
             armor_corner[j].x = target_armor(2*j);
             armor_corner[j].y = target_armor(2*j+1);
         }
         // 绘制测量与预测装甲板间关系
         if (this->match_armors[i].match_type == MatchType::ARMOR)
         {
             for (int j = 0; j < 4; j++)
             {
                 cv::Point2f armor_corner_measure;
                 armor_corner_measure.x = this->match_armors[i].measure_points(2*j);
                 armor_corner_measure.y = this->match_armors[i].measure_points(2*j+1);
                 cv::line(img, armor_corner[j], armor_corner_measure, cv_color, 1);
             }
         }
         else if (this->match_armors[i].match_type == MatchType::LIGHT_LEFT)
         {
             cv::Point2f armor_corner_measure[2];
             for (int j = 0; j < 2; j++)
             {
                 armor_corner_measure[j].x = this->match_armors[i].measure_points(2*j);
                 armor_corner_measure[j].y = this->match_armors[i].measure_points(2*j+1);
             }
             cv::line(img, armor_corner[0], armor_corner_measure[0], cv_color, 1);
             cv::line(img, armor_corner[2], armor_corner_measure[1], cv_color, 1);
         }
         else if (this->match_armors[i].match_type == MatchType::LIGHT_RIGHT)
         {
             cv::Point2f armor_corner_measure[2];
             for (int j = 0; j < 2; j++)
             {
                 armor_corner_measure[j].x = this->match_armors[i].measure_points(2*j);
                 armor_corner_measure[j].y = this->match_armors[i].measure_points(2*j+1);
             }
             cv::line(img, armor_corner[1], armor_corner_measure[0], cv_color, 1);
             cv::line(img, armor_corner[3], armor_corner_measure[1], cv_color, 1);
         }
         // 绘制装甲板
         cv::line(img, armor_corner[0], armor_corner[1], cv_color, 1);
         cv::line(img, armor_corner[2], armor_corner[3], cv_color, 1);
         cv::line(img, armor_corner[0], armor_corner[2], cv_color, 1);
         cv::line(img, armor_corner[1], armor_corner[3], cv_color, 1);
     }
 
     /* 绘制本次更新装甲板 */
     for (int i = 0; i < this->tracked_armors_num; i++)
     {
         // 获取跟踪器装甲板角点
         auto target_armor = Singal_Armor_h(this->target_state, i, MatchType::ARMOR);
         std::vector<cv::Point2f> armor_corner(4);
         for (int j = 0; j < 4; j++)
         {
             armor_corner[j].x = target_armor(2*j);
             armor_corner[j].y = target_armor(2*j+1);
         }
         // 绘制装甲板
         cv::Scalar cv_color;
         if (this->match_armors[i].match_type == MatchType::ARMOR)
         {
             cv_color = cv::Scalar(0, 255, 0);
         }
         else if (this->match_armors[i].match_type == MatchType::LIGHT_LEFT ||
                  this->match_armors[i].match_type == MatchType::LIGHT_RIGHT)
         {
             cv_color = cv::Scalar(100, 149, 237);
         }
         else
         {
             cv_color = cv::Scalar(255, 255, 255);
         }
         cv::line(img, armor_corner[0], armor_corner[1], cv_color, 1);
         cv::line(img, armor_corner[2], armor_corner[3], cv_color, 1);
         cv::line(img, armor_corner[0], armor_corner[2], cv_color, 1);
         cv::line(img, armor_corner[1], armor_corner[3], cv_color, 1);
         // 绘制装甲板信息
         std::stringstream armor_info;
         armor_info << "id: " << i;
         if (this->match_armors[i].match_type != MatchType::NONE)
         {
             armor_info << " | error: " << std::fixed << std::setprecision(1) << this->match_armors[i].error_distance;
         }
         auto armor_info_s = armor_info.str();
         cv::Point2f text_position(armor_corner[2].x, armor_corner[2].y+40);
         cv::putText(img, armor_info_s, text_position, cv::FONT_HERSHEY_SIMPLEX, 0.5, cv_color, 1);
 
         std::stringstream armor_info_2;
         if (this->match_armors[i].match_type == MatchType::ARMOR)
         {
             armor_info_2 << "type: armor";
         }
         else if (this->match_armors[i].match_type == MatchType::LIGHT_LEFT)
         {
             armor_info_2 << "type: left-light";
         }
         else if (this->match_armors[i].match_type == MatchType::LIGHT_RIGHT)
         {
             armor_info_2 << "type: right-light";
         }
         armor_info_s = armor_info_2.str();
         text_position.y += 15;
         cv::putText(img, armor_info_s, text_position, cv::FONT_HERSHEY_SIMPLEX, 0.5, cv_color, 1);
        
         {
            armor_info_s = armor_info_2.str();
            text_position.y += 15;
            cv::putText(img, armor_info_s, text_position, cv::FONT_HERSHEY_SIMPLEX, 0.5, cv_color, 1);
            
            // 显示装甲板高度
            double armor_height;
            if (this->tracked_id == ArmorName::outpost) {
                // 前哨站：根据 circle_index 和 armor_id 获取高度
                armor_height = outpost_height[(circle_index + i) % 3];
            } else {
                // 普通装甲板：根据 armor_id 选择 za1 或 za2
                if (this->tracked_armors_num == 4 && i % 2 == 1) {
                    armor_height = this->target_state(9);  // za2
                } else {
                    armor_height = this->target_state(4);  // za1
                }
            }
            
            std::stringstream armor_info_height;
            armor_info_height << "height: " << std::fixed << std::setprecision(3) << armor_height << "m";
            armor_info_s = armor_info_height.str();
            text_position.y += 15;
            cv::putText(img, armor_info_s, text_position, cv::FONT_HERSHEY_SIMPLEX, 0.5, cv_color, 1);
        }
     }
 }
 

/************************************************************************************************************************
 * @brief 新观测器的target接口实现
 * 
***********************************************************************************************************************/
auto_aim::RWTracker::ShowTargetInterface& auto_aim::RWTracker::get_interface(){
    return this->show_target_interface_;
}

std::vector<Eigen::Vector4d> auto_aim::RWTracker::ShowTargetInterface::armor_xyza_list() const {
    std::vector<Eigen::Vector4d> armor_list;
    const auto& state = this->get_state();  // 通过引用访问 tracker 的状态
    
    // RWTracker 状态: [xc, v_xc, yc, v_yc, za_1, v_za, theta, omega, r1, za_2, r2]
    double xc = state(0);
    double yc = state(2);
    double theta = state(6);
    
    for (int i = 0; i < tracker_.tracked_armors_num; i++) {  // 通过 tracker_ 访问
        double armor_theta = tracker_.normalize_angle(
            theta + 2 * M_PI / static_cast<double>(tracker_.tracked_armors_num) * i);
        
        double r, z;
        if (tracker_.tracked_armors_num == 4 && i % 2 == 1) {
            r = state(10);  // r2
            z = state(9);    // za_2
        } else {
            r = state(8);   // r1
            z = state(4);    // za_1
        }
        
        double armor_x = xc + r * std::cos(armor_theta);
        double armor_y = yc + r * std::sin(armor_theta);
        double armor_z = z;
        
        armor_list.push_back({armor_x, armor_y, armor_z, armor_theta});
    }
    
    return armor_list;
}

// void auto_aim::RWTracker::ShowTargetInterface::predict(double dt){
//     // // Eigen::VectorXd original_state = tracker_.target_state;
//     // // tracker_.target_state = temp_state_;
//     // // tracker_.dt_ = dt;
//     // // temp_state_ = tracker_.ekf.predict();
//     // // tracker_.target_state = original_state;

//     // double temp_dt = tracker_.dt_;
//     // tracker_.dt_ = dt;
//     // temp_state_ = tracker_.ekf.fake_predict();
//     // // tracker_.target_state = original_state;
//     // tracker_.dt_ = temp_dt;
//     Eigen::VectorXd base_state;
//     if (temp_state_.size() == 0) {
//         base_state = tracker_.target_state;
//     } else {
//         base_state = temp_state_;
//     }
    
//     // 手动应用状态转移函数（与 EKF 中的 f 函数相同）
//     // state: [xc, v_xc, yc, v_yc, za_1, v_za, theta, omega, r1, za_2, r2]
//     temp_state_ = base_state;
//     temp_state_(0) += base_state(1) * dt;  // xc += v_xc * dt
//     temp_state_(2) += base_state(3) * dt;  // yc += v_yc * dt
//     temp_state_(4) += base_state(5) * dt;  // za_1 += v_za * dt
//     temp_state_(6) += base_state(7) * dt;  // theta += omega * dt
//     temp_state_(9) += base_state(5) * dt;  // za_2 += v_za * dt
//     // 其他状态保持不变（r1, r2, v_xc, v_yc, v_za, omega）
//     // 归一化角度，防止角度超出合理范围
//     temp_state_(6) = tracker_.normalize_angle(temp_state_(6));
    
//     // 检查预测状态的合理性
//     // 如果位置或高度异常，可能预测出现了问题
//     if (std::abs(temp_state_(0)) > 100 || std::abs(temp_state_(2)) > 100 || 
//         temp_state_(4) < -10 || temp_state_(4) > 10 ||
//         temp_state_(9) < -10 || temp_state_(9) > 10) {
//         // 预测状态异常，重置为原始状态
//         temp_state_ = tracker_.target_state;
//     }
// }

// void auto_aim::RWTracker::ShowTargetInterface::predict(std::chrono::steady_clock::time_point t) const {
//     if (last_predict_time_ == std::chrono::steady_clock::time_point{}) {
//         // 首次调用，初始化
//         last_predict_time_ = t;
//         if (!has_temp_state_) {
//             temp_state_ = tracker_.target_state;
//             has_temp_state_ = true;
//         }
//         return;
//     }
//     auto dt = std::chrono::duration<double>(t - last_predict_time_).count();
//     last_predict_time_ = t;
//     predict(dt);
// }

const Eigen::VectorXd& auto_aim::RWTracker::ShowTargetInterface::get_state() const {
    if(state_.size() == 0) {
        return tracker_.target_state;
    }
    return state_;
}
} //namespace auto_aim