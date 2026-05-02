#include "rw_tracker.hpp"
#include "armor.hpp"
#include "ceres/jet.h"
#include "tools/extended_kalman_filter.hpp"
#include <cmath>
#include <list>
#include <type_traits>
#include <vector>

namespace auto_aim {
namespace {
    struct ConsistentOcclusionPointDebug {
        double cos_side = 0.0;
        double cos_extend = 0.0;
        double alpha_occ = 0.0;
        double alpha_ext = 0.0;
        double side_offset = 0.0;
        double up_offset = 0.0;
        double forward_offset = 0.0;
    };

    struct ConsistentOcclusionModelDebug {
        ConsistentOcclusionPointDebug left;
        ConsistentOcclusionPointDebug right;
    };

    void fill_consistent_occlusion_debug_json_like(
        auto_aim::RWTracker::ConsistentOcclusionDebugInfo& out,
        const ConsistentOcclusionModelDebug& local_debug
    );

    template<typename T>
    Eigen::Matrix<T, Eigen::Dynamic, 1> build_occluded_measurement_ceres(
        const Eigen::Matrix<T, Eigen::Dynamic, 1>& x,
        int armor_id,
        RWTracker::MatchType match_type,
        ArmorName tracked_id,
        int tracked_armors_num,
        int circle_index,
        const int (&circle_type)[3],
        const Eigen::Matrix3d& cam_info,
        const Eigen::Matrix4d& odom_camera_matrix
    ) {
        const T x_c = x(0);
        const T y_c = x(2);

        T theta_pa = tracked_id == ArmorName::outpost ? T(-RWTracker::ARMOR_PITCH)
                                                      : T(RWTracker::ARMOR_PITCH);

        T L_a = T(RWTracker::SMALL_ARMOR_LENGTH);
        T W_a = T(RWTracker::SMALL_ARMOR_WIDTH);
        if (tracked_id == ArmorName::one) {
            L_a = T(RWTracker::LARGE_ARMOR_LENGTH);
            W_a = T(RWTracker::LARGE_ARMOR_WIDTH);
        }

        const T f_x = T(cam_info(0, 0));
        const T f_y = T(cam_info(1, 1));
        const T c_x = T(cam_info(0, 2));
        const T c_y = T(cam_info(1, 2));

        //*视线方向那个向量为基准，逆时针为正
        //*每次观测中单个灯条也会被自动分配自己的id,所以自动计算theta即可
        T theta = x(6) + T(2.0 * M_PI / tracked_armors_num * armor_id);
        T r;
        T z_a;
        if (tracked_armors_num == 4 && armor_id % 2 == 1) {
            r = x(10);
            z_a = x(9);
        } else {
            r = x(8);
            z_a = x(4);
        }
        if (tracked_id == ArmorName::outpost) {
            z_a += T(0.1 * circle_type[(circle_index + armor_id) % 3]);
        }

        Eigen::Matrix<T, 3, 3> R_c_w = odom_camera_matrix.block<3, 3>(0, 0).template cast<T>();
        Eigen::Matrix<T, 3, 1> t_c_w = odom_camera_matrix.block<3, 1>(0, 3).template cast<T>();
        Eigen::Matrix<T, 3, 1> P_cam_w = -R_c_w.transpose() * t_c_w;

        Eigen::Matrix<T, 3, 1> N_w(
            ceres::cos(theta) * ceres::cos(theta_pa),
            ceres::sin(theta) * ceres::cos(theta_pa),
            ceres::sin(theta_pa)
        );
        Eigen::Matrix<T, 3, 1> V_up_w(
            -ceres::cos(theta) * ceres::sin(theta_pa),
            -ceres::sin(theta) * ceres::sin(theta_pa),
            ceres::cos(theta_pa)
        );
        Eigen::Matrix<T, 3, 1> V_side_w = V_up_w.cross(N_w); //* 指向右边
        // Eigen::Matrix<T, 3, 1> V_right_upper_w = (V_up_w + V_side_w)/T(2.0);
        // Eigen::Matrix<T, 3, 1> V_left_upper_w = (V_up_w - V_side_w)/T(2.0);

        Eigen::Matrix<T, 3, 1> center_l(
            x_c + r * ceres::cos(theta) + ceres::sin(theta) * L_a / T(2.0),
            y_c + r * ceres::sin(theta) - ceres::cos(theta) * L_a / T(2.0),
            z_a
        );
        Eigen::Matrix<T, 3, 1> center_r(
            x_c + r * ceres::cos(theta) - ceres::sin(theta) * L_a / T(2.0),
            y_c + r * ceres::sin(theta) + ceres::cos(theta) * L_a / T(2.0),
            z_a
        );

        auto calc_visible_bounds = [&](const Eigen::Matrix<T, 3, 1>& center,
                                       T& side_offset,
                                       T& up_offset,
                                       T& forward_offset,
                                       int if_left) {
            //? if_left左边为1,右边为-1
            const double d_recess = 0.003;
            const T half_w = W_a / T(2.0);
            const T light_width = T(0.007);
            const T light_side_width = T(0.004);
            const T cos_limit = T(1.0 - 1e-3);
            const T min_sin_sq = T(1e-3);

            up_offset = half_w;

            const Eigen::Matrix<T, 3, 1> V_cam = P_cam_w - center;
            //* 看向灯条的视线，所以直接结合视线方向和灯条的左右判断就好了
            // Eigen::Matrix<T, 3, 1> V_cam_unit = V_cam.normalized();

            // T landscape_sheld = V_cam.dot(V_side_w)*d_recess/T(2.0);
            // T landscape_extend = V_cam.dot(V_side_w)*light_side_width/T(2.0);
            //* 先不考虑竖向的遮挡
            //* 但确实横向的遮挡会导致竖向的变化
            // T portrait_sheld =  V_cam.dot(V_right_upper_w)*d_recess;

            const Eigen::Matrix<T, 3, 1> V_sheld_cam =
                V_cam - V_side_w * light_width / T(2.0) * T(if_left);
            const Eigen::Matrix<T, 3, 1> V_extend_cam =
                V_cam + V_side_w * light_width / T(2.0) * T(if_left);
            //*V_sheld_cam.normalized().dot(V_side_w)算是夹角的cos
            const T cos = V_sheld_cam.normalized().dot(V_side_w) * T(if_left);
            const T cos_extend = V_extend_cam.normalized().dot(V_side_w) * T(if_left);
            const T cos_clamped = ceres::fmin(ceres::fmax(cos, T(0.0)), cos_limit);
            const T sin_sq = ceres::fmax(T(1.0) - cos_clamped * cos_clamped, min_sin_sq);
            T landscape_sheld =
                cos_clamped > T(0) ? cos_clamped / ceres::sqrt(sin_sq) * d_recess / T(2.0) : T(0.0);
            //! 延伸相当于角点在灯条法向量上进行延伸
            T landscape_extend = cos_extend < T(0)
                ? -V_extend_cam.normalized().dot(N_w) * light_side_width / T(2.0)
                : T(0.0);
            forward_offset = -landscape_extend;
            side_offset = landscape_sheld;
            up_offset += landscape_sheld;
            // if (side_offset < 0.0)
            //     std::cout << "side_offset in rw_tracker ceres error" << std::endl;
        };

        // T y_l_top, y_l_bot, y_r_top, y_r_bot;
        // T x_l_top, x_l_bot, x_r_top, x_r_bot;
        T side_l_offset = T(0), up_l_offset = T(0), forward_l_offset = T(0);
        T side_r_offset = T(0), up_r_offset = T(0), forward_r_offset = T(0);
        //* 计算遮挡后的三维偏差
        calc_visible_bounds(center_l, side_l_offset, up_l_offset, forward_l_offset, 1);
        calc_visible_bounds(center_r, side_r_offset, up_r_offset, forward_r_offset, -1);

        //* 结合物理过程后得到更真实的角点
        auto project_point = [&](const Eigen::Matrix<T, 3, 1>& center,
                                 const T& side_offset,
                                 const T& up_offset,
                                 const T& forward_offset) -> Eigen::Matrix<T, 2, 1> {
            Eigen::Matrix<T, 4, 1> p_w;
            p_w << center + V_up_w * up_offset + V_side_w * side_offset + N_w * forward_offset,
                T(1.0); //?有遮挡的
            // p_w << center + V_up_w * up_offset, T(1.0);//?无遮挡的，注意把calc_visible_bounds中y_offset += landscape_sheld;去掉
            const Eigen::Matrix<T, 3, 1> p_c =
                (odom_camera_matrix.template cast<T>() * p_w).template head<3>();

            Eigen::Matrix<T, 2, 1> uv;
            uv << f_x * (p_c.x() / p_c.z()) + c_x, f_y * (p_c.y() / p_c.z()) + c_y;
            return uv;
        };

        Eigen::Matrix<T, Eigen::Dynamic, 1> z;
        if (match_type == RWTracker::MatchType::ARMOR) {
            z.resize(8);
            const auto p1 =
                project_point(center_l, side_l_offset, up_l_offset, forward_l_offset); //*左上
            const auto p2 =
                project_point(center_r, -side_r_offset, up_r_offset, forward_r_offset); //*右上
            const auto p3 =
                project_point(center_l, side_l_offset, -up_l_offset, forward_l_offset); //*左下
            const auto p4 =
                project_point(center_r, -side_r_offset, -up_r_offset, forward_r_offset); //*右下
            z << p1.x(), p1.y(), p2.x(), p2.y(), p3.x(), p3.y(), p4.x(), p4.y();
        } else if (match_type == RWTracker::MatchType::LIGHT_LEFT) {
            z.resize(4);
            const auto p1 = project_point(center_l, side_l_offset, up_l_offset, forward_l_offset);
            const auto p3 = project_point(center_l, side_l_offset, -up_l_offset, forward_l_offset);
            z << p1.x(), p1.y(), p3.x(), p3.y();
        } else if (match_type == RWTracker::MatchType::LIGHT_RIGHT) {
            z.resize(4);
            const auto p2 = project_point(center_r, -side_r_offset, up_r_offset, forward_r_offset);
            const auto p4 = project_point(center_r, -side_r_offset, -up_r_offset, forward_r_offset);
            z << p2.x(), p2.y(), p4.x(), p4.y();
        }
        return z;
    }

    template<typename T>
    Eigen::Matrix<T, Eigen::Dynamic, 1> build_occluded_measurement_ceres_consis(
        const Eigen::Matrix<T, Eigen::Dynamic, 1>& x,
        int armor_id,
        RWTracker::MatchType match_type,
        ArmorName tracked_id,
        int tracked_armors_num,
        int circle_index,
        const int (&circle_type)[3],
        const Eigen::Matrix3d& cam_info,
        const Eigen::Matrix4d& odom_camera_matrix,
        ConsistentOcclusionModelDebug* debug = nullptr
    ) {
        auto smooth_sigmoid = [](const T& value, const T& center, const T& sharpness) -> T {
            return T(1.0) / (T(1.0) + ceres::exp(-sharpness * (value - center)));
        };

        auto smooth_pos = [](const T& value, const T& eps) -> T {
            return T(0.5) * (value + ceres::sqrt(value * value + eps * eps));
        };

        auto smooth_sat = [](const T& value, const T& limit) -> T {
            const T ratio = value / limit;
            return value / ceres::sqrt(T(1.0) + ratio * ratio);
        };

        auto safe_normalize = [](const Eigen::Matrix<T, 3, 1>& v) -> Eigen::Matrix<T, 3, 1> {
            const T eps = T(1e-9);
            return v / ceres::sqrt(v.squaredNorm() + eps * eps);
        };

        const T x_c = x(0);
        const T y_c = x(2);

        T theta_pa = tracked_id == ArmorName::outpost ? T(-RWTracker::ARMOR_PITCH)
                                                      : T(RWTracker::ARMOR_PITCH);

        T L_a = T(RWTracker::SMALL_ARMOR_LENGTH);
        T W_a = T(RWTracker::SMALL_ARMOR_WIDTH);
        if (tracked_id == ArmorName::one) {
            L_a = T(RWTracker::LARGE_ARMOR_LENGTH);
            W_a = T(RWTracker::LARGE_ARMOR_WIDTH);
        }

        const T f_x = T(cam_info(0, 0));
        const T f_y = T(cam_info(1, 1));
        const T c_x = T(cam_info(0, 2));
        const T c_y = T(cam_info(1, 2));

        T theta = x(6) + T(2.0 * M_PI / tracked_armors_num * armor_id);
        T r;
        T z_a;
        if (tracked_armors_num == 4 && armor_id % 2 == 1) {
            r = x(10);
            z_a = x(9);
        } else {
            r = x(8);
            z_a = x(4);
        }
        if (tracked_id == ArmorName::outpost) {
            z_a += T(0.1 * circle_type[(circle_index + armor_id) % 3]);
        }

        Eigen::Matrix<T, 3, 3> R_c_w = odom_camera_matrix.block<3, 3>(0, 0).template cast<T>();
        Eigen::Matrix<T, 3, 1> t_c_w = odom_camera_matrix.block<3, 1>(0, 3).template cast<T>();
        Eigen::Matrix<T, 3, 1> P_cam_w = -R_c_w.transpose() * t_c_w;

        Eigen::Matrix<T, 3, 1> N_w(
            ceres::cos(theta) * ceres::cos(theta_pa),
            ceres::sin(theta) * ceres::cos(theta_pa),
            ceres::sin(theta_pa)
        );
        Eigen::Matrix<T, 3, 1> V_up_w(
            -ceres::cos(theta) * ceres::sin(theta_pa),
            -ceres::sin(theta) * ceres::sin(theta_pa),
            ceres::cos(theta_pa)
        );
        Eigen::Matrix<T, 3, 1> V_side_w = V_up_w.cross(N_w);

        Eigen::Matrix<T, 3, 1> center_l(
            x_c + r * ceres::cos(theta) + ceres::sin(theta) * L_a / T(2.0),
            y_c + r * ceres::sin(theta) - ceres::cos(theta) * L_a / T(2.0),
            z_a
        );
        Eigen::Matrix<T, 3, 1> center_r(
            x_c + r * ceres::cos(theta) - ceres::sin(theta) * L_a / T(2.0),
            y_c + r * ceres::sin(theta) + ceres::cos(theta) * L_a / T(2.0),
            z_a
        );

        auto calc_visible_bounds = [&](const Eigen::Matrix<T, 3, 1>& center,
                                       T& side_offset,
                                       T& up_offset,
                                       T& forward_offset,
                                       int if_left,
                                       ConsistentOcclusionPointDebug* point_debug) {
            const T d_recess = T(0.003);
            const T half_w = W_a / T(2.0);
            const T light_width = T(0.007);
            const T light_side_width = T(0.004);

            const T occ_center = T(0.02);
            const T occ_sharpness = T(18.0);
            const T ext_center = T(0.02);
            const T ext_sharpness = T(18.0);
            const T cos_eps = T(0.02);
            const T denom_eps = T(0.08);
            const T sheld_limit = T(0.006);
            const T forward_limit = T(0.004);
            const T up_couple = T(0.65);

            up_offset = half_w;

            const Eigen::Matrix<T, 3, 1> V_cam = P_cam_w - center;
            const Eigen::Matrix<T, 3, 1> V_sheld_cam =
                V_cam - V_side_w * light_width / T(2.0) * T(if_left);
            const Eigen::Matrix<T, 3, 1> V_extend_cam =
                V_cam + V_side_w * light_width / T(2.0) * T(if_left);

            const Eigen::Matrix<T, 3, 1> V_sheld_unit = safe_normalize(V_sheld_cam);
            const Eigen::Matrix<T, 3, 1> V_extend_unit = safe_normalize(V_extend_cam);

            const T cos_side = V_sheld_unit.dot(V_side_w) * T(if_left);
            const T cos_extend = V_extend_unit.dot(V_side_w) * T(if_left);

            const T alpha_occ = smooth_sigmoid(cos_side, occ_center, occ_sharpness);
            const T alpha_ext = smooth_sigmoid(-cos_extend, ext_center, ext_sharpness);

            const T cos_pos = smooth_pos(cos_side, cos_eps);
            const T denom = ceres::sqrt(T(1.0) - cos_side * cos_side + denom_eps * denom_eps);
            const T raw_sheld = d_recess / T(2.0) * cos_pos / denom;
            const T landscape_sheld = smooth_sat(alpha_occ * raw_sheld, sheld_limit);

            const T front_proj = -V_extend_unit.dot(N_w);
            const T front_proj_pos = smooth_pos(front_proj, T(0.02));
            const T raw_extend = front_proj_pos * light_side_width / T(2.0);
            const T landscape_extend = smooth_sat(alpha_ext * raw_extend, forward_limit);

            side_offset = landscape_sheld;
            up_offset = half_w + up_couple * landscape_sheld;
            forward_offset = -landscape_extend;

            if constexpr (std::is_same_v<T, double>) {
                if (point_debug != nullptr) {
                    point_debug->cos_side = cos_side;
                    point_debug->cos_extend = cos_extend;
                    point_debug->alpha_occ = alpha_occ;
                    point_debug->alpha_ext = alpha_ext;
                    point_debug->side_offset = side_offset;
                    point_debug->up_offset = up_offset;
                    point_debug->forward_offset = forward_offset;
                }
            }
        };

        T side_l_offset = T(0), up_l_offset = T(0), forward_l_offset = T(0);
        T side_r_offset = T(0), up_r_offset = T(0), forward_r_offset = T(0);
        calc_visible_bounds(
            center_l,
            side_l_offset,
            up_l_offset,
            forward_l_offset,
            1,
            debug != nullptr ? &debug->left : nullptr
        );
        calc_visible_bounds(
            center_r,
            side_r_offset,
            up_r_offset,
            forward_r_offset,
            -1,
            debug != nullptr ? &debug->right : nullptr
        );

        auto project_point = [&](const Eigen::Matrix<T, 3, 1>& center,
                                 const T& side_offset,
                                 const T& up_offset,
                                 const T& forward_offset) -> Eigen::Matrix<T, 2, 1> {
            Eigen::Matrix<T, 4, 1> p_w;
            p_w << center + V_up_w * up_offset + V_side_w * side_offset + N_w * forward_offset,
                T(1.0);
            const Eigen::Matrix<T, 3, 1> p_c =
                (odom_camera_matrix.template cast<T>() * p_w).template head<3>();

            Eigen::Matrix<T, 2, 1> uv;
            uv << f_x * (p_c.x() / p_c.z()) + c_x, f_y * (p_c.y() / p_c.z()) + c_y;
            return uv;
        };

        Eigen::Matrix<T, Eigen::Dynamic, 1> z;
        if (match_type == RWTracker::MatchType::ARMOR) {
            z.resize(8);
            const auto p1 = project_point(center_l, side_l_offset, up_l_offset, forward_l_offset);
            const auto p2 = project_point(center_r, -side_r_offset, up_r_offset, forward_r_offset);
            const auto p3 = project_point(center_l, side_l_offset, -up_l_offset, forward_l_offset);
            const auto p4 = project_point(center_r, -side_r_offset, -up_r_offset, forward_r_offset);
            z << p1.x(), p1.y(), p2.x(), p2.y(), p3.x(), p3.y(), p4.x(), p4.y();
        } else if (match_type == RWTracker::MatchType::LIGHT_LEFT) {
            z.resize(4);
            const auto p1 = project_point(center_l, side_l_offset, up_l_offset, forward_l_offset);
            const auto p3 = project_point(center_l, side_l_offset, -up_l_offset, forward_l_offset);
            z << p1.x(), p1.y(), p3.x(), p3.y();
        } else if (match_type == RWTracker::MatchType::LIGHT_RIGHT) {
            z.resize(4);
            const auto p2 = project_point(center_r, -side_r_offset, up_r_offset, forward_r_offset);
            const auto p4 = project_point(center_r, -side_r_offset, -up_r_offset, forward_r_offset);
            z << p2.x(), p2.y(), p4.x(), p4.y();
        }
        return z;
    }

} // namespace
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
    target_state_unique(Eigen::VectorXd::Zero(16)),
    tracked_armors_num(4),
    measure_dimension(0),
    show_target_interface_(*this),
    show_target_interface_unique_(*this) {
    auto yaml = YAML::LoadFile(config_path);
    match_thre_distance = yaml["match_thre_distance"].as<float>();
    match_thre_angle = yaml["match_thre_angle"].as<float>();
    match_thre_distance_light = yaml["match_thre_distance_light"].as<float>();
    match_thre_angle_light = yaml["match_thre_angle_light"].as<float>();
    match_thre_length_light = yaml["match_thre_length_light"].as<float>();
    const auto ekf_node = yaml["tracker"]["ekf"];
    const auto ekf_outpost_node = yaml["tracker"]["ekf_outpost"];
    const double q_xyz = ekf_node["sigma2_q_xyz"].as<double>();
    const double q_outpost_xyz = ekf_outpost_node["sigma2_q_xyz"].as<double>();
    s2_normal.q_xyz = q_xyz;
    s2_normal.q_xy = ekf_node["sigma2_q_xy"] ? ekf_node["sigma2_q_xy"].as<double>() : q_xyz;
    s2_normal.q_z = ekf_node["sigma2_q_z"] ? ekf_node["sigma2_q_z"].as<double>() : q_xyz;
    s2_normal.q_yaw = ekf_node["sigma2_q_yaw"].as<double>();
    s2_normal.q_r = ekf_node["sigma2_q_r"].as<double>();
    s2_normal.r_u = ekf_node["sigam2_r_u"].as<double>();
    s2_normal.r_v = ekf_node["sigam2_r_v"].as<double>();

    s2_outpost.q_xyz = q_outpost_xyz;
    s2_outpost.q_xy = ekf_outpost_node["sigma2_q_xy"]
        ? ekf_outpost_node["sigma2_q_xy"].as<double>()
        : q_outpost_xyz;
    s2_outpost.q_z = ekf_outpost_node["sigma2_q_z"]
        ? ekf_outpost_node["sigma2_q_z"].as<double>()
        : q_outpost_xyz;
    s2_outpost.q_yaw = ekf_outpost_node["sigma2_q_yaw"].as<double>();
    s2_outpost.q_r = ekf_outpost_node["sigma2_q_r"].as<double>();
    s2_outpost.r_u = ekf_outpost_node["sigam2_r_u"].as<double>();
    s2_outpost.r_v = ekf_outpost_node["sigam2_r_v"].as<double>();
    r_distance = yaml["tracker"]["r_distance"].as<double>();
    max_armor_distance_ = yaml["tracker"]["max_armor_distance"].as<double>();
    max_match_error = yaml["tracker"]["max_match_error"].as<double>();
    track_thres = yaml["tracker"]["track_thres"].as<double>();
    lost_thres = yaml["tracker"]["lost_thres"].as<double>();
    self_height = yaml["tracker"]["self_height"].as<float>();
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
                Eigen::VectorXd z_singal_armor = use_consistent_occlusion_model_
                    ? Singal_Armor_h_ceres_consis(x, i, this->match_armors[i].match_type)
                    : Singal_Armor_h(x, i, this->match_armors[i].match_type);
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
                Eigen::MatrixXd h_singal_armor = use_consistent_occlusion_model_
                    ? Singal_Armor_jh_ceres_consis(x, i, this->match_armors[i].match_type)
                    : Singal_Armor_jh(x, i, this->match_armors[i].match_type);
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
        double distance = std::hypot(target_state[0], target_state[2], target_state[4]);
        for (int i = 0; i < this->measure_dimension / 2; i++) {
            // r_vec.segment(index, r_point.size()) = r_point * std::pow(distance, 4) * r_distance;//!未测试
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
    if (armors.empty()) {
        return;
    }

    /* 简单选择距离图像中心近装甲板为跟踪对象 */
    double min_distance = DBL_MAX;
    tracked_armor = armors.front();
    for (const auto& armor: armors) {
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
    use_consistent_occlusion_model_ = false;
    consistent_occlusion_debug_.valid = false;
    this->matched = false; // 当前帧是否有匹配的装甲板、灯条
    bool updated_with_measurement = false;
    std::vector<Armor> same_id_armors; // 与当前跟踪相同id的装甲板
    same_id_armors.reserve(this->tracked_armors_num); //* 也就是看到了可用来观测的
    this->match_armors.resize(this->tracked_armors_num); //* 整车观测，记录所有状态
    std::fill(this->match_armors.begin(), this->match_armors.end(), this->MATCH_ARMOR_INIT);

    /* EKF 预测 */
    ekf_prediction = ekf.predict();
    last_true_predict_time_ = std::chrono::steady_clock::now();
    target_state = ekf_prediction;
    double x_c = ekf_prediction(0), y_c = ekf_prediction(2), theta = ekf_prediction(6);

    if (this->tracked_id == ArmorName::outpost) {
        if (outpost_errcount == 1) {
            circle_index = (circle_index - 1) % 3;
        }
        if (outpost_errcount == 2) {
            circle_index = (circle_index - 2) % 3;
        }
        outpost_errcount = 0;
    }

    /* 寻找相同id装甲板 */
    for (const auto& armor: armors) {
        if (armor.name == this->tracked_id) {
            same_id_armors.emplace_back(armor);
        }
    }

    if (!same_id_armors.empty()) {
        /* 根据预测筛选可匹配装甲板     TODO: 装甲板数量不同时阈值修改 */
        //! 这里的坐标变换是新增的
        this->odom2camera_tvec = this->solver.odom2camera_tvec();
        this->odom_camera_matrix = this->solver.odom_camera_matrix();

        double camera_enemy_theta = atan2(
            y_c - this->odom_camera_matrix.block<3, 1>(0, 3)[1],
            x_c - this->odom_camera_matrix.block<3, 1>(0, 3)[0]
        );

        //* 首先根据状态转移预测点
        //? 为前哨站缩小需要匹配范围
        double theta_threshold_max =
            normalize_angle(camera_enemy_theta + M_PI / 2 - (tracked_armors_num - 4) * M_PI / 4);
        double theta_threshold_min =
            normalize_angle(camera_enemy_theta - M_PI / 2 + (tracked_armors_num - 4) * M_PI / 4);
        //? 这个循环给出了待匹配的预测点
        for (int i = 0; i < this->tracked_armors_num; i++) {
            double original_za1 = ekf_prediction(static_cast<int>(State::ZA1));
            //! 这里对前哨站中心临时变化以适应高低差，之后注意要变回去
            if (this->tracked_id == ArmorName::outpost) {
                ekf_prediction(static_cast<int>(State::ZA1)) +=
                    0.1 * circle_type[(circle_index + i) % 3];
            }
            double predict_armor_theta = normalize_angle(
                theta + 2 * M_PI / (static_cast<int>(this->tracked_armors_num)) * i
            );
            // 处理[-pi,pi)区间跳跃
            if (theta_threshold_max > theta_threshold_min) { //* 即自己和目标连线和相机正方向的夹角
                if (predict_armor_theta < theta_threshold_min
                    || predict_armor_theta > theta_threshold_max)
                {
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
                    && predict_armor_theta > theta_threshold_max)
                {
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
            if (this->tracked_id == ArmorName::outpost) {
                ekf_prediction(static_cast<int>(State::ZA1)) -=
                    0.1 * circle_type[(circle_index + i) % 3];
            }
        }

        /* 进行装甲板匹配 */
        //? 通过最小误差选择需要匹配的那个装甲板
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
                    && match_armors[i].match_type == MatchType::NONE)
                {
                    // 检查 predict_points 是否有效
                    if (match_armors[i].predict_points.size() == 0) {
                        continue; // 跳过未初始化的预测点
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
                }
                //! 之前是这样写的，感觉是没问题，为了保底修改下逻辑
                else if (this->tracked_id == ArmorName::outpost)
                {
                    //* 对于前哨站，即使完全匹配错误也只能匹配到这一个
                    bool z_diff_big =
                        measure_points[1] > match_armors[min_error_id].predict_points[1]
                        && measure_points[3] > match_armors[min_error_id].predict_points[3]
                        && measure_points[5] > match_armors[min_error_id].predict_points[5]
                        && measure_points[7] > match_armors[min_error_id].predict_points[7];
                    bool z_diff_small =
                        measure_points[1] > match_armors[min_error_id].predict_points[1]
                        && measure_points[3] > match_armors[min_error_id].predict_points[3]
                        && measure_points[5] > match_armors[min_error_id].predict_points[5]
                        && measure_points[7] > match_armors[min_error_id].predict_points[7];
                    if (z_diff_big || z_diff_small) {
                        outpost_errcount++;
                    }
                    // 添加调试信息：显示为什么匹配失败
                    std::cout << "Match failed for armor: min_error=" << min_error
                              << " (thre=" << match_thre_distance
                              << "), error_angle=" << error_angle << " (thre=" << match_thre_angle
                              << ")" << std::endl;
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
            } else if (match_armors[i].match_type == MatchType::LIGHT_LEFT
                       || match_armors[i].match_type == MatchType::LIGHT_RIGHT)
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
            updated_with_measurement = true;
            std::cout << "measure_dimension: " << measure_dimension << std::endl;
        }
    }

    /* 相关信息打印 */
    for (int i = 0; i < this->tracked_armors_num; i++) {
        std::cout << "armor[" << i << "]: need_match " << match_armors[i].need_match
                  << ", error_distance " << match_armors[i].error_distance << ", error_angle "
                  << match_armors[i].error_angle << ", match_type " << match_armors[i].match_type
                  << std::endl;
    }
    if (!updated_with_measurement) {
        std::cout << "measure_dimension: " << measure_dimension << std::endl;
    }

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
            std::cout << "TRACKINGstate " << this->tracker_state << ", " << this->track_time_count
                      << ", " << this->dt_ << std::endl;
        } else {
            this->track_time_count = 0;
            this->tracker_state = TrackState::LOST;
        }
    } else if (this->tracker_state == TrackState::TRACKING) {
        if (!this->matched) {
            this->lost_time_count += this->dt_;
            this->tracker_state = TrackState::TEMP_LOST;
        }
        std::cout << "TRACKINGstate " << this->tracker_state << ", " << this->track_time_count
                  << ", " << this->dt_ << std::endl;
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
        std::cout << "TRACKINGstate " << this->tracker_state << ", " << this->track_time_count
                  << ", " << this->dt_ << std::endl;
    }
}

void auto_aim::RWTracker::update_ceres(
    const std::list<Armor>& armors,
    std::list<Light> light_array
) {
    use_consistent_occlusion_model_ = true;
    consistent_occlusion_debug_.valid = false;
    this->matched = false; // 当前帧是否有匹配的装甲板、灯条
    bool updated_with_measurement = false;
    std::vector<Armor> same_id_armors; // 与当前跟踪相同id的装甲板
    same_id_armors.reserve(this->tracked_armors_num); //* 也就是看到了可用来观测的
    this->match_armors.resize(this->tracked_armors_num); //* 整车观测，记录所有状态
    std::fill(this->match_armors.begin(), this->match_armors.end(), this->MATCH_ARMOR_INIT);

    /* EKF 预测 */
    ekf_prediction = ekf.predict();
    last_true_predict_time_ = std::chrono::steady_clock::now();
    target_state = ekf_prediction;
    double x_c = ekf_prediction(0), y_c = ekf_prediction(2), theta = ekf_prediction(6);

    if (this->tracked_id == ArmorName::outpost) {
        std::cout<<"out_post err"<<outpost_errcount<<std::endl;
        if (outpost_errcount == 1) {
            circle_index = (circle_index + 1) % 3;
        }
        std::cout<<"circle_index"<<circle_index<<std::endl;
        outpost_errcount = 0;
    }

    /* 寻找相同id装甲板 */
    for (const auto& armor: armors) {
        if (armor.name == this->tracked_id) {
            same_id_armors.emplace_back(armor);
        }
    }

    if (!same_id_armors.empty()) {
        /* 根据预测筛选可匹配装甲板     TODO: 装甲板数量不同时阈值修改 */
        //! 这里的坐标变换是新增的
        // this->odom2camera_tvec = this->solver.odom2camera_tvec();
        this->odom_camera_matrix = this->solver.odom_camera_matrix();
        // auto P_cam_w = -this->odom_camera_matrix.transpose()*this->odom2camera_tvec;

        // double camera_enemy_theta = atan2(
        //     y_c - this->odom_camera_matrix.block<3, 1>(0, 3)[1],
        //     x_c - this->odom_camera_matrix.block<3, 1>(0, 3)[0]
        // );

        const Eigen::Matrix3d R_world2camera = this->odom_camera_matrix.block<3, 3>(0, 0);
        const Eigen::Vector3d t_world2camera = this->odom_camera_matrix.block<3, 1>(0, 3);
        const Eigen::Vector3d P_cam_w = -R_world2camera.transpose() * t_world2camera;

        double camera_enemy_theta = atan2(y_c - P_cam_w.y(), x_c - P_cam_w.x());

        //* 首先根据状态转移预测点
        //? 为前哨站缩小需要匹配范围
        double theta_threshold_max =
            normalize_angle(camera_enemy_theta + M_PI / 2 - (tracked_armors_num - 4) * M_PI / 4);
        double theta_threshold_min =
            normalize_angle(camera_enemy_theta - M_PI / 2 + (tracked_armors_num - 4) * M_PI / 4);
        //? 这个循环给出了待匹配的预测点
        for (int i = 0; i < this->tracked_armors_num; i++) {
            double original_za1 = ekf_prediction(static_cast<int>(State::ZA1));
            //! 这里对前哨站中心临时变化以适应高低差，之后注意要变回去
            if (this->tracked_id == ArmorName::outpost) {
                ekf_prediction(static_cast<int>(State::ZA1)) +=
                    0.1 * circle_type[(circle_index + i) % 3];
            }
            double predict_armor_theta = normalize_angle(
                theta + 2 * M_PI / (static_cast<int>(this->tracked_armors_num)) * i
            );
            // 处理[-pi,pi)区间跳跃
            if (theta_threshold_max > theta_threshold_min) { //* 即自己和目标连线和相机正方向的夹角
                if (predict_armor_theta < theta_threshold_min
                    || predict_armor_theta > theta_threshold_max)
                {
                    // 当前预测装甲板落入观测区间，允许匹配
                    match_armors[i].need_match = true;
                    match_armors[i].predict_points =
                        Singal_Armor_h_ceres_consis(ekf_prediction, i, MatchType::ARMOR);
                } else {
                    // 当前预测装甲板不在观测区间，拒绝匹配
                    match_armors[i].need_match = false;
                }
            } else {
                if (predict_armor_theta < theta_threshold_min
                    && predict_armor_theta > theta_threshold_max)
                {
                    // 当前预测装甲板落入观测区间，允许匹配
                    match_armors[i].need_match = true;
                    match_armors[i].predict_points =
                        Singal_Armor_h_ceres_consis(ekf_prediction, i, MatchType::ARMOR);
                } else {
                    // 当前预测装甲板不在观测区间，拒绝匹配
                    match_armors[i].need_match = false;
                }
            }
            //! 这里状态量变回去
            if (this->tracked_id == ArmorName::outpost) {
                ekf_prediction(static_cast<int>(State::ZA1)) -=
                    0.1 * circle_type[(circle_index + i) % 3];
            }
        }

        /* 进行装甲板匹配 */
        //? 通过最小误差选择需要匹配的那个装甲板
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
                    && match_armors[i].match_type == MatchType::NONE)
                {
                    // 检查 predict_points 是否有效
                    if (match_armors[i].predict_points.size() == 0) {
                        continue; // 跳过未初始化的预测点
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
            // if (min_error_id >= 0 &&this->tracked_id == ArmorName::outpost&& match_armors[min_error_id].predict_points.size() > 0)
            // {
            //     //* 对于前哨站，即使完全匹配错误也只能匹配到这一个
            //     bool z_diff_big =
            //         measure_points[1] > match_armors[min_error_id].predict_points[1]
            //         && measure_points[1] > match_armors[min_error_id].predict_points[3]
            //         && measure_points[1] > match_armors[min_error_id].predict_points[5]
            //         && measure_points[1] > match_armors[min_error_id].predict_points[7]
            //         && measure_points[3] > match_armors[min_error_id].predict_points[1]
            //         && measure_points[3] > match_armors[min_error_id].predict_points[3]
            //         && measure_points[3] > match_armors[min_error_id].predict_points[5]
            //         && measure_points[3] > match_armors[min_error_id].predict_points[7]
            //         && measure_points[5] > match_armors[min_error_id].predict_points[1]
            //         && measure_points[5] > match_armors[min_error_id].predict_points[3]
            //         && measure_points[5] > match_armors[min_error_id].predict_points[5]
            //         && measure_points[5] > match_armors[min_error_id].predict_points[7]
            //         && measure_points[7] > match_armors[min_error_id].predict_points[1]
            //         && measure_points[7] > match_armors[min_error_id].predict_points[3]
            //         && measure_points[7] > match_armors[min_error_id].predict_points[5]
            //         && measure_points[7] > match_armors[min_error_id].predict_points[7];
            //     bool z_diff_small =
            //         measure_points[1] < match_armors[min_error_id].predict_points[1]
            //         && measure_points[1] < match_armors[min_error_id].predict_points[3]
            //         && measure_points[1] < match_armors[min_error_id].predict_points[5]
            //         && measure_points[1] < match_armors[min_error_id].predict_points[7]
            //         && measure_points[3] < match_armors[min_error_id].predict_points[1]
            //         && measure_points[3] < match_armors[min_error_id].predict_points[3]
            //         && measure_points[3] < match_armors[min_error_id].predict_points[5]
            //         && measure_points[3] < match_armors[min_error_id].predict_points[7]
            //         && measure_points[5] < match_armors[min_error_id].predict_points[1]
            //         && measure_points[5] < match_armors[min_error_id].predict_points[3]
            //         && measure_points[5] < match_armors[min_error_id].predict_points[5]
            //         && measure_points[5] < match_armors[min_error_id].predict_points[7]
            //         && measure_points[7] < match_armors[min_error_id].predict_points[1]
            //         && measure_points[7] < match_armors[min_error_id].predict_points[3]
            //         && measure_points[7] < match_armors[min_error_id].predict_points[5]
            //         && measure_points[7] < match_armors[min_error_id].predict_points[7];
            //     if (z_diff_big || z_diff_small) {
            //         outpost_errcount++;
            //         std::cout<<"out_post err"<<std::endl;
            //         return update_ceres(armors,light_array);
            //     }}
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
                }
                //! 之前是这样写的，感觉是没问题，为了保底修改下逻辑

                    // 添加调试信息：显示为什么匹配失败
                    // std::cout << "Match failed for armor: min_error=" << min_error
                    //           << " (thre=" << match_thre_distance
                    //           << "), error_angle=" << error_angle << " (thre=" << match_thre_angle
                    //           << ")" << std::endl;
                // }
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
                Singal_Armor_h_ceres_consis(ekf_prediction, armor_left_id, MatchType::LIGHT_RIGHT);
            armor_right.predict_points =
                Singal_Armor_h_ceres_consis(ekf_prediction, armor_right_id, MatchType::LIGHT_LEFT);
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
            } else if (match_armors[i].match_type == MatchType::LIGHT_LEFT
                       || match_armors[i].match_type == MatchType::LIGHT_RIGHT)
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
            int debug_armor_id = -1;
            MatchType debug_match_type = MatchType::NONE;
            for (int i = 0; i < this->tracked_armors_num; i++) {
                if (this->match_armors[i].match_type == MatchType::ARMOR) {
                    debug_armor_id = i;
                    debug_match_type = MatchType::ARMOR;
                    break;
                }
                if (debug_armor_id < 0 && this->match_armors[i].match_type != MatchType::NONE) {
                    debug_armor_id = i;
                    debug_match_type = this->match_armors[i].match_type;
                }
            }
            if (debug_armor_id >= 0) {
                ConsistentOcclusionModelDebug local_debug;
                build_occluded_measurement_ceres_consis<double>(
                    ekf_prediction,
                    debug_armor_id,
                    debug_match_type,
                    this->tracked_id,
                    this->tracked_armors_num,
                    this->circle_index,
                    this->circle_type,
                    this->cam_info_,
                    this->odom_camera_matrix,
                    &local_debug
                );
                consistent_occlusion_debug_.valid = true;
                consistent_occlusion_debug_.armor_id = debug_armor_id;
                consistent_occlusion_debug_.match_type = static_cast<int>(debug_match_type);
                consistent_occlusion_debug_.theta = ekf_prediction(6);
                if (this->tracked_armors_num == 4 && debug_armor_id % 2 == 1) {
                    consistent_occlusion_debug_.r = ekf_prediction(10);
                    consistent_occlusion_debug_.z = ekf_prediction(9);
                } else {
                    consistent_occlusion_debug_.r = ekf_prediction(8);
                    consistent_occlusion_debug_.z = ekf_prediction(4);
                }
                fill_consistent_occlusion_debug_json_like(consistent_occlusion_debug_, local_debug);
                consistent_occlusion_debug_.jh =
                    Singal_Armor_jh_ceres_consis(ekf_prediction, debug_armor_id, debug_match_type);
                if (last_consistent_occlusion_jh_.rows() == consistent_occlusion_debug_.jh.rows()
                    && last_consistent_occlusion_jh_.cols()
                        == consistent_occlusion_debug_.jh.cols())
                {
                    consistent_occlusion_debug_.djh =
                        consistent_occlusion_debug_.jh - last_consistent_occlusion_jh_;
                } else {
                    consistent_occlusion_debug_.djh = Eigen::MatrixXd::Zero(
                        consistent_occlusion_debug_.jh.rows(),
                        consistent_occlusion_debug_.jh.cols()
                    );
                }
                consistent_occlusion_debug_.jh_norm = consistent_occlusion_debug_.jh.norm();
                consistent_occlusion_debug_.jh_max_abs = consistent_occlusion_debug_.jh.size() > 0
                    ? consistent_occlusion_debug_.jh.cwiseAbs().maxCoeff()
                    : 0.0;
                consistent_occlusion_debug_.djh_norm = consistent_occlusion_debug_.djh.norm();
                consistent_occlusion_debug_.djh_max_abs = consistent_occlusion_debug_.djh.size() > 0
                    ? consistent_occlusion_debug_.djh.cwiseAbs().maxCoeff()
                    : 0.0;
                last_consistent_occlusion_jh_ = consistent_occlusion_debug_.jh;
            }
            // EKF 更新
            if(this->tracked_id == ArmorName::outpost){
                auto fake_state = ekf.getState();
                auto outpost_state = ekf.update(measurement);
                if(std::abs(fake_state[4]-outpost_state[4])>0.15){
                    ekf.setState(fake_state);
                    outpost_errcount++;
                    update_ceres(armors,light_array);
                }
            }else
            {target_state = ekf.update(measurement);}
            updated_with_measurement = true;
            // if (ekf.has_last_nis()) {
            //     std::cout << "measure_dimension: " << measure_dimension
            //               << ", nis: " << ekf.last_nis() << ", nis_norm: " << ekf.last_nis_norm()
            //               << std::endl;
            // } else {
            //     std::cout << "measure_dimension: " << measure_dimension
            //               << ", nis: N/A, nis_norm: N/A" << std::endl;
            // }
        }
    }

    // /* 相关信息打印 */
    // for (int i = 0; i < this->tracked_armors_num; i++) {
    //     std::cout << "armor[" << i << "]: need_match " << match_armors[i].need_match
    //               << ", error_distance " << match_armors[i].error_distance << ", error_angle "
    //               << match_armors[i].error_angle << ", match_type " << match_armors[i].match_type
    //               << std::endl;
    // }
    // if (!updated_with_measurement) {
    //     std::cout << "measure_dimension: " << measure_dimension << std::endl;
    // }

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
            // std::cout << "TRACKINGstate " << this->tracker_state << ", " << this->track_time_count
            //           << ", " << this->dt_ << std::endl;
        } else {
            this->track_time_count = 0;
            this->tracker_state = TrackState::LOST;
        }
    } else if (this->tracker_state == TrackState::TRACKING) {
        if (!this->matched) {
            this->lost_time_count += this->dt_;
            this->tracker_state = TrackState::TEMP_LOST;
        }
        // std::cout << "TRACKINGstate " << this->tracker_state << ", " << this->track_time_count
        //           << ", " << this->dt_ << std::endl;
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
        // std::cout << "TRACKINGstate " << this->tracker_state << ", " << this->track_time_count
        //           << ", " << this->dt_ << std::endl;
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
    double yaw = normalize_angle(armor.ypr_in_world[0] + M_PI);

    target_state = Eigen::VectorXd::Zero(11);
    double r = 0.26; //!这里先给了个先验值
    double xc = xa - r * cos(yaw);
    double yc = ya - r * sin(yaw);
    if (armor.name == ArmorName::outpost) {
        //! 这里应该考虑陀螺仪到转轴的距离和pitch角，但陀螺仪初始化千奇百怪的，只能寄希望于纠错了
        if (std::pow(za - (1.316 - self_height), 2) < std::pow(za - (1.416 - self_height), 2)) {
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
        auto* closest = std::min_element(
            std::begin(outpost_height),
            std::end(outpost_height),
            [za](double a, double b) { return std::abs(za - a) < std::abs(za - b); }
        );
        circle_index = static_cast<int>(closest - std::begin(outpost_height));
    }
    target_state << xc, 0, yc, 0, za, 0, yaw, 0, r, za, r;

    std::cout << "init state: " << xc << ", " << yc << ", " << za << ", " << yaw << ", " << r
              << std::endl;

    ekf.reinitP();
    ekf.setState(target_state);
    ekf_prediction = target_state;
}

const auto_aim::RWTracker::ConsistentOcclusionDebugInfo&
auto_aim::RWTracker::consistent_occlusion_debug() const {
    return consistent_occlusion_debug_;
}

namespace {
    void fill_consistent_occlusion_debug_json_like(
        auto_aim::RWTracker::ConsistentOcclusionDebugInfo& out,
        const ConsistentOcclusionModelDebug& local_debug
    ) {
        out.cos_side_left = local_debug.left.cos_side;
        out.cos_side_right = local_debug.right.cos_side;
        out.cos_extend_left = local_debug.left.cos_extend;
        out.cos_extend_right = local_debug.right.cos_extend;
        out.alpha_occ_left = local_debug.left.alpha_occ;
        out.alpha_occ_right = local_debug.right.alpha_occ;
        out.alpha_ext_left = local_debug.left.alpha_ext;
        out.alpha_ext_right = local_debug.right.alpha_ext;
        out.side_offset_left = local_debug.left.side_offset;
        out.side_offset_right = local_debug.right.side_offset;
        out.up_offset_left = local_debug.left.up_offset;
        out.up_offset_right = local_debug.right.up_offset;
        out.forward_offset_left = local_debug.left.forward_offset;
        out.forward_offset_right = local_debug.right.forward_offset;
    }
} // namespace

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
        z_a += 0.1 * circle_type[(circle_index + armor_id) % 3];
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
Eigen::MatrixXd
auto_aim::RWTracker::Singal_Armor_jh(const Eigen::VectorXd& x, int armor_id, MatchType match_type) {
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

    double f_x = this->cam_info_(0, 0), f_y = this->cam_info_(1, 1);
    //* 标定后得到的像素焦距
    double t_11 = this->odom_camera_matrix(0, 0), t_12 = this->odom_camera_matrix(0, 1),
           t_13 = this->odom_camera_matrix(0, 2);
    double t_21 = this->odom_camera_matrix(1, 0), t_22 = this->odom_camera_matrix(1, 1),
           t_23 = this->odom_camera_matrix(1, 2);
    double t_31 = this->odom_camera_matrix(2, 0), t_32 = this->odom_camera_matrix(2, 1),
           t_33 = this->odom_camera_matrix(2, 2);
    double delta_r = W_a * sin(theta_pa); //* 半径方向的
    double delta_z = W_a * cos(theta_pa); //* 高度方向的
    //* pitch倾角

    // 依据编号赋值当前装甲板 theta、r、z
    theta = x(6) + 2 * M_PI / (static_cast<int>(this->tracked_armors_num)) * armor_id;
    //* 据前面误差阈值的逻辑动态赋值

    if (this->tracked_armors_num == 4 && armor_id % 2 == 1) {
        r = x(10); //当前装甲板半径 r2
        z_a = x(9); //当前装甲板高度 za2
    } else {
        r = x(8); //当前装甲板半径 r1
        z_a = x(4); //当前装甲板高度 za1
    }

    double r_1t = r - delta_r / 2;
    double r_1b = r + delta_r / 2;

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
    if (match_type == MatchType::ARMOR || match_type == MatchType::LIGHT_LEFT) {
        // 计算世界系下装甲板关键点坐标（齐次坐标）
        p_1w << cos(theta) * r_1t + sin(theta) * L_a / 2 + x_c,
            sin(theta) * r_1t - cos(theta) * L_a / 2 + y_c, z_a + delta_z / 2, 1;
        p_3w << cos(theta) * r_1b + sin(theta) * L_a / 2 + x_c,
            sin(theta) * r_1b - cos(theta) * L_a / 2 + y_c, z_a - delta_z / 2, 1;
        p_1c = (this->odom_camera_matrix * p_1w).head<3>();
        p_3c = (this->odom_camera_matrix * p_3w).head<3>();
        // 变换到相机坐标系
        p_1c = (this->odom_camera_matrix * p_1w).head<3>();
        p_3c = (this->odom_camera_matrix * p_3w).head<3>();
        // u1, v1 雅可比计算
        double D_u1_D_x_p1c = f_x / p_1c.z();
        double D_u1_D_z_p1c = -f_x * p_1c.x() / pow(p_1c.z(), 2);
        double D_v1_D_y_p1c = f_y / p_1c.z();
        double D_v1_D_z_p1c = -f_y * p_1c.y() / pow(p_1c.z(), 2);

        double D_x_p1w_D_theta = cos(theta) * L_a / 2 - r_1t * sin(theta);
        double D_y_p1w_D_theta = sin(theta) * L_a / 2 + r_1t * cos(theta);

        double D_x_p1c_D_theta = t_11 * D_x_p1w_D_theta + t_12 * D_y_p1w_D_theta;
        double D_y_p1c_D_theta = t_21 * D_x_p1w_D_theta + t_22 * D_y_p1w_D_theta;
        double D_z_p1c_D_theta = t_31 * D_x_p1w_D_theta + t_32 * D_y_p1w_D_theta;

        double D_x_p1c_D_r1 = t_11 * cos(theta) + t_12 * sin(theta);
        double D_y_p1c_D_r1 = t_21 * cos(theta) + t_22 * sin(theta);
        double D_z_p1c_D_r1 = t_31 * cos(theta) + t_32 * sin(theta);

        D_u1_D_xc = D_u1_D_x_p1c * t_11 + D_u1_D_z_p1c * t_31;
        D_u1_D_yc = D_u1_D_x_p1c * t_12 + D_u1_D_z_p1c * t_32;
        D_u1_D_za1 = D_u1_D_x_p1c * t_13 + D_u1_D_z_p1c * t_33;
        D_u1_D_theta = D_u1_D_x_p1c * D_x_p1c_D_theta + D_u1_D_z_p1c * D_z_p1c_D_theta;
        D_u1_D_r1 = D_u1_D_x_p1c * D_x_p1c_D_r1 + D_u1_D_z_p1c * D_z_p1c_D_r1;

        D_v1_D_xc = D_v1_D_y_p1c * t_21 + D_v1_D_z_p1c * t_31;
        D_v1_D_yc = D_v1_D_y_p1c * t_22 + D_v1_D_z_p1c * t_32;
        D_v1_D_za1 = D_v1_D_y_p1c * t_23 + D_v1_D_z_p1c * t_33;
        D_v1_D_theta = D_v1_D_y_p1c * D_y_p1c_D_theta + D_v1_D_z_p1c * D_z_p1c_D_theta;
        D_v1_D_r1 = D_v1_D_y_p1c * D_y_p1c_D_r1 + D_v1_D_z_p1c * D_z_p1c_D_r1;
        // u3, v3 雅可比计算
        double D_u3_D_x_p3c = f_x / p_3c.z();
        double D_u3_D_z_p3c = -f_x * p_3c.x() / pow(p_3c.z(), 2);
        double D_v3_D_y_p3c = f_y / p_3c.z();
        double D_v3_D_z_p3c = -f_y * p_3c.y() / pow(p_3c.z(), 2);

        double D_x_p3w_D_theta = cos(theta) * L_a / 2 - r_1b * sin(theta);
        double D_y_p3w_D_theta = sin(theta) * L_a / 2 + r_1b * cos(theta);

        double D_x_p3c_D_theta = t_11 * D_x_p3w_D_theta + t_12 * D_y_p3w_D_theta;
        double D_y_p3c_D_theta = t_21 * D_x_p3w_D_theta + t_22 * D_y_p3w_D_theta;
        double D_z_p3c_D_theta = t_31 * D_x_p3w_D_theta + t_32 * D_y_p3w_D_theta;

        double D_x_p3c_D_r1 = t_11 * cos(theta) + t_12 * sin(theta);
        double D_y_p3c_D_r1 = t_21 * cos(theta) + t_22 * sin(theta);
        double D_z_p3c_D_r1 = t_31 * cos(theta) + t_32 * sin(theta);

        D_u3_D_xc = D_u3_D_x_p3c * t_11 + D_u3_D_z_p3c * t_31;
        D_u3_D_yc = D_u3_D_x_p3c * t_12 + D_u3_D_z_p3c * t_32;
        D_u3_D_za1 = D_u3_D_x_p3c * t_13 + D_u3_D_z_p3c * t_33;
        D_u3_D_theta = D_u3_D_x_p3c * D_x_p3c_D_theta + D_u3_D_z_p3c * D_z_p3c_D_theta;
        D_u3_D_r1 = D_u3_D_x_p3c * D_x_p3c_D_r1 + D_u3_D_z_p3c * D_z_p3c_D_r1;

        D_v3_D_xc = D_v3_D_y_p3c * t_21 + D_v3_D_z_p3c * t_31;
        D_v3_D_yc = D_v3_D_y_p3c * t_22 + D_v3_D_z_p3c * t_32;
        D_v3_D_za1 = D_v3_D_y_p3c * t_23 + D_v3_D_z_p3c * t_33;
        D_v3_D_theta = D_v3_D_y_p3c * D_y_p3c_D_theta + D_v3_D_z_p3c * D_z_p3c_D_theta;
        D_v3_D_r1 = D_v3_D_y_p3c * D_y_p3c_D_r1 + D_v3_D_z_p3c * D_z_p3c_D_r1;
    }
    if (match_type == MatchType::ARMOR || match_type == MatchType::LIGHT_RIGHT) {
        // 计算世界系下装甲板关键点坐标（齐次坐标）
        p_2w << cos(theta) * r_1t - sin(theta) * L_a / 2 + x_c,
            sin(theta) * r_1t + cos(theta) * L_a / 2 + y_c, z_a + delta_z / 2, 1;
        p_4w << cos(theta) * r_1b - sin(theta) * L_a / 2 + x_c,
            sin(theta) * r_1b + cos(theta) * L_a / 2 + y_c, z_a - delta_z / 2, 1;

        p_2c = (this->odom_camera_matrix * p_2w).head<3>();
        p_4c = (this->odom_camera_matrix * p_4w).head<3>();
        // u2, v2 雅可比计算
        double D_u2_D_x_p2c = f_x / p_2c.z();
        double D_u2_D_z_p2c = -f_x * p_2c.x() / pow(p_2c.z(), 2);
        double D_v2_D_y_p2c = f_y / p_2c.z();
        double D_v2_D_z_p2c = -f_y * p_2c.y() / pow(p_2c.z(), 2);

        double D_x_p2w_D_theta = -cos(theta) * L_a / 2 - r_1t * sin(theta);
        double D_y_p2w_D_theta = -sin(theta) * L_a / 2 + r_1t * cos(theta);

        double D_x_p2c_D_theta = t_11 * D_x_p2w_D_theta + t_12 * D_y_p2w_D_theta;
        double D_y_p2c_D_theta = t_21 * D_x_p2w_D_theta + t_22 * D_y_p2w_D_theta;
        double D_z_p2c_D_theta = t_31 * D_x_p2w_D_theta + t_32 * D_y_p2w_D_theta;

        double D_x_p2c_D_r1 = t_11 * cos(theta) + t_12 * sin(theta);
        double D_y_p2c_D_r1 = t_21 * cos(theta) + t_22 * sin(theta);
        double D_z_p2c_D_r1 = t_31 * cos(theta) + t_32 * sin(theta);

        D_u2_D_xc = D_u2_D_x_p2c * t_11 + D_u2_D_z_p2c * t_31;
        D_u2_D_yc = D_u2_D_x_p2c * t_12 + D_u2_D_z_p2c * t_32;
        D_u2_D_za1 = D_u2_D_x_p2c * t_13 + D_u2_D_z_p2c * t_33;
        D_u2_D_theta = D_u2_D_x_p2c * D_x_p2c_D_theta + D_u2_D_z_p2c * D_z_p2c_D_theta;
        D_u2_D_r1 = D_u2_D_x_p2c * D_x_p2c_D_r1 + D_u2_D_z_p2c * D_z_p2c_D_r1;

        D_v2_D_xc = D_v2_D_y_p2c * t_21 + D_v2_D_z_p2c * t_31;
        D_v2_D_yc = D_v2_D_y_p2c * t_22 + D_v2_D_z_p2c * t_32;
        D_v2_D_za1 = D_v2_D_y_p2c * t_23 + D_v2_D_z_p2c * t_33;
        D_v2_D_theta = D_v2_D_y_p2c * D_y_p2c_D_theta + D_v2_D_z_p2c * D_z_p2c_D_theta;
        D_v2_D_r1 = D_v2_D_y_p2c * D_y_p2c_D_r1 + D_v2_D_z_p2c * D_z_p2c_D_r1;
        // u4, v4 雅可比计算
        double D_u4_D_x_p4c = f_x / p_4c.z();
        double D_u4_D_z_p4c = -f_x * p_4c.x() / pow(p_4c.z(), 2);
        double D_v4_D_y_p4c = f_y / p_4c.z();
        double D_v4_D_z_p4c = -f_y * p_4c.y() / pow(p_4c.z(), 2);

        double D_x_p4w_D_theta = -cos(theta) * L_a / 2 - r_1b * sin(theta);
        double D_y_p4w_D_theta = -sin(theta) * L_a / 2 + r_1b * cos(theta);

        double D_x_p4c_D_theta = t_11 * D_x_p4w_D_theta + t_12 * D_y_p4w_D_theta;
        double D_y_p4c_D_theta = t_21 * D_x_p4w_D_theta + t_22 * D_y_p4w_D_theta;
        double D_z_p4c_D_theta = t_31 * D_x_p4w_D_theta + t_32 * D_y_p4w_D_theta;

        double D_x_p4c_D_r1 = t_11 * cos(theta) + t_12 * sin(theta);
        double D_y_p4c_D_r1 = t_21 * cos(theta) + t_22 * sin(theta);
        double D_z_p4c_D_r1 = t_31 * cos(theta) + t_32 * sin(theta);

        D_u4_D_xc = D_u4_D_x_p4c * t_11 + D_u4_D_z_p4c * t_31;
        D_u4_D_yc = D_u4_D_x_p4c * t_12 + D_u4_D_z_p4c * t_32;
        D_u4_D_za1 = D_u4_D_x_p4c * t_13 + D_u4_D_z_p4c * t_33;
        D_u4_D_theta = D_u4_D_x_p4c * D_x_p4c_D_theta + D_u4_D_z_p4c * D_z_p4c_D_theta;
        D_u4_D_r1 = D_u4_D_x_p4c * D_x_p4c_D_r1 + D_u4_D_z_p4c * D_z_p4c_D_r1;

        D_v4_D_xc = D_v4_D_y_p4c * t_21 + D_v4_D_z_p4c * t_31;
        D_v4_D_yc = D_v4_D_y_p4c * t_22 + D_v4_D_z_p4c * t_32;
        D_v4_D_za1 = D_v4_D_y_p4c * t_23 + D_v4_D_z_p4c * t_33;
        D_v4_D_theta = D_v4_D_y_p4c * D_y_p4c_D_theta + D_v4_D_z_p4c * D_z_p4c_D_theta;
        D_v4_D_r1 = D_v4_D_y_p4c * D_y_p4c_D_r1 + D_v4_D_z_p4c * D_z_p4c_D_r1;
    }

    Eigen::MatrixXd h;
    if (this->tracked_armors_num == 4 && armor_id % 2 == 1) {
        if (match_type == MatchType::ARMOR) {
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
        } else if (match_type == MatchType::LIGHT_LEFT) {
            h.resize(4, 11);
            // clang-format off
             //      x_c             v_xc,   y_c         v_yc,   z_a1        v_za,   theta,          omega,  r_1,        z_a2,       r_2
             h <<    D_u1_D_xc,      0,      D_u1_D_yc,  0,      0,          0,      D_u1_D_theta,   0,      0,          D_u1_D_za1, D_u1_D_r1,
                     D_v1_D_xc,      0,      D_v1_D_yc,  0,      0,          0,      D_v1_D_theta,   0,      0,          D_v1_D_za1, D_v1_D_r1,
                     D_u3_D_xc,      0,      D_u3_D_yc,  0,      0,          0,      D_u3_D_theta,   0,      0,          D_u3_D_za1, D_u3_D_r1,
                     D_v3_D_xc,      0,      D_v3_D_yc,  0,      0,          0,      D_v3_D_theta,   0,      0,          D_v3_D_za1, D_v3_D_r1;
            // clang-format on
        } else if (match_type == MatchType::LIGHT_RIGHT) {
            h.resize(4, 11);
            // clang-format off
             //      x_c             v_xc,   y_c         v_yc,   z_a1        v_za,   theta,          omega,  r_1,        z_a2,       r_2
             h <<    D_u2_D_xc,      0,      D_u2_D_yc,  0,      0,          0,      D_u2_D_theta,   0,      0,          D_u2_D_za1, D_u2_D_r1,
                     D_v2_D_xc,      0,      D_v2_D_yc,  0,      0,          0,      D_v2_D_theta,   0,      0,          D_v2_D_za1, D_v2_D_r1,
                     D_u4_D_xc,      0,      D_u4_D_yc,  0,      0,          0,      D_u4_D_theta,   0,      0,          D_u4_D_za1, D_u4_D_r1,
                     D_v4_D_xc,      0,      D_v4_D_yc,  0,      0,          0,      D_v4_D_theta,   0,      0,          D_v4_D_za1, D_v4_D_r1;
            // clang-format on
        }
    } else {
        if (match_type == MatchType::ARMOR) {
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
        } else if (match_type == MatchType::LIGHT_LEFT) {
            h.resize(4, 11);
            // clang-format off
             //      x_c             v_xc,   y_c         v_yc,   z_a1        v_za,   theta,          omega,  r_1,        z_a2,       r_2
             h <<    D_u1_D_xc,      0,      D_u1_D_yc,  0,      D_u1_D_za1, 0,      D_u1_D_theta,   0,      D_u1_D_r1,  0,          0,
                     D_v1_D_xc,      0,      D_v1_D_yc,  0,      D_v1_D_za1, 0,      D_v1_D_theta,   0,      D_v1_D_r1,  0,          0,
                     D_u3_D_xc,      0,      D_u3_D_yc,  0,      D_u3_D_za1, 0,      D_u3_D_theta,   0,      D_u3_D_r1,  0,          0,
                     D_v3_D_xc,      0,      D_v3_D_yc,  0,      D_v3_D_za1, 0,      D_v3_D_theta,   0,      D_v3_D_r1,  0,          0;
            // clang-format on
        } else if (match_type == MatchType::LIGHT_RIGHT) {
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

Eigen::VectorXd auto_aim::RWTracker::Singal_Armor_h_ceres(

    const Eigen::VectorXd& x,

    int armor_id,

    MatchType match_type

) {
    return build_occluded_measurement_ceres<double>(

        x,

        armor_id,

        match_type,

        this->tracked_id,

        this->tracked_armors_num,

        this->circle_index,

        this->circle_type,

        this->cam_info_,

        this->odom_camera_matrix

    );
}

Eigen::MatrixXd auto_aim::RWTracker::Singal_Armor_jh_ceres(

    const Eigen::VectorXd& x,

    int armor_id,

    MatchType match_type

) {
    using Jet = ceres::Jet<double, 11>;

    Eigen::Matrix<Jet, Eigen::Dynamic, 1> x_jet(11);

    for (int i = 0; i < 11; ++i) {
        x_jet(i).a = x(i);

        x_jet(i).v.setZero();

        x_jet(i).v[i] = 1.0;
    }

    const auto z_jet = build_occluded_measurement_ceres<Jet>(

        x_jet,

        armor_id,

        match_type,

        this->tracked_id,

        this->tracked_armors_num,

        this->circle_index,

        this->circle_type,

        this->cam_info_,

        this->odom_camera_matrix

    );

    Eigen::MatrixXd h(z_jet.size(), 11);

    for (int row = 0; row < z_jet.size(); ++row) {
        for (int col = 0; col < 11; ++col) {
            h(row, col) = z_jet(row).v[col];
        }
    }

    return h;
}

Eigen::VectorXd auto_aim::RWTracker::Singal_Armor_h_ceres_consis(
    const Eigen::VectorXd& x,
    int armor_id,
    MatchType match_type
) {
    return build_occluded_measurement_ceres_consis<double>(
        x,
        armor_id,
        match_type,
        this->tracked_id,
        this->tracked_armors_num,
        this->circle_index,
        this->circle_type,
        this->cam_info_,
        this->odom_camera_matrix
    );
}

Eigen::MatrixXd auto_aim::RWTracker::Singal_Armor_jh_ceres_consis(
    const Eigen::VectorXd& x,
    int armor_id,
    MatchType match_type
) {
    using Jet = ceres::Jet<double, 11>;

    Eigen::Matrix<Jet, Eigen::Dynamic, 1> x_jet(11);
    for (int i = 0; i < 11; ++i) {
        x_jet(i).a = x(i);
        x_jet(i).v.setZero();
        x_jet(i).v[i] = 1.0;
    }

    const auto z_jet = build_occluded_measurement_ceres_consis<Jet>(
        x_jet,
        armor_id,
        match_type,
        this->tracked_id,
        this->tracked_armors_num,
        this->circle_index,
        this->circle_type,
        this->cam_info_,
        this->odom_camera_matrix
    );

    Eigen::MatrixXd h(z_jet.size(), 11);
    for (int row = 0; row < z_jet.size(); ++row) {
        for (int col = 0; col < 11; ++col) {
            h(row, col) = z_jet(row).v[col];
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
void auto_aim::RWTracker::drawResults(cv::Mat& img) {
    if (this->match_armors.size() != this->tracked_armors_num) {
        return;
    }

    /* 绘制本次预测装甲板 */
    std::cout << "debug: drawResults" << std::endl;
    for (int i = 0; i < this->tracked_armors_num; i++) {
        cv::Scalar cv_color = cv::Scalar(255, 0, 0);
        std::vector<cv::Point2f> armor_corner(4);
        // 获取装甲板角点
        auto target_armor = use_consistent_occlusion_model_
            ? Singal_Armor_h_ceres_consis(this->ekf_prediction, i, MatchType::ARMOR)
            : Singal_Armor_h(this->ekf_prediction, i, MatchType::ARMOR);
        for (int j = 0; j < 4; j++) {
            armor_corner[j].x = target_armor(2 * j);
            armor_corner[j].y = target_armor(2 * j + 1);
        }
        // 绘制测量与预测装甲板间关系
        if (this->match_armors[i].match_type == MatchType::ARMOR) {
            for (int j = 0; j < 4; j++) {
                cv::Point2f armor_corner_measure;
                armor_corner_measure.x = this->match_armors[i].measure_points(2 * j);
                armor_corner_measure.y = this->match_armors[i].measure_points(2 * j + 1);
                cv::line(img, armor_corner[j], armor_corner_measure, cv_color, 1);
            }
        } else if (this->match_armors[i].match_type == MatchType::LIGHT_LEFT) {
            cv::Point2f armor_corner_measure[2];
            for (int j = 0; j < 2; j++) {
                armor_corner_measure[j].x = this->match_armors[i].measure_points(2 * j);
                armor_corner_measure[j].y = this->match_armors[i].measure_points(2 * j + 1);
            }
            cv::line(img, armor_corner[0], armor_corner_measure[0], cv_color, 1);
            cv::line(img, armor_corner[2], armor_corner_measure[1], cv_color, 1);
        } else if (this->match_armors[i].match_type == MatchType::LIGHT_RIGHT) {
            cv::Point2f armor_corner_measure[2];
            for (int j = 0; j < 2; j++) {
                armor_corner_measure[j].x = this->match_armors[i].measure_points(2 * j);
                armor_corner_measure[j].y = this->match_armors[i].measure_points(2 * j + 1);
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
    for (int i = 0; i < this->tracked_armors_num; i++) {
        // 获取跟踪器装甲板角点
        auto target_armor = use_consistent_occlusion_model_
            ? Singal_Armor_h_ceres_consis(this->target_state, i, MatchType::ARMOR)
            : Singal_Armor_h(this->target_state, i, MatchType::ARMOR);
        std::vector<cv::Point2f> armor_corner(4);
        for (int j = 0; j < 4; j++) {
            armor_corner[j].x = target_armor(2 * j);
            armor_corner[j].y = target_armor(2 * j + 1);
        }
        // 绘制装甲板
        cv::Scalar cv_color;
        if (this->match_armors[i].match_type == MatchType::ARMOR) {
            cv_color = cv::Scalar(0, 255, 0);
        } else if (this->match_armors[i].match_type == MatchType::LIGHT_LEFT
                   || this->match_armors[i].match_type == MatchType::LIGHT_RIGHT)
        {
            cv_color = cv::Scalar(100, 149, 237);
        } else {
            cv_color = cv::Scalar(255, 255, 255);
        }
        cv::line(img, armor_corner[0], armor_corner[1], cv_color, 1);
        cv::line(img, armor_corner[2], armor_corner[3], cv_color, 1);
        cv::line(img, armor_corner[0], armor_corner[2], cv_color, 1);
        cv::line(img, armor_corner[1], armor_corner[3], cv_color, 1);
        // 绘制装甲板信息
        std::stringstream armor_info;
        armor_info << "id: " << i;
        if (this->match_armors[i].match_type != MatchType::NONE) {
            armor_info << " | error: " << std::fixed << std::setprecision(1)
                       << this->match_armors[i].error_distance;
        }
        auto armor_info_s = armor_info.str();
        cv::Point2f text_position(armor_corner[2].x, armor_corner[2].y + 40);
        cv::putText(img, armor_info_s, text_position, cv::FONT_HERSHEY_SIMPLEX, 0.5, cv_color, 1);

        std::stringstream armor_info_2;
        if (this->match_armors[i].match_type == MatchType::ARMOR) {
            armor_info_2 << "type: armor";
        } else if (this->match_armors[i].match_type == MatchType::LIGHT_LEFT) {
            armor_info_2 << "type: left-light";
        } else if (this->match_armors[i].match_type == MatchType::LIGHT_RIGHT) {
            armor_info_2 << "type: right-light";
        }
        armor_info_s = armor_info_2.str();
        text_position.y += 15;
        cv::putText(img, armor_info_s, text_position, cv::FONT_HERSHEY_SIMPLEX, 0.5, cv_color, 1);

        {
            armor_info_s = armor_info_2.str();
            text_position.y += 15;
            cv::putText(
                img,
                armor_info_s,
                text_position,
                cv::FONT_HERSHEY_SIMPLEX,
                0.5,
                cv_color,
                1
            );

            // 显示装甲板高度
            double armor_height;
            if (this->tracked_id == ArmorName::outpost) {
                // 前哨站：根据 circle_index 和 armor_id 获取高度
                armor_height = outpost_height[(circle_index + i) % 3];
            } else {
                // 普通装甲板：根据 armor_id 选择 za1 或 za2
                if (this->tracked_armors_num == 4 && i % 2 == 1) {
                    armor_height = this->target_state(9); // za2
                } else {
                    armor_height = this->target_state(4); // za1
                }
            }

            std::stringstream armor_info_height;
            armor_info_height << "height: " << std::fixed << std::setprecision(3) << armor_height
                              << "m";
            armor_info_s = armor_info_height.str();
            text_position.y += 15;
            cv::putText(
                img,
                armor_info_s,
                text_position,
                cv::FONT_HERSHEY_SIMPLEX,
                0.5,
                cv_color,
                1
            );
        }
    }
}

/************************************************************************************************************************
 * @brief 新观测器的target接口实现
 * 
***********************************************************************************************************************/
auto_aim::RWTracker::ShowTargetInterface& auto_aim::RWTracker::get_interface() {
    return this->show_target_interface_;
}

auto_aim::RWTracker::ShowTargetInterfaceUnique& auto_aim::RWTracker::get_interface_unique() {
    return this->show_target_interface_unique_;
}

std::vector<Eigen::Vector4d> auto_aim::RWTracker::ShowTargetInterface::armor_xyza_list() const {
    std::vector<Eigen::Vector4d> armor_list;
    const auto& state = this->get_state(); // 通过引用访问 tracker 的状态

    // RWTracker 状态: [xc, v_xc, yc, v_yc, za_1, v_za, theta, omega, r1, za_2, r2]
    double xc = state(0);
    double yc = state(2);
    double theta = state(6);

    for (int i = 0; i < tracker_.tracked_armors_num; i++) { // 通过 tracker_ 访问
        double armor_theta = tracker_.normalize_angle(
            theta + 2 * M_PI / static_cast<double>(tracker_.tracked_armors_num) * i
        );

        double r, z;
        if (tracker_.tracked_id == ArmorName::outpost) {
            // 前哨站：根据 circle_index 和 armor_id 获取高度
            z = tracker_.outpost_height[(tracker_.circle_index + i) % 3];
            r = state(8); // r1
        } else {
            if (tracker_.tracked_armors_num == 4 && i % 2 == 1) {
                r = state(10); // r2
                z = state(9); // za_2
            } else {
                r = state(8); // r1
                z = state(4); // za_1
            }
        }

        double armor_x = xc + r * std::cos(armor_theta);
        double armor_y = yc + r * std::sin(armor_theta);
        double armor_z = z;

        armor_list.push_back({ armor_x, armor_y, armor_z, armor_theta });
    }

    return armor_list;
}

const Eigen::VectorXd& auto_aim::RWTracker::ShowTargetInterface::get_state() const {
    if (state_.size() == 0) {
        return tracker_.target_state;
    }
    return state_;
}

const Eigen::VectorXd& auto_aim::RWTracker::ShowTargetInterfaceUnique::get_state() const {
    if (state_.size() == 0) {
        return tracker_->target_state_unique;
    }
    return state_;
}
} //namespace auto_aim
