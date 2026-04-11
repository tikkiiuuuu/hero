#include <fmt/core.h>

#include <atomic>
#include <chrono>
#include <deque>
#include <fstream>
#include <numeric>
#include <opencv2/opencv.hpp>
#include <thread>

#include "io/camera.hpp"
#include "io/gimbal/gimbal.hpp"
#include "tasks/auto_aim/planner/planner.hpp"
#include "tasks/auto_aim/rw_tracker.hpp"
#include "tasks/auto_aim/solver.hpp"
// #include "tasks/auto_aim/yolo.hpp"
#include "tasks/auto_aim/detector.hpp"
#include "tools/exiter.hpp"
#include "tools/img_tools.hpp"
#include "tools/logger.hpp"
#include "tools/math_tools.hpp"
#include "tools/plotter.hpp"
#include "tools/recorder.hpp"
#include "tools/thread_safe_queue.hpp"

using namespace std::chrono_literals;

const std::string keys =
  "{help h usage ? |                   | 输出命令行参数说明 }"
    "{config-path c  | configs/standard_rw.yaml | yaml配置文件的路径}"
  "{start-index s  | 0                 | 视频起始帧下标    }"
  "{end-index e    | 0                 | 视频结束帧下标    }"
  "{@input-path    | assets/demo/demo1  | avi和txt文件的路径}";

int main(int argc, char* argv[]) {
    tools::Exiter exiter;
    tools::Plotter plotter;
    //tools::Recorder recorder(60);

    cv::CommandLineParser cli(argc, argv, keys);
    auto config_path = cli.get<std::string>("config-path");
    if (cli.has("help") || config_path.empty()) {
        cli.printMessage();
        return 0;
    }
    auto input_path = cli.get<std::string>(0);
    auto start_index = cli.get<int>("start-index");
    auto end_index = cli.get<int>("end-index");

    auto video_path = fmt::format("{}.avi", input_path);
    auto text_path = fmt::format("{}.txt", input_path);
    cv::VideoCapture video(video_path);
    std::ifstream text(text_path);
    if (!video.isOpened() || !text.is_open()) {
        tools::logger()->error("[rw_tracker_test] Failed to open demo files: {} / {}", video_path, text_path);
        return 1;
    }

    auto_aim::Detector detector(config_path);

    auto_aim::Solver solver(config_path);
    auto_aim::Planner planner(config_path);
    auto_aim::RWTracker rw_tracker(config_path, solver);

    nlohmann::json data;

    cv::Mat img;
    std::chrono::steady_clock::time_point t;
    auto t0 = std::chrono::steady_clock::now();
    std::chrono::steady_clock::time_point last_timestamp = std::chrono::steady_clock::now();
    auto time_offset_us = std::chrono::microseconds(1200);   // 1400
    // auto time_offset_us = std::chrono::microseconds(1200);
    int a = 0;

    //记录上一帧的状态和恢复缓冲帧数
    auto_aim::RWTracker::TrackState last_track_state = auto_aim::RWTracker::TrackState::LOST;
    int recovery_frames = 0;

    float last_plan_yaw = 0.0f;
    float last_plan_pitch = 0.0f;
    bool has_last_plan = false;
    constexpr float MAX_PLAN_YAW_STEP = 0.08f; // 每帧最大变化约 4.6 度，可调
    constexpr float MAX_PLAN_PITCH_STEP = 0.05f; // pitch 变化可稍严一点


    // 最近10帧连续变化之和的限幅 
    std::deque<float> past_yaw_deltas;
    std::deque<float> past_pitch_deltas;
    constexpr int DELTA_WINDOW_SIZE = 10;
    constexpr float MAX_WINDOW_YAW_CHANGE = 0.35f;   // 10帧内总yaw变化约 20度
    constexpr float MAX_WINDOW_PITCH_CHANGE = 0.20f; // 10帧内总pitch变化约 11度

    video.set(cv::CAP_PROP_POS_FRAMES, start_index);
    for (int i = 0; i < start_index; i++) {
        double td, w, x, y, z;
        text >> td >> w >> x >> y >> z;
    }

    for (int frame_count = start_index; !exiter.exit(); frame_count++) {
        if (end_index > 0 && frame_count > end_index) {
            break;
        }

        std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
        video.read(img);
        if (img.empty()) {
            break;
        }

        double td, w, x, y, z;
        if (!(text >> td >> w >> x >> y >> z)) {
            break;
        }
        t = t0 + std::chrono::microseconds(int(td * 1e6));

        Eigen::Quaterniond q(w, x, y, z);
        auto gimbal_euler = tools::eulers(q, 2, 1, 0);
        const float gimbal_yaw = gimbal_euler[0];
        const float gimbal_pitch = gimbal_euler[1];
        const float gimbal_yaw_vel = 0.0f;
        const float gimbal_pitch_vel = 0.0f;
        const bool fire_calm = true;
        const uint8_t enemy_color = 0;

        rw_tracker.set_enemy_color(enemy_color); //设置敌方颜色
        rw_tracker.dt_ = tools::delta_time(t, last_timestamp);
        last_timestamp = t;
        solver.set_R_gimbal2world(q);
        auto [armors, lightbars] = detector.detect_light(img);
        for (auto& armor: armors) {
            solver.solve(armor);
            std::cout << armor.xyz_in_world.z() << std::endl;
        }
        if (rw_tracker.tracker_state == auto_aim::RWTracker::TrackState::LOST) {
            rw_tracker.init(armors);
        }else{
            rw_tracker.update(armors, lightbars);
        }

        // 状态机切换检测，如果刚从 LOST 变成 TRACKING，启动10帧的缓冲期
        if (last_track_state == auto_aim::RWTracker::TrackState::LOST && 
            rw_tracker.tracker_state == auto_aim::RWTracker::TrackState::TRACKING) {
            recovery_frames = 20; // 设置10帧的滤波收敛期
        }
        last_track_state = rw_tracker.tracker_state;

        auto camera_x = rw_tracker.odom_camera_matrix.block<3, 1>(0, 3)[0];
        auto camera_y = rw_tracker.odom_camera_matrix.block<3, 1>(0, 3)[1];
        auto camera_z = rw_tracker.odom_camera_matrix.block<3, 1>(0, 3)[2];
        data["camera_x"] = camera_x;
        data["camera_y"] = camera_y;
        data["camera_z"] = camera_z;
        data["enemy_color"]=enemy_color;
        rw_tracker.drawResults(img);
        std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
        data["dt"] = tools::delta_time(t2, t1);

        bool is_spinning = std::abs(rw_tracker.target_state[7]) > 1;

        auto_aim::Plan plan;
        bool in_recovery_hold = false;
        if (rw_tracker.tracker_state == auto_aim::RWTracker::TrackState::TRACKING ||
            rw_tracker.tracker_state == auto_aim::RWTracker::TrackState::TEMP_LOST) 
        {
            //只在缓冲期内进行限幅和限速
            if (recovery_frames > 0) {
                in_recovery_hold = true;
                // const float MAX_ANGLE_STEP = 0.05f; // 缓冲期的严格角度步长限制(约2.8度)
                
                // plan.yaw = std::clamp(static_cast<float>(plan.yaw), 
                //                       gimbal.state().yaw - MAX_ANGLE_STEP, 
                //                       gimbal.state().yaw + MAX_ANGLE_STEP);
                // plan.pitch = std::clamp(static_cast<float>(plan.pitch), 
                //                         gimbal.state().pitch - MAX_ANGLE_STEP, 
                //                         gimbal.state().pitch + MAX_ANGLE_STEP);

                if (is_spinning) {
                    plan = planner.plan_1(
                        rw_tracker.target_state,
                        rw_tracker.tracked_armors_num,
                        11.5,
                        tools::delta_time(t2, t),
                        rw_tracker.getCircleIndex());
                } else {
                    plan = planner.plan_2(
                        rw_tracker.target_state,
                        rw_tracker.tracked_armors_num,
                        11.5,
                        tools::delta_time(t2, t),
                        rw_tracker.getCircleIndex());
                }
                plan.control=false;
                plan.pitch=gimbal_pitch;
                plan.yaw=gimbal_yaw;
                
                // 刚追踪时速度不可信，直接干掉前馈速度防止疯卷
                plan.yaw_vel = 0.0f;
                plan.pitch_vel = 0.0f;
                plan.yaw_acc = 0.0f;
                plan.pitch_acc = 0.0f;
                plan.fire=0;
                recovery_frames--;

            }

            else{
                if (is_spinning) {
                    plan = planner.plan_1(
                        rw_tracker.target_state,
                        rw_tracker.tracked_armors_num,
                        11.5,
                        tools::delta_time(t2, t),
                        rw_tracker.getCircleIndex());
                } else {
                    plan = planner.plan_2(
                        rw_tracker.target_state,
                        rw_tracker.tracked_armors_num,
                        11.5,
                        tools::delta_time(t2, t),
                        rw_tracker.getCircleIndex());
                }
            } 

            // 在生成完 plan 后对连续两帧的 plan 进行差值限幅
            if (has_last_plan) {
                // 处理角度跳变 (Angle Unwrapping)，保证走最短路径
                float yaw_diff = plan.yaw - last_plan_yaw;
                while (yaw_diff > M_PI) yaw_diff -= 2.0f * M_PI;
                while (yaw_diff < -M_PI) yaw_diff += 2.0f * M_PI;
                 
                float pitch_diff = plan.pitch - last_plan_pitch;

                // 1. 每帧的硬性限幅
                yaw_diff = std::clamp(yaw_diff, -MAX_PLAN_YAW_STEP, MAX_PLAN_YAW_STEP);
                pitch_diff = std::clamp(pitch_diff, -MAX_PLAN_PITCH_STEP, MAX_PLAN_PITCH_STEP);

                // 2. 最近10帧窗口的累积限幅
                float sum_yaw_deltas = std::accumulate(past_yaw_deltas.begin(), past_yaw_deltas.end(), 0.0f);
                float sum_pitch_deltas = std::accumulate(past_pitch_deltas.begin(), past_pitch_deltas.end(), 0.0f);

                if (sum_yaw_deltas + yaw_diff > MAX_WINDOW_YAW_CHANGE) {
                    yaw_diff = MAX_WINDOW_YAW_CHANGE - sum_yaw_deltas;
                } else if (sum_yaw_deltas + yaw_diff < -MAX_WINDOW_YAW_CHANGE) {
                    yaw_diff = -MAX_WINDOW_YAW_CHANGE - sum_yaw_deltas;
                }

                if (sum_pitch_deltas + pitch_diff > MAX_WINDOW_PITCH_CHANGE) {
                    pitch_diff = MAX_WINDOW_PITCH_CHANGE - sum_pitch_deltas;
                } else if (sum_pitch_deltas + pitch_diff < -MAX_WINDOW_PITCH_CHANGE) {
                    pitch_diff = -MAX_WINDOW_PITCH_CHANGE - sum_pitch_deltas;
                }

                // 更新滑动窗口状态
                past_yaw_deltas.push_back(yaw_diff);
                if (past_yaw_deltas.size() > DELTA_WINDOW_SIZE) past_yaw_deltas.pop_front();

                past_pitch_deltas.push_back(pitch_diff);
                if (past_pitch_deltas.size() > DELTA_WINDOW_SIZE) past_pitch_deltas.pop_front();

                // 应用最终过滤的目标角度
                plan.yaw = last_plan_yaw + yaw_diff;
                plan.pitch = last_plan_pitch + pitch_diff;
            }
            last_plan_yaw = plan.yaw;
            last_plan_pitch = plan.pitch;
            has_last_plan = true;
        }
        else {
            plan.control = false; // 处于 LOST 时完全不介入推演
            plan.fire = false;
            has_last_plan = false; // 丢失时重置记录状态
            past_yaw_deltas.clear();   // 清空窗口历史记录
            past_pitch_deltas.clear();
        }

        // 误差评估统一使用最终执行角，避免与 target_yaw/target_pitch 分叉造成观感抖动。
        double actual_yaw_err = std::abs(tools::limit_rad(plan.yaw - gimbal_yaw));
        double actual_pitch_err = std::abs(plan.pitch - gimbal_pitch);

        // 整车中心开火门限：yaw / pitch 双轴同时满足，并限制合成角误差
        constexpr double FIRE_CENTER_YAW_ERR_THRES = 0.035;    // 约 2.0°
        constexpr double FIRE_CENTER_PITCH_ERR_THRES = 0.015;  // 约 0.86°
        constexpr double FIRE_CENTER_TOTAL_ERR_THRES = 0.038;  // 合成角误差门限

        bool is_yaw_ready = actual_yaw_err < FIRE_CENTER_YAW_ERR_THRES;
        bool is_pitch_ready = actual_pitch_err < FIRE_CENTER_PITCH_ERR_THRES;
        double center_angle_err = std::hypot(actual_yaw_err, actual_pitch_err);
        bool is_center_ready = is_yaw_ready && is_pitch_ready && center_angle_err < FIRE_CENTER_TOTAL_ERR_THRES;
        bool is_strict_tracking = rw_tracker.tracker_state == auto_aim::RWTracker::TrackState::TRACKING;

        bool final_fire =
            plan.fire;
            // is_center_ready &&
            // // is_strict_tracking &&
            // (recovery_frames <= 0) &&
            // fire_calm;

        int fire_block_code = 0;
        std::string fire_block_reason = "ALLOW";
        if (rw_tracker.tracker_state != auto_aim::RWTracker::TrackState::TRACKING &&
            rw_tracker.tracker_state != auto_aim::RWTracker::TrackState::TEMP_LOST) {
            fire_block_code = 1;
            fire_block_reason = "NO_TRACK";
        } else if (in_recovery_hold) {
            fire_block_code = 2;
            fire_block_reason = "RECOVERY_HOLD";
        } else if (!final_fire) {
            if (is_spinning) {
                bool not_converged = false;
                if (rw_tracker.target_state.size() > 10 && rw_tracker.tracked_armors_num == 4) {
                    const double r1 = rw_tracker.target_state[8];
                    const double r2 = rw_tracker.target_state[10];
                    not_converged = (r1 < 0.15 || r1 > 0.50 || r2 < 0.15 || r2 > 0.50);
                }
                if (not_converged) {
                    fire_block_code = 3;
                    fire_block_reason = "PLAN1_NOT_CONVERGED";
                } else {
                    fire_block_code = 4;
                    fire_block_reason = "PLAN1_WINDOW_OR_NORMAL";
                }
            } else {
                fire_block_code = 5;
                fire_block_reason = "PLAN2_NOT_READY";
            }
        }

        data["plan_yaw"] = plan.yaw;
        data["plan_yaw_vel"] = plan.yaw_vel;
        data["plan_yaw_acc"] = plan.yaw_acc;
        data["plan_pitch"] = plan.pitch;
        data["plan_pitch_vel"] = plan.pitch_vel;
        data["plan_pitch_acc"] = plan.pitch_acc;
        data["gimbal_yaw"] = gimbal_yaw;
        data["gimbal_yaw_vel"] = gimbal_yaw_vel;
        data["gimbal_pitch"] = gimbal_pitch;
        data["gimbal_pitch_vel"] = gimbal_pitch_vel;
        data["temp_lost"] = rw_tracker.tracker_state == auto_aim::RWTracker::TrackState::TEMP_LOST;
        data["lost"] = rw_tracker.tracker_state == auto_aim::RWTracker::TrackState::LOST;
        data["detecting"] = rw_tracker.tracker_state == auto_aim::RWTracker::TrackState::DETECTING;
        data["tracker_center_x"] = rw_tracker.target_state[0];
        data["tracker_center_vx"] = rw_tracker.target_state[1];
        data["tracker_center_y"] = rw_tracker.target_state[2];
        data["tracker_center_vy"] = rw_tracker.target_state[3];
        data["tracker_center_theta"] = rw_tracker.target_state[6];
        data["tracker_center_omega"] = rw_tracker.target_state[7];
        data["tracker_center_r1"] = rw_tracker.target_state[8];
        data["gimbal_yaw"] = gimbal_yaw;
        data["gimbal_pitch"] = gimbal_pitch;

        data["fire"] = final_fire ? 1 : 0;
        data["plan_fire"] = plan.fire ? 1 : 0;
        data["is_gimbal_ready"] = is_center_ready ? 1 : 0;
        data["is_yaw_ready"] = is_yaw_ready ? 1 : 0;
        data["is_pitch_ready"] = is_pitch_ready ? 1 : 0;
        data["center_angle_err"] = center_angle_err;
        data["target_yaw"] = plan.yaw;
        data["target_pitch"] = plan.pitch;

        data["max_window_yaw_err"] = plan.max_window_yaw_err;
        data["max_window_pitch_err"] = plan.max_window_pitch_err;
        
        
        data["actual_yaw_err"] = actual_yaw_err;
        data["actual_pitch_err"] = actual_pitch_err;
        data["fire_block_code"] = fire_block_code;

        data["fire_calm"] = fire_calm;
        //data["bullet_speed"] = gimbal.state().bullet_speed;
        //data["bullet_count"] = gimbal.state().bullet_count;
        if(rw_tracker.tracker_state == auto_aim::RWTracker::TrackState::TRACKING
            || rw_tracker.tracker_state == auto_aim::RWTracker::TrackState::TEMP_LOST){
                data["send_yaw"]= plan.yaw;
            }
        else{
            data["send_yaw"]= gimbal_yaw;
        }  
        // plotter.plot(data);

        //recorder.record(img, q, t);
        // 在图像上显示当前的时间偏移量
        auto offset_text = fmt::format("Time Offset: {} us", time_offset_us.count());
        cv::putText(img, offset_text, { 10, 60 }, cv::FONT_HERSHEY_SIMPLEX, 1.0, { 0, 255, 0 }, 2);
        auto fire_gate_text = fmt::format("FireGate[{}]: {}", fire_block_code, fire_block_reason);
        cv::putText(img, fire_gate_text, { 10, 95 }, cv::FONT_HERSHEY_SIMPLEX, 0.8, { 0, 255, 255 }, 2);

        // std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
        // data["dt"] = tools::delta_time(t2, t1);
        plotter.plot(data);
        cv::resize(img, img, {}, 0.5, 0.5); // 显示时缩小图片尺寸
        
        // 添加瞄准中心十字
        if (rw_tracker.tracker_state == auto_aim::RWTracker::TrackState::TRACKING || 
            rw_tracker.tracker_state == auto_aim::RWTracker::TrackState::TEMP_LOST) {
            double aim_dist = plan.dist;
            if (aim_dist <= 1e-6) {
                aim_dist = std::hypot(rw_tracker.target_state[0], rw_tracker.target_state[2]);
            }

            Eigen::Vector3d aim_xyz = tools::ypd2xyz({plan.yaw, plan.pitch, aim_dist});
            std::vector<cv::Point3f> target_pts = {
                cv::Point3f(
                    static_cast<float>(aim_xyz.x()),
                    static_cast<float>(aim_xyz.y()),
                    static_cast<float>(aim_xyz.z()))
            };
            auto projected_pts = solver.world2pixel(target_pts);
            if (!projected_pts.empty()) {
                cv::Point center(projected_pts[0].x * 0.5, projected_pts[0].y * 0.5); // 根据上面的 resize 进行等比例缩放
                if(plan.fire==1){             
                int cross_size = 10;
                cv::line(img, cv::Point(center.x - cross_size, center.y), cv::Point(center.x + cross_size, center.y), cv::Scalar(0, 0, 255), 2);
                cv::line(img, cv::Point(center.x, center.y - cross_size), cv::Point(center.x, center.y + cross_size), cv::Scalar(0, 0, 255), 2);
                cv::circle(img, center, cross_size, cv::Scalar(0, 0, 255), 1);
                }
   
            }
        }

        cv::imshow("reprojection", img);
        auto key = cv::waitKey(20);
        // auto key = cv::waitKey(13);
        if (key == '=' || key == '+') {
            time_offset_us += std::chrono::microseconds(100);
            //  += 1;
        }
        if (key == '-') {
            time_offset_us -= std::chrono::microseconds(100);
            // a -= 1;
        }
        if (key == 'q') 
            std::exit(0);
    }
}

