#include <fmt/core.h>

#include <atomic>
#include <chrono>
#include <nlohmann/json.hpp>
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
#include "tools/timer.hpp"
#include "tools/extended_kalman_filter.hpp"

#include "io/ros2/ros2.hpp"

using namespace std::chrono_literals;

const std::string keys =
    "{help h usage ? |                        | 输出命令行参数说明}"
    "{@config-path   | configs/standard_rw.yaml | 位置参数，yaml配置文件路径 }";

int main(int argc, char* argv[]) {
    tools::Exiter exiter;
    tools::Plotter plotter;
    tools::Recorder recorder(60);
    cv::CommandLineParser cli(argc, argv, keys);
    auto config_path = cli.get<std::string>(0);
    if (cli.has("help") || config_path.empty()) {
        cli.printMessage();
        return 0;
    }
    io::ROS2 ros2;
    auto_aim::Detector detector(config_path);
    io::Gimbal gimbal(config_path);
    gimbal.set_interface(ros2.subscribe2nav_->ros2_msg_);
    io::Camera camera(config_path);
    auto_aim::Solver solver(config_path);
    auto_aim::RWTracker rw_tracker(config_path, solver);
    auto_aim::Planner planner(config_path);
    nlohmann::json data;
    tools::KalmanFilter kalman_filter(10,25,1,25,1,25);
    //* 方差计算：[(max-min)/4]^2


    cv::Mat img;
    std::chrono::steady_clock::time_point t;
    std::chrono::steady_clock::time_point last_timestamp = std::chrono::steady_clock::now();
    auto time_offset_us = std::chrono::microseconds(1200);

    double debug_data_time = 0;
    double whole_process_time = 0;
    double before_send_time = 0;
    double predict_send_time = 0;
    double send_time = 0;
    auto_aim::Plan plan;
    while (!exiter.exit()) {
        {    
            tools::ScopedTimer timer("whole_process_time", whole_process_time);
            {    
                tools::ScopedTimer timer("before_send_time", before_send_time);
                camera.read(img, t);//*这里的t就是解码前的时间
                auto q = gimbal.q(t + time_offset_us);
                rw_tracker.dt_ = tools::delta_time(t, last_timestamp);
                last_timestamp = t;
                solver.set_R_gimbal2world(q);
                auto [armors, lightbars] = detector.detect_light(img);
                for (auto& armor : armors) {
                    solver.solve(armor);
                    std::cout<<armor.xyz_in_world.z()<<std::endl;
                  }
                if (rw_tracker.tracker_state == auto_aim::RWTracker::TrackState::LOST) {
                    rw_tracker.init(armors);
                }
                rw_tracker.update(armors, lightbars);
                rw_tracker.drawResults(img);

                kalman_filter.predict();
                kalman_filter.update(Eigen::VectorXd::Constant(1, send_time));
                predict_send_time = kalman_filter.getx();
            }
            {    
                tools::ScopedTimer timer("send_time", send_time);

                // auto plan = planner.plan(rw_tracker.show_target_interface_, gimbal.state().bullet_speed);
                plan = planner.plan(rw_tracker.show_target_interface_, 22.0,predict_send_time+before_send_time);
                //*这里用low_speed的补偿作为延时

                // if(rw_tracker.tracker_state == auto_aim::RWTracker::TrackState::TRACKING || rw_tracker.tracker_state == auto_aim::RWTracker::TrackState::TEMP_LOST){
                //     gimbal.send(
                //     plan.control,
                //     plan.fire,
                //     plan.yaw,
                //     plan.yaw_vel,
                //     plan.yaw_acc,
                //     plan.pitch,
                //     // 0.0,
                //     plan.pitch_vel,
                //     plan.pitch_acc
                //     );
                // }else{
                //     gimbal.send(false, false, 0, 0, 0, 0, 0, 0);
                // }
                if(rw_tracker.tracker_state == auto_aim::RWTracker::TrackState::TRACKING || rw_tracker.tracker_state == auto_aim::RWTracker::TrackState::TEMP_LOST){
                    gimbal.send_sentry(
                    plan.control,
                    plan.fire,
                    plan.yaw,
                    plan.yaw_vel,
                    plan.yaw_acc,
                    plan.pitch,
                    // 0.0,
                    plan.pitch_vel,
                    plan.pitch_acc,
                    std::get<2>(ros2.get_nav_ul_subscribe()),
                    rw_tracker.ekf.getState()[0],
                    rw_tracker.ekf.getState()[2],
                    rw_tracker.ekf.getState()[4],
                    std::get<0>(ros2.get_nav_ul_subscribe()),
                    std::get<1>(ros2.get_nav_ul_subscribe())
                    );
                }else{
                    gimbal.send_sentry(false, false, 0, 0, 0, 0, 0, 0,0,0,0,0,0,0);
                }
            }
        }

        {
            tools::ScopedTimer timer("debug_data_time", debug_data_time);
            data["plan_yaw"] = plan.yaw;
            data["plan_yaw_vel"] = plan.yaw_vel;
            data["plan_yaw_acc"] = plan.yaw_acc;
            data["plan_pitch"] = plan.pitch;
            data["plan_pitch_vel"] = plan.pitch_vel;
            data["plan_pitch_acc"] = plan.pitch_acc;
            data["gimbal_yaw"] = gimbal.state().yaw;
            data["gimbal_yaw_vel"] = gimbal.state().yaw_vel;
            data["gimbal_pitch"] = gimbal.state().pitch;
            data["gimbal_pitch_vel"] = gimbal.state().pitch_vel;
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
            data["gimbal_yaw"] = gimbal.state().yaw;
            data["gimbal_pitch"] = gimbal.state().pitch;

            data["debug_data_time"] = debug_data_time;
            data["whole_process_time"] = whole_process_time;
            plotter.plot(data);

            // recorder.record(img, q, t);
            // 在图像上显示当前的时间偏移量
            auto offset_text = fmt::format("Time Offset: {} us", time_offset_us.count());
            cv::putText(img, offset_text, { 10, 60 }, cv::FONT_HERSHEY_SIMPLEX, 1.0, { 0, 255, 0 }, 2);

            cv::resize(img, img, {}, 0.5, 0.5); // 显示时缩小图片尺寸
            cv::imshow("reprojection", img);
            auto key = cv::waitKey(1);

            if (key == '=' || key == '+') {
                time_offset_us += std::chrono::microseconds(50);
            }
            if (key == '-') {
                time_offset_us -= std::chrono::microseconds(50);
            }
        }
    }

    // auto key = cv::waitKey(13);
    
}
