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
#include "tools/lock_free_snapshot.hpp"

using namespace std::chrono_literals;
 
const std::string keys =
    "{help h usage ? |                        | 输出命令行参数说明}"
    "{@config-path   | configs/standard_rw.yaml | 位置参数，yaml配置文件路径 }";

struct ControlState {
    auto_aim::RWTracker::TrackState tracker_state = auto_aim::RWTracker::TrackState::LOST;
    Eigen::VectorXd target_state;
    int tracked_armors_num = 0;
    std::chrono::steady_clock::time_point timestamp{};
    double dt = 0.0;
    bool has_state = false;
};

struct DisplayState {
    cv::Mat img;
};

struct PlotPacket {
    nlohmann::json data;
};

int main(int argc, char* argv[]) {
    tools::Exiter exiter;
    tools::Plotter plotter;
   

    cv::CommandLineParser cli(argc, argv, keys);
    auto config_path = cli.get<std::string>(0);
    if (cli.has("help") || config_path.empty()) {
        cli.printMessage();
        return 0;
    }
    auto_aim::Detector detector(config_path);
    io::Gimbal gimbal(config_path);
    io::Camera camera(config_path);
    auto_aim::Solver solver(config_path);
    auto_aim::Planner planner(config_path);
    auto_aim::RWTracker rw_tracker(config_path, solver);

    auto time_offset_us = std::chrono::microseconds(1400);   //1200
    
    LockFreeSnapshot<ControlState> shared_control_state; // 无锁控制快照
    LockFreeSnapshot<DisplayState> shared_display_state; // 无锁图像快照
    LockFreeSnapshot<PlotPacket> shared_plot_state;      // 无锁绘图快照
     
    constexpr size_t kPlotQueueMax = 200;

    tools::KalmanFilter time_kf(15,200,1,200,1,200);

    // 1. 视觉线程
    std::thread vision_thread([&]() {
        cv::Mat img;
        std::chrono::steady_clock::time_point t;
        std::chrono::steady_clock::time_point last_timestamp = std::chrono::steady_clock::now();

        while (!exiter.exit()) {
            camera.read(img, t);
            auto q = gimbal.q(t + time_offset_us);

            double dt = tools::delta_time(t, last_timestamp);
            if(dt>0.1){dt=0.1;} //处理相机的底层时间戳 t 与系统的 now() 不是完美同步，防止第一帧卡尔曼暴走
            rw_tracker.dt_ = dt;
            last_timestamp = t;
            
            solver.set_R_gimbal2world(q);
            auto [armors, lightbars] = detector.detect_light(img);

            for (auto& armor: armors) 
            {
                solver.solve(armor);
            }

            if (rw_tracker.tracker_state == auto_aim::RWTracker::TrackState::LOST) {
                rw_tracker.init(armors);
            }
            else{
                rw_tracker.update(armors, lightbars);
            }

            rw_tracker.drawResults(img);

            cv::Mat clone_img = img.clone(); 

            // 更新共享数据
            ControlState next_state;
            next_state.tracker_state = rw_tracker.tracker_state;
            next_state.target_state = rw_tracker.target_state;
            next_state.tracked_armors_num = rw_tracker.tracked_armors_num;
            next_state.timestamp = t;
            next_state.dt = dt;
            next_state.has_state = true;
            shared_control_state.store(next_state);

            // 独立更新显示图像
            DisplayState d_state;
            d_state.img = std::move(clone_img);
            shared_display_state.store(d_state);
        }
    });

    // 2. 控制线程 (高频)
    std::thread control_thread([&]() {
        auto next_tick = std::chrono::steady_clock::now();

        
        while (!exiter.exit()) {
            next_tick += 2ms;
            nlohmann::json data; // 恢复 Plotter 所需数据结构

            ControlState current_state = shared_control_state.load();
            static std::chrono::steady_clock::time_point last_vision_time; // 记录上一次处理的时间戳

            auto_aim::Plan plan;

            bool tracking_ok = current_state.has_state &&
                   current_state.target_state.size() >= 11 &&
                   current_state.tracked_armors_num > 0 &&
                   (current_state.tracker_state == auto_aim::RWTracker::TrackState::TRACKING ||
                    current_state.tracker_state == auto_aim::RWTracker::TrackState::TEMP_LOST);

            if (tracking_ok) 
            {
                //卡尔曼滤波平滑延迟时间
                // 仅当收到全新的视觉帧时，才更新卡尔曼滤波器
                if (current_state.timestamp != last_vision_time) {
                    double process_delay = tools::delta_time(std::chrono::steady_clock::now(), current_state.timestamp);
                    time_kf.predict();
                    time_kf.update(Eigen::VectorXd::Constant(1, process_delay));
                    last_vision_time = current_state.timestamp;
                }
                
                // 实际发弹延迟 = 滤波后的处理延迟 + 这一帧变老的时间
                double age = tools::delta_time(std::chrono::steady_clock::now(), current_state.timestamp);
                double send_time = time_kf.getx() + age;

                if (send_time < 0.0) send_time = 0.0;
                if (send_time > 0.2) send_time = 0.2;

                // 根据 send_time 将状态线性外推到现在 
                Eigen::VectorXd control_predict_state = current_state.target_state;
                if (send_time > 0 && control_predict_state.size() >= 11) {
                    control_predict_state[0] += control_predict_state[1] * send_time;  // xc += v_xc * dt
                    control_predict_state[2] += control_predict_state[3] * send_time;  // yc += v_yc * dt
                    control_predict_state[4] += control_predict_state[5] * send_time;  // za1 += v_za * dt
                    control_predict_state[6] = tools::limit_rad(control_predict_state[6] + control_predict_state[7] * send_time); // theta
                    control_predict_state[9] += control_predict_state[5] * send_time;  // za2 += v_za * dt
                }

                // 基于最新状态进行规划补偿 
                plan = planner.plan(
                    control_predict_state,           
                    current_state.tracked_armors_num,     
                    11.5,                              
                    0     
                );

                double actual_yaw_err = std::abs(tools::limit_rad(plan.target_yaw - gimbal.state().yaw));
                double actual_pitch_err = std::abs(plan.target_pitch - gimbal.state().pitch);   //因为罚单延迟可能导致时间上难以对齐

                bool is_gimbal_ready = actual_yaw_err < 0.035 && actual_pitch_err < 0.015; // 0.04 0.02 约 2.3  1.1 度
                bool final_fire = plan.fire && is_gimbal_ready && gimbal.state().fire_calm; 

                gimbal.send(plan.control, final_fire, plan.yaw, plan.yaw_vel, plan.yaw_acc, plan.pitch, plan.pitch_vel, plan.pitch_acc);
                data["plan_yaw"] = plan.yaw;
                data["actual_yaw_err"] = actual_yaw_err;
                data["send_yaw"] = plan.yaw;
                data["plan_pitch"] = plan.pitch;
            } 
            else {
                gimbal.send_lose(gimbal.state());
                data["send_yaw"] = gimbal.state().yaw;
            }

            // 更新云台状态与绘图（
            data["gimbal_yaw"] = gimbal.state().yaw;
            data["gimbal_pitch"] = gimbal.state().pitch;
            
            // 无锁更新绘图数据
            PlotPacket pkt;
            pkt.data = std::move(data);
            shared_plot_state.store(pkt);
            
            // 保持高频例如 500Hz (2ms)跑一次控制
            std::this_thread::sleep_until(next_tick);
        }
    });

    // 3. 独立绘图线程（建议 50~100Hz）
    std::thread plot_thread([&]() {
        auto next_tick = std::chrono::steady_clock::now();
        while (!exiter.exit()) {
        next_tick += 20ms; // 约 50Hz

            // 无锁获取最新绘图数据
            PlotPacket pkt = shared_plot_state.load();
            if (!pkt.data.empty()) {
                plotter.plot(pkt.data);
            }

            std::this_thread::sleep_until(next_tick);
        }
    });

    // 4. 显示线程                
        while (!exiter.exit()) {
        cv::Mat display_img;
        
        DisplayState current_display = shared_display_state.load();
        if (!current_display.img.empty()) {
            display_img = current_display.img;
        }

        if (!display_img.empty()) {
            auto offset_text = fmt::format("Time Offset: {} us", time_offset_us.count());
            cv::putText(display_img, offset_text, { 10, 60 }, cv::FONT_HERSHEY_SIMPLEX, 1.0, { 0, 255, 0 }, 2);
            cv::resize(display_img, display_img, {}, 0.5, 0.5);
            cv::imshow("reprojection", display_img);
            
            auto key = cv::waitKey(16); // 约 60FPS
            if (key == 'q') std::exit(0); // 触发全体退出
        }else {
                // 如果没有拿到图像，稍微休息一下防止空转吃满单核CPU
                std::this_thread::sleep_for(2ms);
            }
        }
    

    vision_thread.join();
    control_thread.join();
    plot_thread.join();
    return 0;
}