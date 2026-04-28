cmake_minimum_required(VERSION 3.16.3)

project(sp_vision)

# set(CMAKE_INTERPROCEDURAL_OPTIMIZATION TRUE)

if(CMAKE_CXX_COMPILER_ID STREQUAL "GNU" OR CMAKE_CXX_COMPILER_ID STREQUAL "Clang")
    add_compile_options(-Wno-deprecated-enum-enum-conversion)
endif()

# 强制优先使用 ruckig 目录下的 json 库
include_directories(BEFORE SYSTEM /usr/local/include/ruckig/third_party)

# find_program(CMAKE_C_COMPILER_FOUND clang)
# find_program(CMAKE_CXX_COMPILER_FOUND clang++)
# set(CMAKE_C_COMPILER "clang")
# set(CMAKE_CXX_COMPILER "clang++")

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()
message(STATUS "--------------------CMAKE_BUILD_TYPE: ${CMAKE_BUILD_TYPE}--------------------")

if(CMAKE_CXX_COMPILER_ID STREQUAL "GNU" OR CMAKE_CXX_COMPILER_ID STREQUAL "Clang")
    include(CheckCXXCompilerFlag)
    CHECK_CXX_COMPILER_FLAG("-march=native" COMPILER_SUPPORTS_MARCH_NATIVE)
    if(COMPILER_SUPPORTS_MARCH_NATIVE)
        set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -march=native")
        message(STATUS "Compiler supports -march=native, enabling it for architecture-specific optimizations.")
    endif()
endif()

# set(CMAKE_PREFIX_PATH "${CMAKE_PREFIX_PATH};/opt/ros/humble")
# 在 x86-64 架构上禁用 nvpl 查找（nvpl 主要用于 GPU 加速，x86-64 上不需要）
if(CMAKE_SYSTEM_PROCESSOR MATCHES "x86_64")
    set(CMAKE_DISABLE_FIND_PACKAGE_nvpl TRUE)
    message(STATUS "Disabling nvpl search on x86_64 architecture")
endif()

find_package(OpenCV REQUIRED)
find_package(fmt REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(spdlog REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(nlohmann_json REQUIRED)
find_package(fastcdr REQUIRED)
set(OpenVINO_DIR "/opt/intel/openvino_2024.6.0/runtime/cmake/")
find_package(OpenVINO REQUIRED)
find_package(ruckig REQUIRED)

include_directories(${EIGEN3_INCLUDE_DIR})
include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${PROJECT_SOURCE_DIR})


add_subdirectory(tools)
add_subdirectory(io)
add_subdirectory(tasks/auto_aim)
add_subdirectory(tasks/auto_buff)
add_subdirectory(tasks/omniperception)

####################src###################
add_executable(standard src/standard.cpp)
# add_executable(mt_standard src/mt_standard.cpp)
add_executable(standard_mpc src/standard_mpc.cpp)
add_executable(auto_aim_debug_mpc src/auto_aim_debug_mpc.cpp)
# add_executable(mt_auto_aim_debug src/mt_auto_aim_debug.cpp)
add_executable(auto_buff_debug src/auto_buff_debug.cpp)
add_executable(auto_buff_debug_mpc src/auto_buff_debug_mpc.cpp)
add_executable(uav src/uav.cpp)
add_executable(uav_debug src/uav_debug.cpp)

add_executable(sentry_ul src/sentry_ul.cpp)
target_link_libraries(sentry_ul ${OpenCV_LIBS} fmt::fmt yaml-cpp auto_aim tools io)

target_link_libraries(standard ${OpenCV_LIBS} fmt::fmt yaml-cpp auto_aim auto_buff tools io )
# target_link_libraries(mt_standard ${OpenCV_LIBS} fmt::fmt yaml-cpp auto_aim auto_buff tools io)
target_link_libraries(standard_mpc ${OpenCV_LIBS} fmt::fmt yaml-cpp auto_aim auto_buff tools io)
target_link_libraries(auto_aim_debug_mpc ${OpenCV_LIBS} fmt::fmt yaml-cpp auto_aim auto_buff tools io)
# target_link_libraries(mt_auto_aim_debug ${OpenCV_LIBS} fmt::fmt yaml-cpp auto_aim auto_buff tools io)
target_link_libraries(auto_buff_debug ${OpenCV_LIBS} fmt::fmt yaml-cpp auto_buff tools io ${CERES_LIBRARIES})
target_link_libraries(auto_buff_debug_mpc ${OpenCV_LIBS} fmt::fmt yaml-cpp auto_buff tools io ${CERES_LIBRARIES})
target_link_libraries(uav ${OpenCV_LIBS} fmt::fmt yaml-cpp auto_aim auto_buff tools io)
target_link_libraries(uav_debug ${OpenCV_LIBS} fmt::fmt yaml-cpp auto_aim tools io)

##################calibration################
add_executable(capture calibration/capture.cpp)
add_executable(calibrate_camera calibration/calibrate_camera.cpp)
add_executable(calibrate_handeye calibration/calibrate_handeye.cpp)
add_executable(calibrate_robotworld_handeye calibration/calibrate_robotworld_handeye.cpp)
add_executable(split_video calibration/split_video.cpp)

target_link_libraries(capture ${OpenCV_LIBS} fmt::fmt tools io)
target_link_libraries(calibrate_camera ${OpenCV_LIBS} fmt::fmt yaml-cpp tools)
target_link_libraries(calibrate_handeye ${OpenCV_LIBS} fmt::fmt yaml-cpp tools)
target_link_libraries(calibrate_robotworld_handeye ${OpenCV_LIBS} fmt::fmt yaml-cpp tools)
target_link_libraries(split_video ${OpenCV_LIBS} fmt::fmt tools)

##################tests##################
add_executable(auto_aim_test tests/auto_aim_test.cpp)
add_executable(auto_buff_test tests/auto_buff_test.cpp)
add_executable(camera_detect_test tests/camera_detect_test.cpp)
add_executable(camera_bayerdetect_test tests/camera_bayerdetect_test.cpp)
add_executable(camera_test tests/camera_test.cpp)
add_executable(camera_thread_test tests/camera_thread_test.cpp)
add_executable(cboard_test tests/cboard_test.cpp)
add_executable(fire_test tests/fire_test.cpp)
add_executable(detector_video_test tests/detector_video_test.cpp)
add_executable(gimbal_response_test tests/gimbal_response_test.cpp)
add_executable(multi_usbcamera_test tests/multi_usbcamera_test.cpp)
add_executable(usbcamera_detect_test tests/usbcamera_detect_test.cpp)
add_executable(usbcamera_test tests/usbcamera_test.cpp)
add_executable(handeye_test tests/handeye_test.cpp)
add_executable(dm_test tests/dm_test.cpp)
add_executable(minimum_vision_system tests/minimum_vision_system.cpp)
add_executable(auto_rwaim_test tests/auto_rwaim_test.cpp)
add_executable(hero src/hero.cpp)
add_executable(rw_aim_test tests/rw_aim_test.cpp)
add_executable(latency_test tests/latency_test.cpp)
add_executable(mt_rw_tracker_test tests/mt_rw_tracker_test.cpp)
add_executable(rw_demo tests/rw_demo.cpp)

target_link_libraries(auto_aim_test ${OpenCV_LIBS} fmt::fmt yaml-cpp tools io auto_aim)
target_link_libraries(auto_buff_test ${OpenCV_LIBS} fmt::fmt yaml-cpp auto_buff tools io)
target_link_libraries(camera_detect_test ${OpenCV_LIBS} fmt::fmt yaml-cpp tools auto_aim io)
target_link_libraries(camera_bayerdetect_test ${OpenCV_LIBS} fmt::fmt yaml-cpp tools auto_aim io)
target_link_libraries(camera_test ${OpenCV_LIBS} fmt::fmt tools io)
target_link_libraries(camera_thread_test ${OpenCV_LIBS} fmt::fmt auto_aim tools io)
target_link_libraries(cboard_test ${OpenCV_LIBS} fmt::fmt tools io)
target_link_libraries(fire_test ${OpenCV_LIBS} fmt::fmt tools io)
target_link_libraries(detector_video_test ${OpenCV_LIBS} fmt::fmt yaml-cpp tools auto_aim)
target_link_libraries(gimbal_response_test ${OpenCV_LIBS} fmt::fmt yaml-cpp tools io)
target_link_libraries(multi_usbcamera_test ${OpenCV_LIBS} fmt::fmt tools io)
target_link_libraries(usbcamera_detect_test ${OpenCV_LIBS} fmt::fmt yaml-cpp tools io auto_aim)
target_link_libraries(usbcamera_test ${OpenCV_LIBS} fmt::fmt yaml-cpp tools io)
target_link_libraries(handeye_test ${OpenCV_LIBS} fmt::fmt yaml-cpp tools io auto_aim)
target_link_libraries(dm_test ${OpenCV_LIBS} fmt::fmt yaml-cpp tools io)
target_link_libraries(minimum_vision_system ${OpenCV_LIBS} fmt::fmt yaml-cpp tools io auto_aim)
target_link_libraries(auto_rwaim_test ${OpenCV_LIBS} fmt::fmt yaml-cpp tools io auto_aim)
target_link_libraries(hero ${OpenCV_LIBS} fmt::fmt yaml-cpp tools io auto_aim)
target_link_libraries(mt_rw_tracker_test ${OpenCV_LIBS} fmt::fmt yaml-cpp tools io auto_aim)
target_link_libraries(rw_aim_test ${OpenCV_LIBS} fmt::fmt yaml-cpp tools io auto_aim)
target_link_libraries(latency_test ${OpenCV_LIBS} fmt::fmt yaml-cpp tools io auto_aim)
add_executable(gimbal_test tests/gimbal_test.cpp)
target_link_libraries(gimbal_test ${OpenCV_LIBS} fmt::fmt yaml-cpp auto_aim tools io)

target_link_libraries(hero ${OpenCV_LIBS} fmt::fmt yaml-cpp tools io auto_aim)
target_link_libraries(mt_rw_tracker_test ${OpenCV_LIBS} fmt::fmt yaml-cpp tools io auto_aim)
target_link_libraries(rw_demo ${OpenCV_LIBS} fmt::fmt yaml-cpp tools io auto_aim)
target_link_libraries(rw_aim_test ${OpenCV_LIBS} fmt::fmt yaml-cpp tools io auto_aim)
target_link_libraries(latency_test ${OpenCV_LIBS} fmt::fmt yaml-cpp tools io auto_aim)

add_executable(planner_test tests/planner_test.cpp)
target_link_libraries(planner_test ${OpenCV_LIBS} fmt::fmt yaml-cpp auto_aim tools io)

add_executable(planner_test_offline tests/planner_test_offline.cpp)
target_link_libraries(planner_test_offline ${OpenCV_LIBS} fmt::fmt yaml-cpp auto_aim tools io)

add_executable(planner_doubleyaw_test_offline tests/planner_doubleyaw_test_offline.cpp)
target_link_libraries(planner_doubleyaw_test_offline ${OpenCV_LIBS} fmt::fmt yaml-cpp auto_aim tools io)

# 检查 ROS 环境
find_package(ament_cmake QUIET)
find_package(rclcpp QUIET)
find_package(std_msgs QUIET)
find_package(rosidl_typesupport_cpp QUIET)
find_package(geometry_msgs QUIET)
# find_package(sp_msgs QUIET)
# 只有在找到 ROS 环境时才会添加 sentry 和 publish_test
if(ament_cmake_FOUND AND rclcpp_FOUND AND std_msgs_FOUND AND rosidl_typesupport_cpp_FOUND AND geometry_msgs_FOUND) # AND sp_msgs_FOUND
    # 编译 sentry 和 publish_test
    # add_executable(sentry src/sentry.cpp)
    # add_executable(sentry_bp src/sentry_bp.cpp)
    # add_executable(sentry_debug src/sentry_debug.cpp)
    # add_executable(sentry_multithread src/sentry_multithread.cpp)
    add_executable(publish_test tests/publish_test.cpp)
    # add_executable(subscribe_test tests/subscribe_test.cpp)
    # add_executable(topic_loop_test tests/topic_loop_test.cpp)

    # target_link_libraries(sentry ${OpenCV_LIBS} fmt::fmt yaml-cpp auto_aim omniperception tools io)
    # target_link_libraries(sentry_bp ${OpenCV_LIBS} fmt::fmt yaml-cpp auto_aim omniperception tools io)
    # target_link_libraries(sentry_debug ${OpenCV_LIBS} fmt::fmt yaml-cpp auto_aim omniperception tools io)
    # target_link_libraries(sentry_multithread ${OpenCV_LIBS} fmt::fmt yaml-cpp auto_aim omniperception tools io)
    target_link_libraries(publish_test ${OpenCV_LIBS} fmt::fmt tools io auto_aim)
    # target_link_libraries(subscribe_test ${OpenCV_LIBS} fmt::fmt tools io)
    # target_link_libraries(topic_loop_test ${OpenCV_LIBS} fmt::fmt tools io)
else()
    message(STATUS "ROS2 environment not found, skipping ROS2-related code.")
endif()

# 将 compile_commands.json 复制到 build 根目录
if(CMAKE_EXPORT_COMPILE_COMMANDS)
    get_filename_component(BUILD_ROOT_DIR "${CMAKE_BINARY_DIR}" DIRECTORY)
    
    # 检查源文件是否存在
    if(EXISTS "${CMAKE_BINARY_DIR}/compile_commands.json")
        file(COPY "${CMAKE_BINARY_DIR}/compile_commands.json" 
             DESTINATION "${BUILD_ROOT_DIR}" 
             FILES_MATCHING PATTERN "compile_commands.json")
    endif()
endif()