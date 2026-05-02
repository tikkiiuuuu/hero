cmake_minimum_required(VERSION 3.16)

set(OpenVINO_DIR "/opt/intel/openvino_2024.6.0/runtime/cmake")

find_package(OpenVINO REQUIRED COMPONENTS Runtime)

add_subdirectory(planner/tinympc)

set(AUTO_AIM_SOURCES
    armor.cpp
    classifier.cpp
    detector.cpp
    solver.cpp
    aimer.cpp
    target.cpp
    tracker.cpp
    rw_tracker.cpp
    voter.cpp
    shooter.cpp
    yolo.cpp
    yolos/yolov5.cpp
    yolos/yolov8.cpp
    yolos/yolo11.cpp
    multithread/commandgener.cpp
    multithread/mt_detector.cpp
    planner/planner.cpp
)

add_library(auto_aim OBJECT ${AUTO_AIM_SOURCES})
add_library(auto_aim_no_rw OBJECT
    ${AUTO_AIM_SOURCES}
)
# Remove rw_tracker.cpp from auto_aim_no_rw
list(REMOVE_ITEM auto_aim_no_rw rw_tracker.cpp)

target_link_libraries(auto_aim io openvino::runtime tinympcstatic ruckig::ruckig)
target_link_libraries(auto_aim_no_rw io openvino::runtime tinympcstatic ruckig::ruckig)