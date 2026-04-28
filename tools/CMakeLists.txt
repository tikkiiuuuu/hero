
find_package(Ceres REQUIRED)
include_directories(${CERES_INCLUDE_DIRS})


add_library(tools OBJECT
    exiter.cpp
    extended_kalman_filter.cpp
    ransac_sine_fitter.cpp
    img_tools.cpp
    math_tools.cpp
    plotter.cpp
    trajectory.cpp 
    recorder.cpp
    logger.cpp
    pid.cpp
    crc.cpp
)


target_link_libraries(tools
    PRIVATE
    ${CERES_LIBRARIES}
    
)
