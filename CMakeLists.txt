cmake_minimum_required(VERSION 3.15)
project(project)

set(CMAKE_CXX_STANDARD 17)

find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Ceres REQUIRED)

include_directories(
    include
    ${OpenCV_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIR}
    ${CERES_INCLUDE_DIRS}
)

add_executable(camera_calibration_test test/camera_calibration_test.cpp)
target_link_libraries(camera_calibration_test ${OpenCV_LIBS} ${Eigen3_LIBRARIES} ${CERES_LIBRARIES})