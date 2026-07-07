cmake_minimum_required(VERSION 3.8)
project(linetracing_cpp)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)

add_executable(main_node src/main1.cpp)

target_include_directories(main_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  ${EIGEN3_INCLUDE_DIRS}
)

target_link_libraries(main_node
  ${OpenCV_LIBRARIES}
)

ament_target_dependencies(main_node
  rclcpp sensor_msgs geometry_msgs cv_bridge
)

install(TARGETS main_node
  DESTINATION lib/${PROJECT_NAME})

ament_package()
