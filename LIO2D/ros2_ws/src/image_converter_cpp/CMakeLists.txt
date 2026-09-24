cmake_minimum_required(VERSION 3.8)
project(image_converter_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)

add_executable(yuyv_to_bgr_node src/yuyv_to_bgr_node.cpp)

ament_target_dependencies(yuyv_to_bgr_node
  rclcpp
  sensor_msgs
  cv_bridge
)

target_include_directories(yuyv_to_bgr_node PUBLIC
  ${OpenCV_INCLUDE_DIRS}
)

target_link_libraries(yuyv_to_bgr_node
  ${OpenCV_LIBS}
)

install(TARGETS
  yuyv_to_bgr_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
