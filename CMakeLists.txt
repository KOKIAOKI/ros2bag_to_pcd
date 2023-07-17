cmake_minimum_required(VERSION 3.5)
project(ros2bag_to_pcd)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io)
find_package(pcl_conversions REQUIRED)

add_executable(ros2bag_to_pcd 
  src/online.cpp
  src/online_node.cpp
)

target_include_directories(ros2bag_to_pcd 
  PUBLIC ${PROJECT_SOURCE_DIR}/include
)

target_link_libraries(ros2bag_to_pcd 
  ${PCL_LIBRARIES}
)

ament_target_dependencies(ros2bag_to_pcd
  rclcpp 
  sensor_msgs
  pcl_conversions
)

install(TARGETS ros2bag_to_pcd DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

ament_package()
