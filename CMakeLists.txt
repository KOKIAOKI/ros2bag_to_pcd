cmake_minimum_required(VERSION 3.5)
project(ros2bag_to_pcd)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io)
find_package(pcl_conversions REQUIRED)
find_package(rosbag2_cpp REQUIRED)

# online
add_executable(online 
  src/online/online.cpp
  src/online/online_node.cpp
)

target_include_directories(online 
  PUBLIC ${PROJECT_SOURCE_DIR}/include
)

target_link_libraries(online 
  ${PCL_LIBRARIES}
)

ament_target_dependencies(online
  rclcpp
  sensor_msgs
  pcl_conversions
)

install(TARGETS online DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

# offline
add_executable(offline
  src/offline/offline.cpp
)

target_include_directories(offline 
  PUBLIC ${PROJECT_SOURCE_DIR}/include
)

target_link_libraries(offline 
  ${PCL_LIBRARIES}
)

ament_target_dependencies(offline
  rclcpp
  sensor_msgs
  pcl_conversions
  rosbag2_cpp
)

install(TARGETS offline DESTINATION lib/${PROJECT_NAME})

ament_package()