cmake_minimum_required(VERSION 3.5)
project(ros2bag_to_pcd)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io)
find_package(pcl_conversions REQUIRED)
find_package(rosbag2_cpp REQUIRED)
find_package(Eigen3 REQUIRED)

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

# create_map
add_executable(create_map
  src/offline/create_map.cpp
)

target_include_directories(create_map 
  PUBLIC ${PROJECT_SOURCE_DIR}/include
  ${EIGEN3_INCLUDE_DIR}
)

target_link_libraries(create_map 
  ${PCL_LIBRARIES}
)

ament_target_dependencies(create_map
  rclcpp
  sensor_msgs
  geometry_msgs
  pcl_conversions
  rosbag2_cpp
)

install(TARGETS create_map DESTINATION lib/${PROJECT_NAME})


ament_package()