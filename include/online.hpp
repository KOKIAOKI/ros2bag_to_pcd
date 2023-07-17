#pragma once

#include <iostream>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

class Online : public rclcpp::Node {
  struct Params {
    // path
    std::string output_pcd_folder;

    // rosbag information
    std::string lidar_topic_name;
  } params_;

public:
  Online(const rclcpp::NodeOptions& node_options = rclcpp::NodeOptions());
  ~Online();

private:
  // callback
  void points_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr poitns_sub_;

  std::string save_folder_name_;
};
