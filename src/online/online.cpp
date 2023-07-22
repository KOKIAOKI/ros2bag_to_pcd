#include <online/online.hpp>
#include <util.hpp>
#include <boost/filesystem.hpp>

#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>

Online::Online(const rclcpp::NodeOptions& node_options) : Node("online", node_options) {
  params_.output_pcd_folder = this->declare_parameter<std::string>("output_pcd_folder");
  params_.lidar_topic_name = this->declare_parameter<std::string>("lidar_topic_name");
  std::cout << "lidar topic name: " << params_.lidar_topic_name << std::endl;

  // create save file
  std::string date = create_date();
  save_folder_name_ = params_.output_pcd_folder + "/" + date;
  boost::filesystem::create_directory(save_folder_name_);

  // sub
  poitns_sub_ =
    this->create_subscription<sensor_msgs::msg::PointCloud2>(params_.lidar_topic_name, rclcpp::SensorDataQoS(), std::bind(&Online::points_callback, this, std::placeholders::_1));
}

Online::~Online() {}

void Online::points_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
  std::string time_stamp = std::to_string(msg->header.stamp.sec + msg->header.stamp.nanosec / 1e9);
  pcl::PointCloud<pcl::PointXYZ>::Ptr save_points(new pcl::PointCloud<pcl::PointXYZ>());
  pcl::fromROSMsg(*msg, *save_points);

  // pcl::transformPointCloud(*save_points, *save_points, mat);

  pcl::io::savePCDFileBinary(save_folder_name_ + "/" + time_stamp + ".pcd", *save_points);
  std::cout << "saved: " << save_folder_name_ + "/" + time_stamp + ".pcd" << std::endl;
}
