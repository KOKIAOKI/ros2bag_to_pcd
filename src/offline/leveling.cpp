#include <iostream>
#include <memory>
#include <string>
#include <boost/filesystem.hpp>
#include <util.hpp>
#include <rosbag2_cpp/readers/sequential_reader.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/imu.hpp>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/serialization.hpp>

#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/transforms.h>

#include <Eigen/Dense>
#include <Eigen/Geometry>

Eigen::Matrix4f createTransformationMatrix(const sensor_msgs::msg::Imu& imu_msg) {
  Eigen::Vector3f acc(imu_msg.linear_acceleration.x, imu_msg.linear_acceleration.y, imu_msg.linear_acceleration.z);
  Eigen::Vector3f th;
  th.x() = std::atan2(acc.y(), acc.z());
  th.y() = std::atan2(-acc.x(), std::sqrt(acc.y() * acc.y() + acc.z() * acc.z()));
  th.z() = 0.0f;

  Eigen::Matrix3f rot;
  rot = Eigen::AngleAxisf(th.x(), Eigen::Vector3f::UnitX()) * Eigen::AngleAxisf(th.y(), Eigen::Vector3f::UnitY()) * Eigen::AngleAxisf(th.z(), Eigen::Vector3f::UnitZ());
  Eigen::Matrix4f mat = Eigen::Matrix4f::Identity();
  mat.block<3, 3>(0, 0) = rot;

  return mat;
}

int main(int argc, char* argv[]) {
  std::string bag_file = argv[1];
  std::string lidar_topic_name = argv[2];
  std::string imu_topic_name = argv[3];
  std::string save_dir = argv[4];

  // create save file
  std::string date = create_date();
  std::string save_folder_name = save_dir + "/" + date;
  boost::filesystem::create_directory(save_folder_name);

  // Create SequentialReader
  auto reader = std::make_shared<rosbag2_cpp::readers::SequentialReader>();

  // Set up storage options and reader options.
  rosbag2_storage::StorageOptions storage_options{bag_file, ""};
  rosbag2_cpp::ConverterOptions converter_options;
  converter_options.output_serialization_format = "cdr";

  std::vector<double> pose_time_vec;
  pose_time_vec.reserve(500);

  std::vector<sensor_msgs::msg::Imu> imu_msgs;
  imu_msgs.reserve(1000);

  std::vector<sensor_msgs::msg::PointCloud2> lidar_msgs;
  lidar_msgs.reserve(500);

  pcl::PointCloud<pcl::PointXYZ>::Ptr map(new pcl::PointCloud<pcl::PointXYZ>());
  try {
    // Open bagfile
    reader->open(storage_options, converter_options);

    // Read messages from bagfile
    while (reader->has_next()) {
      auto bag_message = reader->read_next();

      if (bag_message->topic_name == imu_topic_name) {
        sensor_msgs::msg::Imu msg;
        rclcpp::Serialization<sensor_msgs::msg::Imu> serialization;
        rclcpp::SerializedMessage extracted_serialized_msg(*bag_message->serialized_data);
        serialization.deserialize_message(&extracted_serialized_msg, &msg);

        pose_time_vec.emplace_back(msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9);
        imu_msgs.emplace_back(msg);
      }

      if (bag_message->topic_name == lidar_topic_name) {
        sensor_msgs::msg::PointCloud2 msg;
        rclcpp::Serialization<sensor_msgs::msg::PointCloud2> serialization;
        rclcpp::SerializedMessage extracted_serialized_msg(*bag_message->serialized_data);
        serialization.deserialize_message(&extracted_serialized_msg, &msg);
        lidar_msgs.emplace_back(msg);
      }
    }
  } catch (std::exception& e) {
    std::cerr << "Error: " << e.what() << std::endl;
    return 1;
  }

  for (const auto& msg : lidar_msgs) {
    double sensor_time = msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9;
    pcl::PointCloud<pcl::PointXYZ>::Ptr raw_points(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::fromROSMsg(msg, *raw_points);

    std::vector<double>::iterator min_error_iter =
      std::min_element(pose_time_vec.begin(), pose_time_vec.end(), [sensor_time](double a, double b) { return std::abs(a - sensor_time) < std::abs(b - sensor_time); });
    int index = std::distance(pose_time_vec.begin(), min_error_iter);
    sensor_msgs::msg::Imu imu_msg = imu_msgs[index];

    pcl::PointCloud<pcl::PointXYZ>::Ptr save_points(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::transformPointCloud(*raw_points, *save_points, createTransformationMatrix(imu_msg));
    pcl::io::savePCDFileBinary(save_folder_name + "/" + std::to_string(sensor_time) + ".pcd", *save_points);
  }

  return 0;
}