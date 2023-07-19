#include <iostream>
#include <memory>
#include <string>
#include <boost/filesystem.hpp>
#include <util.hpp>

#include <rosbag2_cpp/readers/sequential_reader.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/serialization.hpp>

#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/transforms.h>

#include <Eigen/Dense>
#include <Eigen/Geometry>

Eigen::Matrix4f createTransformationMatrix(const geometry_msgs::msg::PoseStamped& pose_msg) {
  // Extract the translation and rotation from the PoseStamped message
  Eigen::Vector3f trans(pose_msg.pose.position.x, pose_msg.pose.position.y, pose_msg.pose.position.z);

  Eigen::Quaternionf quat(pose_msg.pose.orientation.w, pose_msg.pose.orientation.x, pose_msg.pose.orientation.y, pose_msg.pose.orientation.z);

  // Create transformation matrix
  Eigen::Matrix4f transformation_matrix = Eigen::Matrix4f::Identity();

  // Set the translation part
  transformation_matrix.block<3, 1>(0, 3) = trans;

  // Set the rotation part
  transformation_matrix.block<3, 3>(0, 0) = quat.toRotationMatrix();

  return transformation_matrix;
}

int main(int argc, char* argv[]) {
  std::string bag_file = argv[1];
  std::string lidar_topic_name = argv[2];
  std::string trj_topic_name = argv[3];
  std::string save_dir = argv[4];

  // create save file
  std::string date = create_date();
  std::string save_file_name_ = save_dir + "/" + date;

  // Create SequentialReader
  auto reader = std::make_shared<rosbag2_cpp::readers::SequentialReader>();

  // Set up storage options and reader options.
  rosbag2_storage::StorageOptions storage_options{bag_file, ""};
  rosbag2_cpp::ConverterOptions converter_options;
  converter_options.output_serialization_format = "cdr";

  std::vector<double> pose_time_vec;  // TODO num
  pose_time_vec.reserve(10000);
  std::vector<Eigen::Matrix4f> trans_matrix_vec;
  trans_matrix_vec.reserve(10000);

  std::vector<sensor_msgs::msg::PointCloud2> point_cloud_vec;
  point_cloud_vec.reserve(10000);

  pcl::PointCloud<pcl::PointXYZ>::Ptr map(new pcl::PointCloud<pcl::PointXYZ>());
  try {
    // Open bagfile
    reader->open(storage_options, converter_options);

    // Read messages from bagfile
    while (reader->has_next()) {
      auto bag_message = reader->read_next();

      if (bag_message->topic_name == trj_topic_name) {
        geometry_msgs::msg::PoseStamped msg;
        rclcpp::Serialization<geometry_msgs::msg::PoseStamped> serialization;
        rclcpp::SerializedMessage extracted_serialized_msg(*bag_message->serialized_data);
        serialization.deserialize_message(&extracted_serialized_msg, &msg);

        pose_time_vec.emplace_back(msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9);
        trans_matrix_vec.emplace_back(createTransformationMatrix(msg));
      }

      if (bag_message->topic_name == lidar_topic_name) {
        sensor_msgs::msg::PointCloud2 msg;
        rclcpp::Serialization<sensor_msgs::msg::PointCloud2> serialization;
        rclcpp::SerializedMessage extracted_serialized_msg(*bag_message->serialized_data);
        serialization.deserialize_message(&extracted_serialized_msg, &msg);
        point_cloud_vec.emplace_back(msg);
      }
    }
  } catch (std::exception& e) {
    std::cerr << "Error: " << e.what() << std::endl;
    return 1;
  }

  for (const auto& msg : point_cloud_vec) {
    double sensor_time = msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9;
    pcl::PointCloud<pcl::PointXYZ>::Ptr add_points(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::fromROSMsg(msg, *add_points);

    std::vector<double>::iterator min_error_iter =
      std::min_element(pose_time_vec.begin(), pose_time_vec.end(), [sensor_time](double a, double b) { return std::abs(a - sensor_time) < std::abs(b - sensor_time); });
    int index = std::distance(pose_time_vec.begin(), min_error_iter);
    Eigen::Matrix4f trans_matrix = trans_matrix_vec[index];
    pcl::transformPointCloud(*add_points, *add_points, trans_matrix);
    *map += *add_points;
  }

  pcl::io::savePCDFileBinary(save_file_name_ + "_map.pcd", *map);
  return 0;
}