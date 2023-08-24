#include <iostream>
#include <memory>
#include <string>
#include <boost/filesystem.hpp>
#include <util.hpp>
#include <random>
#include <fstream>
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

#define EVA_DISTANE 0.0

std::ofstream ofs_;

void createCsvIndex(std::string output_folder_dir) {
  std::string output_filename = output_folder_dir + "/ground_truth.csv";
  std::cout << "csv file name: " << output_filename.c_str() << std::endl;
  ofs_.open(output_filename.c_str());
  if (!ofs_.is_open()) {
    std::cerr << "can not create csv file" << std::endl;
    exit(1);
  }

  // clang-format off
  ofs_ << "sensor_time" << ","
      << "pose_time" << ","
      << "position_x" << ","
      << "position_y" << ","
      << "position_z" << ","
      << "orientation_x" << ","
      << "orientation_y" << ","
      << "orientation_z" << ","
      << "orientation_w" << ","
      << std::endl;
}  // clang-format on

Eigen::Vector3f QuaternionToEuler(const Eigen::Quaternionf& q) {
  Eigen::Vector3f euler = q.toRotationMatrix().eulerAngles(2, 1, 0);  // z-y-x
  return euler;
}

Eigen::Matrix4f CreateTransformMatrix(const Eigen::Vector3f& euler) {
  Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();

  std::random_device rd;
  std::mt19937 gen(rd());
  std::uniform_real_distribution<float> dis(-0.01, 0.01);

  float roll = euler(2) + dis(gen);
  float pitch = euler(1) + dis(gen);
  // float yaw = euler(0) + dis(gen);

  Eigen::AngleAxisf rollAngle(roll, Eigen::Vector3f::UnitX());
  Eigen::AngleAxisf pitchAngle(pitch, Eigen::Vector3f::UnitY());
  Eigen::AngleAxisf yawAngle(0.0f, Eigen::Vector3f::UnitZ());

  Eigen::Quaternionf q = yawAngle * pitchAngle * rollAngle;
  transform.block<3, 3>(0, 0) = q.matrix();

  return transform;
}

Eigen::Matrix4f createTransformationMatrix(const geometry_msgs::msg::PoseStamped& msg) {
  Eigen::Quaternionf q(msg.pose.orientation.w, msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z);

  Eigen::Vector3f euler = QuaternionToEuler(q);
  Eigen::Matrix4f transform = CreateTransformMatrix(euler);

  return transform;
}

int main(int argc, char* argv[]) {
  std::string bag_file = argv[1];
  std::string lidar_topic_name = argv[2];
  std::string pose_topic_name = argv[3];
  std::string save_dir = argv[4];

  // create save file
  std::string date = create_date();
  std::string save_folder_name = save_dir + "/" + date;
  boost::filesystem::create_directory(save_folder_name);
  createCsvIndex(save_folder_name);

  // Create SequentialReader
  auto reader = std::make_shared<rosbag2_cpp::readers::SequentialReader>();

  // Set up storage options and reader options.
  rosbag2_storage::StorageOptions storage_options{bag_file, ""};
  rosbag2_cpp::ConverterOptions converter_options;
  converter_options.output_serialization_format = "cdr";

  std::vector<double> pose_time_vec;
  pose_time_vec.reserve(500);

  std::vector<geometry_msgs::msg::PoseStamped> pose_msgs;
  pose_msgs.reserve(1000);

  std::vector<sensor_msgs::msg::PointCloud2> lidar_msgs;
  lidar_msgs.reserve(500);

  pcl::PointCloud<pcl::PointXYZ>::Ptr map(new pcl::PointCloud<pcl::PointXYZ>());
  try {
    // Open bagfile
    reader->open(storage_options, converter_options);

    // Read messages from bagfile
    while (reader->has_next()) {
      auto bag_message = reader->read_next();

      if (bag_message->topic_name == pose_topic_name) {
        geometry_msgs::msg::PoseStamped msg;
        rclcpp::Serialization<geometry_msgs::msg::PoseStamped> serialization;
        rclcpp::SerializedMessage extracted_serialized_msg(*bag_message->serialized_data);
        serialization.deserialize_message(&extracted_serialized_msg, &msg);

        pose_time_vec.emplace_back(msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9);
        pose_msgs.emplace_back(msg);
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

  // double distance_from_prior_pose;
  // double prior_x;
  // double prior_y;
  // bool first_count = true;

  for (const auto& msg : lidar_msgs) {
    double sensor_time = msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9;
    pcl::PointCloud<pcl::PointXYZ>::Ptr raw_points(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::fromROSMsg(msg, *raw_points);

    std::vector<double>::iterator min_error_iter =
      std::min_element(pose_time_vec.begin(), pose_time_vec.end(), [sensor_time](double a, double b) { return std::abs(a - sensor_time) < std::abs(b - sensor_time); });
    int index = std::distance(pose_time_vec.begin(), min_error_iter);
    geometry_msgs::msg::PoseStamped pose_msg = pose_msgs[index];

    // if (first_count) {
    //   first_count = false;
    // } else {
    //   double x_dis = pose_msg.pose.position.x - prior_x;
    //   double y_dis = pose_msg.pose.position.y - prior_y;
    //   distance_from_prior_pose += std::sqrt(x_dis * x_dis + y_dis * y_dis);
    // }
    // prior_x = pose_msg.pose.position.x;
    // prior_y = pose_msg.pose.position.y;

    // if (first_count || distance_from_prior_pose > EVA_DISTANE) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr save_points(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::transformPointCloud(*raw_points, *save_points, createTransformationMatrix(pose_msg));
    pcl::io::savePCDFileBinary(save_folder_name + "/" + std::to_string(sensor_time) + ".pcd", *save_points);

    // save data
    // clang-format off
    ofs_ << std::fixed << std::setprecision(10)
          << sensor_time << ","
          << pose_time_vec[index] << ","
          << pose_msg.pose.position.x << "," 
          << pose_msg.pose.position.y << ","
          << pose_msg.pose.position.z <<","
          << pose_msg.pose.orientation.x << ","
          << pose_msg.pose.orientation.y << ","
          << pose_msg.pose.orientation.z << ","
          << pose_msg.pose.orientation.w << ","
          << std::endl;
    // clang-format on
    // distance_from_prior_pose = 0.0;
  }
  // }

  return 0;
}