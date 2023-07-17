#include <iostream>
#include <memory>
#include <string>
#include <boost/filesystem.hpp>
#include <util.hpp>

#include <rosbag2_cpp/readers/sequential_reader.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/serialization.hpp>

#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>

int main(int argc, char* argv[]) {
  std::string bag_file = argv[1];
  std::string topic_name = argv[2];
  std::string save_dir = argv[3];

  // create save file
  std::string date = create_date();
  std::string save_folder_name_ = save_dir + "/" + date;
  boost::filesystem::create_directory(save_folder_name_);

  // Create SequentialReader
  auto reader = std::make_shared<rosbag2_cpp::readers::SequentialReader>();

  // Set up storage options and reader options.
  rosbag2_storage::StorageOptions storage_options{bag_file, ""};
  rosbag2_cpp::ConverterOptions converter_options;
  converter_options.output_serialization_format = "cdr";

  try {
    // Open bagfile
    reader->open(storage_options, converter_options);

    // Read messages from bagfile
    while (reader->has_next()) {
      auto bag_message = reader->read_next();

      if (bag_message->topic_name == topic_name) {
        sensor_msgs::msg::PointCloud2 msg;
        rclcpp::Serialization<sensor_msgs::msg::PointCloud2> serialization;
        rclcpp::SerializedMessage extracted_serialized_msg(*bag_message->serialized_data);
        serialization.deserialize_message(&extracted_serialized_msg, &msg);

        std::string time_stamp = std::to_string(msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9);
        pcl::PointCloud<pcl::PointXYZ>::Ptr save_points(new pcl::PointCloud<pcl::PointXYZ>());
        pcl::fromROSMsg(msg, *save_points);

        pcl::io::savePCDFileBinary(save_folder_name_ + "/" + time_stamp + ".pcd", *save_points);
        std::cout << "saved: " << save_folder_name_ + "/" + time_stamp + ".pcd" << std::endl;
      }
    }
  } catch (std::exception& e) {
    std::cerr << "Error: " << e.what() << std::endl;
    return 1;
  }

  return 0;
}