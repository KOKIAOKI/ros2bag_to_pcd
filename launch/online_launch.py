import launch
import launch_ros.actions
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
  output_pcd_folder = LaunchConfiguration("output_pcd_folder", default ="/home/koki/0_data/output_pcd")
  lidar_topic_name = LaunchConfiguration("lidar_topic_name", default = "/sensing/lidar/top/pointcloud_raw_ex")

  ros2bag_to_pcd = launch_ros.actions.Node(package='ros2bag_to_pcd', executable='ros2bag_to_pcd', output='screen', parameters=[{"lidar_topic_name":lidar_topic_name},{"output_pcd_folder":output_pcd_folder}])
  return launch.LaunchDescription([ros2bag_to_pcd])
