## ros2bag_to_pcd
Convert PointCloud2 in ros2bag to pcd

## Environment
- ubuntu22 ros2 humble

## Install
```
cd ros2_ws/src/
git clone https://github.com/KOKIAOKI/ros2bag_to_pcd.git
cd ../
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
```

## For [3D-BBS](https://github.com/KOKIAOKI/3d_bbs) test data
### Output gravity aligned point cloud
```
source install/setup.bash
ros2 run ros2bag_to_pcd leveling [your bag file path] [lidar topic name] [imu topic name] [save folder path]
```

## Basic Run
### 1. online
Please correct online_launch.py parameter so that the topic can be read.
First terminal
```
source install/setup.bash
ros2 launch ros2bag_to_pcd online_launch.py
```
Second terminal
```
ros2 bag play [your bag file path]
```

### 2. offline
```
source install/setup.bash
ros2 run ros2bag_to_pcd offline [your bag file path] [topic name] [save folder path]
```

or
```
cd ros2_ws
./build/ros2bag_to_pcd/offline [your bag file path] [topic name] [save folder path]
```

### 3. create map
```
source install/setup.bash
ros2 run ros2bag_to_pcd create_map [your bag file path] [topic name] [trajectory topic name] [save folder path]
```

or
```
cd ros2_ws
./build/ros2bag_to_pcd/create_map [your bag file path] [topic name] [trajectory topic name] [save folder path]
```

