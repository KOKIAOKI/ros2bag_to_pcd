## ros2bag_to_pcd

## Environment
- ubuntu22 ros2 humble

## Install
please git clone in ros2_ws/src
```
cd ros2_ws/src/
git clone 
cd ../../
colcon build
```

## Run
### online
Please correct parameter so that the topic can be read.
First terminal
```
source install/setup.bash
ros2 launch ros2bag_to_pcd online_launch.py
```
Second terminal
```
ros2 bag play [your bag file path]
```

### offline
```
source install/setup.bash
ros2 run ros2bag_to_pcd offline [your bag file path] [topic name] [save folder path]
```

or
```
cd ros2_ws
./build/ros2bag_to_pcd/offline [your bag file path] [topic name] [save folder path]
```

### create map
```
source install/setup.bash
ros2 run ros2bag_to_pcd offline [your bag file path] [topic name] [trajectory topic name] [save folder path]
```

or
```
cd ros2_ws
./build/ros2bag_to_pcd/create_map [your bag file path] [topic name] [trajectory topic name] [save folder path]
```