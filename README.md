# Robot Manage ROS

## Build

```bash
mkdir -p rm_ws/src
cd rm_ws/src
git clone https://github.com/565353780/robot-manage-ros.git
cd ..
catkin config --cmake-args -DCMAKE_CXX_STANDARD=17 -DCMAKE_BUILD_TYPE=Release -DCMAKE_EXPORT_COMPILE_COMMANDS=Yes
catkin build
```

## Run

```bash
roslaunch robot_keyboard_controller RobotKeyboardController.launch
roslaunch robot_position_visualizer RobotPositionVisualizer.launch
roslaunch robot_position_loader RobotPositionLoaderServer.launch
```

## Enjoy it~

