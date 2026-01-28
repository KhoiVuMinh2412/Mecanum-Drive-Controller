# Mecanum Drive Controller
## Description
Mecanum Drive Controller within a ros2 workspace with single package.
## Usage
### To build:
Run 
```
colcon build
``` 
at the outermost directory (mecanum-drive-controller).

Then source the install using 
```
source install/setup.bash
``` 
in terminal.
### To launch:
Use:
```
ros2 launch mecanum_driver robot.launch.py 
``` 
### To run separately:
Run **mecanum_node**:
```
ros2 run mecanum_driver mecanum_node
```
Run **keyboard node**:
```
ros2 run mecanum_driver keyboard_node
```
## To-do
- Add the keyboard node to the launch file to open up new terminal upon launch.
- Double check the kinematics of the ```mecanum.cpp```.