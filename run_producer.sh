ROS_DISTRO="humble"
VENV_NAME="gesture_commander_venv"

# After compiling the classification node (requires ready packages)

source /opt/ros/$ROS_DISTRO/setup.bash

echo "Info: You need to build the gesture_recognition package first (as well as the UGV interfaces)"
source ./install/local_setup.bash
ros2 run gesture_recognition producer