# After compiling the classification node (requires ready packages)
source /opt/ros/humble/setup.bash
echo "Info: You need to build the gesture_recognition package first (as well as the UGV interfaces)"
source ./install/local_setup.bash
ros2 run gesture_recognition producer