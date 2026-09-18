source /opt/ros/humble/setup.bash

VENV_NAME="gesture_commander_venv"
source ./$VENV_NAME/bin/activate

echo "Removing build trash ..."
rm -rf build install log

echo "Setup UPC interface ..."
source /home/triffid/upc_ws/install/setup.bash

echo "Building gesture_recognition package ..."
colcon build --packages-select gesture_recognition --cmake-args -DCMAKE_BUILD_TYPE=Release

echo "Installing package ..."
source ./install/local_setup.bash

echo "Running the classification node ..."
ros2 run gesture_recognition gesture_classifier