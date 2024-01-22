# Source ROS setup file
source /opt/ros/${ROS_DISTRO}/setup.bash

# Install all dependencies
rosdep install -y --from-path ~/ModMath2324/src

# Create/Update workspace
cd ~/ModMath2324
rm -rf -v ./build ./devel ./src/CMakeLists.txt
catkin_make

# Source the package setup file
source ~/ModMath2324/devel/setup.bash
echo "source ~/ModMath2324/devel/setup.bash" >> ~/.bashrc