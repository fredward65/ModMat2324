# Create workspace
source /opt/ros/${ROS_DISTRO}/setup.bash

# Install all dependencies
rosdep install -y --from-path ~/ModMath2324/src

# Create/Update workspace
cd ~/ModMath2324 && catkin_make

# Source the package setup file
echo "source ~/ModMath2324/devel/setup.bash" >> ~/.bashrc