# UWBFYP

##Info
This repository is a catkin workspace containing two ROS packages, 'leader' and 'follower', corresponding to the leader and follower quadcopter, each with their own nodes. These can be found under the src directory. The build and devel directories will be created by 'catkin\_make'. Legacy code and scripts have been moved to the python\_scripts and cpp\_scripts directories.

##Installation
Requires ROS-noetic, version 1.15.9. After git cloning, run 'catkin\_make' in this directory to build. Then run 'source ./devel/setup.bash' to ensure the packages are properly found (you will have to run this for each separate terminal you want to run ROS in, consider adding to .bashrc).

##Running
Run 'roscore' on one terminal to create the core, run 'rosrun follower uwb\_node\_reader' on another terminal to create the corresponding node, run 'rostopic echo \uwb\_node\_reading' on another terminal to monitor the published messages.
