# UWBFYP

## Info
This repository is a catkin workspace containing two ROS packages, 'leader' and 'follower', corresponding to the leader and follower quadcopter, each with their own nodes. These can be found under the src directory. The build and devel directories will be created by 'catkin\_make'. Legacy code and scripts have been moved to the python\_scripts and cpp\_scripts directories.

## Installation
Requires ROS-noetic, version 1.15.9. After git cloning, run 'catkin\_make' in this directory to build. Then run 'source devel/setup.bash' to ensure the packages are properly found (you will have to run this for each separate terminal you want to run ROS in, consider adding to your .bashrc).

## Running
Run the provided launch file for the follower package, 'roslaunch follower follower.launch' to launch all the nodes at once.

Additionally, run 'rostopic list' and 'rostopic echo \<topic\>' to monitor what is being published.
