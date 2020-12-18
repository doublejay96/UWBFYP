# UWBFYP

## Info
Code for ultra-wideband localisation for co-ordinated flying of quadcopters, for final-year project. This repository is a catkin workspace containing two ROS packages, 'leader' and 'follower', corresponding to the leader and follower quadcopter, each with their own nodes. These can be found under the src directory. The build and devel directories will be created by 'catkin\_make'. Legacy code and scripts have been moved to the python\_scripts and cpp\_scripts directories.

## Installation
Requires ROS-noetic, version 1.15.9, MAVROS version 1.5.0, gazebo\_ROS version 2.9.1, which are all available as deb packages. After git cloning this repo, run 'catkin\_make' in the top-level directory to build. Then run 'source devel/setup.bash' to ensure the packages are properly found (you will have to run this for each separate terminal you want to run ROS in, consider adding to your .bashrc).

## Running
A number of launch files are provided to launch various combinations of ROS nodes at once for ease of use. These can be run with 'roslaunch [\<leader\follower>] \[\<launch file name>]'.

Provided launch files are:

**Leader**
* leader\_actual: Run this for actual flight of the leader quadcopter, once connected to PX4.
* sim\_leader\_standalone: Run this to start a Gazebo simulation of the leader quadcopter alone. Useful for testing vehicle performance and offboard commands.
* sim\_leader: Do not run this directly, it is called by other launch files to launch the necessary nodes for the simulated leader.
* experimental: Used only for barometer logging.

**Follower**
* follower\_actual: Run this for actual flight of the follower quadcopter, once connected to PX4.
* one\_leader\_one\_follower\_sim\_sitl: Run this to start a Gazebo simulation of one leader and one follower quadcopter following it.
* sim\_follower: Do not run this directly, it is called by other launch files to launch the necessary nodes for the simulated follower.

## Miscellaneous
Waypoints for the leader in simulated flight can be altered under data/waypoints.txt. Format is x,y,z,duration to spend moving to and remaining at that waypoint.

Run 'rostopic echo [\<topic>]' to monitor the various topics being published to. Prepend '/leader' or '/follower1' for multi-vehicle simulations. Some useful ones are:
* /mavros/state
* /mavros/local\_position/pose
* /mavros/setpoint\_raw/local
* /uwb\_node\_reading
* /flight\_status
* /gazebo/model\_states : In simulation only

Change ROSCONSOLE output to show the speaking node's name for easier debugging: 'export ROSCONSOLE\_FORMAT='\[${severity}] \[${time}] \[${node}]: ${message}''
