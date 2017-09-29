## wc gazebo with multiple quadrotors ##

# how to execute??
1. open terminal
2. run roscore
3. launch SITL 
* source environment
- cd catkin_ws/src/Firmware
- source environment
source Tools/setup_gazebo.bash $(pwd) $(pwd)/build_posix_sitl_default
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$(pwd)
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$(pwd)/Tools/sitl_gazebo
* run launch file
roslaunch px4 wc_multi_uav_mavros_sitl.launch

4. run wc_gazebo control node (there are 2 options)
* option 1 (run in pycharm)
- run pycharm in terminal
- open project & build & run

* option 2 (use rosrun)
- go to scripts dir
cd catkin_ws/src/wc_multi_quad/scripts
chmod +x wc_ctrl_quad.py
- run node
rosrun wc_multi_quad wc_ctrl_quad.py





# github upload
1. git add *
2. git commit -m "comments"
3. git push origin master

