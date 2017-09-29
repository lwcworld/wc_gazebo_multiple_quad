## wc gazebo with multiple quadrotors ##

# how to execute??
### open terminal

### run roscore
<pre><code>roscore</code></pre>

### launch SITL 
1. source environment
<pre><code>cd catkin_ws/src/Firmware</code></pre>
<pre><code>source Tools/setup_gazebo.bash $(pwd) $(pwd)/build_posix_sitl_default
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$(pwd)
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$(pwd)/Tools/sitl_gazebo
</code></pre>
2. run launch file
<pre><code> roslaunch px4 wc_multi_uav_mavros_sitl.launch </code></pre>

### run wc_gazebo control node (there are 2 options)
##### option 1 (run in pycharm)
1. run pycharm in terminal
2. open project & build & run

##### option 2 (use rosrun)
1. go to scripts dir
<pre><code>cd catkin_ws/src/wc_multi_quad/scripts</code></pre>
<pre><code>chmod +x wc_ctrl_quad.py</code></pre>
2. run node
<pre><code>rosrun wc_multi_quad wc_ctrl_quad.py</code></pre>


# github upload
1. <pre><code>git add *</code></pre>
2. <pre><code>git commit -m "comments"</code></pre>
3. <pre><code>git push origin master</code></pre>

