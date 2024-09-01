gnome-terminal -- bash -c "cd /home/zw/SDK/oculii_sdk/ros_driver/ros_driver_ws && source devel/setup.sh && roslaunch oculii_sdk_ros_driver oculii.launch; exec bash" &
sleep 3

gnome-terminal -- bash -c "cd /home/zw/catkin_ws && roslaunch radar_graph_slam radar_graph_slam.launch; exec bash" 
