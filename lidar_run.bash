gnome-terminal -- bash -c "cd /home/zw/scout_ws && roslaunch hesai_lidar hesai_lidar.launch lidar_type:="PandarXT-32" frame_id:="laser"; exec bash" &
sleep 3

gnome-terminal -- bash -c "cd /home/zw/catkin_ws && roslaunch aloam_velodyne aloam_hesai_xt32.launch; exec bash" 
