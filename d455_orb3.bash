
gnome-terminal -- bash -c "source ~/tk_fusion/orb3_node/Examples/ROS/ORB_SLAM3/build/devel/setup.bash && rosrun ORB_SLAM3 RGBD Vocabulary/ORBvoc.txt param/d455_gpt.yaml; exec bash" &
sleep 8

gnome-terminal -- bash -c "cd ~/tk_fusion/orb3_node && roslaunch realsense2_camera rs_camera.launch color_width:=640 color_height:=480 enable_sync:=true align_depth:=true; exec bash"


gnome-terminal -- bash -c "rosrun hector_trajectory_server hector_trajectory_server __name:=trajectory_server_loam __ns:=orb3_link _target_frame_name:=map _source_frame_name:=orb3_link _trajectory_update_rate:=10.0 _trajectory_publish_rate:=10.0; exec bash"


gnome-terminal -- bash -c "rosrun rviz rviz -d config/orb_slam3_stereo.rviz; exec bash"




# rosrun ORB_SLAM3 Mono_Inertial Vocabulary/ORBvoc.txt param/RealSense_L515.yaml
# roslaunch realsense2_camera rs_camera.launch color_width:=640 color_height:=480 enable_sync:=true align_depth:=true
# roslaunch realsense2_camera rs_camera.launch color_width:=640 color_height:=480 enable_sync:=true align_depth:=true enable_infra:=true enable_infra1:=true enable_infra2:=true infra_rgb:=true
# roslaunch realsense2_camera rs_camera.launch color_width:=640 color_height:=480 enable_sync:=true align_depth:=true enable_infra:=true enable_infra1:=true enable_infra2:=true 
#gnome-terminal -- bash -c "roscore; exec bash" &
#sleep 3
# gnome-terminal -- bash -c "source ~/tk_fusion/orb3_node/Examples/ROS/ORB_SLAM3/build/devel/setup.bash && rosrun ORB_SLAM3 Stereo Vocabulary/ORBvoc.txt param/d455_stereo.yaml; exec bash" &
#gnome-terminal -- bash -c "source ~/tk_fusion/orb3_node/Examples/ROS/ORB_SLAM3/build/devel/setup.bash && rosrun ORB_SLAM3 Stereo Vocabulary/ORBvoc.txt param/RealSense_D455_recorrected.yaml false; exec bash" &

#gnome-terminal -- bash -c "source ~/tk_fusion/orb3_node/Examples/ROS/ORB_SLAM3/build/devel/setup.bash && rosrun ORB_SLAM3 RGBD Vocabulary/ORBvoc.txt param/d455_2.yaml; exec bash" &


# gnome-terminal -- bash -c "cd ~/tk_fusion/orb3_node && roslaunch realsense2_camera rs_camera.launch color_width:=640 color_height:=480 enable_sync:=true align_depth:=true enable_infra:=true enable_infra1:=true enable_infra2:=true emitter_enable:=false; exec bash"


# gnome-terminal -- bash -c "cd /media/mia/T7 && rosbag record /camera/color/image_raw /camera/aligned_depth_to_color/image_raw; exec bash" 
# gnome-terminal -- bash -c "cd /media/mia/T7 && rosbag record /camera/infra1/image_rect_raw /camera/infra2/image_rect_raw; exec bash" 
#gnome-terminal -- bash -c "cd /media/mia/T7 && rosbag record -O d455_good /camera/color/image_raw /camera/aligned_depth_to_color/image_raw; exec bash" #CHOOSE ANOTHER BAG
# gnome-terminal -- bash -c "cd /media/mia/T7 && rosbag play --clock 2024-02-22-16-03-23.bag; exec bash" 
