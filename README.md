# world_map
Using rtab to SLAM a low-detail map of the environment and then crop to the surrounding area when picking

steps:

sudo apt-get install ros-kinetic-rtabmap-ros

roslaunch realsense2_camera rs_camera.launch align_depth:=true

roslaunch rtabmap_ros rtabmap.launch \
    rtabmap_args:="--delete_db_on_start" \
    depth_topic:=/camera/aligned_depth_to_color/image_raw \
    rgb_topic:=/camera/color/image_raw \
    camera_info_topic:=/camera/color/camera_info \
    approx_sync:=false
    
    
then run the python script
