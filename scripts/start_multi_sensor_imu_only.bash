
camera_fps=60
camera_width=640
camera_height=480
l_depth_camera_no=315122271572
r_depth_camera_no=315122272269

l_serial_port=/dev/ttyUSB2
r_serial_port=/dev/ttyUSB3
sudo chmod a+rw /dev/ttyUSB*
l_fisheye_port=18
r_fisheye_port=6
sudo chmod a+rw /dev/video*

source /opt/ros/noetic/setup.bash && cd ~/pika_ros/install/share/sensor_tools/scripts && chmod 777 usb_camera.py
if [ -n "$1" ]; then
    source ~/pika_ros/install/setup.bash && roslaunch sensor_tools open_multi_sensor_imu_only.launch l_depth_camera_no:=$l_depth_camera_no r_depth_camera_no:=$r_depth_camera_no l_serial_port:=$l_serial_port r_serial_port:=$r_serial_port l_fisheye_port:=$l_fisheye_port r_fisheye_port:=$r_fisheye_port camera_fps:=$camera_fps camera_width:=$camera_width camera_height:=$camera_height name:=$1 name_index:=$1_
else
    source ~/pika_ros/install/setup.bash && roslaunch sensor_tools open_multi_sensor_imu_only.launch l_depth_camera_no:=$l_depth_camera_no r_depth_camera_no:=$r_depth_camera_no l_serial_port:=$l_serial_port r_serial_port:=$r_serial_port l_fisheye_port:=$l_fisheye_port r_fisheye_port:=$r_fisheye_port camera_fps:=$camera_fps camera_width:=$camera_width camera_height:=$camera_height
fi
                
