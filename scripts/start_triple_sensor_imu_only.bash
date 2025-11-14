SCRIPT_DIR=$(dirname "$(readlink -f "$0")")
camera_fps=30
camera_width=640
camera_height=480

l_depth_camera_no=315122271572
r_depth_camera_no=315122272269
l_fisheye_port=34
r_fisheye_port=26
l_serial_port=/dev/ttyUSB3
r_serial_port=/dev/ttyUSB2

pika_l=WH0
pika_r=WH1
pika_base=WH2

sudo chmod a+rw /dev/ttyUSB*
sudo chmod a+rw /dev/video*

source /opt/ros/noetic/setup.bash && cd $SCRIPT_DIR/../install/share/sensor_tools/scripts && chmod 777 usb_camera.py
if [ -n "$1" ]; then
    source $SCRIPT_DIR/../install/setup.bash && roslaunch sensor_tools open_triple_sensor_imu_only.launch l_depth_camera_no:=$l_depth_camera_no r_depth_camera_no:=$r_depth_camera_no l_serial_port:=$l_serial_port r_serial_port:=$r_serial_port l_fisheye_port:=$l_fisheye_port r_fisheye_port:=$r_fisheye_port camera_fps:=$camera_fps camera_width:=$camera_width camera_height:=$camera_height name:=$1 name_index:=$1_ pika_l:=$pika_l pika_r:=$pika_r pika_base:=$pika_base
else
    source $SCRIPT_DIR/../install/setup.bash && roslaunch sensor_tools open_triple_sensor_imu_only.launch l_depth_camera_no:=$l_depth_camera_no r_depth_camera_no:=$r_depth_camera_no l_serial_port:=$l_serial_port r_serial_port:=$r_serial_port l_fisheye_port:=$l_fisheye_port r_fisheye_port:=$r_fisheye_port camera_fps:=$camera_fps camera_width:=$camera_width camera_height:=$camera_height pika_l:=$pika_l pika_r:=$pika_r pika_base:=$pika_base
fi
                
