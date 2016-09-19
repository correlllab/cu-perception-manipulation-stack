# pick_and_place

roslaunch openi2_launch openi2_launch
rosrun image_view image_view image:=/camera/rgb/image_raw
roslaunch teleop_interface teleop_calibration.launch
roslaunch teleop_interface teleop_interface.launch

rosrun finger_sensor sensor.py
for visual: rosrun finger_sensor sensor_visual.py

rosrun finger_sensor stacking_blocks.py

rosrun keyboard keyboard

calibration:
in keyboard window hit c
