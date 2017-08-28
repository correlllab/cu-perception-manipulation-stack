## camera_calibration_tool

### understand and run tf_keyboard_cal package by Dave Coleman.

+ generate a frame transform from root to camera_link
+ hand align the point cloud to the arm in real world by moving the arm in line of sight of the camera
+ note down the transform between the root and the camera_link

### copy this new transform in the transform_camera.py in cu-perception-manipulation-stack/camera_calibration_tool/scripts/

then run,

'''
rosrun camera_calibration_tool  transform_camera.py
'''
