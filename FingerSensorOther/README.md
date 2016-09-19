# FingerSensor
This is a little description

Data from the calibration experiments are in FingerSensor/MATLAB/force_distance_exprmnt/. exp#4 has the values of force and distance at different mixing ratios for a white target; distance_axis[dist(cm)=[0.5:0.5:3 4:1:10], current(mA)=20:20:200], force_axis[frce(g)={0,5,10,20,50,100,200,500}, current(mA)=20:20:200]. exp#5 has sensor values at different thickness of PDMS film for five different color targets; axis[-frce(N)|dist(cm)=-6:1:5, current(mA)=40:40:200].

Data files for the graspEventDetection experiment are in FingerSensor/ros_finger_sensor/scripts/trialGraspEventDetection_dataFiles folder. (CHECK path for saving!.)

controller_1.py is the main pcontroller, mainly used in the grasp event detection experiment. controller.py and controller_2.py are variants of it using fingertip sensors and used in graspsuccess event experiment. (I wont bother checking them.)
