## Finger Sensors

### contains files related to sensors.

## quick Demo

```
roslaunch finger_sensor jaco_fingers.launch
```

This launches the node that reads the sensor values over serial port, processes the signals and generates boolean topic for object and touch detection.

## Note

You might have to change few values (i.e. number of fingers and number of sensors per finger) in the sensor.py and signals.py scripts and play with the threshold/sensitivity values depending upon your application. The above command launches nodes for sensors on two of Jaco finger with one sensor on each finger (last updated 08/27/2017).   
