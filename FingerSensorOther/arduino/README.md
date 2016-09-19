#Arduino Files

## Sensorstrip

Code to read values from Dana's sensor strip. This is based on SparkFun's code for the Infrared Proximity Sensor, but is updated for the 4010 version of the sensor.

## firgelli

A simple controller for the [Firgelli L12-100-50-12-P](http://www.firgelli.com/products.php?id=41) linear actuator using the [Adafruit Motor Shield v1.2](https://www.adafruit.com/products/81). The example pushes the linear actuator to its maximum and minimum limits then to the middle of the limits.

The limit values were found experimentally and are hardcoded into the example file. The limits of the analog read are 0 - 1023.

**NOTE:** The Motor Shield also comes in a v2.0, which we also have in the lab. v2.0 does not work with this code.

## Takkstrip

This implements the demo provided on the [TakkTile](http://www.takktile.com) webpage.

Running Ubuntu you may need to add `java.opts` in your home directory that has a list of Serial Ports. See [this example](http://www.mathworks.com/matlabcentral/answers/25323-using-serial-to-connect-to-an-arduino). `pwd` at the Matlab prompt to see your Matlab home directory.

## Smarthand

Nikolaus' code for reading data from SparkFun [Infrared Proximity sensors](https://www.sparkfun.com/products/10901).

## vn4000_multi

Nikolaus' code for reading 4 of Dana's sensor strips.


