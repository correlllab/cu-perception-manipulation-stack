/**
 * Author(s) : Rebecca Cox
 * Desc      : creates visuals for rviz to display sensor feedback
 * Date      : 2017 - Feb - 02
 */

#ifndef FINGER_SENSOR_PERCEPTION_H
#define FINGER_SENSOR_PERCEPTION_H

#include <ros/ros.h>
#include <rviz_visual_tools/rviz_visual_tools.h>
#include <std_msgs/Int32MultiArray.h>

namespace object_detection
{
const int num_sensors = 16; //the tips should be at index 7 and 15
const bool baxter_gripper = true;

const int threshold = 7000; //nothing
const int three5_cent = 7300; //3.5 centimeter away surface
const int three_cent = 7460; //3 centimeter away surface
const int two5_cent = 7850; //2.5 centimeter away surface
const int two_cent = 8150; //2 centimeter away surface
const int one5_cent = 9700; //1.5 centimeter away surface
const int one_cent = 12600; //1 centimeter away surface
const int half_cent = 17600; //0.5 centimeter away surface

class FingerSensorPerception
{
private:
  double normalize_factors[16];
  bool normalize;


  ros::NodeHandle nh_;
  ros::Subscriber sensor_values;
  ros::Subscriber right_finger_tip;
  rviz_visual_tools::RvizVisualToolsPtr left_tip_;
  rviz_visual_tools::RvizVisualToolsPtr right_tip_;
  //rviz_visual_tools::TFVisualTools tf_visualizer_;

public:
  FingerSensorPerception();
  void processSensorValues(const std_msgs::Int32MultiArray& msg);
  double getOffset(int sensor_value);
  geometry_msgs::Point getTipPoint(int sensor_value);
  geometry_msgs::Point getLeftPoint(int sensor_value, int sensor_index);
  geometry_msgs::Point getRightPoint(int sensor_value, int sensor_index);
};

}
#endif // FINGER_SENSOR_PERCEPTION_H
