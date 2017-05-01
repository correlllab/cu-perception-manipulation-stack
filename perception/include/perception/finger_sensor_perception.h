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
#include <std_msgs/Bool.h>
#include <math.h>


namespace perception
{
const int num_sensors = 16; //the tips should be at index 7 and 15
const bool baxter_gripper = true;
const bool wide_gripper = true;


const int threshold = 300; //do_nothing
const int three5_cent = 250; //3.5 centimeter away surface
const int three_cent = 500; //3 centimeter away surface
const int two5_cent = 750; //2.5 centimeter away surface
const int two_cent = 1000; //2 centimeter away surface
const int one5_cent = 1500; //1.5 centimeter away surface
const int one_cent = 2000; //1 centimeter away surface
const int half_cent = 3000; //0.5 centimeter away surface

class FingerSensorPerception
{
private:
  double normalize_factors[16];
  double log_4;
  double log_20000;

  bool normalize;


  ros::NodeHandle nh_;
  ros::Subscriber enabled_sub_;
  ros::Subscriber sensor_values;
  ros::Subscriber right_finger_tip;
  rviz_visual_tools::RvizVisualToolsPtr left_tip_;
  rviz_visual_tools::RvizVisualToolsPtr right_tip_;
  //rviz_visual_tools::TFVisualTools tf_visualizer_;

  bool visual_tools_enabled;

public:
  double l_gripper_offset;
  double r_gripper_offset;

  FingerSensorPerception();
  void enableProcessing(const std_msgs::Bool &enabled);
  void processSensorValues(const std_msgs::Int32MultiArray& msg);
  double getOffset(int sensor_value);
  geometry_msgs::Point getTipPoint(int sensor_value);
  geometry_msgs::Point getLeftPoint(int sensor_value, int sensor_index);
  geometry_msgs::Point getRightPoint(int sensor_value, int sensor_index);
};

}
#endif // FINGER_SENSOR_PERCEPTION_H
