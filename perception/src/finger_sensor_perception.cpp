/**
 * Author(s) : Rebecca Cox
 * Desc      : creates visuals for rviz to display sensor feedback
 * Date      : 2017 - Feb - 02
 */

#include <perception/finger_sensor_perception.h>

namespace perception
{


FingerSensorPerception::FingerSensorPerception()
 : nh_("~")
{
  //subscribe to left finger tip
  sensor_values = nh_.subscribe("/sensor_values", 1 , &FingerSensorPerception::processSensorValues, this);
  //publish rviz dot
  left_tip_.reset(new rviz_visual_tools::RvizVisualTools("r_gripper_l_finger_tip","/l_finger_sensor_values"));
  left_tip_->enableBatchPublishing();

  right_tip_.reset(new rviz_visual_tools::RvizVisualTools("r_gripper_r_finger_tip","/r_finger_sensor_values"));
  right_tip_->enableBatchPublishing();

  normalize = false;
  visual_tools_enabled = true;
  enabled_sub_ = nh_.subscribe("/finger_perception/enabled", 1, &FingerSensorPerception::enableProcessing, this);
  log_4 = log(0.25);
  log_20000 = log(20000.0);

}

void FingerSensorPerception::enableProcessing(const std_msgs::Bool &enabled)
{
  visual_tools_enabled = enabled.data;
}

void FingerSensorPerception::processSensorValues(const std_msgs::Int32MultiArray& msg)
{
  if (!visual_tools_enabled)
  {
      //std::cout << "not performing perception, cont_run: " << continuous_running << std::endl;
    return;
  }
  int norm_data[16];
  if(normalize) //noramlize before any contact is made
  {


    double ave_left = (msg.data[0]+msg.data[1]+msg.data[2]+msg.data[3]+msg.data[4]+msg.data[5]+msg.data[6])/7.0;
    double ave_right =(msg.data[8]+msg.data[9]+msg.data[10]+msg.data[11]+msg.data[12]+msg.data[13]+msg.data[14])/7.0;

    for(int i=0; i < 15; i++)
    {
      if(i<7)
        normalize_factors[i] = ave_left/msg.data[i];
      else if(i>7)
        normalize_factors[i] = ave_right/msg.data[i];
    }

    double ave_tips = (msg.data[7] + msg.data[15])/2.0;
    normalize_factors[7] = ave_tips/msg.data[7];
    normalize_factors[15] = ave_tips/msg.data[15];
    normalize = false;
  }
  //int left_tip_value = msg.data[7]*normalize_factors[7];
  //int right_tip_value= msg.data[15]*normalize_factors[15];

  for(int i=0; i < 16; i++)
  {
    norm_data[i] = msg.data[i];//*normalize_factors[i];
    //std::cout << msg.data[i] << " ";
  }
 // std::cout << std::endl;

  for(int i=0; i<16; i++)
  {
    if(i<7 && norm_data[i]>threshold)
      right_tip_->publishSphere(getRightPoint(norm_data[i],i+1), rviz_visual_tools::CYAN);//, rviz_visual_tools::XSMALL);
    /*else if(i==7 && norm_data[i]>threshold)
      right_tip_->publishSphere(getTipPoint(norm_data[i]), rviz_visual_tools::WHITE);
    else if(i==15 && norm_data[i]>threshold)
      left_tip_->publishSphere(getTipPoint(norm_data[i]), rviz_visual_tools::WHITE);*/
    else if(i!=15 && i>7 && norm_data[i]>threshold)
      left_tip_->publishSphere(getLeftPoint(norm_data[i],i-7), rviz_visual_tools::PINK);//,rviz_visual_tools::XSMALL);
  }

  left_tip_->triggerBatchPublish();
  right_tip_->triggerBatchPublish();
}


double FingerSensorPerception::getOffset(int sensor_value)
{
  return ((log(sensor_value)-log_20000)/log_4)/100;

}

geometry_msgs::Point FingerSensorPerception::getTipPoint(int sensor_value)
{
  geometry_msgs::Point surface;
  surface.x = 0;
  surface.y = 0;
  surface.z = getOffset(sensor_value);

  return surface;
}

geometry_msgs::Point FingerSensorPerception::getLeftPoint(int sensor_value, int sensor_index)
{
  geometry_msgs::Point surface;
  surface.x = 0;
  surface.y = -getOffset(sensor_value) + l_gripper_offset;// + 0.035
  surface.z = (8-sensor_index)*(-0.01);
  //std::cout <<  "Left: " << sensor_value << std::endl;
  return surface;
}

geometry_msgs::Point FingerSensorPerception::getRightPoint(int sensor_value, int sensor_index)
{
  geometry_msgs::Point surface;
  surface.x = 0;
  surface.y = getOffset(sensor_value) + r_gripper_offset ;// - 0.035;
  surface.z = (8-sensor_index)*(-0.01);
  //std::cout <<  "Rght: " << sensor_value << std::endl;
  return surface;
}

}
