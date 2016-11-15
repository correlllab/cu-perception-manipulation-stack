#ifndef PERCEPTION_PARAM_H
#define PERCEPTION_PARAM_H

// Eigen and TF
#include <eigen_conversions/eigen_msg.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>
#include <tf/transform_datatypes.h>

#include <ctime>
#include <ros/ros.h>
#include <string>

namespace perception
{
  enum task_running { TASK1 = 0, TASK2 = 1, TASK3 = 2} task;

  //cup with spoon
  double cws_height_min;
  double cws_height_max;
  double cws_xy_min;
  double cws_xy_max;
  int cws_objects = 0;

  //spoon without cup split
  double cup_height;

  //plate
  double plate_height_min;
  double plate_height_max;
  double plate_xy_min;
  double plate_xy_max;
  int plate_objects = 0;

  //bowl
  double bowl_height_min;
  double bowl_height_max;
  double bowl_xy_min;
  double bowl_xy_max;
  int bowl_objects = 0;

  //cup
  double cup_height_min;
  double cup_height_max;
  double cup_xy_min;
  double cup_xy_max;
  int cup_objects = 0;

  //shaker
  double shaker_height_min;
  double shaker_height_max;
  double shaker_xy_min;
  double shaker_xy_max;
  int shaker_objects = 0;

  //published labels
  const std::string cws_label = "cup_with_spoon";
  const std::string cup_label = "cup";
  const std::string spoon_label = "spoon";
  const std::string bowl_label = "bowl";
  const std::string plate_label = "plate";
  const std::string shaker_label = "shaker";
  const std::string unknown_label = "unknown";
  int unknown_objects = 0;

  std::vector<std::string> object_labels;
  const std::string base_frame = "camera_rgb_optical_frame";
  //const std::string base_frame = "base";
  const int seconds_keep_alive = 5;
  int blocks = 0;
  int cups = 0;
  struct object_tracking
  {
    std::string name;
    std::time_t timestamp;
    Eigen::Vector3d centroid;

    object_tracking* next;
  };
}


#endif // PERCEPTION_PARAM_H
