#ifndef PERCEPTION_PARAMS_H
#define PERCEPTION_PARAMS_H

#include <ros/ros.h>
#include <string>

namespace teleop_perception
{
  //cup with spoon
  double cws_height_min = .16;
  double cws_height_max = .18;

  //published labels
  const std::string cws_label = "cup_with_spoon";
  const std::string cup_label = "cup";
  const std::string spoon_label = "spoon";
  const std::string bowl_label = "bowl";
  const std::string plate_label = "plate";
  const std::string unknown_label = "unknown";
  //std::ostringstream cws_label;

  std::string* object_labels;

}


#endif // PERCEPTION_PARAMS_H
