#ifndef PERCEPTION_PARAM_H
#define PERCEPTION_PARAM_H

// Eigen and TF
#include <eigen_conversions/eigen_msg.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>
#include <tf/transform_datatypes.h>
#include <pcl_ros/point_cloud.h>
#include <ctime>
#include <ros/ros.h>
#include <string>

namespace perception
{
  bool standalone = false; //running with a robot base or just the camera
  bool continuous_running = false; //continuously run perception or wait for keyboard command
  bool colored_block_detection = false;
  bool save_new = false;
  bool one_of_each = true;
  
  enum task_running { TASK1 = 0, TASK2 = 1, TASK3 = 2} task;

  std::vector<std::string> object_labels;
  std::string base_frame;

  const int seconds_keep_alive = 6; //15
  const int seconds_rename = 10; //12
  double centroid_tracking_distance = 0.025; //meters

  double retest_object_seconds = 2;
  struct object_tracking
  {
    std::string label;
    std::string published_name;
    int id;
    std::time_t timestamp;
    Eigen::Vector3d centroid;

    object_tracking* next;
  };
  struct clustered_objects
  {
    std::string label;
    int id;
    Eigen::Vector3d centroid;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloud;

    clustered_objects* next;
  };
}


#endif // PERCEPTION_PARAM_H
