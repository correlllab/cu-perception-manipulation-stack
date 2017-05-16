/**
 * Author(s) : Rebecca Cox
 * Desc      : object recognition using Correspondence Grouping
 * Src       : http://pointclouds.org/documentation/tutorials/correspondence_grouping.php
 * Date      : 2017 - Feb - 02
 */

#ifndef CORRESPONDENCE_GROUPING_H
#define CORRESPONDENCE_GROUPING_H

#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/correspondence.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/shot_omp.h>
#include <pcl/features/board.h>
//include <pcl/filters/uniform_sampling.h>
#include <pcl/keypoints/uniform_sampling.h>
#include <pcl/recognition/cg/hough_3d.h>
#include <pcl/recognition/cg/geometric_consistency.h>
#include <pcl/recognition/hv/hv_go.h>
#include <pcl/registration/icp.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/kdtree/impl/kdtree_flann.hpp>
#include <pcl/common/transforms.h>
#include <pcl/console/parse.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl_ros/point_cloud.h>

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>

#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>
#include <tf/transform_datatypes.h>
#include <rviz_visual_tools/rviz_visual_tools.h>
#include <rviz_visual_tools/tf_visual_tools.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/vtk_lib_io.h>
#include <pcl/io/ply_io.h>
#include <ros/ros.h>
#include <pcl/visualization/histogram_visualizer.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/vfh.h>

namespace object_detection
{
typedef pcl::PointXYZRGB PointType;
typedef pcl::Normal NormalType;
typedef pcl::ReferenceFrame RFType;
typedef pcl::SHOT352 DescriptorType;
//Algorithm params

//Calculates descriptors from object database
struct model_object
{
  std::string name;
  pcl::PointCloud<PointType>::Ptr raw_cloud;
  pcl::PointCloud<PointType>::Ptr keypoints;
  pcl::PointCloud<NormalType>::Ptr normals;
  pcl::PointCloud<DescriptorType>::Ptr descriptors;
  int correspondence_needed;

  model_object* next;
};

class ObjectDetection
{
private:
  model_object* models_linkedlist;
  void load_model_objects();

  //main function for determining if objects match
  bool is_object(model_object*  unknown, model_object* model);
  model_object* compute_descriptors(pcl::PointCloud<PointType>::Ptr point_cloud);
public:
  explicit ObjectDetection();
  std::string label_object(pcl::PointCloud<PointType>::Ptr unknown);
protected:
  ros::NodeHandle nh_;
  ros::Publisher found_match_cloud_pub_; //publishes keypoints for debugging purposes
  ros::Publisher poseArrayPub;  //publishes surface normals for debugging purposes or pretty pictures
  geometry_msgs::PoseArray poseArray;

};



}


#endif //CORRESPONDENCE_GROUPING_H
