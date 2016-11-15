
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

namespace object_detection
{
typedef pcl::PointXYZRGB PointType;
typedef pcl::Normal NormalType;
typedef pcl::ReferenceFrame RFType;
typedef pcl::SHOT352 DescriptorType;

//std::string models[] = {"cup.pcd"};
/*
struct CloudStyle
{
    double r;
    double g;
    double b;
    double size;

    CloudStyle (double r,
                double g,
                double b,
                double size) :
        r (r),
        g (g),
        b (b),
        size (size)
    {
    }
};

CloudStyle style_white (255.0, 255.0, 255.0, 4.0);
CloudStyle style_red (255.0, 0.0, 0.0, 3.0);
CloudStyle style_green (0.0, 255.0, 0.0, 5.0);
CloudStyle style_cyan (93.0, 200.0, 217.0, 4.0);
CloudStyle style_violet (255.0, 0.0, 255.0, 8.0);
*/
//Algorithm params

struct model_object
{
  std::string name;
  pcl::PointCloud<PointType>::Ptr raw_cloud;
  pcl::PointCloud<PointType>::Ptr keypoints;
  pcl::PointCloud<NormalType>::Ptr normals;
  pcl::PointCloud<DescriptorType>::Ptr descriptors;

  model_object* next;
};

class ObjectDetection
{
private:
  bool cup_loaded;
  pcl::PointCloud<PointType>::Ptr cup;
  pcl::PointCloud<PointType>::Ptr cup_keypoints;
  pcl::PointCloud<NormalType>::Ptr cup_normals;
  pcl::PointCloud<DescriptorType>::Ptr cup_descriptors;

  model_object* models_linkedlist;
  void load_model_objects();
  bool is_object(model_object*  unknown, model_object* model);
  model_object* compute_descriptors(pcl::PointCloud<PointType>::Ptr point_cloud);
public:
  explicit ObjectDetection();
  void compute_cup();
  bool is_cup(boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB> > unknown);
  std::string label_object(pcl::PointCloud<PointType>::Ptr unknown);
protected:
  ros::NodeHandle nh_;
  ros::Publisher found_match_cloud_pub_;

};



}


#endif //CORRESPONDENCE_GROUPING_H
