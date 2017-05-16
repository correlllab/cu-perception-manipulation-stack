/**
 * Author(s) : Rebecca Cox, Jorge Canardo, Andy McEvoy
 * Desc      : perception server for object recognition
 * Date      : 2017 - Feb - 02
 */

#ifndef PERCEPTION_PARAM_H
#define PERCEPTION_PARAM_H

//ROS
#include <ros/ros.h>
#include <ros/package.h>
#include <rviz_visual_tools/rviz_visual_tools.h>
#include <rviz_visual_tools/tf_visual_tools.h>
#include <pcl_ros/point_cloud.h>

//Eigen and TF
#include <eigen_conversions/eigen_msg.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>
#include <tf/transform_datatypes.h>

//PCL
#include <pcl/PCLPointCloud2.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/common/common.h>
#include <pcl/common/geometry.h>
#include <pcl/common/centroid.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h> //print pcd to file
#include <pcl/search/search.h>
#include <pcl/search/kdtree.h>
#include <pcl/surface/convex_hull.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/sac_segmentation.h> //RANSAC
#include <pcl/segmentation/extract_polygonal_prism_data.h>
#include <pcl/segmentation/region_growing_rgb.h> //color-based segmentation

//perception
#include <perception/correspondence_grouping.h>
#include <perception/finger_sensor_perception.h>

//general
#include <std_msgs/Int64.h>
#include <std_msgs/Bool.h>
#include <math.h>
#include <ctime>
#include <string>
#include <boost/lexical_cast.hpp>

namespace perception
{
  bool standalone = false; //running with a robot base or just the camera
  bool continuous_running = true; //continuously run perception or wait for command
  bool begin_from_start = false; //begin from start, or wait for command
  bool colored_block_detection = false; //further segments unknown objects by their color
  bool only_blocks = false; //keeps blocks numbered between 0-N, where N is the number of blocks
  bool save_new = false; //save unknown point clouds to a file
  bool one_of_each = true; //only 1 unique object. used for labeling objects with a specified name (rather than object_some_number)
  //bool find_basket = false; //only finds the basket corners if true
  bool publish_handle = true; //publishes cup handle when a cup is found
  std::vector<std::string> object_labels;
  std::string base_frame;

  const int seconds_keep_alive = 6; //15
  const int seconds_rename = 10; //12
  double centroid_tracking_distance = 0.025; //meters

  double retest_object_seconds = 2;
  struct single_object_ll
  {
    std::string label; //object identified as. plate, bowl, cup, etc.
    std::string published_name; //name published as such as plate_5
    int id; //unique id
    std::time_t timestamp; //time since last identified
    Eigen::Vector3d centroid; //centroid last seen
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloud; 

    single_object_ll* next;
  };

class Perception
{
private:
    boost::shared_ptr<FingerSensorPerception> fingerSensorsPtr;
    boost::shared_ptr<object_detection::ObjectDetection> ObjectDetectionPtr;
    ros::NodeHandle nh_;
    std::string previous_frame;
    ros::Subscriber raw_cloud_sub_;
    ros::Subscriber enabled_sub_;
    ros::Publisher not_table_cloud_pub_;
    ros::Publisher z_filtered_objects_cloud_pub_;
    ros::Publisher roi_cloud_pub_;
    ros::Publisher objects_cloud_pub_;
    ros::Publisher number_of_objects_pub_;
    single_object_ll* tracked_objects;
    bool objects_detected_;
    std::vector<Eigen::Affine3d> object_poses_;

    bool image_processing_enabled_;
    bool clear_tf_buffer;

    rviz_visual_tools::TFVisualTools tf_visualizer_;
    tf::TransformListener tf_listener_;
    Eigen::Vector3f table_orientation;
    rviz_visual_tools::RvizVisualToolsPtr visual_tools_;

    
    bool timed_out(std::time_t timestamp, double seconds_to_timeout);
    void add_tracking_object(Eigen::Vector3d centroid, std::string label, int id, std::string published_name);
    int get_unique_id();
    void remove_outdated_objects();
    void update_tracked_object(single_object_ll* object, Eigen::Vector3d new_centroid, std::string label, std::string  published_name);
    single_object_ll* near_centroid_object(Eigen::Vector3d centroid);
    Eigen::Vector3f calucalateTableNormal(pcl::PointCloud<pcl::PointXYZRGB>::Ptr table, pcl::PointIndices::Ptr inliers);
    single_object_ll* get_colored_segments(pcl::PointCloud<pcl::PointXYZRGB>::Ptr single_object, int &unique_id);

public:
    Perception(int test);
    ~Perception(){}
  
    void processPointCloud(const sensor_msgs::PointCloud2ConstPtr& msg);
    void enableProcessing(const std_msgs::Bool &enabled);
  
};

}


#endif // PERCEPTION_PARAM_H
