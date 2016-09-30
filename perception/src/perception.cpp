/**
 * Author(s) : Andy McEvoy
 * Desc      : perception server for teleop demo
 * Date      : 2016 - May - 21
 */

#include <boost/lexical_cast.hpp>

#include <ros/ros.h>
#include <std_msgs/Int64.h>
#include <std_msgs/Bool.h>
#include <math.h>
//#include <rviz_visual_tools/rviz_visual_tools.h>
#include <rviz_visual_tools/tf_visual_tools.h>

// Eigen and TF
#include <eigen_conversions/eigen_msg.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>
#include <tf/transform_datatypes.h>

// Image processing
#include <pcl_ros/point_cloud.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/common/centroid.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/search/search.h>
#include <pcl/search/kdtree.h>
#include <pcl/surface/convex_hull.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_polygonal_prism_data.h>

//object identification
//#include <perception/identified_object.h>
#include <pcl/common/geometry.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/common/common.h>

//dynamic reconfiguration of variables during runtime
#include <dynamic_reconfigure/server.h>
#include <perception/perception_paramConfig.h>
#include <perception/perception_param.h>

namespace perception
{
void callback(teleop_interface::perception_paramConfig &config, uint32_t level)
{
  std::ostringstream ss;
  ss << "Reconfigure request, min_cup_height: " << config.min_cws_height << ", max_cup_height: " << config.max_cws_height;
  ROS_INFO_STREAM_NAMED("ppc",ss.str());
  //cup with spoon
  cws_height_min = config.min_cws_height;
  cws_height_max = config.max_cws_height;
  cws_xy_min = config.min_cws_xy;
  cws_xy_max = config.max_cws_xy;
  cup_height = config.cup_height;

  //plate
  plate_height_min = config.min_plate_height;
  plate_height_max = config.max_plate_height;
  plate_xy_min = config.min_plate_xy;
  plate_xy_max = config.max_plate_xy;

  //bowl
  bowl_height_min = config.min_bowl_height;
  bowl_height_max = config.max_bowl_height;
  bowl_xy_min = config.min_bowl_xy;
  bowl_xy_max = config.max_bowl_xy;

  task = static_cast<task_running>(config.task);
}

class PerceptionTester
{
private:
  ros::NodeHandle nh_;

  ros::Subscriber raw_cloud_sub_;
  ros::Subscriber enabled_sub_;
  ros::Publisher not_table_cloud_pub_;
  ros::Publisher z_filtered_objects_cloud_pub_;
  ros::Publisher roi_cloud_pub_;
  ros::Publisher objects_cloud_pub_;
  ros::Publisher number_of_objects_pub_;
  ros::Publisher identified_objects_; //rebecca

  bool objects_detected_;
  std::vector<Eigen::Affine3d> object_poses_;

  bool image_processing_enabled_;

  rviz_visual_tools::TFVisualTools tf_visualizer_;
  tf::TransformListener tf_listener_;

  //rviz_visual_tools::RvizVisualToolsPtr visual_tools_;
public:
  PerceptionTester(int test)
    : nh_("~")
  {
    ROS_INFO_STREAM_NAMED("constructor","starting PerceptionTester...");

    image_processing_enabled_ = false;

    // point clouds
    raw_cloud_sub_ = nh_.subscribe("/camera/depth_registered/points", 1, &PerceptionTester::processPointCloud, this);
    // Enable/disable point cloud processing
    enabled_sub_ = nh_.subscribe("/perception/enabled", 1, &PerceptionTester::enableProcessing, this);
    // Debuggin clouds/publishers
    not_table_cloud_pub_ = nh_.advertise<pcl::PointCloud<pcl::PointXYZRGB> >("/not_table_cloud", 1);
    z_filtered_objects_cloud_pub_ = nh_.advertise<pcl::PointCloud<pcl::PointXYZRGB> >("/z_filt_objects_cloud", 1);
    roi_cloud_pub_ = nh_.advertise<pcl::PointCloud<pcl::PointXYZRGB> >("/roi_cloud", 1);
    // Final clouds/publishers
    objects_cloud_pub_ = nh_.advertise<pcl::PointCloud<pcl::PointXYZRGB> >("/objects_cloud", 1);
    number_of_objects_pub_ = nh_.advertise<std_msgs::Int64>("/num_objects", 1);

    //identified_objects_ = nh_.advertise<perception::identified_object>("/identified_objects", 1);

    dynamic_reconfigure::Server<teleop_interface::perception_paramConfig> srv;
    dynamic_reconfigure::Server<teleop_interface::perception_paramConfig>::CallbackType f;
    f = boost::bind(&callback, _1, _2);
    srv.setCallback(f);

    //visual_tools_->enableBatchPublishing();

    ROS_DEBUG_STREAM_NAMED("constructor","waiting for pubs and subs to come online... (5s)");
    ros::Duration(5).sleep();

    objects_detected_ = false;

    ros::Rate loop_rate(50);
    while(ros::ok())
    {
      //ROS_INFO_STREAM_NAMED("PercConstr", "While Loop");
      if (objects_detected_)
      {
        // TODO: we're using camera_rgb_optical_frame all the time, but the
        // 3d images and the color seem to be shifted. Is that the right frame? Maybe
        // the camera needs to be calibrated?
        std_msgs::Int64 msg;
        msg.data = object_poses_.size();
        number_of_objects_pub_.publish(msg);
        std::size_t idx = 0;
        for (std::vector<Eigen::Affine3d>::const_iterator it = object_poses_.begin(); it != object_poses_.end(); it++ )
        {
          // Can't get this to work :S
          // ROS_ERROR(boost::lexical_cast<std::string>(object_poses_.size()));
          tf_visualizer_.publishTransform(*it, "base", object_labels[idx]);
          idx++;
        }
      }

      loop_rate.sleep();
    }
  }

  ~PerceptionTester()
  {
  }

  void enableProcessing(const std_msgs::Bool &enabled)
  {
    image_processing_enabled_ = enabled.data;
  }

  void processPointCloud(const sensor_msgs::PointCloud2ConstPtr& msg)
  {
    /*
     * The process is as follows:
     * 1) Read cloud
     * 2) Keep points that are within a certain distance of the camera (removes
     *    far background, very close points)
     * 3) Fit a plane using RANSAC to detect table top
     * 4) Create a prism whose base is the convex hull of the plane found above,
     *    and that is tall enough to contain all objects of interest.
     * 5) However, that prism tends to contain the table too, and even if we increase
     *    the lower limit, it's slightly tilted for some reason and is hard to remove the
     *    whole table top without removing part of some objects. Thus, we fit a plane using
     *    RANSAC again, but the input is the points contained in the ROI prism.
     * 5) Keep points on ROI that are not part of the new plane.
     * 6) Cluster them.
     * 7) Compute their centroids.
     */

    if (!image_processing_enabled_)
    {
      return;
    }

    // Read raw point cloud
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr raw_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::fromROSMsg(*msg, *raw_cloud);


    ROS_DEBUG_STREAM_NAMED("ppc","starting segmentation");

    // z-filtering
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr z_filtered_objects (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PassThrough<pcl::PointXYZRGB> pass;
    pass.setInputCloud(raw_cloud);
    pass.setFilterFieldName("z");
    // TODO: read parameters in a way that allows dynamic changes
    pass.setFilterLimits(0.5, 1.2);
    pass.filter(*z_filtered_objects);
    ROS_INFO_STREAM_NAMED("ppc", "point cloud after z filtering has " << z_filtered_objects->width * z_filtered_objects->height);
    z_filtered_objects->header.frame_id = "camera_rgb_optical_frame";
    z_filtered_objects_cloud_pub_.publish(z_filtered_objects);

    // Table top segmentation
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients ());
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices ());
    pcl::SACSegmentation <pcl::PointXYZRGB> table_segmenter;
    table_segmenter.setOptimizeCoefficients(true);
    table_segmenter.setModelType(pcl::SACMODEL_PLANE);
    table_segmenter.setMethodType(pcl::SAC_RANSAC);
    table_segmenter.setMaxIterations(1000);
    table_segmenter.setDistanceThreshold(0.01);

    table_segmenter.setInputCloud(z_filtered_objects);
    table_segmenter.segment(*inliers, *coefficients);

    if (inliers->indices.size () == 0)
    {
      ROS_ERROR_STREAM_NAMED("ppc", "Could not estimate a planar model.");
      return;
    }

    // Extract indices not from table
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr not_table (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::ExtractIndices<pcl::PointXYZRGB> eifilter;
    eifilter.setInputCloud(z_filtered_objects);
    eifilter.setIndices(inliers);
    eifilter.setNegative(true);
    eifilter.filter(*not_table);
    ROS_INFO_STREAM_NAMED("ppc", "point cloud after table top filtering has " << not_table->width * not_table->height);
    not_table->header.frame_id = "camera_rgb_optical_frame";
    // objects is the set of points not on the table
    not_table_cloud_pub_.publish(not_table);

    // on top of table filtering. First extract plane
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr plane (new pcl::PointCloud<pcl::PointXYZRGB>);
    eifilter.setInputCloud(z_filtered_objects);
    eifilter.setIndices(inliers);
    eifilter.setNegative(false);
    eifilter.filter(*plane);
    plane->header.frame_id = "camera_rgb_optical_frame";
    ROS_INFO_STREAM_NAMED("ppc", "plane has " << plane->width * plane->height);

    // now create a 2D convex hull of the plane
    pcl::ConvexHull<pcl::PointXYZRGB> conv;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr convex_hull (new pcl::PointCloud<pcl::PointXYZRGB>);
    conv.setInputCloud(plane);
    conv.setDimension(2);
    conv.reconstruct(*convex_hull);
    // and make a prism that encloses the ROI
    pcl::ExtractPolygonalPrismData<pcl::PointXYZRGB> prism;
    prism.setInputCloud(z_filtered_objects);
    prism.setInputPlanarHull(convex_hull);
    // TODO set them easily
    prism.setHeightLimits(-0.002, 0.06);
    pcl::PointIndices::Ptr obj_indices (new pcl::PointIndices);
    prism.segment(*obj_indices);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr objects (new pcl::PointCloud<pcl::PointXYZRGB>);
    eifilter.setInputCloud(z_filtered_objects);
    eifilter.setIndices(obj_indices);
    eifilter.setNegative(false);
    eifilter.filter(*objects);
    ROS_INFO_STREAM_NAMED("ppc", "point cloud after ROI filtering has " << objects->width * objects->height);
    objects->header.frame_id = "camera_rgb_optical_frame";
    roi_cloud_pub_.publish(objects);

    // The prism is not great at removing the whole plane, even if we increase the lower height
    // limit to 0.005 m, so let's get its output, fit a plane, and get everything that's not a
    // plane. Repetitive code. There should be a better pcl API
    table_segmenter.setDistanceThreshold(0.01);
    table_segmenter.setInputCloud(objects);
    table_segmenter.segment(*inliers, *coefficients);

    if (inliers->indices.size () == 0)
    {
      ROS_ERROR_STREAM_NAMED("ppc", "Could not estimate a planar model.");
      return;
    }

    // Extract indices not from table
    eifilter.setInputCloud(objects);
    eifilter.setIndices(inliers);
    eifilter.setNegative(true);
    eifilter.filter(*not_table);
    ROS_INFO_STREAM_NAMED("ppc", "final point cloud has " << not_table->width * not_table->height);
    not_table->header.frame_id = "camera_rgb_optical_frame";
    // not_table is the set of points not on the table
    //objects_cloud_pub_.publish(not_table);

    // Cluster objects
    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB>);
    tree->setInputCloud(not_table);

    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> ec;
    // SI units
    //ec.setClusterTolerance(0.02);//original before rebecca came
    ec.setClusterTolerance(0.05);
    /* From experiments:
     * - Tiny wood cubes have about 300 points
     * - Cubeletes have about 800 points
     * Thus, let's set the following values:
     */
    /*TODO:
     * -    draw ROI above table. filter out arm and other objects
     * -    if objects are too close they become one
     *          -   if it's too large, it gets filtered out
     * Idea: filter known objects and use RANSAC to find a match. Others, do more processing on
     *  -start to save objects
     */
    ec.setMinClusterSize(200);  // less than a wood cube
    ec.setMaxClusterSize(15000);  // a plate is lots
    ec.setSearchMethod(tree);
    ec.setInputCloud(not_table);
    ec.extract(cluster_indices);

    //TODO: Old poses continue to be published?
    std::vector<Eigen::Affine3d> local_poses;
    object_poses_ = local_poses;
    std::size_t idx = 0;

    cws_objects = 0;
    plate_objects = 0;
    bowl_objects = 0;
    unknown_objects = 0;

    int objects_found = cluster_indices.end() - cluster_indices.begin();
    //object_labels->clear();
    //object_labels = new std::string[objects_found+1];
    Eigen::Vector4d useless_centroid;
    Eigen::Vector3d object_centroid;
    tf::StampedTransform qr_transform;
    Eigen::Affine3d object_pose;
    visual_tools_->deleteAllMarkers();

    tf_listener_.waitForTransform("base", "camera_rgb_optical_frame", ros::Time(0), ros::Duration(1.0));
    try
    {
      tf_listener_.lookupTransform("base", "camera_rgb_optical_frame", ros::Time(0), qr_transform);
    }
    catch (tf::TransformException ex)
    {
      ROS_ERROR("%s", ex.what());
      //ros::Duration(1.0).sleep();
    }

    for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin();
         it != cluster_indices.end();
         ++it)
    {
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr single_object (new pcl::PointCloud<pcl::PointXYZRGB>);
      for (std::vector<int>::const_iterator pit = it->indices.begin();
           pit != it->indices.end();
           ++pit)
      {
        single_object->points.push_back(not_table->points[*pit]);
      }
      single_object->width = single_object->points.size();
      single_object->height = 1;
      single_object->is_dense = true;
      single_object->header.frame_id = "camera_rgb_optical_frame";

      //removing noise
      pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> sor;

      sor.setInputCloud (single_object);
      sor.setMeanK (50);
      sor.setStddevMulThresh (1.0);
      sor.filter (*single_object);

      // Compute single_object centroid (i.e., our estimated position in the
      // camera_rgb_optical_frame).
      // TODO: why does centroid 3D need a 4 vector, but the last value seems
      // useless (?)

      object_pose = Eigen::Affine3d::Identity();
      pcl::compute3DCentroid(*not_table,
                             *it,  // indices to be used from cloud. Checked that matches output of compute3dcentroid(single_object)
                             useless_centroid);
      //pcl::compute3DCentroid(*single_object, useless_centroid);
      object_centroid << useless_centroid(0), useless_centroid(1), useless_centroid(2); //x, y, z

      tf::transformTFToEigen(qr_transform, object_pose);
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr single_object_transformed (new pcl::PointCloud<pcl::PointXYZRGB>);
      pcl::transformPointCloud (*single_object, *single_object_transformed, object_pose);
      object_pose.translation() = object_centroid;
      single_object_transformed->header.frame_id = "base";

      debug_object_cloud_.publish(single_object_transformed);

      object_pose = Eigen::Affine3d::Identity();
      pcl::compute3DCentroid(*single_object_transformed, useless_centroid);
      object_centroid << useless_centroid(0), useless_centroid(1), useless_centroid(2); //x, y, z
      object_pose.translation() = object_centroid;
      local_poses.push_back(object_pose);
      ROS_INFO_STREAM_NAMED("ppc", "Object " << idx << " has " << single_object_transformed->width * single_object_transformed->height << " points");

      ROS_INFO_STREAM_NAMED("ppc", "useless_centroid_0: " << useless_centroid(0)<<"1: "<< useless_centroid(1)<<"2: " << useless_centroid(2));

      /*******************************REBECCA'S PERCEPTION ADDITIONS**********************************************************************/

      std::string object_identity = object_recognition( object_pose, single_object_transformed, idx, useless_centroid_table(0));
      object_labels.push_back(object_identity);
      if(object_identity == "cup_with_spoon_0")
      {
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr spoon_object (new pcl::PointCloud<pcl::PointXYZRGB>);
        ROS_INFO_STREAM_NAMED("ppc", "object point cloud size: " << single_object_transformed->points.size());

        for (int pit = 0;
             pit < single_object_transformed->points.size();
             ++pit)
        {
          if(single_object_transformed->points[pit].z > cup_min_z+cup_height)
            spoon_object->points.push_back(single_object_transformed->points[pit]);
        }
        ROS_INFO_STREAM_NAMED("ppc", "object point cloud size: " << spoon_object->points.size());
        object_labels.push_back(spoon_label);

        object_pose = Eigen::Affine3d::Identity();
        pcl::compute3DCentroid(*spoon_object, useless_centroid);
        object_centroid << useless_centroid(0), useless_centroid(1), useless_centroid(2); //x, y, z
        object_pose.translation() = object_centroid;

        local_poses.push_back(object_pose);

        ROS_INFO_STREAM_NAMED("ppc", "Spoon " << idx << " has " << spoon_object->width * spoon_object->height << " points");
        ROS_INFO_STREAM_NAMED("ppc", "useless_centroid_0: " << useless_centroid(0)<<"1: "<< useless_centroid(1)<<"2: " << useless_centroid(2));

      }

      /*******************************END REBECCA'S PERCEPTION ADDITIONS*******************************************************************/

      objects_cloud_pub_.publish(single_object);

      idx++;
      objects_detected_ = true;
    }

    object_poses_ = local_poses;
    visual_tools_->triggerBatchPublish();

    ROS_DEBUG_STREAM_NAMED("pcc","finished segmentation");
  }

  double cup_min_z;

  std::string object_recognition(Eigen::Affine3d object_pose, pcl::PointCloud<pcl::PointXYZRGB>::Ptr single_object, int index, double table_z)
  {
    pcl::PointXYZRGB min, max;

    pcl::getMinMax3D(*single_object, min, max);
    cup_min_z = min.z;
    double height = max.z-min.z;
    double depth = max.x-min.x;
    double width = max.y-min.y;

    ROS_INFO_STREAM_NAMED("ppc", "Min: " << min << ", Max: " << max);
    ROS_INFO_STREAM_NAMED("ppc", "Height: " << height << ", Width: " << width << ", Depth: " << depth );

    switch(task)
    {
    case TASK1:
      return task_1_object_id(object_pose, depth, width, height, index);
    case TASK0:
    default:
      return task_1_object_id(object_pose, depth, width, height, index);
      break;
    }

  }

  std::string task_1_object_id(Eigen::Affine3d object_pose, double depth, double width, double height, int index)
  {
    std::ostringstream ss;
    if((cws_height_min < height) && (height < cws_height_max)
       && (cws_xy_min < width) && (width < cws_xy_max)
       && (cws_xy_min < depth) && (depth < cws_xy_max) )
    {
      ss << cws_label << "_" << cws_objects;
      cws_objects++;
    }
    else if((bowl_height_min < height) && (height < bowl_height_max)
            && (bowl_xy_min < width) && (width < bowl_xy_max)
            && (bowl_xy_min < depth) && (depth < bowl_xy_max))
    {
      ss << bowl_label << "_" << bowl_objects;
      bowl_objects++;
    }
    else if((plate_height_min < height) && (height < plate_height_max)
            && (plate_xy_min < width) && (width < plate_xy_max)
            && (plate_xy_min < depth) && (depth < plate_xy_max))
    {
      ss << plate_label << "_" << plate_objects;
      plate_objects++;
    }
    else
    {
      ss << unknown_label << "_" << unknown_objects;
      unknown_objects++;
    }
    ROS_INFO_STREAM_NAMED("ppc", "Object Identity: " << ss.str());
    return ss.str();
  }

}; // end class PerceptionTester
} // end namespace perception


int main(int argc, char** argv)
{
  ros::init(argc, argv, "perception");
  ros::AsyncSpinner spinner(1);
  spinner.start();

  int test = 1;
  perception::PerceptionTester tester(test);

  return 0;
}
