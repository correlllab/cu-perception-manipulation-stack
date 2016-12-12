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
#include <rviz_visual_tools/rviz_visual_tools.h>
#include <rviz_visual_tools/tf_visual_tools.h>

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
#include <pcl/io/pcd_io.h> //print pcd to file
#include <pcl/common/geometry.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/common/common.h>

//dynamic reconfiguration of variables during runtime
#include <dynamic_reconfigure/server.h>
#include <perception/perception_paramConfig.h>
#include <perception/perception_param.h>
#include <perception/correspondence_grouping.h>

//color-based segmentation
#include <pcl/segmentation/region_growing_rgb.h>
namespace perception
{
void callback(perception::perception_paramConfig &config, uint32_t level)
{
  std::ostringstream ss;
  ss << "Reconfigure request, min_cup_height: " << config.min_cws_height << ", max_cup_height: " << config.max_cws_height;
  ROS_INFO_STREAM_NAMED("ppc",ss.str());

  task = static_cast<task_running>(config.task);
}

class PerceptionTester
{
private:
  boost::shared_ptr<object_detection::ObjectDetection> ObjectDetectionPtr;
  ros::NodeHandle nh_;

  ros::Subscriber raw_cloud_sub_;
  ros::Subscriber enabled_sub_;
  ros::Publisher not_table_cloud_pub_;
  ros::Publisher z_filtered_objects_cloud_pub_;
  ros::Publisher roi_cloud_pub_;
  ros::Publisher objects_cloud_pub_;
  ros::Publisher number_of_objects_pub_;

  bool objects_detected_;
  std::vector<Eigen::Affine3d> object_poses_;

  bool image_processing_enabled_;

  rviz_visual_tools::TFVisualTools tf_visualizer_;
  tf::TransformListener tf_listener_;

  rviz_visual_tools::RvizVisualToolsPtr visual_tools_;
public:
  PerceptionTester(int test)
    : nh_("~")
  {
    if(standalone)
      base_frame = "camera_rgb_optical_frame";
    else
      base_frame = "base";

    objects_linkedlist = NULL;
    ROS_INFO_STREAM_NAMED("constructor","starting PerceptionTester...");
    ObjectDetectionPtr.reset(new object_detection::ObjectDetection());
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

    dynamic_reconfigure::Server<perception::perception_paramConfig> srv;
    dynamic_reconfigure::Server<perception::perception_paramConfig>::CallbackType f;
    f = boost::bind(&callback, _1, _2);
    srv.setCallback(f);

    visual_tools_.reset(new rviz_visual_tools::RvizVisualTools(base_frame,"/bounding_boxes"));
    visual_tools_->enableBatchPublishing();
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
          tf_visualizer_.publishTransform(*it, base_frame, object_labels[idx]);
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
  Eigen::Vector3d cup_centroid;

  object_tracking* objects_linkedlist;
  object_tracking* near_centroid_object(Eigen::Vector3d centroid)
  {
    object_tracking* iterator = objects_linkedlist;
    double x_dif, y_dif, z_dif;
    while(iterator)
    {
      x_dif = (centroid[0]-iterator->centroid[0]);
      y_dif = (centroid[1]-iterator->centroid[1]);
      z_dif = (centroid[2]-iterator->centroid[2]);

      if((x_dif*x_dif + y_dif*y_dif + z_dif*z_dif) < centroid_tracking_distance*centroid_tracking_distance)
        return iterator;

      iterator = iterator->next;
    }
    return iterator;
  }

  bool timed_out(std::time_t timestamp, double seconds_to_timeout)
  {
    double seconds_since_updated = difftime( time(0), timestamp);

    return (seconds_since_updated >= seconds_to_timeout);
  }

  void add_tracking_object(Eigen::Vector3d centroid, std::string label, int id)
  {
    object_tracking* new_node(new object_tracking());
    new_node->label = label;
    new_node->centroid = centroid;
    new_node->id = id;
    new_node->timestamp = std::time(NULL);
    new_node->next = NULL;

    if(!objects_linkedlist)
      objects_linkedlist = new_node;
    else
    {
      object_tracking* iterator = objects_linkedlist;
      while(iterator->next)
      {
        iterator = iterator->next;
      }
      iterator->next = new_node;
    }

  }

  int get_unique_id()
  {
    object_tracking* iterator;
    int id = 0;
    while(id < 100)
    {
      iterator = objects_linkedlist;
      //std::cout << "unique checking:" << id;
      while(iterator)
      {
        //std::cout << "numbers:" << iterator->id;
        if(iterator->id == id)
        {
          break;
        }
        iterator = iterator->next;
      }
      if(!iterator)
        return id;
      id++;
    }
    return id;
  }

  void remove_outdated_objects()
  {
    if(!objects_linkedlist)
      return;
    object_tracking* previous = objects_linkedlist;
    object_tracking* next = previous->next;
    std::ostringstream ss;
    while(next)
    {
      //std::cout << previous->label << "_" << previous->id;
      if(timed_out(next->timestamp, seconds_keep_alive))
      {
        previous->next = next->next;

        ss << next->label << "_" << next->id;
        //std::cout << "removing " << ss.str() << endl;
        tf_visualizer_.publishTransform(Eigen::Affine3d::Identity(), base_frame, ss.str());
        delete next;
        next = previous->next;
        ss.str("");
      }
      else
      {
        previous = next;
        next = next->next;
      }
    }
    if(timed_out(objects_linkedlist->timestamp, seconds_keep_alive))
    {
      previous = objects_linkedlist;
      objects_linkedlist = objects_linkedlist->next;
      ss << previous->label << "_" << previous->id;
      tf_visualizer_.publishTransform(Eigen::Affine3d::Identity(), base_frame, ss.str());
      delete previous;
    }
  }

  void update_tracked_object(object_tracking* object, Eigen::Vector3d new_centroid, std::string label)
  {
    object->centroid = new_centroid;
    object->timestamp = std::time(NULL);
    object->label = label;
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

    if (!image_processing_enabled_ && !continuous_running)
    {
      return;
    }

    // Read raw point cloud
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr raw_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::fromROSMsg(*msg, *raw_cloud);


    //ROS_DEBUG_STREAM_NAMED("ppc","starting segmentation");

    // z-filtering
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr z_filtered_objects (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PassThrough<pcl::PointXYZRGB> pass;
    pass.setInputCloud(raw_cloud);
    pass.setFilterFieldName("z");
    // TODO: read parameters in a way that allows dynamic changes
    pass.setFilterLimits(0.5, 1.2);
    pass.filter(*z_filtered_objects);
    //ROS_INFO_STREAM_NAMED("ppc", "point cloud after z filtering has " << z_filtered_objects->width * z_filtered_objects->height);
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
      //ROS_ERROR_STREAM_NAMED("ppc", "Could not estimate a planar model.");
      return;
    }

    // Extract indices not from table
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr not_table (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::ExtractIndices<pcl::PointXYZRGB> eifilter;
    eifilter.setInputCloud(z_filtered_objects);
    eifilter.setIndices(inliers);
    eifilter.setNegative(true);
    eifilter.filter(*not_table);
    //ROS_INFO_STREAM_NAMED("ppc", "point cloud after table top filtering has " << not_table->width * not_table->height);
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
    //ROS_INFO_STREAM_NAMED("ppc", "plane has " << plane->width * plane->height);

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
    prism.setHeightLimits(-0.002, 0.15);
    pcl::PointIndices::Ptr obj_indices (new pcl::PointIndices);
    prism.segment(*obj_indices);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr objects (new pcl::PointCloud<pcl::PointXYZRGB>);
    eifilter.setInputCloud(z_filtered_objects);
    eifilter.setIndices(obj_indices);
    eifilter.setNegative(false);
    eifilter.filter(*objects);
    //ROS_INFO_STREAM_NAMED("ppc", "point cloud after ROI filtering has " << objects->width * objects->height);
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
      //ROS_ERROR_STREAM_NAMED("ppc", "Could not estimate a planar model.");
      return;
    }

    // Extract indices not from table
    eifilter.setInputCloud(objects);
    eifilter.setIndices(inliers);
    eifilter.setNegative(true);
    eifilter.filter(*not_table);
    //ROS_INFO_STREAM_NAMED("ppc", "final point cloud has " << not_table->width * not_table->height);
    not_table->header.frame_id = "camera_rgb_optical_frame";
    // not_table is the set of points not on the table
    //objects_cloud_pub_.publish(not_table);
/*
    //down sample
   /* float model_ss_ (0.01f);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr not_table_keypoints(new pcl::PointCloud<pcl::PointXYZRGB>());
    pcl::UniformSampling<pcl::PointXYZRGB> uniform_sampling;
    uniform_sampling.setInputCloud (not_table);
    uniform_sampling.setRadiusSearch (model_ss_);
    pcl::PointCloud<int> keypointIndices1;
    uniform_sampling.compute(keypointIndices1);
    pcl::copyPointCloud(*not_table, keypointIndices1.points, *not_table_keypoints);
    //std::cout << "Model total points: " << new_node->raw_cloud->size () << "; Selected Keypoints: " << new_node->keypoints->size () << std::endl;
    not_table_keypoints->header.frame_id = "camera_rgb_optical_frame";*/
    // Cluster objects
    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB>);
    tree->setInputCloud(not_table);

    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> ec;
    // SI units
    //ec.setClusterTolerance(0.02);//original before rebecca came
    ec.setClusterTolerance(0.03);
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
    ec.setMinClusterSize(100);  // less than a wood cube
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

    //int objects_found = cluster_indices.end() - cluster_indices.begin();
    object_labels.clear();
    Eigen::Vector4d useless_centroid;
    Eigen::Vector3d object_centroid;
    tf::StampedTransform qr_transform;
    Eigen::Affine3d object_pose;
    visual_tools_->deleteAllMarkers();

    
    clustered_objects* segmented_objects = NULL;
    clustered_objects* iterator;
    pcl::PointXYZRGB min, max;
    int unique_id = 0;
    //separate objects into a linked list to make processing more simple
    //perform object labeling
    //if unknown, attempt to separate clusters by color and re-label
    for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin();
         it != cluster_indices.end();
         ++it)
    {
      clustered_objects* new_object(new clustered_objects());
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
      std::string label = ObjectDetectionPtr->label_object(single_object);
      if(colored_block_detection)
      {
        //label = ObjectDetectionPtr->label_object(single_object);
        pcl::getMinMax3D(*single_object, min, max);
        double height = max.z-min.z;
        double depth = max.x-min.x;
        double width = max.y-min.y;
        std::cout << label << " height: " << height << ", depth: " << depth << ", width: " << width;
        if(label == "unknown")
        {
          new_object = get_colored_segments(single_object, unique_id);

        }
        else
        {
          new_object->label = label;
          new_object->point_cloud = single_object;
          new_object->id = unique_id;
          unique_id++;
          //get_colored_segments_2(single_object, unique_id);
        }

      }
      else
      { //just label and add object
        //label = ObjectDetectionPtr->label_object(single_object);
        new_object->label = label;
        new_object->point_cloud = single_object;
        new_object->id = unique_id;
        unique_id++;
      }

      if(!segmented_objects)
      {
        segmented_objects = new_object;
        iterator = segmented_objects;
      }
      else
      {
        while(iterator->next)
          iterator = iterator->next;
        iterator->next = new_object;
      }
    }

    iterator = segmented_objects;
    

    while(iterator)
    {
      tf_listener_.waitForTransform(base_frame, "camera_rgb_optical_frame", ros::Time(0), ros::Duration(1.0));
      try
      {
        tf_listener_.lookupTransform(base_frame, "camera_rgb_optical_frame", ros::Time(0), qr_transform);
      }
      catch (tf::TransformException ex)
      {
        ROS_ERROR("%s", ex.what());
        //ros::Duration(1.0).sleep();
      }
      object_pose = Eigen::Affine3d::Identity();
      pcl::compute3DCentroid(*iterator->point_cloud, useless_centroid);
      object_centroid << useless_centroid(0), useless_centroid(1), useless_centroid(2); //x, y, z

      tf::transformTFToEigen(qr_transform, object_pose);
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr single_object_transformed (new pcl::PointCloud<pcl::PointXYZRGB>);
      pcl::transformPointCloud (*iterator->point_cloud, *single_object_transformed, object_pose);
      object_pose.translation() = object_centroid;
      single_object_transformed->header.frame_id = base_frame;

      object_pose = Eigen::Affine3d::Identity();
      pcl::compute3DCentroid(*single_object_transformed, useless_centroid);
      object_centroid << useless_centroid(0), useless_centroid(1), useless_centroid(2); //x, y, z
      object_pose.translation() = object_centroid;
      

      /*object_pose = Eigen::Affine3d::Identity();
      tf::transformTFToEigen(qr_transform, object_pose);
      //Eigen::Affine3d new_pose = object_pose;

      //transform object to base frame for proper pose
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr single_object_transformed (new pcl::PointCloud<pcl::PointXYZRGB>);
      pcl::transformPointCloud (*iterator->point_cloud, *single_object_transformed, object_pose);
      single_object_transformed->header.frame_id = base_frame;

      //coputing the centroid in base frame coordinates and push to local_pushes
      pcl::compute3DCentroid(*single_object_transformed, useless_centroid);
      object_centroid << useless_centroid(0), useless_centroid(1), useless_centroid(2); //x, y, z
      object_pose.translation() = object_centroid;
      local_poses.push_back(object_pose);*/

      //ROS_INFO_STREAM_NAMED("ppc", "\n\nObject " << iterator->id << " has " << single_object_transformed->width * single_object_transformed->height << " points");// << '\n');
      //ROS_INFO_STREAM_NAMED("ppc", "useless_centroid_" << iterator->id << ": "<< useless_centroid(0)<<", "<< useless_centroid(1)<<", " << useless_centroid(2));

      //object tracking
      std::ostringstream ss;
      object_tracking* matching_centroid = near_centroid_object(object_centroid);
      std::string label = iterator->label;
      rviz_visual_tools::colors color = rviz_visual_tools::MAGENTA;
      if(matching_centroid)
      {
        ///bug: object's label gets pushed into tf transform publisher, but we removed it at the end of this fuction
        ///   this results in removed objects being sent to base, then back to where is was
        if(matching_centroid->label == "unknown" || timed_out(matching_centroid->timestamp,retest_object_seconds))
        {
          //label = ObjectDetectionPtr->label_object(single_object);
          if(matching_centroid->label == "unknown" || label != "unknown" )
          {
            if(matching_centroid->label != label)
            {
              ss << matching_centroid->label << "_" << matching_centroid->id;
              tf_visualizer_.publishTransform(Eigen::Affine3d::Identity(), base_frame, ss.str());
              ss.str("");
            }
            update_tracked_object(matching_centroid, object_centroid, label);
          }
          else if(timed_out(matching_centroid->timestamp,seconds_rename))
          {
            ss << matching_centroid->label << "_" << matching_centroid->id;
            tf_visualizer_.publishTransform(Eigen::Affine3d::Identity(), base_frame, ss.str());
            ss.str("");
            update_tracked_object(matching_centroid, object_centroid, label);
          }
          else
          {
            label = matching_centroid->label;
          }
        }
        else
          label = matching_centroid->label;
        ss << label << "_" << matching_centroid->id;
      }
      else
      {
        int id = get_unique_id();
        ss << label << "_" << id;
        add_tracking_object(object_centroid, label, id);
      }
      if(label == "unknown")
         color = rviz_visual_tools::CYAN;
      local_poses.push_back(object_pose);
      object_labels.push_back(ss.str());
      
      //ROS_INFO_STREAM_NAMED("ppc", "Object Name" << ss.str());
      //pcl::PointXYZRGB min, max;
      pcl::getMinMax3D(*single_object_transformed, min, max);
      double height = max.z-min.z;
      double depth = max.x-min.x;
      double width = max.y-min.y;
      visual_tools_->publishWireframeCuboid(object_pose, depth, width, height, color);
      
      //pcl::io::savePCDFileASCII("/home/rebecca/ros/sandbox_ws/src/cu-perception-manipulation-stack/perception/object_database/"+ss.str()+".pcd", *iterator->point_cloud);

      objects_cloud_pub_.publish(iterator->point_cloud);

      iterator = iterator->next;
    }
    remove_outdated_objects();
    /*
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

      /*removing noise
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
      single_object_transformed->header.frame_id = base_frame;

      object_pose = Eigen::Affine3d::Identity();
      pcl::compute3DCentroid(*single_object_transformed, useless_centroid);
      object_centroid << useless_centroid(0), useless_centroid(1), useless_centroid(2); //x, y, z
      object_pose.translation() = object_centroid;
      local_poses.push_back(object_pose);
      ROS_INFO_STREAM_NAMED("ppc", "\n\nObject " << idx << " has " << single_object_transformed->width * single_object_transformed->height << " points");// << '\n');

      ROS_INFO_STREAM_NAMED("ppc", "\n\nuseless_centroid_" << idx << ": "<< useless_centroid(0)<<", "<< useless_centroid(1)<<", " << useless_centroid(2));

      //*******************************REBECCA'S PERCEPTION ADDITIONS**********************************************************************
      std::ostringstream ss;
      object_tracking* matching_centroid = near_centroid_object(object_centroid);
      std::string label = ObjectDetectionPtr->label_object(single_object);
      //colored object detection and tracking
      if(colored_block_detection && label == "unknown")
      {
        pcl::RegionGrowingRGB<pcl::PointXYZRGB> reg;
        reg.setInputCloud(single_object);
        reg.setSearchMethod(tree);
        reg.setDistanceThreshold(0.02);
        reg.setPointColorThreshold(10);
        reg.setRegionColorThreshold(10);
        reg.setMinClusterSize(50);

        std::vector <pcl::PointIndices> clusters;
        reg.extract (clusters);
        for (std::vector<pcl::PointIndices>::const_iterator it_2 = clusters.begin();
             it_2 != clusters.end();
             ++it_2)
        {
          pcl::PointCloud<pcl::PointXYZRGB>::Ptr single_block (new pcl::PointCloud<pcl::PointXYZRGB>);
          for (std::vector<int>::const_iterator pit = it_2->indices.begin();
               pit != it_2->indices.end();
               ++pit)
          {
            single_block->points.push_back(single_object->points[*pit]);
          }
          single_block->width = single_block->points.size();
          single_block->height = 1;
          single_block->is_dense = true;
          single_block->header.frame_id = "camera_rgb_optical_frame";
          std::string label_2 = ObjectDetectionPtr->label_object(single_block);
          if(label_2 == "block")
          {

          }
        }
      }
      else if(matching_centroid)
      {
        ///bug: object's label gets pushed into tf transform publisher, but we removed it at the end of this fuction
        ///   this results in removed objects being sent to base, then back to where is was
        if(matching_centroid->label == "unknown" || timed_out(matching_centroid->timestamp,retest_object_seconds))
        {
          //label = ObjectDetectionPtr->label_object(single_object);
          if(matching_centroid->label == "unknown" || label != "unknown" )
          {
            if(matching_centroid->label != label)
            {
              ss << matching_centroid->label << "_" << matching_centroid->id;
              tf_visualizer_.publishTransform(Eigen::Affine3d::Identity(), base_frame, ss.str());
              ss.str("");
            }
            update_tracked_object(matching_centroid, object_centroid, label);
          }
          else if(timed_out(matching_centroid->timestamp,seconds_rename))
          {
            ss << matching_centroid->label << "_" << matching_centroid->id;
            tf_visualizer_.publishTransform(Eigen::Affine3d::Identity(), base_frame, ss.str());
            ss.str("");
            update_tracked_object(matching_centroid, object_centroid, label);
          }
          else
          {
            label = matching_centroid->label;
          }
        }
        else
          label = matching_centroid->label;
        ss << label << "_" << matching_centroid->id;
      }
      else
      {
        //label = ObjectDetectionPtr->label_object(single_object);
        int id = get_unique_id();
        ss << label << "_" << id;
        add_tracking_object(object_centroid, label, id);
      }
      pcl::PointXYZRGB min, max;
      pcl::getMinMax3D(*single_object, min, max);
      double height = max.z-min.z;
      double depth = max.x-min.x;
      double width = max.y-min.y;
      visual_tools_->publishWireframeCuboid(object_pose, height, depth, width, rviz_visual_tools::RAND);
      object_labels.push_back(ss.str());
      //pcl::io::savePCDFileASCII("/home/rebecca/ros/"+ss.str()+".pcd", *single_object);
      remove_outdated_objects();
      objects_cloud_pub_.publish(single_object);
      //ROS_INFO_STREAM_NAMED("ppc", ss.str() + " published");
      idx++;

    }*/

//    object_tracking* iterator = objects_linkedlist;
//    while(iterator)
//    {
//      std::cout << "objects: " << iterator->label << "_" << iterator->id << endl;
//      iterator = iterator->next;
//    }
    objects_detected_ = true;
    object_poses_ = local_poses;
    visual_tools_->triggerBatchPublish();

//    iterator = objects_linkedlist;
//    while(iterator)
//    {
//      std::cout << "objects: " << iterator->label << "_" << iterator->id << endl;
//      iterator = iterator->next;
//    }
    ROS_DEBUG_STREAM_NAMED("pcc","finished segmentation");
  }

  clustered_objects* get_colored_segments(pcl::PointCloud<pcl::PointXYZRGB>::Ptr single_object, int &unique_id)
  {
    pcl::RegionGrowingRGB<pcl::PointXYZRGB> reg;
    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB>);
    tree->setInputCloud(single_object);
    reg.setInputCloud(single_object);
    //reg.setIndices(indices);
    reg.setSearchMethod(tree);
    reg.setDistanceThreshold(1); //10
    reg.setPointColorThreshold(6); //6
    reg.setRegionColorThreshold(5); //5
    reg.setMinClusterSize(50);

    std::vector <pcl::PointIndices> clusters;
    reg.extract (clusters);
    clustered_objects* return_object = NULL;
    clustered_objects* iterator;
    for (std::vector<pcl::PointIndices>::const_iterator it = clusters.begin();
         it != clusters.end();
         ++it)
    {
      clustered_objects* new_object(new clustered_objects());
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr part_object (new pcl::PointCloud<pcl::PointXYZRGB>);
      for (std::vector<int>::const_iterator pit = it->indices.begin();
           pit != it->indices.end();
           ++pit)
      {
        part_object->points.push_back(single_object->points[*pit]);
      }
      part_object->width = part_object->points.size();
      part_object->height = 1;
      part_object->is_dense = true;
      part_object->header.frame_id = "camera_rgb_optical_frame";

      new_object->label = ObjectDetectionPtr->label_object(part_object);
      pcl::PointXYZRGB min, max;
      pcl::getMinMax3D(*part_object, min, max);
      double height = max.z-min.z;
      double depth = max.x-min.x;
      double width = max.y-min.y;
      std::cout << new_object->label << " height: " << height << ", depth: " << depth << ", width: " << width;

      if(height < 0.025 && height > 0.015)
      {
        std::cout << "renaming object to block based on height" << endl;
        new_object->label = "block";
      }
      new_object->point_cloud = part_object;
      new_object->id = unique_id;
      unique_id++;
      std::cout << new_object->label << std::endl;
      //get_colored_segments_2(part_object, unique_id);
      if(!return_object)
      {
        return_object = new_object;
        iterator = return_object;
      }
      else
      {
        while(iterator->next)
          iterator = iterator->next;
        iterator->next = new_object;
      }
    }
    return return_object;
  }

}; // end class PerceptionTester
} // end namespace perception


int main(int argc, char** argv)
{
  ros::init(argc, argv, "perception");
  ros::AsyncSpinner spinner(3);
  spinner.start();

  int test = 1;
  perception::PerceptionTester tester(test);

  return 0;
}
