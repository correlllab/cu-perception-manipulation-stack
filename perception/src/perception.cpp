/**
 * Author(s) : Rebecca Cox, Jorge Canardo, Andy McEvoy
 * Desc      : perception server for object recognition
 * Date      : 2017 - Feb - 02
 */

#include <perception/perception.h>
#include <exception>
#include <dynamic_reconfigure/server.h>
#include <perception/sensor_paramConfig.h>

namespace perception
{

double l_gripper_offset;
double r_gripper_offset;

void callback(perception::sensor_paramConfig &config, uint32_t level)
{
    l_gripper_offset = config.left_gripper_offset_y;
    r_gripper_offset = config.right_gripper_offset_y;
    // std::cout << "received new gripper offsets" << std::endl;
    // std::cout << "Left: " << l_gripper_offset << std::endl;
    // std::cout << "Right: " << r_gripper_offset << std::endl;
}

Perception::Perception(int test)
    : nh_("~")
  {
    table_orientation(0) = 0;
    previous_frame = "";
    base_frame = "camera_rgb_optical_frame";
    tracked_objects = NULL;
    ROS_INFO_STREAM_NAMED("constructor","starting Perception...");
    fingerSensorsPtr.reset(new FingerSensorPerception());
    fingerSensorsPtr->l_gripper_offset = 0;
    fingerSensorsPtr->r_gripper_offset = 0;

    ObjectDetectionPtr.reset(new object_detection::ObjectDetection());
    image_processing_enabled_ = begin_from_start;

    // point clouds
    raw_cloud_sub_ = nh_.subscribe("/camera/depth_registered/points", 1, &Perception::processPointCloud, this);
    // Enable/disable point cloud processing
    enabled_sub_ = nh_.subscribe("/perception/enabled", 1, &Perception::enableProcessing, this);
    // Debuggin clouds/publishers
    not_table_cloud_pub_ = nh_.advertise<pcl::PointCloud<pcl::PointXYZRGB> >("/not_table_cloud", 1);
    z_filtered_objects_cloud_pub_ = nh_.advertise<pcl::PointCloud<pcl::PointXYZRGB> >("/z_filt_objects_cloud", 1);
    roi_cloud_pub_ = nh_.advertise<pcl::PointCloud<pcl::PointXYZRGB> >("/roi_cloud", 1);
    // Final clouds/publishers
    objects_cloud_pub_ = nh_.advertise<pcl::PointCloud<pcl::PointXYZRGB> >("/objects_cloud", 1);
    number_of_objects_pub_ = nh_.advertise<std_msgs::Int64>("/num_objects", 1);
    visual_tools_.reset(new rviz_visual_tools::RvizVisualTools(base_frame,"/bounding_boxes"));
    visual_tools_->enableBatchPublishing();
    ROS_DEBUG_STREAM_NAMED("constructor","waiting for pubs and subs to come online... (5s)");
    ros::Duration(5).sleep();

    objects_detected_ = false;
    clear_tf_buffer = false;

    dynamic_reconfigure::Server<perception::sensor_paramConfig> srv;
    dynamic_reconfigure::Server<perception::sensor_paramConfig>::CallbackType f;
    f = boost::bind(&callback, _1, _2);
    srv.setCallback(f);

    ros::Rate loop_rate(50);
    while(ros::ok())
    {
      fingerSensorsPtr->l_gripper_offset = l_gripper_offset;
      fingerSensorsPtr->r_gripper_offset = r_gripper_offset;
      //ROS_INFO_STREAM_NAMED("PercConstr", "While Loop");
      if (objects_detected_)
      {
        // TODO: we're using camera_rgb_optical_frame all the time, but the
        // 3d images and the color seem to be shifted. Is that the right frame? Maybe
        // the camera needs to be calibrated?
        std::size_t idx = 0;
        std_msgs::Int64 msg;
        msg.data = object_poses_.size();
        number_of_objects_pub_.publish(msg);
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


  void Perception::enableProcessing(const std_msgs::Bool &enabled)
  {
    image_processing_enabled_ = enabled.data;
  }

  single_object_ll* Perception::near_centroid_object(Eigen::Vector3d centroid)
  {
    single_object_ll* iterator = tracked_objects;
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

  bool Perception::timed_out(std::time_t timestamp, double seconds_to_timeout)
  {
    double seconds_since_updated = difftime( time(0), timestamp);

    return (seconds_since_updated >= seconds_to_timeout);
  }

  void Perception::add_tracking_object(Eigen::Vector3d centroid, std::string label, int id, std::string published_name)
  {
    single_object_ll* new_node(new single_object_ll());
    new_node->label = label;
    new_node->centroid = centroid;
    new_node->id = id;
    new_node->timestamp = std::time(NULL);
    new_node->published_name = published_name;
    new_node->next = NULL;

    if(!tracked_objects)
      tracked_objects = new_node;
    else
    {
      single_object_ll* iterator = tracked_objects;
      while(iterator->next)
      {
        iterator = iterator->next;
      }
      iterator->next = new_node;
    }

  }

  int Perception::get_unique_id()
  {
    single_object_ll* iterator;
    int id = 0;
    while(id < 100)
    {
      iterator = tracked_objects;
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

  void Perception::remove_outdated_objects()
  {
    if(!tracked_objects)
      return;
    single_object_ll* previous = tracked_objects;
    single_object_ll* next = previous->next;
    std::ostringstream ss;
    while(next)
    {
      //std::cout << previous->label << "_" << previous->id;
      if(timed_out(next->timestamp, seconds_keep_alive))
      {
        previous->next = next->next;
        if(one_of_each && next->label != "unknown" && next->label != "block")
        {
          ss << next->label << "_position";
        }
        else
        {
          ss << next->label << "_" << next->id;
          //std::cout << "removing " << ss.str() << endl;
        }
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
    if(timed_out(tracked_objects->timestamp, seconds_keep_alive))
    {
      previous = tracked_objects;
      tracked_objects = tracked_objects->next;
      if(one_of_each && next->label != "unknown" && next->label != "block")
      {
        ss << previous->label << "_position";
      }
      else
      {
        ss << previous->label << "_" << previous->id;
      }

      tf_visualizer_.publishTransform(Eigen::Affine3d::Identity(), base_frame, ss.str());
      delete previous;
    }
  }

  void Perception::update_tracked_object(single_object_ll* object, Eigen::Vector3d new_centroid, std::string label, std::string  published_name)
  {
    object->centroid = new_centroid;
    object->timestamp = std::time(NULL);
    object->label = label;
    object->published_name = published_name;
  }

  void Perception::processPointCloud(const sensor_msgs::PointCloud2ConstPtr& msg)
  {
      if (!image_processing_enabled_)
      {
        //std::cout << "not performing perception, cont_run: " << continuous_running << std::endl;
        return;
      }
      objects_detected_ = false;
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

    if(standalone)
       base_frame = "camera_rgb_optical_frame";
    else
       base_frame = "root";

    if(base_frame != previous_frame)
    {
       previous_frame = base_frame;
       visual_tools_.reset(new rviz_visual_tools::RvizVisualTools(base_frame,"/bounding_boxes"));
       visual_tools_->enableBatchPublishing();
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
    pass.setFilterLimits(0.0, 1); // 0.5, 1.2
    pass.filter(*z_filtered_objects);
    //ROS_INFO_STREAM_NAMED("ppc", "point cloud after z filtering has " << z_filtered_objects->width * z_filtered_objects->height);
    z_filtered_objects->header.frame_id = "camera_rgb_optical_frame";
    z_filtered_objects_cloud_pub_.publish(z_filtered_objects);

    // Table top segmentation
    //TODO: Try to optimize, no reason to rerun ransac for every frame when the table and camera don't move
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients ());
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices ());
    pcl::SACSegmentation <pcl::PointXYZRGB> table_segmenter;
    if(table_orientation(0) != 0)
    {
      table_segmenter.setAxis(table_orientation);
      table_segmenter.setModelType(pcl::SACMODEL_PARALLEL_PLANE);
    }
    else
      table_segmenter.setModelType(pcl::SACMODEL_PLANE);
    table_segmenter.setOptimizeCoefficients(true);

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

    //finding a normal perpendicular to the table
    if(table_orientation(0) == 0)
    {
      table_orientation = calucalateTableNormal(objects, inliers);
      visual_tools_->triggerBatchPublish();
    }
    if (inliers->indices.size () == 0)
    {
      //ROS_ERROR_STREAM_NAMED("ppc", "Could not estimate a planar model.");
      return;
    }


    std::cout << "perception running..." << std::endl;

    // Extract indices not from table
    eifilter.setInputCloud(objects);
    eifilter.setIndices(inliers);
    eifilter.setNegative(true);
    eifilter.filter(*not_table);
    //ROS_INFO_STREAM_NAMED("ppc", "final point cloud has " << not_table->width * not_table->height);
    not_table->header.frame_id = "camera_rgb_optical_frame";
    not_table_cloud_pub_.publish(not_table);
   /* if(find_basket)
    {
      return;
    }*/
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
    ec.setClusterTolerance(0.025);
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
    if(false)//find_basket)
    {
        ec.setMinClusterSize(1000);  // less than a wood cube
        ec.setMaxClusterSize(20000);  // a plate is lots
    }
    else
    {
        ec.setMinClusterSize(250);  // less than a wood cube
        ec.setMaxClusterSize(20000);  // a plate is lots
    }
    ec.setSearchMethod(tree);
    ec.setInputCloud(not_table);
    ec.extract(cluster_indices);
    pcl::PointXYZRGB min, max;
    /*
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr single_object (new pcl::PointCloud<pcl::PointXYZRGB>);
    for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin();
         it != cluster_indices.end();
         ++it)
    {
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

    }
    objects_cloud_pub_.publish(single_object);
    std::cout << "Object size:" << single_object->points.size() << std::endl;

    pcl::getMinMax3D(*single_object, min, max);
    double height0 = max.z-min.z;
    double depth0 = max.x-min.x;
    double width0 = max.y-min.y;
    visual_tools_->publishWireframeCuboid(Eigen::Affine3d::Identity(), depth0, width0, height0, rviz_visual_tools::CYAN);
    visual_tools_->triggerBatchPublish();
    return;*/
    //TODO: Old poses continue to be published?
    std::vector<Eigen::Affine3d> local_poses;
    object_poses_ = local_poses;
    std::size_t idx = 0;

    //int objects_found = cluster_indices.end() - cluster_indices.begin();
    object_labels.clear();
    Eigen::Vector4d useless_centroid;
    Eigen::Vector3d object_centroid, handle;
    tf::StampedTransform qr_transform;
    Eigen::Affine3d object_pose;
    visual_tools_->deleteAllMarkers();


    single_object_ll* segmented_objects = NULL;
    single_object_ll* iterator;
    //pcl::PointXYZRGB min, max;
    int unique_id = 0;
    //separate objects into a linked list to make processing more simple
    //perform object labeling
    //if unknown, attempt to separate clusters by color and re-label
    for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin();
         it != cluster_indices.end();
         ++it)
    {
      single_object_ll* new_object(new single_object_ll());
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
      if(!(label == "unknown" && only_blocks))
      {
      if(colored_block_detection)
      {
        //label = ObjectDetectionPtr->label_object(single_object);
        pcl::getMinMax3D(*single_object, min, max);
        double height = max.z-min.z;
        double depth = max.x-min.x;
        double width = max.y-min.y;
        //std::cout << label << " height: " << height << ", depth: " << depth << ", width: " << width;
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

      //object tracking
      std::ostringstream ss;
      single_object_ll* matching_centroid = near_centroid_object(object_centroid);
      std::string label = iterator->label;
      rviz_visual_tools::colors color = rviz_visual_tools::MAGENTA;
      if(matching_centroid)
      {
        ///bug: object's label gets pushed into tf transform publisher, but we removed it at the end of this fuction
        ///   this results in removed objects being sent to base, then back to where is was
        ss << matching_centroid->published_name;
        if(matching_centroid->label == "unknown" || timed_out(matching_centroid->timestamp,retest_object_seconds))
        {
          if(matching_centroid->label == "unknown" || label != "unknown" )
          {
            if(matching_centroid->label != label)
            {
              tf_visualizer_.publishTransform(Eigen::Affine3d::Identity(), base_frame, ss.str());
              ss.str("");
              if(one_of_each && label != "unknown" && label != "block")
              {
                  ss << label << "_position";
              }
              else
              {
                  ss << label << "_" << matching_centroid->id;
              }
            }
            update_tracked_object(matching_centroid, object_centroid, label, ss.str());
          }
          else if(timed_out(matching_centroid->timestamp,seconds_rename))
          {
            //ss << matching_centroid->published_name;
            tf_visualizer_.publishTransform(Eigen::Affine3d::Identity(), base_frame, ss.str());
            ss.str("");
            if(one_of_each && label != "unknown" && label != "block")
            {
                ss << label << "_position";
            }
            else
            {
                ss << label << "_" << iterator->id;
            }
            update_tracked_object(matching_centroid, object_centroid, label, ss.str());
          }
          else
          {
            label = matching_centroid->label;
          }
        }
        else
        {
          label = matching_centroid->label;
          //ss << matching_centroid->published_name;
        }
      }
      else
      {
        int id = get_unique_id();
        if(one_of_each && label != "unknown" && label != "block")
        {
            ss << label << "_position";
        }
        else
        {
            ss << label << "_" << id;
        }
        //std::cout << "object id" << id << std::endl;
        add_tracking_object(object_centroid, label, id, ss.str());
      }
      if(label == "unknown")
         color = rviz_visual_tools::CYAN;

      local_poses.push_back(object_pose);
      object_labels.push_back(ss.str());

      pcl::getMinMax3D(*single_object_transformed, min, max);
      double height = max.z-min.z;
      double depth = max.x-min.x;
      double width = max.y-min.y;
      visual_tools_->publishWireframeCuboid(object_pose, depth, width, height, color);

      if(publish_handle && label.find("cup") != std::string::npos)
      {

        double x,y,z;
        z = object_centroid[2];
        /*
        std::cout << "object_centroid:" << object_centroid[0] << ", " <<object_centroid[1] << ", " << object_centroid[2] <<std::endl;
        std::cout << "x_dif:" << abs(10000*(object_centroid[0]-max.x)) << ", x_diff" <<abs(10000*(object_centroid[0]-min.x)) <<std::endl;
        std::cout << "y_dif:" << abs(10000*(object_centroid[1]-max.y)) << ", y_diff" <<abs(10000*(object_centroid[1]-min.y)) <<std::endl;
        if(abs(10000*(object_centroid[0]-max.x)) > abs(10000*(object_centroid[0]-min.x)))
        {
            x = max.x;
        }
        else
        {
            x = min.x;
        }
        if(abs(10000*(object_centroid[1]-max.y)) > abs(10000*(object_centroid[1]-min.y)))
        {
            y = max.y;
        }
        else
        {
            y = min.y;
        }*/
        double x_diff, y_diff;
        double xloc, yloc;
        double max_diff=0;
        for (size_t i = 0; i < single_object_transformed->points.size (); ++i)
        {
            x_diff = std::pow(100*single_object_transformed->points[i].x - 100*object_centroid[0],2);
            y_diff = std::pow(100*single_object_transformed->points[i].y - 100*object_centroid[1],2);

            if((x_diff + y_diff) > max_diff)
            {
                max_diff = x_diff + y_diff;
                xloc = single_object_transformed->points[i].x;
                yloc = single_object_transformed->points[i].y;
            }
            if(max_diff > 45.0)
                break;
        }
        std::cout << "max handle distance: " << max_diff;
        handle << xloc,yloc,z;
        object_pose.translation() = handle;
        local_poses.push_back(object_pose);
        object_labels.push_back("handle");
      }

      //std::cout << "object ss: " << ss.str()  << std::endl;

      if(save_new && label == "unknown")
      {
        std::cout << ros::package::getPath("perception") << "/object_database/new/"<<ss.str()<<".pcd" << endl;
        pcl::io::savePCDFileASCII(ros::package::getPath("perception") + "/object_database/new/"+ss.str()+".pcd", *iterator->point_cloud);
      }

      objects_cloud_pub_.publish(iterator->point_cloud);

      iterator = iterator->next;
    }
    remove_outdated_objects();

    if(local_poses.size() > 0) objects_detected_ = true;

    object_poses_ = local_poses;
    visual_tools_->triggerBatchPublish();

    ROS_DEBUG_STREAM_NAMED("pcc","finished segmentation");

  }


  tf::Vector3 table_axis;
  geometry_msgs::PoseStamped pose;

  Eigen::Affine3d create_rotation_matrix(double ax, double ay, double az) {
    Eigen::Affine3d rx =
        Eigen::Affine3d(Eigen::AngleAxisd(ax, Eigen::Vector3d(1, 0, 0)));
    Eigen::Affine3d ry =
        Eigen::Affine3d(Eigen::AngleAxisd(ay, Eigen::Vector3d(0, 1, 0)));
    Eigen::Affine3d rz =
        Eigen::Affine3d(Eigen::AngleAxisd(az, Eigen::Vector3d(0, 0, 1)));
    //std::cout<< "rx " << rx.rotation() << endl;
    //std::cout<< "ry " << ry.rotation() << endl;
    //std::cout<< "rz " << rz.rotation() << endl;
    return rx * ry * rz ;
  }
  Eigen::Vector3f Perception::calucalateTableNormal(pcl::PointCloud<pcl::PointXYZRGB>::Ptr table, pcl::PointIndices::Ptr inliers)
  {
    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal> ());
    Eigen::Vector3f axis;
    pcl::NormalEstimationOMP<pcl::PointXYZRGB, pcl::Normal> norm_est;
    norm_est.setKSearch(10);
    norm_est.setIndices(inliers);
    norm_est.setInputCloud(table);
    norm_est.compute(*cloud_normals);
    double x=0, y=0, z=0;
    for(size_t i = 0; i<cloud_normals->points.size(); i++)
    {
      x += cloud_normals->points[i].normal[0];
      y += cloud_normals->points[i].normal[1];
      z += cloud_normals->points[i].normal[2];
      //tf::Vector3 axis_vector(cloud_normals->points[i].normal[0], cloud_normals->points[i].normal[1], cloud_normals->points[i].normal[2]);


    }
    axis(0) = x/cloud_normals->points.size();
    axis(1) = y/cloud_normals->points.size();
    axis(2) = z/cloud_normals->points.size();
    //std::cout << "averages, x: " <<x/cloud_normals->points.size()<< ", y: " << y/cloud_normals->points.size()<< ", z: " <<z/cloud_normals->points.size() << endl;

    geometry_msgs::Quaternion msg;
    // extracting surface normals

    tf::Vector3 axis_vector(x/cloud_normals->points.size(), y/cloud_normals->points.size(), z/cloud_normals->points.size());
    tf::Vector3 up_vector( 1.0, 0.0, 0.0); // (0,0,1)

    tf::Vector3 right_vector = axis_vector.cross(up_vector);
    right_vector.normalized();
    tf::Quaternion q(right_vector, -1.0*acos(axis_vector.dot(up_vector)));
    q.normalize();
    tf::quaternionTFToMsg(q, msg);

    //adding pose to pose array

    Eigen::Vector4d useless_centroid;
    Eigen::Vector3d table_centroid;

    pcl::compute3DCentroid(*table, *inliers, useless_centroid);
    table_centroid << useless_centroid(0), useless_centroid(1), useless_centroid(2); //x, y, z

    pose.pose.position.x = table_centroid(0);
    pose.pose.position.y = table_centroid(1);
    pose.pose.position.z = table_centroid(2);
    pose.pose.orientation = msg;


    visualization_msgs::Marker marker;
    marker.header.frame_id = "camera_rgb_optical_frame";
    marker.header.stamp = ros::Time();
    marker.ns = "my_namespace";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::ARROW;
    marker.action = visualization_msgs::Marker::MODIFY;
    marker.pose = pose.pose;
    marker.scale.x = 0.15;
    marker.scale.y = 0.01;
    marker.scale.z = 0.01;
    marker.color.a = 1.0; // Don't forget to set the alpha!
    marker.color.r = 0.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;
    visual_tools_->publishMarker(marker);

    return axis;
  }

  single_object_ll* Perception::get_colored_segments(pcl::PointCloud<pcl::PointXYZRGB>::Ptr single_object, int &unique_id)
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
    single_object_ll* return_object = NULL;
    single_object_ll* iterator;
    for (std::vector<pcl::PointIndices>::const_iterator it = clusters.begin();
         it != clusters.end();
         ++it)
    {
      single_object_ll* new_object(new single_object_ll());
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

      if(new_object->label == "unknown" && height < 0.025 && height > 0.015)
      {
        std::cout << "renaming object to block based on height" << endl;
        new_object->label = "block";
      }
      new_object->point_cloud = part_object;
      new_object->id = unique_id;
      unique_id++;
      std::cout << new_object->label << std::endl;
      //get_colored_segments_2(part_object, unique_id);
     // if(!(new_object->label == "unknown" && only_blocks))
      //{
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
      //}
    }
    return return_object;
  }


} // end namespace perception


int main(int argc, char** argv)
{
  ros::init(argc, argv, "perception");
  ros::AsyncSpinner spinner(3);
  spinner.start();

  int test = 1;
  perception::Perception tester(test);

  return 0;
}
