/*
 * Author(s) : Andy McEvoy ( mcevoy.andy@gmail.com )
 * Desc      : Simple pick and place for baxter and finger sensor
 * Created   : 2016 - 04 - 14
 */
#include <iostream>
#include <ros/ros.h>
#include <rviz_visual_tools/rviz_visual_tools.h>

#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>

#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/common/common.h>
#include <pcl/conversions.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/kdtree/kdtree_flann.h>

#include <Eigen/Core>

#include <iostream>

namespace ros_finger_sensor
{

class FingerSensorTest
{
private:
  std::string qr_marker_;

  ros::NodeHandle nh_;

  rviz_visual_tools::RvizVisualToolsPtr visual_tools_;

  tf::TransformListener tf_listener_;
  tf::StampedTransform qr_transform_;

  Eigen::Affine3d roi_pose_;
  Eigen::Affine3d qr_pose_;

  double roi_depth_, roi_width_, roi_height_;
  double qr_offset_x_, qr_offset_y_, qr_offset_z_;
  double roi_padding_x_, roi_padding_y_, roi_padding_z_;

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr roi_cloud_;
  //std::vector<pcl::PointIndices> cluster_indices;
  //pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> ec;

  ros::Publisher roi_cloud_pub_;
  ros::Subscriber raw_cloud_sub_;
  ros::Publisher filtered_pub_; // filtered point cloud for testing the algorithms
  ros::Publisher plane_pub_; // points that were recognized as part of the table
  ros::Publisher block_pose_pub_; // publishes to the block logic server

  double block_size;
  std::string arm_link;
  //std::vector<geometry_msgs::Pose> block_poses_;
  geometry_msgs::PoseArray block_poses_;


public:
  // Constructor
  FingerSensorTest(int test)
    : nh_("~")
  {
    // all the markers have names, probably want this in config so it's easily changed...
    qr_marker_ = "ar_marker_6";

    visual_tools_.reset(new rviz_visual_tools::RvizVisualTools("base", "visual_tools"));
    visual_tools_->deleteAllMarkers();

    // Setup to filter workspace
    // TODO: add these values to config file in case setup moves...
    roi_depth_ = 0.45; //region of interest (table + cubes)
    roi_width_ = 0.7;
    roi_height_ = 0.05;
    qr_offset_x_ = -0.075; //offsets from corner of table
    qr_offset_y_ = 0.035;
    qr_offset_z_ = -0.10;
    roi_padding_x_ = 0.5;
    roi_padding_y_ = 0.5;
    roi_padding_z_ = 0.5;

    arm_link = "/base_link";
    block_size = 0.04;
    block_poses_.header.stamp = ros::Time::now();
    block_poses_.header.frame_id = arm_link;

    showRegionOfInterest();

    // point cloud
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr roi_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    roi_cloud_ = roi_cloud;
    // Publish table segmented cloud
    roi_cloud_pub_ = nh_.advertise<pcl::PointCloud<pcl::PointXYZRGB> >("roi_cloud", 1);
    // Publish a point cloud of filtered data that was not part of table
    filtered_pub_ = nh_.advertise< pcl::PointCloud<pcl::PointXYZRGB> >("block_output", 1);
    // Publish a point cloud of data that was considered part of the plane
    plane_pub_ = nh_.advertise< pcl::PointCloud<pcl::PointXYZRGB> >("plane_output", 1);
    // Publish interactive markers for blocks
    block_pose_pub_ = nh_.advertise< geometry_msgs::PoseArray >("/", 1, true);
    //subscribe to raw point cloud
    raw_cloud_sub_ = nh_.subscribe("/camera/depth_registered/points", 1, &FingerSensorTest::processPointCloud, this);


    while(ros::ok())
    {

    }

  }

  void processPointCloud(const sensor_msgs::PointCloud2ConstPtr& msg)
  {
    // get point cloud in qr_marker_ coordinate frame
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr raw_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::fromROSMsg(*msg, *raw_cloud);
    tf_listener_.waitForTransform(qr_marker_, raw_cloud->header.frame_id, msg->header.stamp, ros::Duration(2.0));
    if (!pcl_ros::transformPointCloud(qr_marker_, *raw_cloud, *roi_cloud_, tf_listener_))
    {
      ROS_ERROR_STREAM_NAMED("processPointCloud","Error converting to desired frame");
    }

    segmentRegionOfInterest();
    int method = 2; //select method of segmentation
    segmentTableBlock(method);
    //roi_cloud_pub_.publish(roi_cloud_);

  }

  void segmentTableBlock(int method)
  {
    if (method == 1)
    {
      // TODO: would need a way to initialize this and store segmentation limit value.
      ROS_WARN_STREAM_NAMED("segmentTable","Table must be clear for this method!");
      pcl::PointXYZRGB min_point, max_point;
      pcl::getMinMax3D(*roi_cloud_, min_point, max_point);
      pcl::PassThrough<pcl::PointXYZRGB> pass_z;

      pass_z.setInputCloud(roi_cloud_);
      pass_z.setFilterFieldName("z");
      pass_z.setFilterLimits(0.0075, roi_height_);
      pass_z.filter(*roi_cloud_);
    }

    if (method == 2)
    {
      // http://www.pointclouds.org/documentation/tutorials/planar_segmentation.php
      pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
      pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_plane(new pcl::PointCloud<pcl::PointXYZRGB>());

      // Create the segmentation object for the planar model and set all the parameters
      pcl::SACSegmentation<pcl::PointXYZRGB> seg;
      seg.setOptimizeCoefficients(true);
      seg.setModelType(pcl::SACMODEL_PLANE);
      seg.setMethodType(pcl::SAC_RANSAC); // robustness estimator - RANSAC is simple
      seg.setMaxIterations(200);
      seg.setDistanceThreshold(0.005); // determines how close a point must be to the model in order to be considered an inlier

      int nr_points = roi_cloud_->points.size();

      // http://pointclouds.org/documentation/tutorials/extract_indices.php
      pcl::ExtractIndices<pcl::PointXYZRGB> extract;
      extract.setInputCloud(roi_cloud_);
      extract.setIndices(inliers);
      extract.setNegative(true);
      extract.filter(*roi_cloud_);

      // Segment cloud until there are less than 30% of points left
      while(roi_cloud_->points.size() > 0.3 * nr_points)
      {

        // Segment the largest planar component from the remaining cloud (find the table)
        seg.setInputCloud(roi_cloud_);
        seg.segment(*inliers, *coefficients);

        if(inliers->indices.size() == 0)
        {
          ROS_ERROR("[block detection] Could not estimate a planar model for the given dataset.");
          return;
        }

        // Extract the planar inliers from the input cloud
        pcl::ExtractIndices<pcl::PointXYZRGB> extract;
        extract.setInputCloud(roi_cloud_);
        extract.setIndices(inliers);
        extract.setNegative(false);

        // Write the planar inliers to disk
        extract.filter(*cloud_plane);

        // Remove the planar inliers, extract the rest
        extract.setNegative(true);
        extract.filter(*roi_cloud_);

        // Debug output - DTC
        // Show the contents of the inlier set, together with the estimated plane parameters, in ax+by+cz+d=0 form (general equation of a plane)
        std::cerr << "Model coefficients: " << coefficients->values[0] << " "
                  << coefficients->values[1] << " "
                  << coefficients->values[2] << " "
                  << coefficients->values[3] << std::endl;

      }

      std::vector<pcl::PointIndices> cluster_indices;
      pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> ec;
      ec.setClusterTolerance(0.005);
      ec.setMinClusterSize(100);
      ec.setMaxClusterSize(25000);
      //ec.setSearchMethod(tree);
      ec.setInputCloud(roi_cloud_);
      ec.extract(cluster_indices);

      // Publish point cloud data
      filtered_pub_.publish(roi_cloud_);
      plane_pub_.publish(cloud_plane);


      // for each cluster, see if it is a block
      for(size_t c = 0; c < cluster_indices.size(); ++c)
      {
        // find the outer dimensions of the cluster
        float xmin = 0; float xmax = 0;
        float ymin = 0; float ymax = 0;
        float zmin = 0; float zmax = 0;
        for(size_t i = 0; i < cluster_indices[c].indices.size(); i++)
        {
          int j = cluster_indices[c].indices[i];
          float x = roi_cloud_->points[j].x;
          float y = roi_cloud_->points[j].y;
          float z = roi_cloud_->points[j].z;
          if(i == 0)
          {
            xmin = xmax = x;
            ymin = ymax = y;
            zmin = zmax = z;
          }
          else
          {
            xmin = std::min(xmin, x);
            xmax = std::max(xmax, x);
            ymin = std::min(ymin, y);
            ymax = std::max(ymax, y);
            zmin = std::min(zmin, z);
            zmax = std::max(zmax, z);
          }
        }

        // Check if these dimensions make sense for the block size specified
        float xside = xmax-xmin;
        float yside = ymax-ymin;
        float zside = zmax-zmin;

        const float tol = 0.01; // 1 cm error tolerance
        // In order to be part of the block, xside and yside must be between
        // blocksize and blocksize*sqrt(2)
        // z must be equal to or smaller than blocksize
        if(xside > block_size-tol &&
           xside < block_size*sqrt(2)+tol &&
                   yside > block_size-tol &&
           yside < block_size*sqrt(2)+tol &&
                   zside > tol && zside < block_size+tol)
        {
          // If so, then figure out the position and the orientation of the block
          float angle = atan(block_size/((xside+yside)/2));

          if(yside < block_size)
            angle = 0.0;

          ROS_INFO_STREAM("[block detection] xside: " << xside << " yside: " << yside << " zside " << zside << " angle: " << angle);
          // Then add it to our set
          addBlock( xmin+(xside)/2.0, ymin+(yside)/2.0, zmax - block_size/2.0, angle);

        }
      }

      if(block_poses_.poses.size() > 0)
      {
        block_pose_pub_.publish(block_poses_);
        ROS_INFO("[block detection] Finished");
      }
      else
      {
        ROS_INFO("[block detection] Couldn't find any blocks this iteration!");
      }

    }
  }


  void addBlock(float x, float y, float z, float angle)
  {

    geometry_msgs::Pose block_pose;
    block_pose.position.x = x;
    block_pose.position.y = y;
    block_pose.position.z = z;

    Eigen::Quaternionf quat(Eigen::AngleAxis<float>(angle, Eigen::Vector3f(0,0,1)));

    block_pose.orientation.x = quat.x();
    block_pose.orientation.y = quat.y();
    block_pose.orientation.z = quat.z();
    block_pose.orientation.w = quat.w();

    // Discard noise
    if( block_pose.position.y > 10 || block_pose.position.y < -10 )
    {
      ROS_WARN_STREAM("Rejected block: " << block_pose );
    }

    ROS_INFO_STREAM("Added block: \n" << block_pose );
    //visual_tools->publishWireframeCuboid(block_pose, 0.4,0.4,0.4,rviz_visual_tools::RED);
    block_poses_.poses.push_back(block_pose);
  }

  void showRegionOfInterest()
  {
    // get tf to qr code
    ROS_DEBUG_STREAM_NAMED("constructor","waiting for qr transform to be published...");
    tf_listener_.waitForTransform("/base", qr_marker_, ros::Time(0), ros::Duration(1.0));
    tf_listener_.lookupTransform("/base", qr_marker_, ros::Time(0), qr_transform_);
    tf::transformTFToEigen(qr_transform_, qr_pose_);

    // add offsets and display region of interset
    Eigen::Affine3d pose_offset = Eigen::Affine3d::Identity();
    pose_offset.translation()[0] += roi_depth_ / 2.0 + qr_offset_x_;
    pose_offset.translation()[1] -= roi_width_ / 2.0 - qr_offset_y_;
    pose_offset.translation()[2] += roi_height_ / 2.0 + qr_offset_z_; // roi_pose_ in qr coord. frame
    roi_pose_ = qr_pose_ * pose_offset; // roi_pose_ in base coord. frame

    visual_tools_->publishAxisLabeled(roi_pose_, "roi_pose");
    visual_tools_->publishWireframeCuboid(roi_pose_, roi_depth_, roi_width_, roi_height_, rviz_visual_tools::CYAN);
    visual_tools_->publishWireframeCuboid(roi_pose_,
                                          roi_depth_ - 2 * roi_padding_x_,
                                          roi_width_ - 2 * roi_padding_y_,
                                          roi_height_ - 2 * roi_padding_z_,
                                          rviz_visual_tools::MAGENTA);
  }

  void segmentRegionOfInterest()
  {

    ROS_INFO("[start region segmentation] Starting with, %d points", (int) roi_cloud_->points.size());
    // Filter based on qr location
    pcl::PassThrough<pcl::PointXYZRGB> pass_x;
    pass_x.setInputCloud(roi_cloud_);
    pass_x.setFilterFieldName("x");
    pass_x.setFilterLimits(qr_offset_x_ - roi_padding_x_, roi_depth_ + qr_offset_x_ - roi_padding_x_);
    pass_x.filter(*roi_cloud_);

    pcl::PassThrough<pcl::PointXYZRGB> pass_y;
    pass_y.setInputCloud(roi_cloud_);
    pass_y.setFilterFieldName("y");
    pass_y.setFilterLimits(qr_offset_y_ - roi_padding_y_, roi_width_ + qr_offset_y_ - roi_padding_y_);
    pass_y.filter(*roi_cloud_);

    pcl::PassThrough<pcl::PointXYZRGB> pass_z;
    pass_z.setInputCloud(roi_cloud_);
    pass_z.setFilterFieldName("z");
    pass_z.setFilterLimits(qr_offset_z_ - roi_padding_z_, roi_height_ + qr_offset_z_ - roi_padding_z_);
    pass_z.filter(*roi_cloud_);

    // Check if any points remain
    if( roi_cloud_->points.size() == 0 )
    {
      ROS_ERROR("0 points left");
      return;
    }
    else
    {
      ROS_INFO("[start block detection] For now filtered, %d points left", (int) roi_cloud_->points.size());
    }

  }

};

}

int main(int argc, char *argv[])
{
  ROS_INFO_STREAM_NAMED("main","Starting perception server for finger sensor...");
  ros::init(argc, argv, "finger_sensor_test");
  ros::AsyncSpinner spinner(2);
  spinner.start();

  int test = 1;
  ros_finger_sensor::FingerSensorTest tester(test);

  return 0;
}
