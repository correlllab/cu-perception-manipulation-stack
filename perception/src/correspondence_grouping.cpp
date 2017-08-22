/**
 * Author(s) : Rebecca Cox
 * Desc      : object recognition using Correspondence Grouping
 * Src       : http://pointclouds.org/documentation/tutorials/correspondence_grouping.php
 * Date      : 2017 - Feb - 02
 */

#include <perception/correspondence_grouping.h>
#include <ros/package.h>

#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

namespace object_detection
{
bool show_keypoints_ (false);
bool use_hough_ (true);
float model_ss_ (0.01f);
float scene_ss_ (0.0125f);
//float model_ss_ (0.01f);
//float scene_ss_ (0.03f);
float rf_rad_ (0.08f);
float descr_rad_ (0.08f);
float cg_size_ (0.05f);
float cg_thresh_ (10.0f);
int icp_max_iter_ (5);
float icp_corr_distance_ (0.005f);
float hv_clutter_reg_ (5.0f);
float hv_inlier_th_ (0.005f);
float hv_occlusion_th_ (0.01f);
float hv_rad_clutter_ (0.03f);
float hv_regularizer_ (3.0f);
float hv_rad_normals_ (0.05);
bool hv_detect_clutter_ (true);

std::string path_to_models = "/object_database/";

/**
 * `models` is the array of names of files in the "object_database" folder, without
 * the ".pcd" file extension. For example, if the folder looks like this:
 *
 *   object_database
 *     /plate.pcd
 *     /cup.pcd
 *
 * then models should be set to `{"plate", "cup"}`
 */
std::string models[] =         {"cup", "plate", "bowl_farther_half", "spoon"};
/**
 * Correspondences needed for each associated model.
 * ex: if `models = {"plate", "cup"}`, this can be set to `{100, 200}` for a
 *     required correspondence of 100 needed to match a plate and 200 to match a
 *     cup
 */
int correspondences_needed[] = {75,200,70,5};

ObjectDetection::ObjectDetection()
    : nh_("~")
  {
    ROS_INFO_STREAM_NAMED("constructor","starting ObjectDetection...");
    show_keypoints_ = true;
    use_hough_ = true;
    found_match_cloud_pub_ = nh_.advertise<pcl::PointCloud<pcl::PointXYZRGB> >("/correspondence_keypoints", 10);
    //publish surface normals of objects
    poseArrayPub = nh_.advertise<visualization_msgs::Marker>( "/normal_vectors", 0 );

    std::string path = ros::package::getPath("perception");
    path_to_models = path + path_to_models;
    models_linkedlist = NULL;
    if(sizeof(models) > 0)
    {
      load_model_objects();
    }


}

/* Code to load a stl model of the object for comparison. Correspondences were not being found, so this
 * method is not being used
 * pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGBA>());
    pcl::PolygonMesh triangles;
    (pcl::io::loadPolygonFileSTL(path_to_models + models[a] + "/meshes/poisson_mesh.stl", triangles) >= 0)
pcl::fromPCLPointCloud2(triangles.cloud, *cloud);
pcl::fromROSMsg(triangles.cloud, *cloud);
    */

void ObjectDetection::load_model_objects()
{
   model_object* new_node;
   pcl::PointCloud<PointType>::Ptr cloud;
   if(sizeof(models) <= 0) return; //no models to load

   for( unsigned int a = 0; a < sizeof(models)/sizeof(models[0]); a++ )
   {
     //std::cout << "sizeof(models)" << sizeof(models)/sizeof(models[0]) << std::endl;
     //std::cout << models[a] << endl;
     cloud.reset(new pcl::PointCloud<PointType> ());
     if( pcl::io::loadPCDFile (path_to_models + models[a] + ".pcd", *cloud) >= 0)
     {
       new_node = compute_descriptors(cloud);
       new_node->name = models[a];
       new_node->correspondence_needed = correspondences_needed[a];
       if(!models_linkedlist)
       {
         models_linkedlist = new_node;
       }
       else
       {
         model_object* iterator = models_linkedlist;
         while(iterator->next != NULL)
         {
           iterator = iterator->next;
         }
         iterator->next = new_node;

       }
      //  std::cout << "Added model " << new_node->name << std::endl;

     }
     else
       std::cout << "Failed to load model " << models[a]  << endl;
   }
}


std::string ObjectDetection::label_object(pcl::PointCloud<PointType>::Ptr unknown)
{
  model_object* unknown_descriptors = compute_descriptors(unknown);
  model_object* iterator = models_linkedlist;

  while(iterator != NULL)
  {

    if(is_object(unknown_descriptors, iterator))
      return iterator->name;

    iterator = iterator->next;
  }
  // pcl::io::savePCDFileASCII ("test_pcd.pcd", *unknown);
  return "unknown";
}

bool ObjectDetection::is_object(model_object* unknown, model_object* model)
{

  if(!model || !unknown)
  {
    std::cout << "Null object received. Object detection not happening!" << std::endl;
    return false;
  }
  found_match_cloud_pub_.publish(unknown->keypoints);
  //found_match_cloud_pub_.publish(model->keypoints);

  /**
   *  Find Model-Scene Correspondences with KdTree
   */
  pcl::CorrespondencesPtr model_unknown_corrs (new pcl::Correspondences ());
  pcl::KdTreeFLANN<DescriptorType> match_search;
  match_search.setInputCloud (model->descriptors);
  std::vector<int> model_good_keypoints_indices;
  std::vector<int> unknown_good_keypoints_indices;

  for (size_t i = 0; i < unknown->descriptors->size (); ++i)
  {
    std::vector<int> neigh_indices (1);
    std::vector<float> neigh_sqr_dists (1);
    if (!pcl_isfinite (unknown->descriptors->at (i).descriptor[0]))  //skipping NaNs
    {
      continue;
    }
    int found_neighs = match_search.nearestKSearch (unknown->descriptors->at (i), 1, neigh_indices, neigh_sqr_dists);
    if (found_neighs == 1 && neigh_sqr_dists[0] < 0.25f)
    {
      pcl::Correspondence corr (neigh_indices[0], static_cast<int> (i), neigh_sqr_dists[0]);
      model_unknown_corrs->push_back (corr);
      model_good_keypoints_indices.push_back (corr.index_query);
      unknown_good_keypoints_indices.push_back (corr.index_match);
    }
  }
  pcl::PointCloud<PointType>::Ptr model_good_kp (new pcl::PointCloud<PointType> ());
  pcl::PointCloud<PointType>::Ptr unknown_good_kp (new pcl::PointCloud<PointType> ());
  pcl::copyPointCloud (*model->keypoints, model_good_keypoints_indices, *model_good_kp);
  pcl::copyPointCloud (*unknown->keypoints, unknown_good_keypoints_indices, *unknown_good_kp);

  std::cout << "Correspondences of " << model_unknown_corrs->size () << " found with " << model->name << std::endl;

  /**
   *  Clustering
   */
  //std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f> > rototranslations;
  //std::vector < pcl::Correspondences > clustered_corrs;
/* NOTE: Issues with clustering. finding clusters was unsuccessful, even though we found plenty of
 * matching correspondences. This is only needed to find orientation. Just skipping this part for now
  if (use_hough_)
  {
    pcl::PointCloud<RFType>::Ptr cup_rf (new pcl::PointCloud<RFType> ());
    pcl::PointCloud<RFType>::Ptr unknown_rf (new pcl::PointCloud<RFType> ());

    pcl::BOARDLocalReferenceFrameEstimation<PointType, NormalType, RFType> rf_est;
    rf_est.setFindHoles (true);
    rf_est.setRadiusSearch (rf_rad_);

    rf_est.setInputCloud (cup_keypoints);
    rf_est.setInputNormals (cup_normals);
    rf_est.setSearchSurface (cup);
    rf_est.compute (*cup_rf);

    rf_est.setInputCloud (unknown_keypoints);
    rf_est.setInputNormals (unknown_normals);
    rf_est.setSearchSurface (unknown);
    rf_est.compute (*unknown_rf);

    //  Clustering
    pcl::Hough3DGrouping<PointType, PointType, RFType, RFType> clusterer;
    clusterer.setHoughBinSize (cg_size_);
    clusterer.setHoughThreshold (cg_thresh_);
    clusterer.setUseInterpolation (true);
    clusterer.setUseDistanceWeight (false);

    clusterer.setInputCloud (cup_keypoints);
    clusterer.setInputRf (cup_rf);
    clusterer.setSceneCloud (unknown_keypoints);
    clusterer.setSceneRf (unknown_rf);
    clusterer.setModelSceneCorrespondences (cup_unknown_corrs);

    clusterer.recognize (rototranslations, clustered_corrs);
  }
  else
  {
    pcl::GeometricConsistencyGrouping<PointType, PointType> gc_clusterer;
    gc_clusterer.setGCSize (cg_size_);
    gc_clusterer.setGCThreshold (cg_thresh_);

    gc_clusterer.setInputCloud (cup_keypoints);
    gc_clusterer.setSceneCloud (unknown_keypoints);
    gc_clusterer.setModelSceneCorrespondences (cup_unknown_corrs);

    gc_clusterer.recognize (rototranslations, clustered_corrs);
  }

  /**
   * Stop if no instances.
   * Uses a threshold for matching correspondance rather than the commented out clustering methods
   */
  if (/*rototranslations.size () <= 0 && */(model_unknown_corrs->size () < model->correspondence_needed ))
  {
    //std::cout << "No groups found " << std::endl;
    return false;
  }
  else
  {
     /* Commented code publishes pretty arrows that show the surface normals
    poseArray.poses.clear(); // Clear last block perception result
    poseArray.header.stamp = ros::Time::now();
    poseArray.header.frame_id = "camera_rgb_optical_frame";
    for(size_t i = 0; i<unknown->normals->points.size()+25; i=i+5)
      {
        //new_node->normals->points[i].x = new_node->raw_cloud->points[i].x;
        //new_node->normals->points[i].y = new_node->raw_cloud->points[i].y;
        //new_node->normals->points[i].z = new_node-    >raw_cloud->points[i].z;

        geometry_msgs::PoseStamped pose;
        geometry_msgs::Quaternion msg;

        // extracting surface normals
        tf::Vector3 axis_vector(unknown->normals->points[i].normal[0], unknown->normals->points[i].normal[1], unknown->normals->points[i].normal[2]);
        tf::Vector3 up_vector( 1.0, 0.0, 0.0); // (0,0,1)

        tf::Vector3 right_vector = axis_vector.cross(up_vector);
        right_vector.normalized();
        tf::Quaternion q(right_vector, -1.0*acos(axis_vector.dot(up_vector)));
        q.normalize();
        tf::quaternionTFToMsg(q, msg);

        //adding pose to pose array

        pose.pose.position.x = unknown->raw_cloud->points[i].x;
        pose.pose.position.y = unknown->raw_cloud->points[i].y;
        pose.pose.position.z = unknown->raw_cloud->points[i].z;
        pose.pose.orientation = msg;
        if (!(isinf(pose.pose.orientation.x) || isnan(pose.pose.orientation.x) ||
              isinf(pose.pose.orientation.y) || isnan(pose.pose.orientation.y) ||
              isinf(pose.pose.orientation.z) || isnan(pose.pose.orientation.z) ||
              isinf(pose.pose.orientation.w) || isnan(pose.pose.orientation.w) ))
        {
          visualization_msgs::Marker marker;
          marker.header.frame_id = "camera_rgb_optical_frame";
          marker.header.stamp = ros::Time();
          marker.ns = "my_namespace";
          marker.id = i;
          marker.type = visualization_msgs::Marker::ARROW;
          marker.action = visualization_msgs::Marker::MODIFY;
          marker.pose = pose.pose;
          marker.scale.x = 0.01;
          marker.scale.y = 0.001;
          marker.scale.z = 0.001;
          marker.color.a = 1.0; // Don't forget to set the alpha!
          marker.color.r = 0.0;
          marker.color.g = 1.0;
          marker.color.b = 0.0;
          poseArrayPub.publish( marker );
          //poseArray.poses.push_back(pose.pose);

        }
      }*/
    return true;
    /**
     * Generates clouds for each instances found, commented out because instances are not getting found

    //std::vector<pcl::PointCloud<PointType>::ConstPtr> instances;
    for (size_t i = 0; i < rototranslations.size (); ++i)
    {
      pcl::PointCloud<PointType>::Ptr rotated_model (new pcl::PointCloud<PointType> ());
      pcl::transformPointCloud (*cup, *rotated_model, rototranslations[i]);
      //instances.push_back (rotated_model);
      found_match_cloud_pub_.publish(rotated_model);
    }*/
  }
}

model_object* ObjectDetection::compute_descriptors(pcl::PointCloud<PointType>::Ptr point_cloud)
{
  model_object* new_node(new model_object());
  new_node->raw_cloud = point_cloud;
  new_node->descriptors.reset(new pcl::PointCloud<DescriptorType>);
  new_node->keypoints.reset(new pcl::PointCloud<PointType>);
  new_node->normals.reset(new pcl::PointCloud<NormalType>);
  //std::cout << "init new_node " << std::endl;
  /**
   * Compute Normals
   */
  pcl::NormalEstimationOMP<PointType, NormalType> norm_est;
  norm_est.setKSearch (10);
  norm_est.setInputCloud (new_node->raw_cloud);
  norm_est.compute (*new_node->normals);

  /**
   *  Downsample Clouds to Extract keypoints
   */
  pcl::UniformSampling<PointType> uniform_sampling;
  uniform_sampling.setInputCloud (new_node->raw_cloud);
  uniform_sampling.setRadiusSearch (model_ss_);
  //uniform_sampling.filter (*cup_keypoints);
  pcl::PointCloud<int> keypointIndices1;
  uniform_sampling.compute(keypointIndices1);
  pcl::copyPointCloud(*new_node->raw_cloud, keypointIndices1.points, *new_node->keypoints);
  //std::cout << "Model total points: " << new_node->raw_cloud->size () << "; Selected Keypoints: " << new_node->keypoints->size () << std::endl;
  new_node->keypoints->header.frame_id = "camera_rgb_optical_frame";
  /**
   *  Compute Descriptor for keypoints
   */
  pcl::SHOTEstimationOMP<PointType, NormalType, DescriptorType> descr_est;
  descr_est.setRadiusSearch (descr_rad_);
  descr_est.setInputCloud (new_node->keypoints);
  descr_est.setInputNormals (new_node->normals);
  descr_est.setSearchSurface (new_node->raw_cloud);
  descr_est.compute (*new_node->descriptors);
  new_node->next = NULL;
  return new_node;

}

} // end namespace object_detection
