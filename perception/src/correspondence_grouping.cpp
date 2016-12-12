
#include <perception/correspondence_grouping.h>
#include <ros/package.h>

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

//std::string path_to_models = "/home/rebecca/ros/sandbox_ws/src/cu-perception-manipulation-stack/perception/object_database/";//ycb/";
std::string path_to_models = "/object_database/";
std::string models[] = {"cup","block","plate","bowl"};
int correspondences_needed[] = {50, 7, 150, 100};

ObjectDetection::ObjectDetection()
    : nh_("~")
  {
    ROS_INFO_STREAM_NAMED("constructor","starting ObjectDetection...");
    show_keypoints_ = true;
    use_hough_ = true;
    found_match_cloud_pub_ = nh_.advertise<pcl::PointCloud<pcl::PointXYZRGB> >("/correspondence_keypoints", 10);
    std::string path = ros::package::getPath("perception");
    path_to_models = path + path_to_models;
    models_linkedlist = NULL;
    cout << path_to_models << endl;
    if(sizeof(models) > 0)
    {
      load_model_objects();
    }


}

/* Code to load a 3D model of the object for comparison. Correspondences were not being found, so this
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
       //std::cout << "Added model " << new_node->name;

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

  std::cout << "Correspondences found: " << model_unknown_corrs->size () << std::endl;

  /**
   *  Clustering
   */
  //std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f> > rototranslations;
  //std::vector < pcl::Correspondences > clustered_corrs;
/* Issues with clustering. finding clusters was unsuccessful, even though we found plenty of
 * matching correspondences. Just skipping this part for not
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
   * Stop if no instances
   */
  if (/*rototranslations.size () <= 0 && */(model_unknown_corrs->size () < model->correspondence_needed ))
  {
    std::cout << "No groups found " << std::endl;
    return false;
  }
  else
  {
    std::cout << "MAAAAAAAAAAAAAAAAAAAAAAAAAAAAAATCH " << std::endl;
    return true;
    /**
     * Generates clouds for each instances found

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
