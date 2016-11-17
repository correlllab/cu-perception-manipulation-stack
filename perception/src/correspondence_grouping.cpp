
#include <perception/correspondence_grouping.h>

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
std::string path_to_models = "/home/rebecca/ros/sandbox_ws/src/cu-perception-manipulation-stack/perception/object_database/";
std::string models[] = {"cup", "block"};
//other option is to try a hue histogram
//http://stackoverflow.com/questions/3018313/algorithm-to-convert-rgb-to-hsv-and-hsv-to-rgb-in-range-0-255-for-both

//http://david.latotzky.de/content/Bachelor-Juehnemann.pdf
void transformed_color_histograms(pcl::PointCloud<pcl::PointXYZRGB> object)
{
  /*accumulator_set<double, stats<tag::variance> > acc_r, acc_b, acc_g;
  for_each(a_vec.begin(), a_vec.end(), bind<void>(ref(acc_r), _1));
  mean(acc_r)
  sqrt(variance(acc_r))*/
  //for each channel, r g b
  //    compute mean and standard deviation
  //    (R - mean)/deviation

  //compute histogram
}

ObjectDetection::ObjectDetection()
    : nh_("~")
  {
    ROS_INFO_STREAM_NAMED("constructor","starting ObjectDetection...");
    show_keypoints_ = true;
    use_hough_ = true;
    found_match_cloud_pub_ = nh_.advertise<pcl::PointCloud<pcl::PointXYZRGB> >("/correspondence_keypoints", 10);
    models_linkedlist = NULL;
    if(sizeof(models) > 0) //tabletop detection
    {
      compute_cup();
      load_model_objects();
    }


}

void ObjectDetection::load_model_objects()
{
   model_object* new_node;
   pcl::PointCloud<PointType>::Ptr cloud;//(new pcl::PointCloud<PointType> ());
   if(sizeof(models) <= 0) return;
   for( unsigned int a = 0; a < sizeof(models)/sizeof(models[0]); a++ )
   {
     std::cout << "sizeof(models)" << sizeof(models)/sizeof(models[0]) << std::endl;
     std::cout << models[a] << endl;
     cloud.reset(new pcl::PointCloud<PointType> ());
     if( pcl::io::loadPCDFile (path_to_models + models[a] + ".pcd", *cloud) >= 0)
     {
       std::cout << "loaded " << models[a] << std::endl;
       new_node = compute_descriptors(cloud);
       std::cout << "Model total points: " << new_node->raw_cloud->size () << "; Selected Keypoints: " << new_node->keypoints->size () << std::endl;
       new_node->name = models[a];
       std::cout << "name set" << std::endl;
       if(!models_linkedlist)
       {
        // std::cout << "7" << std::endl;
         models_linkedlist = new_node;
         //std::cout << "8" << std::endl;
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
       //std::cout << "9" << std::endl;
       //std::cout << "Added model " << new_node->name;

     }
     else
       std::cout << "Failed to load model" << endl;
   }
   std::cout << "10" << std::endl;
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
  found_match_cloud_pub_.publish(model->keypoints);

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

  //std::cout << "Correspondences found: " << model_unknown_corrs->size () << std::endl;

  /**
   *  Clustering
   */
  std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f> > rototranslations;
  std::vector < pcl::Correspondences > clustered_corrs;
/*
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
  if (/*rototranslations.size () <= 0 && */(model_unknown_corrs->size () < 5 ))
  {
    //std::cout << "No groups found " << std::endl;
    return false;
  }
  else
  {
    //std::cout << "MAAAAAAAAAAAAAAAAAAAAAAAAAAAAAATCH " << std::endl;
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
  //std::cout << "about to init node " << std::endl;
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
  //std::cout << "1" << std::endl;
  /**
   *  Compute Descriptor for keypoints
   */
  pcl::SHOTEstimationOMP<PointType, NormalType, DescriptorType> descr_est;
  descr_est.setRadiusSearch (descr_rad_);
  //std::cout << "2" << std::endl;

  descr_est.setInputCloud (new_node->keypoints);
  //std::cout << "3" << std::endl;
  descr_est.setInputNormals (new_node->normals);
  //std::cout << "4" << std::endl;
  descr_est.setSearchSurface (new_node->raw_cloud);
  //std::cout << "5" << std::endl;
  descr_est.compute (*new_node->descriptors);
  //std::cout << "6" << std::endl;
  new_node->next = NULL;
  //std::cout << "new node being returned" << std::endl;
  return new_node;

}

void ObjectDetection::compute_cup()
{
  cup.reset(new pcl::PointCloud<PointType>);
  cup_keypoints.reset(new pcl::PointCloud<PointType>);
  cup_normals.reset(new pcl::PointCloud<NormalType>);
  cup_descriptors.reset(new pcl::PointCloud<DescriptorType>);

  if( pcl::io::loadPCDFile ("/home/rebecca/ros/sandbox_ws/src/cu-perception-manipulation-stack/perception/object_database/cup.pcd", *cup) < 0)
  {
    std::cout << "Error loading model cloud." << std::endl;
    return;
  }
  cup_loaded = true;
  /**
   * Compute Normals
   */
  pcl::NormalEstimationOMP<PointType, NormalType> norm_est;
  norm_est.setKSearch (10);
  norm_est.setInputCloud (cup);
  norm_est.compute (*cup_normals);

  /**
   *  Downsample Clouds to Extract keypoints
   */
  pcl::UniformSampling<PointType> uniform_sampling;
  uniform_sampling.setInputCloud (cup);
  uniform_sampling.setRadiusSearch (model_ss_);
  //uniform_sampling.filter (*cup_keypoints);
  pcl::PointCloud<int> keypointIndices1;
  uniform_sampling.compute(keypointIndices1);
  pcl::copyPointCloud(*cup, keypointIndices1.points, *cup_keypoints);
  std::cout << "Model total points: " << cup->size () << "; Selected Keypoints: " << cup_keypoints->size () << std::endl;
  cup_keypoints->header.frame_id = "camera_rgb_optical_frame";
  /**
   *  Compute Descriptor for keypoints
   */
  pcl::SHOTEstimationOMP<PointType, NormalType, DescriptorType> descr_est;
  descr_est.setRadiusSearch (descr_rad_);

  descr_est.setInputCloud (cup_keypoints);
  descr_est.setInputNormals (cup_normals);
  descr_est.setSearchSurface (cup);
  descr_est.compute (*cup_descriptors);
  std::cout << "cup looooooaded." << std::endl;
}


bool ObjectDetection::is_cup(boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB> > unknown)
  {
    //compute_cup();
    if(!cup_loaded || !unknown)
    {
      std::cout << "Cup model not loaded or empty point cloud object received!" << std::endl;
      return false;
    }
    std::cout << "Model total points: " << cup->size () << "; Selected Keypoints: " << cup_keypoints->size () << std::endl;

    pcl::PointCloud<PointType>::Ptr unknown_keypoints (new pcl::PointCloud<PointType> ());
    pcl::PointCloud<NormalType>::Ptr unknown_normals (new pcl::PointCloud<NormalType> ());
    pcl::PointCloud<DescriptorType>::Ptr unknown_descriptors (new pcl::PointCloud<DescriptorType> ());

    /**
     * Compute Normals
     */
    pcl::NormalEstimationOMP<PointType, NormalType> norm_est;
    norm_est.setKSearch (10);
    norm_est.setInputCloud (unknown);
    norm_est.compute (*unknown_normals);

    /**
     *  Downsample Clouds to Extract keypoints
     */
    pcl::UniformSampling<PointType> uniform_sampling;
    uniform_sampling.setInputCloud (unknown);
    uniform_sampling.setRadiusSearch (scene_ss_);
    pcl::PointCloud<int> keypointIndices2;
    uniform_sampling.compute(keypointIndices2);
    pcl::copyPointCloud(*unknown, keypointIndices2.points, *unknown_keypoints);
    std::cout << "Unknown total points: " << unknown->size () << "; Selected Keypoints: " << unknown_keypoints->size () << std::endl;
    found_match_cloud_pub_.publish(unknown_keypoints);
    found_match_cloud_pub_.publish(cup_keypoints);
    /**
     *  Compute Descriptor for keypoints
     */
    pcl::SHOTEstimationOMP<PointType, NormalType, DescriptorType> descr_est;
    descr_est.setRadiusSearch (descr_rad_);
    descr_est.setInputCloud (unknown_keypoints);
    descr_est.setInputNormals (unknown_normals);
    descr_est.setSearchSurface (unknown);
    descr_est.compute (*unknown_descriptors);

    /**
     *  Find Model-Scene Correspondences with KdTree
     */
    pcl::CorrespondencesPtr cup_unknown_corrs (new pcl::Correspondences ());
    pcl::KdTreeFLANN<DescriptorType> match_search;
    match_search.setInputCloud (cup_descriptors);
    std::vector<int> cup_good_keypoints_indices;
    std::vector<int> unknown_good_keypoints_indices;

    for (size_t i = 0; i < unknown_descriptors->size (); ++i)
    {
      std::vector<int> neigh_indices (1);
      std::vector<float> neigh_sqr_dists (1);
      if (!pcl_isfinite (unknown_descriptors->at (i).descriptor[0]))  //skipping NaNs
      {
        continue;
      }
      int found_neighs = match_search.nearestKSearch (unknown_descriptors->at (i), 1, neigh_indices, neigh_sqr_dists);
      if (found_neighs == 1 && neigh_sqr_dists[0] < 0.25f)
      {
        pcl::Correspondence corr (neigh_indices[0], static_cast<int> (i), neigh_sqr_dists[0]);
        cup_unknown_corrs->push_back (corr);
        cup_good_keypoints_indices.push_back (corr.index_query);
        unknown_good_keypoints_indices.push_back (corr.index_match);
      }
    }
    pcl::PointCloud<PointType>::Ptr cup_good_kp (new pcl::PointCloud<PointType> ());
    pcl::PointCloud<PointType>::Ptr unknown_good_kp (new pcl::PointCloud<PointType> ());
    pcl::copyPointCloud (*cup_keypoints, cup_good_keypoints_indices, *cup_good_kp);
    pcl::copyPointCloud (*unknown_keypoints, unknown_good_keypoints_indices, *unknown_good_kp);

    std::cout << "Correspondences found: " << cup_unknown_corrs->size () << std::endl;

    /**
     *  Clustering
     */
    std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f> > rototranslations;
    std::vector < pcl::Correspondences > clustered_corrs;
/*
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
    if (/*rototranslations.size () <= 0 && */(cup_unknown_corrs->size () < 5 ))
    {
      std::cout << "No groups found " << std::endl;
      return false;
    }
    else
    {
      std::cout << "MAAAAAATCH " << std::endl;
      return true;
      /**
       * Generates clouds for each instances found
       */
      //std::vector<pcl::PointCloud<PointType>::ConstPtr> instances;
      for (size_t i = 0; i < rototranslations.size (); ++i)
      {
        pcl::PointCloud<PointType>::Ptr rotated_model (new pcl::PointCloud<PointType> ());
        pcl::transformPointCloud (*cup, *rotated_model, rototranslations[i]);
        //instances.push_back (rotated_model);
        found_match_cloud_pub_.publish(rotated_model);
      }

      return true;
    }
  }
/*
bool identify_objects(pcl::PointCloud<PointType>::Ptr unknown)
{
  pcl::PointCloud<PointType>::Ptr unknown_keypoints (new pcl::PointCloud<PointType> ());
  pcl::PointCloud<NormalType>::Ptr unknown_normals (new pcl::PointCloud<NormalType> ());
  pcl::PointCloud<DescriptorType>::Ptr unknown_descriptors (new pcl::PointCloud<DescriptorType> ());

  /**
   * Compute Normals

  pcl::NormalEstimationOMP<PointType, NormalType> norm_est;
  norm_est.setKSearch (10);
  norm_est.setInputCloud (unknown);
  norm_est.compute (*unknown_normals);

  /**
   *  Downsample Clouds to Extract keypoints

  pcl::UniformSampling<PointType> uniform_sampling;
  uniform_sampling.setInputCloud (unknown);
  uniform_sampling.setRadiusSearch (scene_ss_);
  pcl::PointCloud<int> keypointIndices2;
  uniform_sampling.compute(keypointIndices2);
  pcl::copyPointCloud(*unknown, keypointIndices2.points, *unknown_keypoints);
  std::cout << "Unknown total points: " << unknown->size () << "; Selected Keypoints: " << unknown_keypoints->size () << std::endl;

  /**
   *  Compute Descriptor for keypoints
   *
  pcl::SHOTEstimationOMP<PointType, NormalType, DescriptorType> descr_est;
  descr_est.setRadiusSearch (descr_rad_);
  descr_est.setInputCloud (unknown_keypoints);
  descr_est.setInputNormals (unknown_normals);
  descr_est.setSearchSurface (unknown);
  descr_est.compute (*unknown_descriptors);

  /**
   *  Find Model-Scene Correspondences with KdTree
   *
  pcl::CorrespondencesPtr cup_unknown_corrs (new pcl::Correspondences ());
  pcl::KdTreeFLANN<DescriptorType> match_search;
  match_search.setInputCloud (cup_descriptors);
  std::vector<int> cup_good_keypoints_indices;
  std::vector<int> unknown_good_keypoints_indices;

  for (size_t i = 0; i < unknown_descriptors->size (); ++i)
  {
    std::vector<int> neigh_indices (1);
    std::vector<float> neigh_sqr_dists (1);
    if (!pcl_isfinite (unknown_descriptors->at (i).descriptor[0]))  //skipping NaNs
    {
      continue;
    }
    int found_neighs = match_search.nearestKSearch (unknown_descriptors->at (i), 1, neigh_indices, neigh_sqr_dists);
    if (found_neighs == 1 && neigh_sqr_dists[0] < 0.25f)
    {
      pcl::Correspondence corr (neigh_indices[0], static_cast<int> (i), neigh_sqr_dists[0]);
      cup_unknown_corrs->push_back (corr);
      cup_good_keypoints_indices.push_back (corr.index_query);
      unknown_good_keypoints_indices.push_back (corr.index_match);
    }
  }
  pcl::PointCloud<PointType>::Ptr cup_good_kp (new pcl::PointCloud<PointType> ());
  pcl::PointCloud<PointType>::Ptr unknown_good_kp (new pcl::PointCloud<PointType> ());
  pcl::copyPointCloud (*cup_keypoints, cup_good_keypoints_indices, *cup_good_kp);
  pcl::copyPointCloud (*unknown_keypoints, unknown_good_keypoints_indices, *unknown_good_kp);

  std::cout << "Correspondences found: " << cup_unknown_corrs->size () << std::endl;

  /**
   *  Clustering
   *
  std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f> > rototranslations;
  std::vector < pcl::Correspondences > clustered_corrs;

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
   *
  if (rototranslations.size () <= 0)
  {
    return false;
  }
  else
  {
    /**
     * Generates clouds for each instances found
     *
    //std::vector<pcl::PointCloud<PointType>::ConstPtr> instances;
    for (size_t i = 0; i < rototranslations.size (); ++i)
    {
      pcl::PointCloud<PointType>::Ptr rotated_model (new pcl::PointCloud<PointType> ());
      pcl::transformPointCloud (*cup, *rotated_model, rototranslations[i]);
      //instances.push_back (rotated_model);
      found_match_cloud_pub_.publish(rotated_model);
    }

    return true;
  }
}*/

} // end namespace object_detection


