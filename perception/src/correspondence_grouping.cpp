
#include <perception/correspondence_grouping.h>

namespace perception
{

ObjectDetection::ObjectDetection()
    : nh_("~")
  {
    ROS_INFO_STREAM_NAMED("constructor","starting ObjectDetection...");
    show_keypoints_ = true;
    use_hough_ = true;
    found_match_cloud_pub_ = nh_.advertise<pcl::PointCloud<pcl::PointXYZRGB> >("/correspondence_keypoints", 10);
    if(true) //tabletop detection
    {
      pcl::PointCloud<PointType>::Ptr temp ( new pcl::PointCloud<PointType> ());
      pcl::PointCloud<PointType>::Ptr temp_keypoints (new pcl::PointCloud<PointType> ());
      pcl::PointCloud<NormalType>::Ptr temp_normals (new pcl::PointCloud<NormalType> ());
      pcl::PointCloud<DescriptorType>::Ptr temp_descriptors (new pcl::PointCloud<DescriptorType> ());

      if( pcl::io::loadPCDFile ("cup.pcd", *temp) < 0)
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
      norm_est.setInputCloud (temp);
      norm_est.compute (*temp_normals);

      /**
       *  Downsample Clouds to Extract keypoints
       */
      pcl::UniformSampling<PointType> uniform_sampling;
      uniform_sampling.setInputCloud (temp);
      uniform_sampling.setRadiusSearch (model_ss_);
      //uniform_sampling.filter (*model_keypoints);
      pcl::PointCloud<int> keypointIndices1;
      uniform_sampling.compute(keypointIndices1);
      pcl::copyPointCloud(*temp, keypointIndices1.points, *temp_keypoints);
      std::cout << "Model total points: " << temp->size () << "; Selected Keypoints: " << temp_keypoints->size () << std::endl;

      /**
       *  Compute Descriptor for keypoints
       */
      pcl::SHOTEstimationOMP<PointType, NormalType, DescriptorType> descr_est;
      descr_est.setRadiusSearch (descr_rad_);

      descr_est.setInputCloud (temp_keypoints);
      descr_est.setInputNormals (temp_normals);
      descr_est.setSearchSurface (temp);
      descr_est.compute (*temp_descriptors);

      cup = temp;
      cup_keypoints = temp_keypoints;
      cup_normals = temp_normals;
      cup_descriptors = temp_descriptors;
    }

  }
bool ObjectDetection::is_cup(pcl::PointCloud<pcl::PointXYZRGB>::Ptr unknown)
  {
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
    if (rototranslations.size () <= 0)
    {
      return false;
    }
    else
    {
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

} // end namespace perception

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "object_detection");
  ros::AsyncSpinner spinner(1);
  spinner.start();

  int test = 1;
  perception::ObjectDetection tester;

  return 0;
}

