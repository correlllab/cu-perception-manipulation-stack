/**
 * Author(s) : Andy McEvoy
 * Desc      : Camera alignment tester
 * Date      : 2016 - May - 26
 */

#include <ros/ros.h>
#include <rviz_visual_tools/rviz_visual_tools.h>
#include <rviz_visual_tools/tf_visual_tools.h>

#include <std_msgs/Bool.h>

#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <tf_conversions/tf_eigen.h>

#include <Eigen/Core>

namespace camera_alignment
{

class CameraAlignmentTester
{
private:
  ros::NodeHandle nh_;
  ros::Subscriber align_sub_;

  rviz_visual_tools::RvizVisualToolsPtr visual_tools_;
  rviz_visual_tools::TFVisualTools tf_visualizer_;

  std::string qr_marker_;
  std::string base_cf_;
  std::string camera_cf_;

  tf::TransformListener tf_listener_;
  Eigen::Affine3d camera_pose_;
  Eigen::Affine3d camera_refined_pose_; // so callback isn't trying to modify variable while main loop is publishing
  std::vector<Eigen::Matrix4d> all_camera_matrix_poses_;
  bool refined_;
  bool previously_refined_;

public:
  CameraAlignmentTester(int test)
  : nh_("~")
  {
    ROS_INFO_STREAM_NAMED("constructor","starting CameraAlignmentTester...");

    qr_marker_ = "ar_marker_4";
    camera_cf_ = "camera_link";
    base_cf_ = "base";

    align_sub_ = nh_.subscribe("/alignment/doit", 1, &CameraAlignmentTester::alignCamera, this);

    visual_tools_.reset(new rviz_visual_tools::RvizVisualTools(base_cf_, "visual_tools"));
    visual_tools_->deleteAllMarkers();

    camera_pose_ = Eigen::Affine3d::Identity();
    refined_ = false;
    previously_refined_ = false;

    ros::Rate loop_rate(60);
    while(ros::ok())
    {
      if (refined_)
      {
        camera_pose_ = camera_refined_pose_;
        refined_ = false;
      }
      tf_visualizer_.publishTransform(camera_pose_, base_cf_, camera_cf_);
      loop_rate.sleep();
    }
  }   

  void alignCamera(const std_msgs::Bool &msg)
  {
    // TODO: need to add refined poses instead of replacing so estimate gets better. 
    // Add std::vector of poses and just keep the average?
    // The mean (centroid) minimizes the sum of square distances, so let's try doing that.
    if (msg.data)
    {
      ROS_DEBUG_STREAM_NAMED("ea","Alignment requested, enabling...");
      camera_refined_pose_ = refineCameraPose();
      all_camera_matrix_poses_.push_back(camera_refined_pose_.matrix());
      if (previously_refined_)
      {
        Eigen::Matrix4d avg = Eigen::Matrix4d::Zero();
        for (std::vector<Eigen::Matrix4d>::const_iterator it = all_camera_matrix_poses_.begin();
             it != all_camera_matrix_poses_.end();
             ++it)
        {
          avg += *it;
        }
        avg /= all_camera_matrix_poses_.size();
        camera_refined_pose_.matrix() = avg;
      }
      refined_ = true;
      previously_refined_ = true;
    }
  }

  Eigen::Affine3d refineCameraPose()
  {
    ROS_DEBUG_STREAM_NAMED("rcp","refining camera pose...");
    tf::StampedTransform qr_transform;
    Eigen::Affine3d qr_marker_poses[25];
    Eigen::Matrix4d avg_pose = Eigen::Matrix4d::Zero();
    
    // compute average qr pose over 25 readings...
    
    for (std::size_t i = 0; i < 25; i++)
    {
      ROS_DEBUG_STREAM_NAMED("rcp","getting frame " << i << " of 25");
      tf_listener_.waitForTransform(camera_cf_, qr_marker_, ros::Time(0), ros::Duration(1.0));
      try
      {
        tf_listener_.lookupTransform(camera_cf_, qr_marker_, ros::Time(0), qr_transform);
      }
      catch (tf::TransformException ex)
      {
        ROS_ERROR("%s", ex.what());
        ros::Duration(1.0).sleep();
        continue;
      }

      tf::transformTFToEigen(qr_transform, qr_marker_poses[i]);    
      ros::Duration(0.1).sleep();
    }

    for (std::size_t i = 0; i < 25; i++)
    {
      avg_pose += qr_marker_poses[i].matrix();
    }
    avg_pose /= 25;

    Eigen::Affine3d qr_pose; 
    qr_pose.matrix() = avg_pose; // qr pose in w.r.t. camera frame
    ROS_DEBUG_STREAM_NAMED("rcp","qr_pose = \n" << qr_pose.matrix());

    // compute pose of camera w.r.t. base frame
    tf::StampedTransform rh_transform;
    Eigen::Affine3d rh_pose;
    Eigen::Affine3d qr_location = Eigen::Affine3d::Identity();
    Eigen::Affine3d rh_to_qr = Eigen::Affine3d::Identity();
    Eigen::Affine3d camera_pose = Eigen::Affine3d::Identity();

    // TODO I think order is swapped here to?
    // arguments are (target, source, etc). Also shouldn't it use ros::Time::now()???
    tf_listener_.waitForTransform(base_cf_, "right_hand", ros::Time(0), ros::Duration(1.0));
    tf_listener_.lookupTransform(base_cf_, "right_hand", ros::Time(0), rh_transform);
    tf::transformTFToEigen(rh_transform, rh_pose);    

    // location of qr tag on laser cut plate
    rh_to_qr.translation()[0] -= 0.08;
    rh_to_qr.translation()[2] += 0.0028;
    // TODO isn't the order swapped? Same below.
    qr_location = rh_pose * rh_to_qr;
    visual_tools_->publishAxisLabeled(qr_location, "qr_location");

    camera_pose = qr_location * qr_pose.inverse();
    visual_tools_->publishAxisLabeled(camera_pose, "camera_pose");

    ROS_DEBUG_STREAM_NAMED("rcp", "camera_pose = \n" << camera_pose.matrix());
    return camera_pose;
  }

}; // end class CameraAlignmentTester
} // end namespace camera_alignment


int main(int argc, char** argv)
{
  ros::init(argc, argv, "camera_alignment");
  ros::AsyncSpinner spinner(2);
  spinner.start();

  int test = 1;
  camera_alignment::CameraAlignmentTester tester(test);

  return 0;
}
