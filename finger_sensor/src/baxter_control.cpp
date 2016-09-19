/*
 * Author(s) : Andy McEvoy ( mcevoy.andy@gmail.com )
 * Desc      : Simple position control for baxter
 * Created   : 2016 - 04 - 21
 */

#include <ros/ros.h>
#include <rviz_visual_tools/rviz_visual_tools.h>

#include <sensor_msgs/JointState.h>
#include <baxter_core_msgs/SolvePositionIK.h>
#include <baxter_core_msgs/SolvePositionIKRequest.h>

#include <tf_conversions/tf_eigen.h>
#include <geometry_msgs/PoseStamped.h>

#include <Eigen/Core>
#include <eigen_conversions/eigen_msg.h>

namespace ros_finger_sensor
{

class BaxterControlTest
{
private:
  ros::NodeHandle nh_;
  ros::Publisher planning_state_pub_;
  ros::Subscriber block_pose_sub_;
  unsigned long int seq_;
  sensor_msgs::JointState planning_msg_;

  rviz_visual_tools::RvizVisualToolsPtr visual_tools_;

  double joint_limits_[15][2];

  ros::ServiceClient ik_client_right_;
  baxter_core_msgs::SolvePositionIK ik_srv_;

  //std::vector<geometry_msgs::Pose> block_poses_;
  std::vector<geometry_msgs::Pose> block_poses_;

public:
  //Constructor
  BaxterControlTest(int test)
    : nh_("~")
  {
    ROS_DEBUG_STREAM_NAMED("constructor","starting test " << test);

    visual_tools_.reset(new rviz_visual_tools::RvizVisualTools("base", "visual_tools"));
    visual_tools_->deleteAllMarkers();

    void setJointLimts();
    void initializePlanningMsg();



    // Set up subs/pubs
    seq_ = 0;
    planning_state_pub_ = nh_.advertise<sensor_msgs::JointState>("my_states", 20);
    // Subsribe to block poses
    block_pose_sub_ = nh_.subscribe("/finger_sensor_test/blockpose", 1, &BaxterControlTest::callback, this );

    // Set up services
    ik_client_right_ = nh_.serviceClient<baxter_core_msgs::SolvePositionIK>("/ExternalTools/right/PositionKinematicsNode/IKService");
    testIKServiceCall();


    while (ros::ok())
    {

    }
  }

  void callback(const geometry_msgs::PoseArray& msg)
  {
    block_poses_ = msg.poses;
  }

  void testIKServiceCall()
  {
    // define goal pose for right hand end effector
    Eigen::Affine3d goal_pose = Eigen::Affine3d::Identity();
    goal_pose *= Eigen::AngleAxisd(3.141593, Eigen::Vector3d::UnitY());
    goal_pose.translation()[0] = 0.6;
    goal_pose.translation()[1] = -0.5;
    goal_pose.translation()[2] = 0.1;
    visual_tools_->publishAxisLabeled(goal_pose, "goal_pose");

    // call ik service
    geometry_msgs::PoseStamped tf2_msg;
    tf2_msg.header.stamp = ros::Time::now();
    tf::poseEigenToMsg(goal_pose, tf2_msg.pose);

    ik_srv_.request.pose_stamp.push_back(tf2_msg);


    // TODO: not the right way to call this? compile error on this section...
    if (ik_client_right_.call(ik_srv_))
    {
      ROS_DEBUG_STREAM_NAMED("testIKServiceCall","ik service called successfully.");
    }
    else
    {
      ROS_WARN_STREAM_NAMED("testIKServiceCall","Failed to call ik service...");
    }

  }

  void publishPlanningState()
  {
    planning_msg_.header.seq = seq_;
    seq_++;
    planning_msg_.header.stamp = ros::Time::now();

    planning_state_pub_.publish(planning_msg_);
  }

  void maxMinLimits()
  {
    // just a simple loop going from min to max joint limits then resetting
    for (std::size_t i = 0; i < 15; i++)
    {
      double delta = (joint_limits_[i][1] - joint_limits_[i][0]) / 25.0;
      ROS_DEBUG_STREAM_NAMED("maxMinLimits","delta = " << delta);
      planning_msg_.position[i] += delta;
      if (planning_msg_.position[i] > joint_limits_[i][1])
      {
        planning_msg_.position[i] = joint_limits_[i][0];
      }
      ROS_DEBUG_STREAM_NAMED("maxMinLimits","position = " << i << ", value = " << planning_msg_.position[i]);
    }
  }

  void initializePlanningMsg()
  {

    for (int i = 0; i < 15; i++)
    {
      planning_msg_.position.push_back(0.0);
      planning_msg_.velocity.push_back(0.0);
      planning_msg_.effort.push_back(0.0);
    }

    // setup planning message
    planning_msg_.name.push_back("head_pan");
    planning_msg_.name.push_back("right_s0");
    planning_msg_.name.push_back("right_s1");
    planning_msg_.name.push_back("right_e0");
    planning_msg_.name.push_back("right_e1");
    planning_msg_.name.push_back("right_w0");
    planning_msg_.name.push_back("right_w1");
    planning_msg_.name.push_back("right_w2");
    planning_msg_.name.push_back("left_s0");
    planning_msg_.name.push_back("left_s1");
    planning_msg_.name.push_back("left_e0");
    planning_msg_.name.push_back("left_e1");
    planning_msg_.name.push_back("left_w0");
    planning_msg_.name.push_back("left_w1");
    planning_msg_.name.push_back("left_w2");

  }

  void setJointLimits()
  {
    // define limits (from baxter urdf)
    // head_pan
    joint_limits_[0][0] = -1.57079632679;
    joint_limits_[0][1] = 1.57079632679;
    //right_s0
    joint_limits_[1][0] = -1.70167993878;
    joint_limits_[1][1] = 1.7016799387;
    //right_s1
    joint_limits_[2][0] = -2.147;
    joint_limits_[2][1] = 1.047;
    //right_e0
    joint_limits_[3][0] = -3.05417993878;
    joint_limits_[3][1] = 3.05417993878;
    //right_e1
    joint_limits_[4][0] = -0.05;
    joint_limits_[4][1] = 2.618;
    //right_w0
    joint_limits_[5][0] = -3.059;
    joint_limits_[5][1] = 3.059;
    //right_w1
    joint_limits_[6][0] = -1.57079632679;
    joint_limits_[6][1] = 2.094;
    //right_w2
    joint_limits_[7][0] = -3.059;
    joint_limits_[7][1] = 3.059;
    //left_s0
    joint_limits_[8][0] = -1.70167993878;
    joint_limits_[8][1] = 1.70167993878;
    //left_s1
    joint_limits_[9][0] = -2.147;
    joint_limits_[9][1] = 1.047;
    //left_e0
    joint_limits_[10][0] = -3.05417993878;
    joint_limits_[10][1] = 3.05417993878;
    //left_e1
    joint_limits_[11][0] = -0.05;
    joint_limits_[11][1] = 2.618;
    //left_w0
    joint_limits_[12][0] = -3.059;
    joint_limits_[12][1] = 3.059;
    //left_w1
    joint_limits_[13][0] = -1.57079632679;
    joint_limits_[13][1] = 2.094;
    //left_w2
    joint_limits_[14][0] = -3.059;
    joint_limits_[14][1] = 3.059;

  }

};

}

int main(int argc, char *argv[])
{
  ROS_INFO_STREAM_NAMED("main","Starting baxter control test for finger sensor...");
  ros::init(argc, argv, "baxter_control_test");
  ros::AsyncSpinner spinner(2);
  spinner.start();

  int test = 1;
  ros_finger_sensor::BaxterControlTest tester(test);

}
