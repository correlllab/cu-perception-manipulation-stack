#include <ros/ros.h>

#include <eigen_conversions/eigen_msg.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>
#include <tf/transform_datatypes.h>

#include <pcl/io/ply_io.h>


class ObjectRecognizer
{
  ros::NodeHandle nh_;
  ros::Subscriber objects_cloud_pub_;


}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "object_recognition");
  ros::AsyncSpinner spinner(1);
  spinner.start();

  perception::PerceptionTester tester(test);

  return 0;
}
