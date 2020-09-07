#include <ros/ros.h>
#include "example.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "openpose_class_node", ros::init_options::AnonymousName);

  if (!ros::ok())
  {
    ROS_ERROR("ros::ok failed!");
    return -1;
  }

  example_namespace::ExampleClassNode exampleClassNode;
  if (exampleClassNode.start())
  {
    ros::spin();

    exampleClassNode.stop();
  }

  ros::shutdown();
  return 0;
}
