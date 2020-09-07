// ros
#include <ros/ros.h>
#include <std_msgs/Header.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

namespace example_namespace
{

  class ExampleClassNode
  {
  private:
    int initialize();
    int readParameters();

    ros::NodeHandle m_nh;
    ros::NodeHandle m_priv_nh;
    ros::Subscriber m_sub_camera_info;
    image_transport::ImageTransport m_it;
    image_transport::Subscriber m_it_sub_image;
    image_transport::Publisher m_it_pub_image;

    std::string m_topic_sub_image;
    std::string m_topic_sub_camera_info;
    std::string m_topic_pub_image;

    bool m_running;

    void subscriberCallbackImage(const sensor_msgs::ImageConstPtr &msg_image);
    void subscriberCallbackCameraInfo(const sensor_msgs::CameraInfoConstPtr &msg_camera_info);

  public:
    ExampleClassNode(const ros::NodeHandle &nh = ros::NodeHandle(), const ros::NodeHandle &priv_nh = ros::NodeHandle("~"));
    ~ExampleClassNode() = default;
    bool start();
    bool stop();
  };

} // namespace example_namespace
