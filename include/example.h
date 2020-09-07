// ros
#include <ros/ros.h>
#include <std_msgs/Header.h>

namespace example_namespace
{

  class ExampleClassNode
  {
  private:
    int initialize();
    int readParameters();

    ros::NodeHandle m_nh;
    ros::NodeHandle m_priv_nh;
    ros::Subscriber m_sub_human_list;
    ros::Publisher m_pub_human_list;

    std::string m_topic_sub;
    std::string m_topic_pub;

    bool m_running;

  public:
    ExampleClassNode(const ros::NodeHandle &nh = ros::NodeHandle(), const ros::NodeHandle &priv_nh = ros::NodeHandle("~"));
    ~ExampleClassNode() = default;
    bool start();
    bool stop();
  };

} // namespace example_namespace
