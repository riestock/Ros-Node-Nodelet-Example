// ros
#include <ros/ros.h>
#include <std_msgs/Header.h>
#include <std_srvs/Trigger.h>

namespace example_namespace
{

  class ExampleClassNode
  {
  private:
    int initialize();
    int readParameters();
    void initializeSubscribers();
    void initializePublishers();
    void initializeServices();
    void subscriberCallback(const std_msgs::HeaderConstPtr &msg);
    void publisherCallback(const std_msgs::HeaderConstPtr &msg);
    bool serviceCallback(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);

    ros::NodeHandle m_nh;
    ros::NodeHandle m_priv_nh;
    ros::Subscriber m_sub;
    ros::Publisher m_pub;
    ros::ServiceServer m_service;

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
