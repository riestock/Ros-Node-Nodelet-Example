#include "example.h"
#include <nodelet/nodelet.h>

namespace example_namespace
{

  ExampleClassNode::ExampleClassNode(const ros::NodeHandle &nh_, const ros::NodeHandle &priv_nh_) : m_nh(nh_),
                                                                                                    m_priv_nh(priv_nh_),
                                                                                                    m_topic_sub("/sub_topic"),
                                                                                                    m_topic_pub("/pub_topic"),
                                                                                                    m_running(false)

  {
  }

  bool ExampleClassNode::start()
  {
    if (m_running)
    {
      ROS_INFO("[ExampleClassNode] is already running!");
      return false;
    }
    if (!initialize())
    {
      ROS_ERROR("[ExampleClassNode] Initialization failed!");
      return false;
    }

    m_running = true;
    return true;
  }

  bool ExampleClassNode::stop()
  {
    if (!m_running)
    {
      ROS_WARN("[ExampleClassNode] is not running!");
      return false;
    }

    m_nh.shutdown();
    m_priv_nh.shutdown();

    m_running = false;
    return true;
  }

  int ExampleClassNode::initialize()
  {
    if (!readParameters())
    {
      ROS_ERROR("[ExampleClassNode] Could not read parameters!");
    }

    initializeSubscribers();
    initializePublishers();
    initializeServices();

    return true;
  }

  int ExampleClassNode::readParameters()
  {
    return m_priv_nh.getParam("topic_sub", m_topic_sub) &&
           m_priv_nh.getParam("topic_pub", m_topic_pub);
  }

  void ExampleClassNode::initializeSubscribers()
  {
    ROS_INFO("[ExampleClassNode] Initializing Subscribers");
    m_sub = m_nh.subscribe(m_topic_sub, 1, &example_namespace::ExampleClassNode::subscriberCallback, this);
  }

  void ExampleClassNode::initializePublishers()
  {
    ROS_INFO("[ExampleClassNode] Initializing Publishers");
    m_pub = m_nh.advertise<std_msgs::Header>(m_topic_pub, 1);
  }

  void ExampleClassNode::initializeServices()
  {
    ROS_INFO("[ExampleClassNode] Initializing Services");
    m_service = m_nh.advertiseService("example_node_service", &example_namespace::ExampleClassNode::serviceCallback, this);
  }

  void ExampleClassNode::subscriberCallback(const std_msgs::HeaderConstPtr &msg)
  {
    ROS_INFO_ONCE("[ExampleClassNodelet] Firs time subscriberCallback being called!");

    // todo
  }

  void ExampleClassNode::publisherCallback(const std_msgs::HeaderConstPtr &msg)
  {
    ROS_INFO_ONCE("[ExampleClassNodelet] Firs time publisherCallback being called!");

    // todo
  }

  bool ExampleClassNode::serviceCallback(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res)
  {
    ROS_INFO("[ExampleClassNode] ServiceCallback being called!");

    // todo

    res.success = true;
    res.message = ros::NodeHandle().getNamespace();
    return true;
  }

  class ExampleClassNodelet : public nodelet::Nodelet
  {
  private:
    ExampleClassNode *example_class_node_ptr;

  public:
    ExampleClassNodelet() : Nodelet(), example_class_node_ptr(nullptr)
    {
      ROS_INFO("[ExampleClassNodelet] Constructor call");
    }

    ~ExampleClassNodelet() override
    {
      ROS_INFO("[ExampleClassNodelet] Destructor call");
      if (example_class_node_ptr)
      {
        ROS_INFO("[ExampleClassNodelet] Have something to destruct");
        example_class_node_ptr->stop();
        delete example_class_node_ptr;
      }
    }

    void onInit() override
    {
      ROS_INFO("[ExampleClassNodelet] onInit");
      example_class_node_ptr = new ExampleClassNode(getNodeHandle(), getPrivateNodeHandle());
      if (!example_class_node_ptr->start())
      {
        delete example_class_node_ptr;
        example_class_node_ptr = nullptr;
        throw nodelet::Exception("[ExampleClassNodelet] Could not start nodelet");
      }
    }
  };

} // namespace example_namespace

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(example_namespace::ExampleClassNodelet, nodelet::Nodelet)