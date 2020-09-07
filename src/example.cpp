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

    return true;
  }

  int ExampleClassNode::readParameters()
  {
    return m_priv_nh.getParam("topic_sub", m_topic_sub) &&
           m_priv_nh.getParam("topic_pub", m_topic_pub);
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