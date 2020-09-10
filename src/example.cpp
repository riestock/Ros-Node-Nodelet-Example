#include "example.h"
#include <nodelet/nodelet.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <compressed_image_transport/compressed_subscriber.h>

namespace example_namespace
{

  ExampleClassNode::ExampleClassNode(const ros::NodeHandle &nh_, const ros::NodeHandle &priv_nh_) : m_nh(nh_),
                                                                                                    m_priv_nh(priv_nh_),
                                                                                                    m_it(nh_),
                                                                                                    m_topic_sub_image("/image_raw"),
                                                                                                    m_topic_sub_camera_info("/camera_info"),
                                                                                                    m_topic_pub_image("/pub_topic"),
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
    return m_priv_nh.getParam("topic_sub_image", m_topic_sub_image) &&
           m_priv_nh.getParam("topic_sub_camera_info", m_topic_sub_camera_info) &&
           m_priv_nh.getParam("topic_pub_image", m_topic_pub_image);
  }

  void ExampleClassNode::initializeSubscribers()
  {
    ROS_INFO("[ExampleClassNode] Initializing Subscribers");

    image_transport::TransportHints hints("compressed");
    m_it_sub_image = m_it.subscribe(m_topic_sub_image, 1, &example_namespace::ExampleClassNode::subscriberCallbackImage, this);
    m_sub_camera_info = m_nh.subscribe(m_topic_sub_camera_info, 1, &example_namespace::ExampleClassNode::subscriberCallbackCameraInfo, this);
  }

  void ExampleClassNode::initializePublishers()
  {
    ROS_INFO("[ExampleClassNode] Initializing Publishers");
    m_it_pub_image = m_it.advertise(m_topic_pub_image, 1);
  }

  void ExampleClassNode::initializeServices()
  {
    ROS_INFO("[ExampleClassNode] Initializing Services");
    m_service = m_nh.advertiseService("example_node_service", &example_namespace::ExampleClassNode::serviceCallback, this);
  }

  void ExampleClassNode::subscriberCallbackImage(const sensor_msgs::ImageConstPtr &msg)
  {
    ROS_INFO_ONCE("[ExampleClassNode] First time subscriberCallbackImage being called!");

    try
    {
      m_cv_image_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception &e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

    // // Draw an example circle on the video stream
    if (m_cv_image_ptr->image.rows > 60 && m_cv_image_ptr->image.cols > 60)
      cv::circle(m_cv_image_ptr->image, cv::Point(50, 50), 10, CV_RGB(255, 0, 0));

    // // Output modified video stream
    m_it_pub_image.publish(m_cv_image_ptr->toImageMsg());
  }

  void ExampleClassNode::subscriberCallbackCameraInfo(const sensor_msgs::CameraInfoConstPtr &msg)
  {
    ROS_INFO_ONCE("[ExampleClassNode] First time subscriberCallbackCameraInfo being called!");

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