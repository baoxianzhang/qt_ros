#include <ros/ros.h>
#include <ros/network.h>
#include <string>
#include <std_msgs/String.h>
#include <sstream>
#include "qnode.h"

namespace qt_ros
{
QNode::QNode(int argc, char** argv) : init_argc_(argc), init_argv_(argv)
{
}

QNode::~QNode()
{
  if (ros::isStarted())
  {
    ros::shutdown();  // explicitly needed since we use ros::start();
    ros::waitForShutdown();
  }
  wait();
}

bool QNode::init()
{
  ros::init(init_argc_, init_argv_, "qt_ros");
  if (!ros::master::check())
  {
    return false;
  }
  ros::start();  // explicitly needed since our nodehandle is going out of scope.
  rosCommsInit();
  start();
  return true;
}

bool QNode::init(const std::string& master_url, const std::string& host_url)
{
  std::map<std::string, std::string> remappings;
  remappings["__master"] = master_url;
  remappings["__hostname"] = host_url;
  ros::init(remappings, "qt_ros");
  if (!ros::master::check())
  {
    return false;
  }
  ros::start();  // explicitly needed since our nodehandle is going out of scope.
  rosCommsInit();
  start();
  return true;
}

void QNode::rosCommsInit()
{
  // TODO
  ros::NodeHandle n;
  // Add your ros communications here.
  chatter_publisher_ = n.advertise<std_msgs::String>("chatter", 1000);
}

void QNode::run()
{
  ros::Rate loop_rate(1);
  int count = 0;
  while (ros::ok())
  {
    std_msgs::String msg;
    std::stringstream ss;
    ss << "hello world " << count;
    msg.data = ss.str();
    chatter_publisher_.publish(msg);
    log(Info, std::string("I sent: ") + msg.data);
    ros::spinOnce();
    loop_rate.sleep();
    ++count;
  }
  std::cout << "Ros shutdown, proceeding to close the gui." << std::endl;
  Q_EMIT rosShutdown();  // used to signal the gui for a shutdown (useful to roslaunch)
}

void QNode::log(const LogLevel& level, const std::string& msg)
{
  logging_model_.insertRows(logging_model_.rowCount(), 1);
  std::stringstream logging_model_msg;
  switch (level)
  {
    case (Debug):
    {
      ROS_DEBUG_STREAM(msg);
      logging_model_msg << "[DEBUG] [" << ros::Time::now() << "]: " << msg;
      break;
    }
    case (Info):
    {
      ROS_INFO_STREAM(msg);
      logging_model_msg << "[INFO] [" << ros::Time::now() << "]: " << msg;
      break;
    }
    case (Warn):
    {
      ROS_WARN_STREAM(msg);
      logging_model_msg << "[INFO] [" << ros::Time::now() << "]: " << msg;
      break;
    }
    case (Error):
    {
      ROS_ERROR_STREAM(msg);
      logging_model_msg << "[ERROR] [" << ros::Time::now() << "]: " << msg;
      break;
    }
    case (Fatal):
    {
      ROS_FATAL_STREAM(msg);
      logging_model_msg << "[FATAL] [" << ros::Time::now() << "]: " << msg;
      break;
    }
  }
  QVariant new_row(QString(logging_model_msg.str().c_str()));
  logging_model_.setData(logging_model_.index(logging_model_.rowCount() - 1), new_row);
  Q_EMIT loggingUpdated();  // used to readjust the scrollbar
}

}  // namespace qt_ros
