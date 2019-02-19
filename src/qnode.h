#ifndef QT_ROS_QNODE_HPP_
#define QT_ROS_QNODE_HPP_

// To workaround boost/qt4 problems that won't be bugfixed. Refer to
//    https://bugreports.qt.io/browse/QTBUG-22829
#ifndef Q_MOC_RUN
#include <ros/ros.h>
#endif
#include <string>
#include <QThread>
#include <QStringListModel>

namespace qt_ros
{
class QNode : public QThread
{
  Q_OBJECT
public:
  QNode(int argc, char** argv);
  virtual ~QNode();
  bool init();
  bool init(const std::string& master_url, const std::string& host_url);
  void run();
  void rosCommsInit();

  enum LogLevel
  {
    Debug,
    Info,
    Warn,
    Error,
    Fatal
  };

  QStringListModel* loggingModel()
  {
    return &logging_model_;
  }
  void log(const LogLevel& level, const std::string& msg);

Q_SIGNALS:
  void loggingUpdated();
  void rosShutdown();

private:
  int init_argc_;
  char** init_argv_;
  ros::Publisher chatter_publisher_;
  QStringListModel logging_model_;
  // NOTE: please not to add ros::NodeHandle here, or it will occur
  // "Couldn't find an AF_INET address for []" "
  // it occur because you should ros::init(argc, argv, "xxx"); first
  // use the pointer is ok, init after ros::init

  // ros::NodeHandle nh_;  // xxx no
  // ros::NodeHandle* nh_;  // xxx yes
};

}  // namespace qt_ros

#endif /* QT_ROS_QNODE_HPP_ */
