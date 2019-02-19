#ifndef QT_ROS_MAIN_WINDOW_H
#define QT_ROS_MAIN_WINDOW_H

#include <QtWidgets/QMainWindow>
#include "ui_main_window.h"
#include "qnode.h"

// NOTE: when use ui file, you can also code for the ui
namespace qt_ros
{
class MainWindow : public QMainWindow
{
  Q_OBJECT

public:
  MainWindow(int argc, char** argv, QWidget* parent = 0);
  ~MainWindow();

  void readSettings();   // load up qt program settings at startup
  void writeSettings();  // save qt program settings when closing

  void closeEvent(QCloseEvent* event);  // Overloaded function
  void showNoMasterMessage();

public Q_SLOTS:
  /******************************************
  ** Auto-connections (connectSlotsByName())
  *******************************************/
  void on_actionAbout_triggered();
  void on_button_connect_clicked(bool check);
  void on_checkbox_use_environment_stateChanged(int state);

  /******************************************
  ** Manual connections
  *******************************************/
  void updateLoggingView();  // no idea why this can't connect automatically

private:
  Ui::MainWindowDesign ui_;
  QNode* qnode_;
};

}  // namespace qt_ros

#endif  // QT_ROS_MAIN_WINDOW_H
