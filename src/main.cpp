#include <QtGui>
#include <QApplication>
#include "main_window.h"

int main(int argc, char** argv)
{
  QApplication app(argc, argv);
  qt_ros::MainWindow w(argc, argv);
  w.show();
  app.connect(&app, SIGNAL(lastWindowClosed()), &app, SLOT(quit()));
  int result = app.exec();

  return result;
}
