#include <QtWidgets>
#include <QMessageBox>
#include <iostream>
#include "main_window.h"

namespace qt_ros
{
using namespace Qt;

MainWindow::MainWindow(int argc, char** argv, QWidget* parent) : QMainWindow(parent), qnode_(new QNode(argc, argv))
{
  ui_.setupUi(this);  // Calling this incidentally connects all ui_'s triggers to on_...() callbacks in this class.
  QObject::connect(ui_.actionAbout_Qt, SIGNAL(triggered(bool)), qApp,
                   SLOT(aboutQt()));  // qApp is a global variable for the application

  readSettings();
  setWindowIcon(QIcon(":/images/icon.png"));
  ui_.tab_manager->setCurrentIndex(0);  // ensure the first tab is showing - qt-designer should have this already
                                        // hardwired, but often loses it (settings?).
  QObject::connect(qnode_, SIGNAL(rosShutdown()), this, SLOT(close()));

  ui_.view_logging->setModel(qnode_->loggingModel());
  QObject::connect(qnode_, SIGNAL(loggingUpdated()), this, SLOT(updateLoggingView()));

  if (ui_.checkbox_remember_settings->isChecked())
  {
    on_button_connect_clicked(true);
  }
}

MainWindow::~MainWindow()
{
}

void MainWindow::showNoMasterMessage()
{
  QMessageBox msgBox;
  msgBox.setText("Couldn't find the ros master.");
  msgBox.exec();
  close();
}

/*
 * These triggers whenever the button is clicked, regardless of whether it
 * is already checked or not.
 */

void MainWindow::on_button_connect_clicked(bool check)
{
  if (ui_.checkbox_use_environment->isChecked())
  {
    if (!qnode_->init())
    {
      showNoMasterMessage();
    }
    else
    {
      ui_.button_connect->setEnabled(false);
    }
  }
  else
  {
    if (!qnode_->init(ui_.line_edit_master->text().toStdString(), ui_.line_edit_host->text().toStdString()))
    {
      showNoMasterMessage();
    }
    else
    {
      ui_.button_connect->setEnabled(false);
      ui_.line_edit_master->setReadOnly(true);
      ui_.line_edit_host->setReadOnly(true);
      ui_.line_edit_topic->setReadOnly(true);
    }
  }
}

void MainWindow::on_checkbox_use_environment_stateChanged(int state)
{
  bool enabled;
  if (state == 0)
  {
    enabled = true;
  }
  else
  {
    enabled = false;
  }
  ui_.line_edit_master->setEnabled(enabled);
  ui_.line_edit_host->setEnabled(enabled);
  // ui_.line_edit_topic->setEnabled(enabled);
}

/*****************************************************************************
** Implemenation [Slots][manually connected]
*****************************************************************************/

/**
 * This function is signalled by the underlying model. When the model changes,
 * this will drop the cursor down to the last line in the QListview to ensure
 * the user can always see the latest log message.
 */
void MainWindow::updateLoggingView()
{
  ui_.view_logging->scrollToBottom();
}

/*****************************************************************************
** Implementation [Menu]
*****************************************************************************/

void MainWindow::on_actionAbout_triggered()
{
  QMessageBox::about(this, tr("About ..."),
                     tr("<h2>PACKAGE_NAME Test Program 0.10</h2><p>Copyright Yujin Robot</p><p>This package needs an "
                        "about description.</p>"));
}

/*****************************************************************************
** Implementation [Configuration]
*****************************************************************************/

void MainWindow::readSettings()
{
  QSettings settings("Qt-Ros Package", "qt_ros");
  restoreGeometry(settings.value("geometry").toByteArray());
  restoreState(settings.value("windowState").toByteArray());
  QString master_url = settings.value("master_url", QString("http://192.168.1.2:11311/")).toString();
  QString host_url = settings.value("host_url", QString("192.168.1.3")).toString();
  // QString topic_name = settings.value("topic_name", QString("/chatter")).toString();
  ui_.line_edit_master->setText(master_url);
  ui_.line_edit_host->setText(host_url);
  // ui_.line_edit_topic->setText(topic_name);
  bool remember = settings.value("remember_settings", false).toBool();
  ui_.checkbox_remember_settings->setChecked(remember);
  bool checked = settings.value("use_environment_variables", false).toBool();
  ui_.checkbox_use_environment->setChecked(checked);
  if (checked)
  {
    ui_.line_edit_master->setEnabled(false);
    ui_.line_edit_host->setEnabled(false);
    // ui_.line_edit_topic->setEnabled(false);
  }
}

void MainWindow::writeSettings()
{
  QSettings settings("Qt-Ros Package", "qt_ros");
  settings.setValue("master_url", ui_.line_edit_master->text());
  settings.setValue("host_url", ui_.line_edit_host->text());
  // settings.setValue("topic_name",ui_.line_edit_topic->text());
  settings.setValue("use_environment_variables", QVariant(ui_.checkbox_use_environment->isChecked()));
  settings.setValue("geometry", saveGeometry());
  settings.setValue("windowState", saveState());
  settings.setValue("remember_settings", QVariant(ui_.checkbox_remember_settings->isChecked()));
}

void MainWindow::closeEvent(QCloseEvent* event)
{
  writeSettings();
  QMainWindow::closeEvent(event);
}

}  // namespace qt_ros
