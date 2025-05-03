/**
 * @file /include/robot_master/main_window.hpp
 *
 * @brief Qt based gui for robot_master.
 *
 * @date January 2025
 **/

#ifndef robot_master_MAIN_WINDOW_H
#define robot_master_MAIN_WINDOW_H

/*****************************************************************************
** Includes
*****************************************************************************/

#include <QDebug>
#include <QMainWindow>

#include "QIcon"
#include "qnode.hpp"
#include "ui_mainwindow.h"
#include <QTime>
#include <QScrollBar>
/*****************************************************************************
** Interface [MainWindow]
*****************************************************************************/
/**
 * @brief Qt central, all operations relating to the view part here.
 */

namespace robot_master {

class MainWindow : public QMainWindow {
  Q_OBJECT

 public:
  MainWindow(QWidget* parent = nullptr);
  ~MainWindow();
  QNode* qnode;

 private:
  Ui::MainWindowDesign* ui;
  void closeEvent(QCloseEvent* event);

 public Q_SLOTS:
  void updateData();
  void appendLog(const QString& message);
  void updateTaskState(int state);

  // 방향 버튼 핸들러
  void on_forwardButton_pressed();
  void on_backwardButton_pressed();
  void on_leftButton_pressed();
  void on_rightButton_pressed();
  void on_stopButton_clicked();
  void on_emergencyStopButton_clicked();

  // 리프트 버튼 핸들러
  void on_liftUpButton_clicked();
  void on_liftDownButton_clicked();

  // 검색 버튼 핸들러
  void on_pushButton_findParcel_clicked();
};

}  // namespace robot_master
#endif  // robot_master_MAIN_WINDOW_H