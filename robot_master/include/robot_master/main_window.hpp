/**
 * @file /include/robot_master/main_window.hpp
 *
 * @brief Qt based gui for %(package)s.
 *
 * @date January 2025
 **/

#ifndef robot_master_MAIN_WINDOW_H
#define robot_master_MAIN_WINDOW_H

/*****************************************************************************
** Includes
*****************************************************************************/

#include <QMainWindow>

#include "QIcon"
#include "qnode.hpp"
#include "ui_mainwindow.h"
#include <QDebug>
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
  void updateData(void);
  void on_pushButton_clicked();
  void on_pushButton_findParcel_clicked();
};

}  // namespace robot_master
#endif  // robot_master_MAIN_WINDOW_H
