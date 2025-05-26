// =============================================================================
// main_window.hpp
// =============================================================================
#ifndef robot_master_MAIN_WINDOW_H
#define robot_master_MAIN_WINDOW_H

#include <QMainWindow>
#include <QTimer>

#include "qnode.hpp"
#include "ui_mainwindow.h"
namespace robot_master {

class MainWindow : public QMainWindow {
  Q_OBJECT

 public:
  MainWindow(QWidget* parent = nullptr);
  ~MainWindow();

 private Q_SLOTS:
  void updateData();
  void appendLog(const QString& message);
  void updateWorkState(int state);

  // 로봇 조종 버튼들
  void on_forwardButton_pressed();
  void on_backwardButton_pressed();
  void on_leftButton_pressed();
  void on_rightButton_pressed();
  void on_stopButton_clicked();
  void on_emergencyStopButton_clicked();

  // 리프트 버튼들
  void on_liftUpButton_pressed();
  void on_liftDownButton_pressed();
  void on_liftUpButton_released();
  void on_liftDownButton_released();

  // 물품 검색
  void on_pushButton_findParcel_clicked();

 protected:
  void closeEvent(QCloseEvent* event);

 private:
  Ui::MainWindowDesign* ui;
  QNode* qnode;
  QTimer* lift_timer;
  double current_lift_height;
  bool lift_moving_up;
  bool lift_moving_down;

  void updateLiftHeight();
};

}  // namespace robot_master

#endif  // robot_master_MAIN_WINDOW_H