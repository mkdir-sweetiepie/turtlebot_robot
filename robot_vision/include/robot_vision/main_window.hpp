#ifndef ROBOT_VISION_MAIN_WINDOW_H
#define ROBOT_VISION_MAIN_WINDOW_H

#include <QCloseEvent>
#include <QIcon>
#include <QMainWindow>
#include <QString>
#include <opencv2/opencv.hpp>

#include "qnode.hpp"
#include "ui_mainwindow.h"

namespace robot_vision {

class MainWindow : public QMainWindow {
  Q_OBJECT

 public:
  MainWindow(QWidget* parent = nullptr);
  ~MainWindow();

  QNode* qnode;
  cv::Mat clone_mat;

 private:
  Ui::MainWindowDesign* ui;
  bool detection_active_;

  void closeEvent(QCloseEvent* event);

 public Q_SLOTS:
  void slotUpdateImg();
  void slotUpdateOCRResult(const QString& text, bool detected, float confidence, int processing_time);
  void slotUpdateCameraFPS(int fps);
  void onStartDetection();
  void onStopDetection();
};

}  // namespace robot_vision

#endif  // ROBOT_VISION_MAIN_WINDOW_H