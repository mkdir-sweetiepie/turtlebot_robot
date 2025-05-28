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
  cv::Mat clone_mat;  // 이미지 클론을 위한 cv::Mat 객체

 private:
  Ui::MainWindowDesign* ui;
  bool detection_active_;  // OCR 탐지 활성화 여부

  void closeEvent(QCloseEvent* event);

 public Q_SLOTS:
  void slotUpdateImg();                                                                                 // 이미지 업데이트
  void slotUpdateOCRResult(const QString& text, bool detected, float confidence, int processing_time);  // OCR 결과 업데이트
  void slotUpdateCameraFPS(int fps);                                                                    // 카메라 FPS 업데이트
  void onStartDetection();                                                                              // OCR 탐지 시작
  void onStopDetection();                                                                               // OCR 탐지 중지
};

}  // namespace robot_vision

#endif  // ROBOT_VISION_MAIN_WINDOW_H