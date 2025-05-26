#include "../include/robot_vision/main_window.hpp"

#include <QGroupBox>
#include <QHBoxLayout>
#include <QLabel>
#include <QPushButton>
#include <QVBoxLayout>

namespace robot_vision {

MainWindow::MainWindow(QWidget* parent) : QMainWindow(parent), ui(new Ui::MainWindowDesign), detection_active_(false) {
  ui->setupUi(this);

  QIcon icon("://ros-icon.png");
  this->setWindowIcon(icon);

  qnode = new QNode();
  move(820, 330);

  QObject::connect(qnode, &QNode::rosShutDown, this, &MainWindow::close);
  QObject::connect(qnode, &QNode::sigRcvImg, this, &MainWindow::slotUpdateImg);
  QObject::connect(qnode, &QNode::sigOCRResult, this, &MainWindow::slotUpdateOCRResult);
  QObject::connect(qnode, &QNode::sigCameraFPS, this, &MainWindow::slotUpdateCameraFPS);

  QObject::connect(ui->startOCRButton, &QPushButton::clicked, this, &MainWindow::onStartDetection);
  QObject::connect(ui->stopOCRButton, &QPushButton::clicked, this, &MainWindow::onStopDetection);

  ui->ocrStatusLabel->setText("대기 중");
  ui->fps_label->setText("0");
  ui->stopOCRButton->setEnabled(false);
}

void MainWindow::closeEvent(QCloseEvent* event) { QMainWindow::closeEvent(event); }

MainWindow::~MainWindow() { delete ui; }

void MainWindow::slotUpdateImg() {
  clone_mat = qnode->imgRaw->clone();
  cv::resize(clone_mat, clone_mat, cv::Size(760, 360), 0, 0, cv::INTER_CUBIC);

  QImage RGB_im((const unsigned char*)(clone_mat.data), clone_mat.cols, clone_mat.rows, QImage::Format_RGB888);
  ui->label->setPixmap(QPixmap::fromImage(RGB_im));

  delete qnode->imgRaw;
  qnode->imgRaw = nullptr;
  qnode->isreceived = false;
}

void MainWindow::slotUpdateCameraFPS(int fps) { ui->fps_label->setText(QString::number(fps)); }

void MainWindow::slotUpdateOCRResult(const QString& text, bool detected, float confidence, int processing_time) {
  if (detected) {
    QString result_text;
    if (processing_time > 0) {
      result_text = QString("인식됨: %1 (%2%) [%3초]").arg(text).arg(QString::number((int)(confidence * 100))).arg(QString::number(processing_time / 1000.0, 'f', 1));
    } else {
      result_text = QString("인식됨: %1 (%2%)").arg(text).arg(QString::number((int)(confidence * 100)));
    }

    ui->ocrStatusLabel->setText(result_text);
    ui->ocrStatusLabel->setStyleSheet("color: #a3be8c; font-weight: bold;");
  } else if (detection_active_) {
    if (processing_time > 0) {
      ui->ocrStatusLabel->setText(QString("탐지 중... [처리시간: %1초]").arg(QString::number(processing_time / 1000.0, 'f', 1)));
    } else {
      ui->ocrStatusLabel->setText("탐지 중...");
    }
    ui->ocrStatusLabel->setStyleSheet("color: #ebcb8b;");
  }
}

void MainWindow::onStartDetection() {
  detection_active_ = true;
  qnode->enableDetection(true);

  ui->startOCRButton->setEnabled(false);
  ui->stopOCRButton->setEnabled(true);
  ui->ocrStatusLabel->setText("OCR 인식 시작됨");
  ui->ocrStatusLabel->setStyleSheet("color: #a3be8c;");

  statusBar()->showMessage("OCR 물품 인식이 시작되었습니다 (5초 간격으로 처리)", 3000);
}

void MainWindow::onStopDetection() {
  detection_active_ = false;
  qnode->enableDetection(false);

  ui->startOCRButton->setEnabled(true);
  ui->stopOCRButton->setEnabled(false);
  ui->ocrStatusLabel->setText("대기 중");
  ui->ocrStatusLabel->setStyleSheet("color: #d8dee9;");

  statusBar()->showMessage("OCR 물품 인식이 중지되었습니다", 3000);
}

}  // namespace robot_vision