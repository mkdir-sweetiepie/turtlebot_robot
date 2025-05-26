// =============================================================================
// src/main_window.cpp (수정된 리프트 버튼 핸들러)
// =============================================================================
#include "../include/robot_master/main_window.hpp"

#include <QCloseEvent>
#include <QMessageBox>
#include <QScrollBar>
#include <QString>
#include <QTime>

namespace robot_master {

MainWindow::MainWindow(QWidget* parent) : QMainWindow(parent), ui(new Ui::MainWindowDesign), current_lift_height(0.0), lift_moving_up(false), lift_moving_down(false) {
  ui->setupUi(this);

  QIcon icon("://ros-icon.png");
  this->setWindowIcon(icon);

  qnode = new QNode();

  // 리프트 높이 업데이트용 타이머 (UI 표시용)
  lift_timer = new QTimer(this);
  connect(lift_timer, &QTimer::timeout, this, &MainWindow::updateLiftHeight);
  lift_timer->start(50);  // 50ms마다 업데이트

  move(820, 0);

  // 시그널 연결
  QObject::connect(qnode, SIGNAL(rosShutDown()), this, SLOT(close()));
  QObject::connect(qnode, SIGNAL(dataReceived()), this, SLOT(updateData()));
  QObject::connect(qnode, SIGNAL(logMessage(QString)), this, SLOT(appendLog(QString)));
  QObject::connect(qnode, SIGNAL(workStateChanged(int)), this, SLOT(updateWorkState(int)));

  appendLog("시스템이 초기화되었습니다");
}

MainWindow::~MainWindow() { delete ui; }

void MainWindow::closeEvent(QCloseEvent* event) { QMainWindow::closeEvent(event); }

void MainWindow::updateData() {
  // 모터 상태 업데이트
  ui->label_linear_x->setText(QString::number(qnode->driving_.motor_value_.linear.x));
  ui->label_angular_z->setText(QString::number(qnode->driving_.motor_value_.angular.z));

  // 실제 리프트 높이를 QNode에서 가져오기
  current_lift_height = qnode->getLiftHeight();
  ui->liftHeightValue->setText(QString::number(current_lift_height, 'f', 2));

  // 리프트 높이에 따른 색상 변경
  if (current_lift_height > 0.4) {
    ui->liftHeightValue->setStyleSheet("color: #a3be8c;");  // 녹색
  } else if (current_lift_height > 0.2) {
    ui->liftHeightValue->setStyleSheet("color: #ebcb8b;");  // 노랑
  } else {
    ui->liftHeightValue->setStyleSheet("color: #d8dee9;");  // 기본
  }
}

void MainWindow::updateLiftHeight() {
  // 이제 실제 리프트 높이는 QNode에서 관리하므로
  // 여기서는 UI 업데이트만 수행
  current_lift_height = qnode->getLiftHeight();
}

void MainWindow::appendLog(const QString& message) {
  QString current_time = QTime::currentTime().toString("hh:mm:ss");
  QString log_entry = QString("[%1] %2").arg(current_time).arg(message);

  ui->textEdit_debugLog->append(log_entry);
  ui->textEdit_debugLog->verticalScrollBar()->setValue(ui->textEdit_debugLog->verticalScrollBar()->maximum());
}

void MainWindow::updateWorkState(int state) {
  QNode::WorkState work_state = static_cast<QNode::WorkState>(state);

  switch (work_state) {
    case QNode::WorkState::IDLE:
      ui->statusValueLabel->setText("대기 중");
      ui->statusValueLabel->setStyleSheet("color: #d8dee9;");
      break;
    case QNode::WorkState::WORKING:
      ui->statusValueLabel->setText("작업 수행 중");
      ui->statusValueLabel->setStyleSheet("color: #a3be8c;");
      break;
    case QNode::WorkState::COMPLETED:
      ui->statusValueLabel->setText("작업 완료");
      ui->statusValueLabel->setStyleSheet("color: #a3be8c;");
      break;
  }
}

// 로봇 조종 버튼 핸들러들
void MainWindow::on_forwardButton_pressed() {
  RobotDriving::start = true;
  qnode->driving_.setSpeed(0.1, 0.0);
  ui->statusValueLabel->setText("이동 중: 전진");
  ui->statusValueLabel->setStyleSheet("color: #a3be8c;");
  appendLog("전진 명령 실행");
}

void MainWindow::on_backwardButton_pressed() {
  RobotDriving::start = true;
  qnode->driving_.setSpeed(-0.1, 0.0);
  ui->statusValueLabel->setText("이동 중: 후진");
  ui->statusValueLabel->setStyleSheet("color: #a3be8c;");
  appendLog("후진 명령 실행");
}

void MainWindow::on_leftButton_pressed() {
  RobotDriving::start = true;
  qnode->driving_.setSpeed(0.0, 1.0);
  ui->statusValueLabel->setText("이동 중: 좌회전");
  ui->statusValueLabel->setStyleSheet("color: #a3be8c;");
  appendLog("좌회전 명령 실행");
}

void MainWindow::on_rightButton_pressed() {
  RobotDriving::start = true;
  qnode->driving_.setSpeed(0.0, -1.0);
  ui->statusValueLabel->setText("이동 중: 우회전");
  ui->statusValueLabel->setStyleSheet("color: #a3be8c;");
  appendLog("우회전 명령 실행");
}

void MainWindow::on_stopButton_clicked() {
  RobotDriving::start = false;
  qnode->driving_.setSpeed(0.0, 0.0);
  ui->statusValueLabel->setText("정지");
  ui->statusValueLabel->setStyleSheet("color: #d8dee9;");
  appendLog("정지 명령 실행");
}

void MainWindow::on_emergencyStopButton_clicked() {
  RobotDriving::start = false;
  qnode->driving_.setSpeed(0.0, 0.0);
  ui->statusValueLabel->setText("비상 정지");
  ui->statusValueLabel->setStyleSheet("color: #bf616a;");

  // 모든 작업 취소 (리프트 포함)
  qnode->cancelTask();

  appendLog("비상 정지 명령 실행");
}

// 🔧 수정된 리프트 버튼 핸들러들 (실제 ROS 메시지 발행)
void MainWindow::on_liftUpButton_pressed() {
  qnode->liftUp();  // 실제 LiftController 호출
  appendLog("리프트 올림 명령 전송");
}

void MainWindow::on_liftDownButton_pressed() {
  qnode->liftDown();  // 실제 LiftController 호출
  appendLog("리프트 내림 명령 전송");
}

void MainWindow::on_liftUpButton_released() {
  qnode->liftStop();  // 실제 LiftController 정지
  appendLog("리프트 정지 명령 전송");
}

void MainWindow::on_liftDownButton_released() {
  qnode->liftStop();  // 실제 LiftController 정지
  appendLog("리프트 정지 명령 전송");
}

void MainWindow::on_pushButton_findParcel_clicked() {
  QString parcelInfo = ui->lineEdit_parcelInfo->text();

  if (parcelInfo.trimmed().isEmpty()) {
    ui->label_parcelResult->setText("택배 정보를 입력하세요.");
    appendLog("오류: 빈 택배 정보");
    return;
  }

  QString resultMsg = "입력한 택배 정보: " + parcelInfo;
  ui->label_parcelResult->setText(resultMsg);

  qnode->startFindParcelTask(parcelInfo.toStdString());
  appendLog("물품 검색 작업 시작: " + parcelInfo);
}

}  // namespace robot_master