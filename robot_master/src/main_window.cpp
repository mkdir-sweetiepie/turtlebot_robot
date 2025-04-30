/**
 * @file /src/main_window.cpp
 *
 * @brief Implementation for the qt gui.
 *
 * @date January 2025
 **/
/*****************************************************************************
** Includes
*****************************************************************************/

#include "../include/robot_master/main_window.hpp"
namespace robot_master {

MainWindow::MainWindow(QWidget* parent) : QMainWindow(parent), ui(new Ui::MainWindowDesign) {
  ui->setupUi(this);

  QIcon icon("://ros-icon.png");
  this->setWindowIcon(icon);

  qnode = new QNode();

  move(820, 0);  // ui위치
  QObject::connect(qnode, SIGNAL(rosShutDown()), this, SLOT(close()));
  QObject::connect(qnode, SIGNAL(dataReceived()), this, SLOT(updateData()));
}

void MainWindow::closeEvent(QCloseEvent* event) { QMainWindow::closeEvent(event); }

MainWindow::~MainWindow() { delete ui; }

void MainWindow::updateData() {
  // DRIVING MSG
  
}

void MainWindow::on_pushButton_clicked() {
  button_clicked = !button_clicked;
  ui->pushButton->setText(button_clicked ? "stop" : "start");
  RobotDriving::start = button_clicked;
  ui->label_linear_x->setText(QString::number(qnode->driving_.motor_value_.linear.x));
  ui->label_angular_z->setText(QString::number(qnode->driving_.motor_value_.angular.z));
}

void MainWindow::on_pushButton_findParcel_clicked() {
  QString parcelInfo = ui->lineEdit_parcelInfo->text();

  if (parcelInfo.trimmed().isEmpty()) {
    ui->label_parcelResult->setText("택배 정보를 입력하세요.");
    return;
  }

  QString resultMsg = "입력한 택배 정보: " + parcelInfo;
  ui->label_parcelResult->setText(resultMsg);

  qnode->setItemInfo(parcelInfo.toStdString());
}

}  // namespace robot_master