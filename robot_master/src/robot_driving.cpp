#include "../include/robot_master/robot_driving.hpp"

namespace robot_master {

bool RobotDriving::start = false;

RobotDriving::RobotDriving() : situation(Situation::NONE) {
  // 모터 값 초기화
  motor_value_.linear.x = 0.0;
  motor_value_.linear.y = 0.0;
  motor_value_.linear.z = 0.0;
  motor_value_.angular.x = 0.0;
  motor_value_.angular.y = 0.0;
  motor_value_.angular.z = 0.0;

  // 마스터 메시지 초기화
  master_msg_.slam = false;
  master_msg_.qr = false;
  master_msg_.lift = false;
  master_msg_.item = "";
}

// robot driving 모터 속도 설정
void RobotDriving::setSpeed(double linear, double angular) {
  motor_value_.linear.x = linear;
  motor_value_.angular.z = angular;
}

// robot driving 모터 동작 시작
void RobotDriving::go() {
  if (!start) {
    setSpeed(0.0, 0.0);
    return;
  }
  // 모터 속도 설정
  analyzeSituation();
}

// master_msg_ flag
void RobotDriving::analyzeSituation() {
  if (master_msg_.slam) {
    situation = Situation::SLAM;
  } else if (master_msg_.qr) {
    situation = Situation::RECOGNIZE_QR;
  } else if (master_msg_.lift) {
    situation = Situation::LIFT_PARCEL;
  } else {
    situation = Situation::NONE;
  }
}

}  // namespace robot_master
