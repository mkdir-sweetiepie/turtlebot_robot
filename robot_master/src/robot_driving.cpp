#include "../include/robot_master/robot_driving.hpp"

namespace robot_master {

bool RobotDriving::start = false;

RobotDriving::RobotDriving() {
  // 모터 값 초기화
  motor_value_.linear.x = 0.0;
  motor_value_.linear.y = 0.0;
  motor_value_.linear.z = 0.0;
  motor_value_.angular.x = 0.0;
  motor_value_.angular.y = 0.0;
  motor_value_.angular.z = 0.0;
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
}

}  // namespace robot_master
