#include "../include/robot_master/robot_driving.hpp"

namespace robot_master {

bool RobotDriving::start = false;

RobotDriving::RobotDriving() : situation(NONE) {
  master_msg_.slam = false;
  master_msg_.qr = false;
  master_msg_.lift = false;
}

void RobotDriving::setSpeed(double linear, double angular) {
  motor_value_.linear.x = linear;
  motor_value_.angular.z = angular;
}

void RobotDriving::go() {
  if (!start) {
    setSpeed(0.0, 0.0);
    return;
  } else {
    analyzeSituation();
    setSpeed(1.0, 1.0);
  }

}

void RobotDriving::analyzeSituation() {
  if(!master_msg_.slam){
    situation = SLAM;
  }
}


}  // namespace robot_master