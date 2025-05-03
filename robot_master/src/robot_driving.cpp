/**
 * @file /src/robot_driving.cpp
 *
 * @brief Implementation for robot driving control.
 *
 * @date May 2025
 **/

#include "../include/robot_master/robot_driving.hpp"

namespace robot_master {

bool RobotDriving::start = false;

RobotDriving::RobotDriving() : situation(Situation::NONE) {
  // Initialize motor values to 0
  motor_value_.linear.x = 0.0;
  motor_value_.linear.y = 0.0;
  motor_value_.linear.z = 0.0;
  motor_value_.angular.x = 0.0;
  motor_value_.angular.y = 0.0;
  motor_value_.angular.z = 0.0;

  // Initialize master message flags
  master_msg_.slam = false;
  master_msg_.qr = false;
  master_msg_.lift = false;
  master_msg_.item = "";
}

void RobotDriving::setSpeed(double linear, double angular) {
  motor_value_.linear.x = linear;
  motor_value_.angular.z = angular;
}

void RobotDriving::go() {
  if (!start) {
    // If not started, ensure robot is stopped
    setSpeed(0.0, 0.0);
    return;
  }

  // If manual control is active, the motor values are set directly
  // by the button handlers in MainWindow. We don't need to modify
  // them here.

  // For autonomous operation, we would analyze the situation and set speeds
  // based on the current task
  analyzeSituation();
}

void RobotDriving::analyzeSituation() {
  // This is where the autonomous driving logic would go
  // For now, we'll just have a placeholder implementation

  // Flag-based state transitions
  if (master_msg_.slam) {
    situation = Situation::SLAM;
  } else if (master_msg_.qr) {
    situation = Situation::RECOGNIZE_QR;
  } else if (master_msg_.lift) {
    situation = Situation::LIFT_PARCEL;
  }

  // In a full implementation, this would have more sophisticated
  // logic for path planning, obstacle avoidance, etc.
}

}  // namespace robot_master