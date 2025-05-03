/**
 * @file /include/robot_master/robot_driving.hpp
 *
 * @brief Header for robot driving control.
 *
 * @date May 2025
 **/

#ifndef ROBOT_MASTER_ROBOT_DRIVING_HPP
#define ROBOT_MASTER_ROBOT_DRIVING_HPP

#include <string>

#include "geometry_msgs/msg/twist.hpp"
#include "robot_msgs/msg/master_msg.hpp"

namespace robot_master {

// 열거형 이름 변경: DrivingSituation -> Situation
enum class Situation { NONE, SLAM, SEARCH_PARCEL, RECOGNIZE_QR, NAVIGATION, LIFT_PARCEL, RETURN_TO_BASE };

class RobotDriving {
 public:
  RobotDriving();

  void go();
  void setSpeed(double linear, double angular);
  void analyzeSituation();

  // Robot speed command
  geometry_msgs::msg::Twist motor_value_;

  // Master message for communication
  robot_msgs::msg::MasterMsg master_msg_;

  // Static control flag
  static bool start;

 private:
  Situation situation;  // DrivingSituation -> Situation으로 변경
};

}  // namespace robot_master

#endif  // ROBOT_MASTER_ROBOT_DRIVING_HPP