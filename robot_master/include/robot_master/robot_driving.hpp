// =============================================================================
// robot_driving.hpp
// =============================================================================
#ifndef robot_master_ROBOT_DRIVING_HPP_
#define robot_master_ROBOT_DRIVING_HPP_

#include "geometry_msgs/msg/twist.hpp"
#include "robot_msgs/msg/master_msg.hpp"

namespace robot_master {

enum class Situation { NONE, SLAM, RECOGNIZE_QR, LIFT_PARCEL };

class RobotDriving {
 public:
  static bool start;  // 로봇 주행 시작 여부

  RobotDriving();

  void setSpeed(double linear, double angular);  // 로봇 주행 속도 설정
  void go();                                     // 로봇 주행 시작

  geometry_msgs::msg::Twist motor_value_;  // 로봇 모터 값
  robot_msgs::msg::MasterMsg master_msg_;  // 로봇 마스터 메시지

 private:
  Situation situation;      // 현재 상황
  void analyzeSituation();  // 현재 상황 분석
};

}  // namespace robot_master

#endif  // robot_master_ROBOT_DRIVING_HPP_