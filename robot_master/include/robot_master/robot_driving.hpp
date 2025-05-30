#ifndef robot_master_ROBOT_DRIVING_HPP_
#define robot_master_ROBOT_DRIVING_HPP_

#include "geometry_msgs/msg/twist.hpp"

namespace robot_master {

class RobotDriving {
 public:
  static bool start;  // 로봇 주행 시작 여부

  RobotDriving();

  void setSpeed(double linear, double angular);  // 로봇 주행 속도 설정
  void go();                                     // 로봇 주행 시작

  geometry_msgs::msg::Twist motor_value_;  // 로봇 모터 값

 private:
};

}  // namespace robot_master

#endif  // robot_master_ROBOT_DRIVING_HPP_