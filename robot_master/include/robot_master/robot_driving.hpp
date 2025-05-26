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
  static bool start;

  RobotDriving();

  void setSpeed(double linear, double angular);
  void go();

  geometry_msgs::msg::Twist motor_value_;
  robot_msgs::msg::MasterMsg master_msg_;

 private:
  Situation situation;
  void analyzeSituation();
};

}  // namespace robot_master

#endif  // robot_master_ROBOT_DRIVING_HPP_