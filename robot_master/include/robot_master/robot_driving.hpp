#ifndef ROBOT_DRIVING_HPP
#define ROBOT_DRIVING_HPP

#include <rclcpp/rclcpp.hpp>

#include "geometry_msgs/msg/twist.hpp"
#include "robot_msgs/msg/master_msg.hpp"
#include "robot_msgs/msg/vision_msg.hpp"

namespace robot_master {

class RobotDriving {
 public:
  RobotDriving();
  void go();
  void setSpeed(double linear, double angular);

  robot_msgs::msg::MasterMsg master_msg_;
  robot_msgs::msg::VisionMsg Vision_msg_;
  geometry_msgs::msg::Twist motor_value_;

  static bool start;

 private:
  void analyzeSituation();

  int situation;
  enum Situation { NONE = 0, SLAM, QR, LIFT };
};

}  // namespace robot_master

#endif  // ROBOT_DRIVING_HPP