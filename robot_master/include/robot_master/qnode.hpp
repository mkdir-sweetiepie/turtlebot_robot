/**
 * @file /include/robot_master/qnode.hpp
 *
 * @brief Communications central!
 *
 * @date January 2025
 **/
/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef robot_master_QNODE_HPP_
#define robot_master_QNODE_HPP_

/*****************************************************************************
** Includes
*****************************************************************************/
#ifndef Q_MOC_RUN
#include <rclcpp/rclcpp.hpp>
#endif
#include <QThread>
#include <opencv2/opencv.hpp>
#include <turtlebot3_msgs/msg/sensor_state.hpp>

#include "geometry_msgs/msg/twist.hpp"
#include "robot_driving.hpp"
#include "robot_msgs/msg/master_msg.hpp"
#include "robot_msgs/msg/vision_msg.hpp"
#include "std_msgs/msg/string.hpp"
/*****************************************************************************
** Class
*****************************************************************************/
namespace robot_master {

extern bool button_clicked;
class QNode : public QThread {
  Q_OBJECT
 public:
  QNode();
  ~QNode();

  RobotDriving driving_;
  void setItemInfo(const std::string& item);

 protected:
  void run();

 private:
  std::shared_ptr<rclcpp::Node> node;

  void initPubSub();
  void visionCallback(const std::shared_ptr<robot_msgs::msg::VisionMsg> vision_msg);
  void turtleRun();

  // topic
  rclcpp::Publisher<robot_msgs::msg::MasterMsg>::SharedPtr pub_master;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_motor;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_parcel_info;

  rclcpp::Subscription<robot_msgs::msg::VisionMsg>::SharedPtr sub_vision;

 Q_SIGNALS:
  void rosShutDown();
  void dataReceived();
};

}  // namespace robot_master
#endif /* robot_master_QNODE_HPP_ */
