/**
 * @file /include/robot_master/task_manager.hpp
 *
 * @brief Header for the task manager.
 *
 * @date May 2025
 **/

#ifndef ROBOT_MASTER_TASK_MANAGER_HPP
#define ROBOT_MASTER_TASK_MANAGER_HPP

#include <functional>
#include <map>
#include <memory>
#include <string>

#include "lift_controller.hpp"
#include "rclcpp/rclcpp.hpp"
#include "robot_msgs/srv/navigate_to_parcel.hpp"
#include "robot_msgs/srv/recognize_item.hpp"
#include "yaml-cpp/yaml.h"

namespace robot_master {

enum class TaskState { IDLE, SEARCHING, NAVIGATING_TO_PARCEL, RECOGNIZING_PARCEL, LIFTING_PARCEL, DELIVERING, DROPPING_PARCEL, COMPLETED, ERROR };

struct Location {
  double x;
  double y;
  double yaw;
};

class TaskManager {
 public:
  explicit TaskManager(std::shared_ptr<rclcpp::Node> node);
  ~TaskManager();

  // Task control methods
  void startFindParcelTask(const std::string& item_id);
  void cancelCurrentTask();

  // Status methods
  TaskState getCurrentState() const;
  std::string getCurrentItemId() const;

  // 이 메서드 추가: 리프트 컨트롤러 반환
  std::shared_ptr<LiftController> getLiftController() const { return lift_controller_; }

 private:
  // Node pointer
  std::shared_ptr<rclcpp::Node> node_;

  // Lift controller
  std::shared_ptr<LiftController> lift_controller_;

  // Current state information
  TaskState current_state_;
  std::string current_item_id_;

  // Location information
  std::map<std::string, Location> parcel_locations_;
  Location delivery_location_;

  // Service clients
  rclcpp::Client<robot_msgs::srv::RecognizeItem>::SharedPtr recognize_item_client_;
  rclcpp::Client<robot_msgs::srv::NavigateToParcel>::SharedPtr navigate_client_;

  // Timer for checking lift status
  rclcpp::TimerBase::SharedPtr lift_timer_;

  // Private methods
  void loadLocations();
  void processCurrentState();

  // State machine methods
  void navigateToSearchArea();
  void navigateToParcelCallback(const rclcpp::Client<robot_msgs::srv::NavigateToParcel>::SharedFuture future);

  void recognizeParcel();
  void recognizeParcelCallback(const rclcpp::Client<robot_msgs::srv::RecognizeItem>::SharedFuture future);

  void liftParcel();
  void checkLiftStatus();

  void navigateToDelivery();
  void navigateToDeliveryCallback(const rclcpp::Client<robot_msgs::srv::NavigateToParcel>::SharedFuture future);

  void dropParcel();
  void checkDropStatus();
};

}  // namespace robot_master

#endif  // ROBOT_MASTER_TASK_MANAGER_HPP