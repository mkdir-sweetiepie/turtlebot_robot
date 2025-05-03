/**
 * @file /src/task_manager.cpp
 *
 * @brief Implementation for the task manager.
 *
 * @date May 2025
 **/

#include "../include/robot_master/task_manager.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>  // 상단으로 이동
#include <filesystem>

namespace robot_master {

TaskManager::TaskManager(std::shared_ptr<rclcpp::Node> node) : node_(node), current_state_(TaskState::IDLE), current_item_id_(""), lift_controller_(std::make_shared<LiftController>(node)) {
  // Load predefined parcel and delivery locations from parameters
  loadLocations();

  // Set up service clients
  recognize_item_client_ = node_->create_client<robot_msgs::srv::RecognizeItem>("recognize_item");
  navigate_client_ = node_->create_client<robot_msgs::srv::NavigateToParcel>("navigate_to_parcel");

  // Set up ROS2 logger
  RCLCPP_INFO(node_->get_logger(), "Task manager initialized");
}

TaskManager::~TaskManager() {
  // Make sure all tasks are properly stopped
  if (current_state_ != TaskState::IDLE) {
    cancelCurrentTask();
  }
}

void TaskManager::loadLocations() {
  // 패키지 경로와 설정 파일 경로 구성
  std::string package_share_dir;
  std::string parcel_config_file;
  std::string delivery_config_file;

  try {
    // 패키지 경로 얻기
    package_share_dir = ament_index_cpp::get_package_share_directory("robot_master");
    RCLCPP_INFO(node_->get_logger(), "Package share directory: %s", package_share_dir.c_str());

    // 설정 파일 경로 구성
    parcel_config_file = package_share_dir + "/config/parcel_locations.yaml";
    delivery_config_file = package_share_dir + "/config/delivery_pose.yaml";
  } catch (const std::exception& e) {
    RCLCPP_ERROR(node_->get_logger(), "Failed to get package share directory: %s", e.what());
    // 실패하면 상대 경로 사용
    parcel_config_file = node_->declare_parameter<std::string>("parcel_locations_file", "config/parcel_locations.yaml");
    delivery_config_file = node_->declare_parameter<std::string>("delivery_location_file", "config/delivery_pose.yaml");
  }

  RCLCPP_INFO(node_->get_logger(), "Loading parcel locations from: %s", parcel_config_file.c_str());

  try {
    YAML::Node config = YAML::LoadFile(parcel_config_file);
    for (const auto& location : config["locations"]) {
      std::string id = location["id"].as<std::string>();
      double x = location["x"].as<double>();
      double y = location["y"].as<double>();
      double yaw = location["yaw"].as<double>();

      parcel_locations_[id] = {x, y, yaw};
      RCLCPP_INFO(node_->get_logger(), "Loaded parcel location: %s (%.2f, %.2f, %.2f)", id.c_str(), x, y, yaw);
    }
  } catch (const YAML::Exception& e) {
    RCLCPP_ERROR(node_->get_logger(), "Failed to load parcel locations: %s", e.what());

    // 기본값 설정
    RCLCPP_INFO(node_->get_logger(), "Using default parcel locations");
    parcel_locations_["DRINK-001"] = {1.0, 2.0, 90.0};
    parcel_locations_["BOOK-001"] = {3.0, 1.5, 0.0};
    parcel_locations_["FOOD-001"] = {2.5, 3.5, 180.0};
  }

  RCLCPP_INFO(node_->get_logger(), "Loading delivery location from: %s", delivery_config_file.c_str());

  try {
    YAML::Node config = YAML::LoadFile(delivery_config_file);
    delivery_location_.x = config["delivery"]["x"].as<double>();
    delivery_location_.y = config["delivery"]["y"].as<double>();
    delivery_location_.yaw = config["delivery"]["yaw"].as<double>();

    RCLCPP_INFO(node_->get_logger(), "Loaded delivery location: (%.2f, %.2f, %.2f)", delivery_location_.x, delivery_location_.y, delivery_location_.yaw);
  } catch (const YAML::Exception& e) {
    RCLCPP_ERROR(node_->get_logger(), "Failed to load delivery location: %s", e.what());

    // 기본값 설정
    RCLCPP_INFO(node_->get_logger(), "Using default delivery location");
    delivery_location_ = {0.0, 0.0, 0.0};
  }
}

void TaskManager::startFindParcelTask(const std::string& item_id) {
  if (current_state_ != TaskState::IDLE) {
    RCLCPP_WARN(node_->get_logger(), "Cannot start new task while another task is active");
    return;
  }

  current_item_id_ = item_id;
  current_state_ = TaskState::SEARCHING;

  RCLCPP_INFO(node_->get_logger(), "Starting task to find parcel with ID: %s", item_id.c_str());

  // Start state machine
  processCurrentState();
}

void TaskManager::cancelCurrentTask() {
  RCLCPP_INFO(node_->get_logger(), "Canceling current task");

  // Additional cancellation logic would go here (e.g., cancel navigation)

  current_state_ = TaskState::IDLE;
  current_item_id_ = "";
}

void TaskManager::processCurrentState() {
  switch (current_state_) {
    case TaskState::IDLE:
      // Nothing to do
      break;

    case TaskState::SEARCHING:
      // Start by navigating to search areas
      navigateToSearchArea();
      break;

    case TaskState::NAVIGATING_TO_PARCEL:
      // Already navigating, wait for callback
      break;

    case TaskState::RECOGNIZING_PARCEL:
      // Start recognizing parcel
      recognizeParcel();
      break;

    case TaskState::LIFTING_PARCEL:
      // Lift the parcel
      liftParcel();
      break;

    case TaskState::DELIVERING:
      // Navigate to delivery location
      navigateToDelivery();
      break;

    case TaskState::DROPPING_PARCEL:
      // Drop the parcel
      dropParcel();
      break;

    case TaskState::COMPLETED:
      // Task completed, reset to idle
      RCLCPP_INFO(node_->get_logger(), "Task completed successfully");
      current_state_ = TaskState::IDLE;
      current_item_id_ = "";
      break;

    case TaskState::ERROR:
      // Handle error state
      RCLCPP_ERROR(node_->get_logger(), "Task failed, resetting to idle state");
      current_state_ = TaskState::IDLE;
      current_item_id_ = "";
      break;
  }
}

void TaskManager::navigateToSearchArea() {
  // Check if we have a predefined location for this item
  if (parcel_locations_.find(current_item_id_) != parcel_locations_.end()) {
    // We have a predefined location, navigate directly to it
    auto location = parcel_locations_[current_item_id_];

    RCLCPP_INFO(node_->get_logger(), "Navigating to predefined location for %s: (%.2f, %.2f, %.2f)", current_item_id_.c_str(), location.x, location.y, location.yaw);

    auto request = std::make_shared<robot_msgs::srv::NavigateToParcel::Request>();
    request->x = location.x;
    request->y = location.y;
    request->yaw = location.yaw;

    current_state_ = TaskState::NAVIGATING_TO_PARCEL;

    // Send the navigation request
    auto future = navigate_client_->async_send_request(request, std::bind(&TaskManager::navigateToParcelCallback, this, std::placeholders::_1));
  } else {
    // No predefined location, this would require a more complex search strategy
    RCLCPP_WARN(node_->get_logger(), "No predefined location for %s, cannot search", current_item_id_.c_str());
    current_state_ = TaskState::ERROR;
    processCurrentState();
  }
}

void TaskManager::navigateToParcelCallback(const rclcpp::Client<robot_msgs::srv::NavigateToParcel>::SharedFuture future) {
  auto result = future.get();
  if (result->success) {
    RCLCPP_INFO(node_->get_logger(), "Successfully navigated to parcel location");
    current_state_ = TaskState::RECOGNIZING_PARCEL;
    processCurrentState();
  } else {
    RCLCPP_ERROR(node_->get_logger(), "Failed to navigate to parcel location: %s", result->message.c_str());
    current_state_ = TaskState::ERROR;
    processCurrentState();
  }
}

void TaskManager::recognizeParcel() {
  RCLCPP_INFO(node_->get_logger(), "Starting parcel recognition for %s", current_item_id_.c_str());

  auto request = std::make_shared<robot_msgs::srv::RecognizeItem::Request>();
  request->expected_id = current_item_id_;

  auto future = recognize_item_client_->async_send_request(request, std::bind(&TaskManager::recognizeParcelCallback, this, std::placeholders::_1));
}

void TaskManager::recognizeParcelCallback(const rclcpp::Client<robot_msgs::srv::RecognizeItem>::SharedFuture future) {
  auto result = future.get();
  if (result->found && result->id == current_item_id_) {
    RCLCPP_INFO(node_->get_logger(), "Successfully recognized parcel with ID: %s", result->id.c_str());
    current_state_ = TaskState::LIFTING_PARCEL;
    processCurrentState();
  } else {
    RCLCPP_ERROR(node_->get_logger(), "Failed to recognize parcel or ID mismatch: found=%d, id=%s", result->found, result->id.c_str());
    current_state_ = TaskState::ERROR;
    processCurrentState();
  }
}

void TaskManager::liftParcel() {
  RCLCPP_INFO(node_->get_logger(), "Lifting parcel");

  lift_controller_->moveUp();

  // Use a timer to check when lifting is complete
  lift_timer_ = node_->create_wall_timer(std::chrono::milliseconds(500), std::bind(&TaskManager::checkLiftStatus, this));
}

void TaskManager::checkLiftStatus() {
  auto status = lift_controller_->getStatus();

  if (status == LiftStatus::UP) {
    // Lift is up, proceed to delivery
    lift_timer_->cancel();
    RCLCPP_INFO(node_->get_logger(), "Lift is up, proceeding to delivery");
    current_state_ = TaskState::DELIVERING;
    processCurrentState();
  } else if (status == LiftStatus::DOWN) {
    // Lift failed to go up
    lift_timer_->cancel();
    RCLCPP_ERROR(node_->get_logger(), "Lift failed to go up");
    current_state_ = TaskState::ERROR;
    processCurrentState();
  }
  // If still moving, continue waiting
}

void TaskManager::navigateToDelivery() {
  RCLCPP_INFO(node_->get_logger(), "Navigating to delivery location: (%.2f, %.2f, %.2f)", delivery_location_.x, delivery_location_.y, delivery_location_.yaw);

  auto request = std::make_shared<robot_msgs::srv::NavigateToParcel::Request>();
  request->x = delivery_location_.x;
  request->y = delivery_location_.y;
  request->yaw = delivery_location_.yaw;

  // Send the navigation request
  auto future = navigate_client_->async_send_request(request, std::bind(&TaskManager::navigateToDeliveryCallback, this, std::placeholders::_1));
}

void TaskManager::navigateToDeliveryCallback(const rclcpp::Client<robot_msgs::srv::NavigateToParcel>::SharedFuture future) {
  auto result = future.get();
  if (result->success) {
    RCLCPP_INFO(node_->get_logger(), "Successfully navigated to delivery location");
    current_state_ = TaskState::DROPPING_PARCEL;
    processCurrentState();
  } else {
    RCLCPP_ERROR(node_->get_logger(), "Failed to navigate to delivery location: %s", result->message.c_str());
    current_state_ = TaskState::ERROR;
    processCurrentState();
  }
}

void TaskManager::dropParcel() {
  RCLCPP_INFO(node_->get_logger(), "Dropping parcel");

  lift_controller_->moveDown();

  // Use a timer to check when dropping is complete
  lift_timer_ = node_->create_wall_timer(std::chrono::milliseconds(500), std::bind(&TaskManager::checkDropStatus, this));
}

void TaskManager::checkDropStatus() {
  auto status = lift_controller_->getStatus();

  if (status == LiftStatus::DOWN) {
    // Lift is down, task complete
    lift_timer_->cancel();
    RCLCPP_INFO(node_->get_logger(), "Lift is down, task complete");
    current_state_ = TaskState::COMPLETED;
    processCurrentState();
  } else if (status == LiftStatus::UP) {
    // Lift failed to go down
    lift_timer_->cancel();
    RCLCPP_ERROR(node_->get_logger(), "Lift failed to go down");
    current_state_ = TaskState::ERROR;
    processCurrentState();
  }
  // If still moving, continue waiting
}

TaskState TaskManager::getCurrentState() const { return current_state_; }

std::string TaskManager::getCurrentItemId() const { return current_item_id_; }

}  // namespace robot_master