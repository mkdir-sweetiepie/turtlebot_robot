#ifndef robot_master_QNODE_HPP_
#define robot_master_QNODE_HPP_

#include <QThread>
#include <chrono>
#include <memory>
#include <string>

#include "geometry_msgs/msg/twist.hpp"
#include "lift_controller.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "robot_driving.hpp"
#include "robot_msgs/action/precise_control.hpp"
#include "robot_msgs/msg/log_message.hpp"
#include "robot_msgs/msg/ocr_request.hpp"
#include "robot_msgs/msg/ocr_result.hpp"
#include "robot_msgs/msg/vision_msg.hpp"
#include "robot_msgs/srv/item_info.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/u_int8.hpp"

namespace robot_master {

class QNode : public QThread {
  Q_OBJECT

 public:
  // 작업 상태 정의
  enum class WorkState { IDLE, WORKING, COMPLETED };

  // 액션 타입 정의
  using PreciseControlAction = robot_msgs::action::PreciseControl;
  using GoalHandlePreciseControl = rclcpp_action::ServerGoalHandle<PreciseControlAction>;

  QNode();
  ~QNode();

  void startFindParcelTask(const std::string& item);
  void cancelTask();
  void performItemFoundActions();

  // 리프트 제어
  void liftUp();
  void liftDown();
  void liftStop();
  double getLiftHeight();

  RobotDriving driving_;
  WorkState getCurrentState() const { return current_work_state_; }

 Q_SIGNALS:
  void rosShutDown();
  void dataReceived();
  void logMessage(const QString& message);
  void workStateChanged(int state);

 protected:
  void run();

 private:
  // 상태 관리
  int precise_step_;

  // ROS2 관련
  std::shared_ptr<rclcpp::Node> node;

  // Publishers & Subscribers
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_motor;
  rclcpp::Subscription<robot_msgs::msg::VisionMsg>::SharedPtr sub_vision;

  // 네비게이션 시스템과 통신용
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr search_request_pub;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr search_result_sub;

  // OCR 토픽 통신
  rclcpp::Subscription<robot_msgs::msg::OCRRequest>::SharedPtr ocr_request_sub_;
  rclcpp::Publisher<robot_msgs::msg::OCRResult>::SharedPtr ocr_result_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr ocr_control_pub_;

  // 통합 로그 시스템
  rclcpp::Publisher<robot_msgs::msg::LogMessage>::SharedPtr system_log_pub_;
  rclcpp::Subscription<robot_msgs::msg::LogMessage>::SharedPtr system_log_sub_;

  // Service
  rclcpp::Client<robot_msgs::srv::ItemInfo>::SharedPtr item_info_client_;
  rclcpp::Service<robot_msgs::srv::ItemInfo>::SharedPtr item_info_service_;

  // Action Server
  rclcpp_action::Server<PreciseControlAction>::SharedPtr precise_control_action_server_;
  std::shared_ptr<GoalHandlePreciseControl> current_goal_handle_;

  std::shared_ptr<LiftController> lift_controller_;

  // 상태 관리
  WorkState current_work_state_;
  std::string target_item_;
  bool lift_performing_action_;
  bool ocr_scan_active_;
  bool action_in_progress_;
  bool navigation_mode_;
  bool arrive;

  // OCR 토픽 관련
  int64_t current_request_id_;
  std::string current_location_;
  std::chrono::steady_clock::time_point scan_start_time_;
  std::chrono::steady_clock::time_point precise_control_start_time_;

  // 메서드
  void initPubSub();
  void turtleRun();
  void setState(WorkState new_state);

  // 로그 시스템
  void publishSystemLog(const std::string& level, const std::string& message);
  void systemLogCallback(const robot_msgs::msg::LogMessage::SharedPtr msg);

  // Service 처리
  void queryItemInfo(const std::string& item_id);
  void handleItemInfoServiceRequest(const std::shared_ptr<robot_msgs::srv::ItemInfo::Request> request, std::shared_ptr<robot_msgs::srv::ItemInfo::Response> response);

  // Action 처리
  rclcpp_action::GoalResponse handleGoal(const rclcpp_action::GoalUUID& uuid, std::shared_ptr<const PreciseControlAction::Goal> goal);
  rclcpp_action::CancelResponse handleCancel(const std::shared_ptr<GoalHandlePreciseControl> goal_handle);
  void handleAccepted(const std::shared_ptr<GoalHandlePreciseControl> goal_handle);
  void executePreciseControlAction(const std::shared_ptr<GoalHandlePreciseControl> goal_handle);
  void publishActionFeedback(const std::string& step, int step_num, float progress);

  // 네비게이션 통신
  void searchResultCallback(const std_msgs::msg::String::SharedPtr msg);

  // OCR 처리
  void ocrRequestCallback(const robot_msgs::msg::OCRRequest::SharedPtr msg);
  void sendOCRResult(bool found, const std::string& detected_text, float confidence, const std::string& message);
  void visionCallback(const std::shared_ptr<robot_msgs::msg::VisionMsg> vision_msg);

  // 텍스트 매칭
  std::string normalizeString(const std::string& str);
  double calculateSimilarity(const std::string& str1, const std::string& str2);
  bool isTextMatch(const std::string& detected_text, const std::string& target_text, float confidence);
};

}  // namespace robot_master

#endif  // robot_master_QNODE_HPP_