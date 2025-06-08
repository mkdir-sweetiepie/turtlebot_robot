/**
 * @file qnode.hpp
 * @date May 2025
 */

#ifndef ROBOT_VISION_QNODE_HPP_
#define ROBOT_VISION_QNODE_HPP_

#include <QStringListModel>
#include <QThread>
#include <chrono>
#include <memory>
#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>
#include <robot_msgs/msg/log_message.hpp>
#include <robot_msgs/msg/vision_msg.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/bool.hpp>

namespace robot_vision {

class QNode : public QThread {
  Q_OBJECT

 public:
  QNode();
  virtual ~QNode();
  void run() override;

  bool isOCREnabled() const;
  bool isOCRProcessing() const;
  void enableOCRManually(bool enabled);
  void enableDetection(bool enabled);

 Q_SIGNALS:
  void rosShutDown();
  void dataReceived();
  void sigRcvImg();
  void sigOCRResult(QString text, bool detected, float confidence, int processing_time);
  void sigCameraFPS(int fps);

 public:
  cv::Mat* imgRaw = nullptr;
  bool isreceived;

 private:
  void initPubSub();
  void publishSystemLog(const std::string& level, const std::string& message);

  void ocrControlCallback(const std_msgs::msg::Bool::SharedPtr msg);
  void callbackImage(const sensor_msgs::msg::Image::SharedPtr msg_img);
  void sendImageToOCR();
  void ocrResultCallback(const robot_msgs::msg::VisionMsg::SharedPtr msg);

  std::shared_ptr<rclcpp::Node> node;

  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr ocr_request_pub;
  rclcpp::Publisher<robot_msgs::msg::LogMessage>::SharedPtr system_log_pub_;

  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr ocr_control_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub;
  rclcpp::Subscription<robot_msgs::msg::VisionMsg>::SharedPtr ocr_result_sub;

  rclcpp::TimerBase::SharedPtr ocr_image_timer;

  bool ocr_processing_;
  bool ocr_enabled_;
  sensor_msgs::msg::Image::SharedPtr latest_image_msg;
  robot_msgs::msg::VisionMsg latest_ocr_result_;

  int camera_fps_count_;
  int current_camera_fps_;
  std::chrono::steady_clock::time_point last_camera_time_;
};

}  // namespace robot_vision

#endif  // ROBOT_VISION_QNODE_HPP_