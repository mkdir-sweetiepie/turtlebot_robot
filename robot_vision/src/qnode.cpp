#include "../include/robot_vision/qnode.hpp"

#include <cv_bridge/cv_bridge.h>

namespace robot_vision {

QNode::QNode() : ocr_processing_(false), detection_enabled_(false), camera_fps_count_(0), current_camera_fps_(0) {
  int argc = 0;
  char** argv = nullptr;
  rclcpp::init(argc, argv);
  node = rclcpp::Node::make_shared("robot_vision");

  last_camera_time_ = std::chrono::steady_clock::now();

  initPubSub();
  this->start();
}

QNode::~QNode() {
  if (rclcpp::ok()) {
    rclcpp::shutdown();
  }
}

void QNode::run() {
  rclcpp::WallRate loop_rate(30);
  while (rclcpp::ok()) {
    rclcpp::spin_some(node);
    loop_rate.sleep();
  }
  rclcpp::shutdown();
  Q_EMIT rosShutDown();
}

void QNode::initPubSub() {
  vision_pub = node->create_publisher<robot_msgs::msg::VisionMsg>("turtle_vision", 100);

  image_sub = node->create_subscription<sensor_msgs::msg::Image>("camera1/image_raw", 10, std::bind(&QNode::callbackImage, this, std::placeholders::_1));

  ocr_request_pub = node->create_publisher<sensor_msgs::msg::Image>("/camera/ocr_request", 10);

  ocr_result_sub = node->create_subscription<robot_msgs::msg::VisionMsg>("turtle_vision", 10, std::bind(&QNode::ocrResultCallback, this, std::placeholders::_1));

  ocr_timer = node->create_wall_timer(std::chrono::milliseconds(5000), std::bind(&QNode::requestOCRInference, this));
}

void QNode::callbackImage(const sensor_msgs::msg::Image::SharedPtr msg_img) {
  // FPS 카운터는 항상 증가
  auto current_time = std::chrono::steady_clock::now();
  camera_fps_count_++;

  // 1초마다 FPS 계산
  auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(current_time - last_camera_time_);
  if (duration.count() >= 1000) {
    current_camera_fps_ = (camera_fps_count_ * 1000) / duration.count();
    camera_fps_count_ = 0;
    last_camera_time_ = current_time;
    Q_EMIT sigCameraFPS(current_camera_fps_);
  }

  // 이미지 처리는 별도로 (isreceived 조건 제거)
  try {
    cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg_img, sensor_msgs::image_encodings::RGB8);

    if (imgRaw != nullptr) {
      delete imgRaw;  // 이전 이미지 메모리 해제
    }

    imgRaw = new cv::Mat(cv_ptr->image);
    latest_image_msg = msg_img;
    Q_EMIT sigRcvImg();

  } catch (cv_bridge::Exception& e) {
    RCLCPP_ERROR(node->get_logger(), "cv_bridge exception: %s", e.what());
  }
}
void QNode::requestOCRInference() {
  if (detection_enabled_ && !ocr_processing_ && latest_image_msg) {
    ocr_processing_ = true;
    ocr_request_pub->publish(*latest_image_msg);
    RCLCPP_DEBUG(node->get_logger(), "OCR 추론 요청 전송");
  }
}

void QNode::ocrResultCallback(const robot_msgs::msg::VisionMsg::SharedPtr msg) {
  ocr_processing_ = false;
  latest_ocr_result_ = *msg;
  vision_pub->publish(*msg);

  float confidence = 0.0f;
  std::string display_text = msg->ocr_data;

  if (msg->ocr_detected) {
    size_t pipe1 = msg->ocr_data.find('|');
    if (pipe1 != std::string::npos) {
      display_text = msg->ocr_data.substr(0, pipe1);
      size_t pipe2 = msg->ocr_data.find('|', pipe1 + 1);
      if (pipe2 != std::string::npos) {
        std::string conf_str = msg->ocr_data.substr(pipe2 + 1);
        try {
          confidence = std::stof(conf_str);
        } catch (const std::exception& e) {
          confidence = 0.8f;
        }
      }
    }
  }

  Q_EMIT sigOCRResult(QString::fromStdString(display_text), msg->ocr_detected, confidence, msg->fps);

  if (msg->ocr_detected) {
    RCLCPP_INFO(node->get_logger(), "물품 인식: %s (확률: %.0f%%)", display_text.c_str(), confidence * 100);
  }
}

void QNode::enableDetection(bool enabled) {
  detection_enabled_ = enabled;
  if (enabled) {
    RCLCPP_INFO(node->get_logger(), "OCR 탐지가 활성화되었습니다");
  } else {
    RCLCPP_INFO(node->get_logger(), "OCR 탐지가 비활성화되었습니다");
    latest_ocr_result_ = robot_msgs::msg::VisionMsg();
  }
}

}  // namespace robot_vision