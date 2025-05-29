#include "../include/robot_vision/qnode.hpp"

#include <cv_bridge/cv_bridge.h>

namespace robot_vision {

QNode::QNode() : ocr_processing_(false), detection_enabled_(false), camera_fps_count_(0), current_camera_fps_(0), isreceived(false) {
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
  // ocr output pub
  vision_pub = node->create_publisher<robot_msgs::msg::VisionMsg>("turtle_vision", 100);
  // ocr request pub
  ocr_request_pub = node->create_publisher<sensor_msgs::msg::Image>("/ocr_request", 10);
  // usb_cam sub
  image_sub = node->create_subscription<sensor_msgs::msg::Image>("camera1/image_raw", 10, std::bind(&QNode::callbackImage, this, std::placeholders::_1));
  // ocr result sub
  ocr_result_sub = node->create_subscription<robot_msgs::msg::VisionMsg>("turtle_vision", 10, std::bind(&QNode::ocrResultCallback, this, std::placeholders::_1));
  // 5초마다 ocr 추론 요청 타이머 설정
  ocr_timer = node->create_wall_timer(std::chrono::milliseconds(5000), std::bind(&QNode::requestOCRInference, this));
}

// usb_cam 카메라 이미지 콜백 함수
void QNode::callbackImage(const sensor_msgs::msg::Image::SharedPtr msg_img) {
  if (!isreceived) {
    isreceived = true;

    // 카메라 FPS 계산
    auto current_time = std::chrono::steady_clock::now();
    camera_fps_count_++;

    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(current_time - last_camera_time_);
    if (duration.count() >= 1000) {
      current_camera_fps_ = (camera_fps_count_ * 1000) / duration.count();
      camera_fps_count_ = 0;
      last_camera_time_ = current_time;
      Q_EMIT sigCameraFPS(current_camera_fps_);
    }

    // 이미지 처리
    try {
      cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg_img, sensor_msgs::image_encodings::RGB8);

      if (imgRaw != nullptr) {
        delete imgRaw;
      }

      imgRaw = new cv::Mat(cv_ptr->image);
      latest_image_msg = msg_img;
      Q_EMIT sigRcvImg();

    } catch (cv_bridge::Exception& e) {
      RCLCPP_ERROR(node->get_logger(), "cv_bridge exception: %s", e.what());
    }
  }
}

// OCR 추론 요청 함수 (5초마다 호출)
void QNode::requestOCRInference() {
  if (detection_enabled_ && !ocr_processing_ && latest_image_msg) {
    ocr_processing_ = true;
    ocr_request_pub->publish(*latest_image_msg);
    RCLCPP_DEBUG(node->get_logger(), "OCR 추론 요청 전송");
  }
}

// OCR 결과 콜백 함수
void QNode::ocrResultCallback(const robot_msgs::msg::VisionMsg::SharedPtr msg) {
  ocr_processing_ = false;
  latest_ocr_result_ = *msg;
  vision_pub->publish(*msg);

  float confidence = 0.0f;
  std::string display_text = msg->ocr_text;

  if (msg->ocr_detected) {
    display_text = msg->ocr_text;
    confidence = msg->confidence;
  }

  Q_EMIT sigOCRResult(QString::fromStdString(display_text), msg->ocr_detected, confidence, msg->fps);
}

// OCR 탐지 활성화/비활성화 함수
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
