# robot_msgs/msg/VisionMsg.msg
# OCR 결과를 전달하는 메시지

bool ocr_detected      # OCR로 텍스트가 감지되었는지 여부
string ocr_text        # 감지된 텍스트 내용
float32 confidence     # 인식 신뢰도 (0.0 ~ 1.0)
int32 fps             # 처리 시간(ms) 또는 FPSu