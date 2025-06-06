# robot_msgs/msg/OCRResult.msg  
# OCR 스캔 결과 메시지

int64 request_id          # 요청 ID (매칭용)
bool item_found           # 물품을 찾았는지 여부
string detected_text      # 감지된 텍스트
float32 confidence        # 신뢰도
string target_item_id     # 검색했던 물품 ID
string current_location   # 스캔한 위치
string message           # 상태 메시지