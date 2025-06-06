# robot_msgs/msg/OCRRequest.msg
# OCR 스캔 요청 메시지

string target_item_id      # 찾고자 하는 물품 ID
string current_location    # 현재 위치 이름 (예: "거점1", "거점2")
int32 location_index       # 거점 인덱스 (0, 1, 2, 3)
int64 request_id          # 요청 ID (타임스탬프 등)