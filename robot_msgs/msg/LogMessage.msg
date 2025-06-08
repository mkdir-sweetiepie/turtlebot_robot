# robot_msgs/msg/LogMessage.msg
# 통합 로그 메시지 정의

string node_name         # 로그를 전송한 노드 이름 ("master", "navigation", "vision")
string level            # 로그 레벨 ("INFO", "WARN", "ERROR", "DEBUG")
string message          # 로그 메시지 내용
builtin_interfaces/Time timestamp  # 타임스탬프