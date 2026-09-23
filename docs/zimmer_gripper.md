# Zimmer GEP2006IO-00-B Digital I/O 단독 시험

사용자가 배선 및 펜던트 동작을 확인한 구성: DO1=열기(white), DO2=닫기(black).
DO0은 다른 장치용이므로 절대 쓰지 않습니다. IO-Link/WEISS 코드는 사용하지 않습니다.
전원과 배선 변경은 이 코드 범위 밖입니다.

UR driver의 `/io_and_status_controller/set_io`와 `/io_and_status_controller/io_states`를
사용합니다. MoveIt이나 팔 이동 명령은 사용하지 않으며 full 시나리오에 아직 연결하지 않았습니다.

```bash
ros2 run ur3_moveit_examples zimmer_gripper open
ros2 run ur3_moveit_examples zimmer_gripper close
```

위는 dry-run으로 출력/ROS 연결을 하지 않습니다. 실행은 `--execute`가 필요하며
open/close에는 펜던트에서 확인한 충분한 출력 유지 시간을 `--hold-time`으로 지정합니다.
예를 들어 **1초가 실제 조건에서 확인된 경우에만**:

```bash
ros2 run ur3_moveit_examples zimmer_gripper open --execute --hold-time 1.0
ros2 run ur3_moveit_examples zimmer_gripper close --execute --hold-time 1.0
# 두 명령 신호 해제 (기계적 정지/그립 해제 보장 아님)
ros2 run ur3_moveit_examples zimmer_gripper off --execute
```

각 명령은 DO1/DO2 LOW를 서비스 응답 및 새 피드백으로 확인하고 50ms 대기 후
선택한 출력 하나만 HIGH로 만듭니다. 유지 시간이 지나면 두 출력을 LOW로 해제합니다.
피드백 단절/서비스 실패/중단 시 LOW를 최선 노력으로 시도합니다.
출력 성공은 그리퍼 열림/집기 완료를 의미하지 않습니다. 별도 완료 센서는 없습니다.

**중요:** 네트워크 단절, 강제 종료, 지연된 요청, 다른 출력 제어자 때문에 LOW를
보장할 수 없습니다. 서비스 timeout은 요청 취소가 아닙니다. 불확실 시 펜던트로
DO1/DO2를 확인해야 하며 자동 재시도하지 않습니다. OFF는 비상정지가 아닙니다.
펜던트/PLC/UR 프로그램 등에서 DO1/DO2를 동시에 제어하지 마세요. 로컬 잠금은
이 명령의 중복 실행만 제한합니다. 손과 공작물을 치우고 단독 시험부터 수행하세요.

새 그리퍼 장착으로 카메라 위치가 변했다면 핸드아이 보정, TCP, payload 및 충돌 모델을
재확인하기 전 기존 자동 팔 접근 시나리오를 실행하지 마세요.

참고: [UR I/O controller](https://docs.universal-robots.com/Universal_Robots_ROS2_Documentation/doc/ur_robot_driver/ur_controllers/doc/index.html),
[Zimmer GEP2000IO 설명서](https://www.zimmer-group.com/fileadmin/pim/ZIM/DOK/MON/ZIM_DOK_MON_DDOC02562__SALL__APD__V1.pdf).
제조사 설명서는 반대 명령 전 신호 reset 및 10ms 휴지를 요구합니다. 본 코드는
50ms를 사용합니다. 변형 모델/도표별 방향 표기가 있으므로 핀 방향은 사용자 실측 매핑을
기준으로 고정했으며 동작 유지 시간을 제조사 정격값으로 가정하지 않습니다.
