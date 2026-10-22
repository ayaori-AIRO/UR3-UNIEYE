# 관절 목표 반복성 시험

기존 가위/그리퍼 코드를 변경하지 않는 별도 시험입니다. 2026-10-08 제공한
HOME/A/B 관절값을 사용합니다. 실제 끝단 측정기가 아니라 엔코더와 TF 기준이며
그리퍼 끝이 아닌 base_link → tool0를 기록합니다. ISO 인증/정격 검증이 아닙니다.

Driver, 실제 보정값을 사용하는 MoveIt을 준비하세요. 실행에는 External Control이
필요합니다. 카메라/YOLO/그리퍼 출력 명령은 사용하지 않습니다.
새 그리퍼가 장착돼 있다면 충돌 형상과 payload를 확인하고 경로 전체를 점검하세요.

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
cd ~/ros2_ws/src/ur3_moveit_examples
# 현재 -> HOME 계획만; 전체 경로가 검증되는 것은 아닙니다.
python3 -m ur3_moveit_examples.demos.joint_repeatability --plan-target home
# 현재 -> A 또는 B 계획만
python3 -m ur3_moveit_examples.demos.joint_repeatability --plan-target a
python3 -m ur3_moveit_examples.demos.joint_repeatability --plan-target b
```

기본 이동량 한도 2.20rad, 속도/가속도 배율 각 0.03입니다. A↔B shoulder_pan
차이가 약 2.133rad이지만 실제 계획이 더 길면 중단합니다. 한도는 자동으로 늘리지 않습니다.

전체 경로 확인 후 첫 실행:

```bash
python3 -m ur3_moveit_examples.demos.joint_repeatability --execute --cycles 1
```

실제 순서는 HOME → A → B → A, 이후 A에서 종료하며 HOME 복귀하지 않습니다.
한 번 시험이 적절한 것을 확인한 뒤 --cycles 30으로 늘릴 수 있습니다.
매 구간은 새로 계획하므로 궤적 자체가 매회 동일하다고 보장하지 않습니다.
실패/측정 불가 시 다음 이동을 중단합니다. Ctrl+C가 하드웨어 정지를 보장하지 않으므로
펜던트 정지 수단을 준비하고 실제 정지를 확인하세요.

도착 후 기본 2초 기다리고 최근 약 1초 관절 변동이 0.001rad 이내인지 확인합니다.
신선한 관절 메시지와 동일한 시각의 TF만 사용합니다. 목표 관절 허용 오차는
기존 controller 설정을 사용합니다. HOME에서는 측정하지 않습니다.

출력: ~/ur3_repeatability/고유실행폴더/

- settings.json: 목표/속도/검사 설정
- samples.csv: cycle, 도착점, 출발점, 관절 시각, 6개 관절값, XYZ/쿼터니언, 목표 관절 최대 오차
- summary.json: 성공/실패 사유, 출발점별 XYZ 평균(m), 변동 폭(mm), 표본 표준편차(mm)

HOME→A 초기 측정은 B→A 반복 측정과 분리합니다. 한 번 측정의 표준편차는 null입니다.
CSV는 매 측정마다 flush하며 실패해도 앞선 기록은 유지합니다. 강제 종료/전원 차단은
최종 summary 저장을 보장하지 않습니다. 물리적 유격·변형은 외부 측정기가 필요합니다.
