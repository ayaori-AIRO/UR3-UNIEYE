# Eye-in-Hand HOME → 가위 관찰 위치

## 전체 시나리오 한 번 실행

Driver/MoveIt/D405(정렬 깊이 포함)는 미리 실행합니다. 기존 scissors_position과
YOLO 라이브 노드는 종료하세요. full은 전용 자동 검출 프로세스를 직접 시작합니다.

```bash
ros2 run ur3_moveit_examples eye_in_hand_pick_and_place \
  --stage full --execute --approach-height 0.20 \
  --confidence 0.4 --candidate-timeout 0 \
  --max-joint-travel 2.10 --velocity-scaling 0.03 --acceleration-scaling 0.03
```

HOME → 관찰 위치 도착 확인 → 1초 정착 → 자동 검출 실행 → 새 후보점 하나 고정
→ 검출 프로세스 종료 → 현재 관절 seed IK → 접근 후 종료합니다.
그리퍼, 하강, 자동 HOME 복귀는 없습니다. 추가 승인창 없이 각 단계를 실행합니다.
후보점 대기만 무기한이며 검출 프로세스 종료/IK/이동 실패 시 시나리오를 중단합니다.
소유한 검출 프로세스만 종료하며 기존 Driver/MoveIt/카메라는 종료하지 않습니다.
full에 --execute가 없으면 현재 → HOME만 계획하며 검출을 시작하지 않습니다.
별도 자동 후보 발행자가 있으면 잘못된 출처의 점을 받을 수 있으므로 반드시 종료하세요.
접근 높이는 실제 카메라/작업대 여유를 보장하지 않습니다.

## 클릭한 가위 위쪽 접근 (별도 stage)

관찰 위치에서 로봇과 가위를 정지시키고 `scissors_position`을 실행합니다.
다음 명령은 **계획만** 합니다. 0.20 m는 예시 높이이며 실제 카메라/작업대
간격을 보장하지 않습니다. 목표는 가위 X/Y, 가위 Z + 높이의 tool0 위치입니다.

```bash
ros2 run ur3_moveit_examples eye_in_hand_pick_and_place \
  --stage approach --approach-height 0.20 \
  --max-joint-travel 2.10 --velocity-scaling 0.03 --acceleration-scaling 0.03
```

명령이 기다리는 동안 카메라 창에서 `R → S → 표면 한 번 클릭`합니다.
명령 시작 이후 촬영한 첫 유효 후보만 고정하고 구독을 종료합니다.
base_link 프레임, 유한 좌표, 최대 촬영 나이 15초를 검사하며 최대 60초 기다립니다.
이전 로그 좌표나 오래된 고정 화면은 사용하지 않습니다.
IK는 fresh current joints를 seed로 사용하고 방향은 저장된 관찰 quaternion입니다.
현재 방향을 측정해서 유지하는 것과 다르므로 관찰 자세에서 시작하세요.

실제 이동은 위 명령에 `--execute`를 추가한 뒤 **다시 새로 촬영·클릭**해야 합니다.
그 경우 재계획하며 이전 미리보기와 동일 경로를 보장하지 않습니다.
검사된 해당 실행 궤적만 실행합니다. 움직이는 물체를 추적하지 않습니다.
sequence는 기존대로 HOME → 관찰까지만 실행하며 approach를 자동 호출하지 않습니다.

높이는 최소 0.10 m를 요구하지만 이는 입력 제한일 뿐 충돌 안전 판정이 아닙니다.
관절 경로는 직선/수직 접근이 아닙니다. 작업대 등 실제 장애물이 MoveIt에 모델링되지
않았다면 충돌 검사에도 반영되지 않습니다. 카메라/케이블 여유 공간과 전체 경로를
확인하세요. 하강, 접촉, 그립, 자동 HOME 복귀는 없습니다.

`eye_in_hand_pick_and_place`는 HOME 관절 목표 도착을 확인한 뒤,
관찰 TCP 목표를 티칭 관절 seed로 IK 계산합니다. YOLO 실행이나 그리퍼
동작은 아직 포함하지 않습니다. 관찰 위치에서 종료합니다.

- 관찰 목표 (`base_link → tool0`, m): `-0.102946, -0.333081, 0.244248`
- 목표 quaternion (xyzw): `0.454060, 0.890464, 0.016411, 0.025199`
- IK seed (UR 관절 순서, rad): `1.580786, -1.751029, -1.695628, -1.270584, 1.622455, 0.954812`

Seed는 원하는 관절 구성 근처로 IK를 유도하며 최단 경로나 최소 관절 이동을
보장하지 않습니다. 기존 controller의 충돌 고려 IK, 현재 관절과의 차이 제한,
궤적 검사 및 실행 후 목표 검증을 사용합니다. 직선 TCP 이동은 아닙니다.
기본 속도/가속도 배율은 각각 0.03, 관절 이동 제한은 2.10 rad입니다.

HOME에서 관찰 위치까지 계획만 검사:

```bash
ros2 run ur3_moveit_examples eye_in_hand_pick_and_place --stage observe
```

관찰 위치만 실제 이동 (현재 위치에서 새로 계획):

```bash
ros2 run ur3_moveit_examples eye_in_hand_pick_and_place --stage observe --execute
```

전체 HOME → 관찰 위치 실행:

```bash
ros2 run ur3_moveit_examples eye_in_hand_pick_and_place --stage sequence --execute
```

HOME만 이동하려면 `--stage home --execute`를 사용합니다.
기본 stage는 sequence입니다. 단, `--execute` 없는 sequence는 현재 → HOME만
검사하고 종료합니다. 실제로 HOME에 이동하지 않았으므로 다음 구간까지
검사했다고 간주하지 않습니다. 단계 실패 시 후속 이동은 실행하지 않습니다.

실행 전 Driver/MoveIt/External Control 상태, 카메라 고정 및 케이블과 작업대의
여유 공간을 확인하세요. 계획 성공은 모델에 없는 장애물의 안전을 보장하지 않습니다.
