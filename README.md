# UR3 MoveIt 2 제어 예제 (ROS 2 Humble)

실제 Universal Robots UR3를 ROS 2 Humble과 MoveIt 2로 제어하기 위한 예제 패키지입니다.

이 패키지는 MoveIt의 `moveit_msgs/action/MoveGroup` 액션을 사용합니다. 코드에서
`FollowJointTrajectory` 액션을 직접 전송하지 않으며, MoveIt이 다음 작업을 수행합니다.

- 현재 로봇 상태 확인
- 역기구학(IK) 계산
- 충돌 검사와 경로 계획
- trajectory 생성
- 설정된 controller를 통한 실제 로봇 실행
- 계획 및 실행 결과 반환

현재 UR MoveIt 설정에서는 실행 trajectory가
`scaled_joint_trajectory_controller`로 전달됩니다.

> **안전 주의:** 기본 동작은 실제 로봇을 움직이지 않는 Plan-only입니다. RViz에서 목표와
> 계획 경로가 안전한지 확인한 후에만 `--execute` 옵션을 사용하세요.

## 요구 환경

- Ubuntu 22.04
- ROS 2 Humble
- MoveIt 2
- `ur_robot_driver`
- `ur_moveit_config`
- Universal Robots UR3
- Planning group: `ur_manipulator`
- Planning frame: `base_link`
- End-effector link: `tool0`

## 빌드

```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select ur3_moveit_examples --symlink-install
source ~/ros2_ws/install/setup.bash
```

빌드 후 새 터미널을 열었다면 다음 환경을 다시 불러와야 합니다.

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
```

## 실행 전 확인

UR 드라이버와 MoveIt을 먼저 실행하고, teach pendant에서 External Control 프로그램을
Play 상태로 만듭니다. 이후 다음 명령으로 ROS 상태를 확인합니다.

```bash
ros2 action list -t | grep move_action
ros2 action info /move_action
ros2 control list_controllers
ros2 topic echo /joint_states --once
```

다음 항목이 확인되어야 합니다.

```text
/move_action [moveit_msgs/action/MoveGroup]
scaled_joint_trajectory_controller ... active
```

`ros2 control` 명령이 없다면 CLI 패키지를 설치할 수 있습니다.

```bash
sudo apt install ros-humble-ros2controlcli
```

`scaled_joint_trajectory_controller`가 `inactive`인 상태에서는 계획에 성공하더라도 실제
실행 단계에서 `CONTROL_FAILED`가 발생합니다.

## 현재 TCP pose 확인

목표 orientation을 임의로 정하지 말고, 먼저 현재 `tool0`의 안전한 quaternion을 확인하는
것을 권장합니다.

```bash
ros2 run tf2_ros tf2_echo base_link tool0
```

출력되는 quaternion의 순서는 다음과 같습니다.

```text
qx qy qz qw
```

## 현재 로봇 상태 출력

이 패키지의 읽기 전용 노드로 UR3의 6축 joint position과 현재 TCP pose를 한 번에 확인할
수 있습니다. 이 명령은 로봇에 이동 명령을 보내지 않습니다.

```bash
ros2 run ur3_moveit_examples robot_state
```

joint position은 rad와 degree로 출력되며, TCP pose는 `base_link → tool0` 기준의 위치와
quaternion으로 출력됩니다. 기본 TF frame이나 대기 시간을 변경할 수도 있습니다.

```bash
ros2 run ur3_moveit_examples robot_state \
  --base-frame base_link \
  --ee-frame tool0 \
  --timeout 10.0
```

## 1단계: Plan-only

아래 인자의 순서는 `x y z qx qy qz qw`입니다. 위치 단위는 m이며 quaternion은 코드에서
자동으로 정규화합니다. `QX QY QZ QW`를 실제로 확인한 안전한 값으로 변경하세요.

```bash
ros2 run ur3_moveit_examples move_to_pose \
  -0.40 0.10 0.35 \
  QX QY QZ QW
```

`--execute` 옵션이 없으므로 MoveIt이 계획만 수행하고 실제 UR3는 움직이지 않습니다.

성공 시 예상 출력:

```text
MoveIt succeeded: SUCCESS; planning_time=...; trajectory_points=...; mode=PLAN ONLY
```

계획에 실패하면 `NO_IK_SOLUTION`, `GOAL_IN_COLLISION`, `START_STATE_IN_COLLISION`,
`PLANNING_FAILED` 등의 MoveIt 오류가 출력됩니다.

## 2단계: 실제 로봇 실행

Plan-only 결과와 RViz 경로를 확인한 후, 동일한 목표에 `--execute` 옵션을 추가합니다.

```bash
ros2 run ur3_moveit_examples move_to_pose \
  -0.40 0.10 0.35 \
  QX QY QZ QW \
  --execute \
  --velocity-scaling 0.1 \
  --acceleration-scaling 0.1
```

MoveIt은 경로를 계획하고 설정된 controller manager를 통해 trajectory를 실행한 뒤 완료
결과를 기다립니다.

성공 시 예상 출력:

```text
MoveIt succeeded: SUCCESS; ...; mode=PLAN + EXECUTE
Goal verification succeeded: position_error=... m, orientation_error=... rad
```

MoveIt 실행 성공 후 최신 TF를 조회하여 실제 TCP가 목표 위치 및 자세 허용 오차 안에
도달했는지 추가로 검사합니다. 계획, controller 실행 또는 최종 상태 검증에 실패하면
오류를 출력하고 종료 코드 `1`로 종료합니다.

`--execute`를 사용해도 MoveIt에는 먼저 Plan-only 요청을 보냅니다. 반환된 trajectory의
관절 이동량을 검사한 뒤, 검사한 동일 trajectory를 `/execute_trajectory` 액션으로
실행합니다. 실행 단계에서 IK나 경로를 다시 계산하지 않습니다.

Pose 목표는 `/compute_ik` 서비스에 최신 `/joint_states`를 seed로 제공하여 먼저
collision-aware IK joint 해로 변환합니다. 반환된 IK 해가 현재 관절 자세에서 설정된 최대
이동량보다 멀면 계획 전에 거부합니다. 통과한 IK 해를 명시적인 joint goal로 사용하므로
pose constraint sampler가 먼 elbow/wrist configuration이나 `2π` winding 해를 다시
선택하지 않습니다.

기본적으로 한 관절이라도 계획 시작점에서 `0.5rad`보다 멀리 움직이는 계획은 실행 전에
차단됩니다.

```text
Execution blocked: planned joint travel exceeds the configured limit.
```

의도적으로 더 먼 이동이 필요할 때만 Plan-only 결과를 확인하고 제한값을 명시적으로
높이세요.

```bash
ros2 run ur3_moveit_examples move_to_pose \
  X Y Z QX QY QZ QW \
  --execute \
  --max-joint-travel 1.0
```

## Joint 목표 이동

`move_to_joint`는 UR3의 6축 joint 목표를 MoveIt으로 계획합니다. 입력 순서는 다음과
같습니다.

```text
shoulder_pan_joint shoulder_lift_joint elbow_joint
wrist_1_joint wrist_2_joint wrist_3_joint
```

기본 입력 단위는 rad이며, `--execute`가 없으면 Plan-only로 동작합니다. 아래의 `J1`부터
`J6`까지는 임의의 예제 값이 아니라 `robot_state`로 확인한 현재 값과 가까운 안전한 목표로
교체해야 합니다.

```bash
ros2 run ur3_moveit_examples move_to_joint \
  J1 J2 J3 J4 J5 J6
```

degree 단위로 입력하려면 `--degrees`를 사용합니다.

```bash
ros2 run ur3_moveit_examples move_to_joint \
  J1 J2 J3 J4 J5 J6 \
  --degrees
```

Plan-only와 RViz 경로를 확인한 후 동일한 목표를 실제로 실행합니다.

```bash
ros2 run ur3_moveit_examples move_to_joint \
  J1 J2 J3 J4 J5 J6 \
  --execute \
  --velocity-scaling 0.1 \
  --acceleration-scaling 0.1
```

실행 후에는 최신 `/joint_states`를 이용하여 모든 joint가 목표 허용 오차 안에 들어왔는지
검사합니다. 기본 검증 허용 오차는 `0.01rad`, 제한 시간은 3초입니다.

최초 실제 시험에서는 현재 joint 값 중 하나만 약 `0.03~0.05rad` 변경하는 것을
권장합니다. MoveIt은 robot model의 joint limit과 충돌 여부를 검사하며, 유효하지 않은
목표나 경로는 실행하지 않습니다.

## 주요 옵션

```text
--execute                       실제 trajectory 실행
--velocity-scaling VALUE        속도 배율, 기본값 0.1
--acceleration-scaling VALUE    가속도 배율, 기본값 0.1
--planning-time VALUE           최대 계획 시간, 기본값 5.0초
--planning-attempts VALUE       계획 시도 횟수, 기본값 5
--ik-timeout VALUE              IK 계산 제한 시간, 기본값 1초
--ik-joint-tolerance VALUE      계획할 IK joint 목표 오차, 기본값 0.001rad
--verify-joint-tolerance VALUE  실행 후 joint 검증 오차, 기본값 0.01rad
--verify-position-tolerance VALUE
                                실행 후 TCP 위치 검증 오차, 기본값 0.005m
--verify-orientation-tolerance VALUE
                                실행 후 TCP 자세 검증 오차, 기본값 0.02rad
--verify-timeout VALUE          실행 후 목표 검증 제한 시간, 기본값 3초
--max-joint-travel VALUE        관절별 최대 계획 이동량, 기본값 0.5rad
```

속도 및 가속도 배율은 `(0, 1]` 범위만 허용됩니다.

## 안전 수칙

- 최초 실제 실행은 reduced/manual mode에서 수행하세요.
- 작업 영역에서 사람과 불필요한 장애물을 제거하세요.
- teach pendant와 비상정지를 즉시 사용할 수 있게 유지하세요.
- 장착 공구, 테이블 및 작업물이 MoveIt planning scene에 없다면 충분한 안전거리를 두세요.
- 현재 TCP pose와 가까운 목표부터 낮은 속도로 시험하세요.
- 실행 전 `scaled_joint_trajectory_controller`가 `active`인지 확인하세요.
- 실제 실행 전에 항상 동일한 목표를 Plan-only로 먼저 검증하세요.

기본 실행 후 위치 검증 오차는 5mm, orientation 검증 오차는 0.02rad입니다. 목표 pose는
`base_link` 좌표계에서 표현한 `tool0`의 pose입니다.

## Python 클래스 API

여러 동작을 하나의 ROS 노드에서 연속 실행할 때는 `UR3MoveItController` 클래스를 사용할
수 있습니다. 호출 측에서 `rclpy.init()`과 종료 처리를 담당합니다.

```python
import rclpy

from ur3_moveit_examples import UR3MoveItController


rclpy.init()
robot = UR3MoveItController(
    velocity_scaling=0.05,
    acceleration_scaling=0.05,
    max_joint_travel=0.15,
)

try:
    joints = robot.get_current_joint_positions()
    pose = robot.get_current_pose()

    planned = robot.move_to_pose(
        x=0.01,
        y=-0.34,
        z=0.59,
        qx=0.18,
        qy=0.69,
        qz=-0.57,
        qw=0.39,
        execute=False,
    )

    # 실제 실행은 동일한 호출에 execute=True를 명시합니다.
    # 내부에서 다시 계획하지만, 그 계획을 검사한 뒤 동일 trajectory만 실행합니다.
finally:
    robot.destroy_node()
    rclpy.shutdown()
```

제공 메서드:

```text
get_current_joint_positions()
get_current_pose()
move_to_joint(joint_positions, execute=False)
move_to_pose(x, y, z, qx, qy, qz, qw, execute=False)
move_relative_tool(dx, dy, dz, execute=False)
move_cartesian_relative_tool(dx, dy, dz, execute=False)
move_cartesian_to_pose(x, y, z, qx, qy, qz, qw, execute=False)
```

기존 `ros2 run` 명령도 동일하게 유지되며, 내부적으로 이 클래스를 호출합니다.

## 연속 동작 예제

`sequence_demo`는 실행 시작 시점의 TCP pose를 저장하고, `base_link`의 Z축 방향으로
10mm 상승한 뒤 원래 pose로 복귀합니다. 먼저 상승 경로만 Plan-only로 확인합니다.

```bash
ros2 run ur3_moveit_examples sequence_demo
```

Plan-only가 성공하고 RViz에서 경로를 확인한 뒤 실제 연속 동작을 실행합니다.

```bash
ros2 run ur3_moveit_examples sequence_demo \
  --execute \
  --lift-distance 0.01 \
  --max-joint-travel 0.15 \
  --velocity-scaling 0.05 \
  --acceleration-scaling 0.05
```

각 단계는 별도로 IK, collision-aware planning, trajectory 이동량 검사, 실행 및 TCP 도달
검증을 수행합니다. 상승 단계가 실패하면 복귀 명령은 실행하지 않습니다. Plan-only에서는
로봇의 실제 상태가 변하지 않으므로 복귀 단계는 생략합니다.

## 공구 좌표계 기준 상대 이동

`move_relative_tool`은 현재 `tool0` 좌표계에서 지정한 거리만큼 TCP를 평행 이동합니다.
단위는 metre이며 현재 TCP orientation은 유지됩니다. 예를 들어 다음 명령은 공구의
`+Z`축 방향으로 5mm 이동하는 경로만 계획합니다.

```bash
ros2 run ur3_moveit_examples move_relative_tool 0 0 0.005
```

로그에 공구 좌표계 이동량과 `base_link`로 변환된 이동량이 모두 출력됩니다. RViz에서
방향과 경로를 확인한 뒤 실행합니다.

```bash
ros2 run ur3_moveit_examples move_relative_tool \
  0 0 0.005 \
  --execute \
  --max-joint-travel 0.15 \
  --velocity-scaling 0.05 \
  --acceleration-scaling 0.05
```

반대 방향은 `dz`에 음수를 입력합니다. 실제 장착 공구의 접근 방향이 `tool0`의 `+Z`인지
`-Z`인지는 Plan-only 경로와 TF 축을 통해 반드시 먼저 확인해야 합니다. 이 명령은 목표
pose까지 MoveIt 일반 planning을 수행하며 TCP가 직선을 따라간다는 보장은 없습니다.
5mm 시험의 기본 위치 검증 허용오차는 3mm입니다.

## 공구축 Cartesian 직선 이동

`move_cartesian_tool`은 현재 공구 좌표계의 상대 이동 목표를 계산한 뒤 MoveIt Humble의
`/compute_cartesian_path`를 사용하여 TCP 직선 경로를 생성합니다. 먼저 공구 `+Z` 방향
5mm 경로를 Plan-only로 확인합니다.

```bash
ros2 run ur3_moveit_examples move_cartesian_tool \
  0 0 0.005 \
  --max-step 0.001 \
  --max-joint-travel 0.15
```

경로 fraction이 기본값 `0.999` 이상이어야 수락되며 collision avoidance와 joint jump
검사를 적용합니다. RViz에서 직선 방향과 주변 간섭을 확인한 뒤 실행합니다.

```bash
ros2 run ur3_moveit_examples move_cartesian_tool \
  0 0 0.005 \
  --execute \
  --max-step 0.001 \
  --max-joint-travel 0.15 \
  --velocity-scaling 0.05 \
  --acceleration-scaling 0.05
```

Humble의 Cartesian 서비스가 생성한 시간화 trajectory를 지정한 scaling 이하가 되도록
더 느리게 조정하고, 검사한 동일 trajectory를 실행합니다. `fraction < 0.999`, 충돌,
joint jump 또는 최대 관절 이동량 초과가 발생하면 실행하지 않습니다.

## 접근·후퇴 시나리오

`approach_retreat_demo`는 실행 시작 pose를 저장하고, 공구 Z축으로 Cartesian 직선 접근한
뒤 같은 거리만큼 반대 방향으로 후퇴합니다. 빈 공간에서만 시험하세요. 기본값은 공구
`+Z` 방향 5mm이며 Plan-only에서는 접근 경로만 계산합니다.

```bash
ros2 run ur3_moveit_examples approach_retreat_demo \
  --distance 0.005
```

RViz에서 접근 방향과 직선 경로를 확인한 뒤 실제 시나리오를 실행합니다.

```bash
ros2 run ur3_moveit_examples approach_retreat_demo \
  --distance 0.005 \
  --execute \
  --dwell-time 1.0 \
  --max-step 0.001 \
  --max-joint-travel 0.15 \
  --velocity-scaling 0.05 \
  --acceleration-scaling 0.05
```

접근이 성공한 경우에만 저장한 시작 TCP pose를 절대 Cartesian 목표로 삼아 후퇴하며,
마지막에는 시작 pose와 최종 pose의 위치 및 자세 오차를 검사합니다. 반대 방향으로
접근하려면 `--distance -0.005`를 사용합니다.

## 발표용 3단계 관절 이동

`three_move_demo`는 현재 joint 자세를 저장하고 선택한 관절을 시작값 기준 `+offset`,
`-offset`, 시작값 순서로 이동합니다. 기본 관절은 `shoulder_pan_joint`, offset은
`0.05rad`(약 2.86도)입니다. 기본 Plan-only에서는 첫 번째 목표만 계획합니다.

```bash
ros2 run ur3_moveit_examples three_move_demo \
  --joint shoulder_pan_joint \
  --offset 0.05 \
  --max-joint-travel 0.15
```

RViz에서 첫 번째 경로와 주변 작업 공간을 확인하고, scaled trajectory controller가
active인지 확인한 뒤 세 동작을 실행합니다.

```bash
ros2 run ur3_moveit_examples three_move_demo \
  --joint shoulder_pan_joint \
  --offset 0.05 \
  --execute \
  --dwell-time 0.5 \
  --max-joint-travel 0.15 \
  --velocity-scaling 0.05 \
  --acceleration-scaling 0.05
```

두 번째 동작은 `+offset`에서 `-offset`까지 이동하므로 이동량이 offset의 두 배입니다.
따라서 `2 * offset <= max_joint_travel` 조건을 만족하지 않으면 시작 전에 오류로
차단합니다. 각 단계는 별도로 MoveIt 계획, trajectory 검사, 실행 및 joint 도달 검증을
수행하며 실패하면 다음 단계로 진행하지 않습니다.

## 발표용 전체 팔 협조 이동

`arm_sweep_demo`는 `shoulder_lift_joint`, `elbow_joint`, `wrist_1_joint`을 동시에 조금씩
변경하여 자세 A, 반대 방향의 자세 B, 저장한 시작 자세 순서로 이동합니다. 관절 범위
끝에 가까운 `shoulder_pan_joint`는 변경하지 않습니다. 먼저 자세 A만 Plan-only로
확인합니다.

```bash
ros2 run ur3_moveit_examples arm_sweep_demo \
  --motion-scale 1.0 \
  --max-joint-travel 0.15 \
  --velocity-scaling 0.05 \
  --acceleration-scaling 0.05
```

RViz와 로그에서 경로, 충돌 여부, 계획 시작·종료점 검증, 최대 관절 이동량을 확인한 뒤
실행합니다.

```bash
ros2 run ur3_moveit_examples arm_sweep_demo \
  --motion-scale 1.0 \
  --execute \
  --dwell-time 0.7 \
  --max-joint-travel 0.15 \
  --velocity-scaling 0.05 \
  --acceleration-scaling 0.05
```

기본 자세 A 변화량은 shoulder lift `+0.04rad`, elbow `-0.06rad`, wrist 1
`+0.02rad`이며 자세 B는 반대 부호입니다. A에서 B까지 elbow 이동량은 `0.12rad`입니다.
`--motion-scale`을 높이면 동작이 커지지만 A-B 최대 이동량이 안전 제한을 넘으면 시작
전에 차단합니다.

## 관절 티칭 기반 Pick & Place

`pick_and_place_fk_demo`는 카메라 없이 미리 티칭한 관절 위치를 사용하는 Pick & Place
프로그램입니다. 현재 첫 단계로 실제 UR3에서 측정한 `HOME_JOINTS`만 등록되어 있습니다.
Pick과 Place 관절 위치는 아직 등록하지 않았으므로 실행되지 않습니다.

먼저 홈 이동을 Plan-only로 검사합니다.

```bash
ros2 run ur3_moveit_examples pick_and_place_fk_demo \
  --max-joint-travel 0.30 \
  --velocity-scaling 0.03 \
  --acceleration-scaling 0.03
```

RViz에서 계획 경로와 로그의 시작점·최종점·최대 관절 이동량을 확인한 후 실제 홈 이동을
실행합니다.

```bash
ros2 run ur3_moveit_examples pick_and_place_fk_demo \
  --execute \
  --max-joint-travel 0.30 \
  --velocity-scaling 0.03 \
  --acceleration-scaling 0.03
```

현재 자세에서 홈까지 어느 한 관절이라도 `--max-joint-travel`을 초과하면 실행이
차단됩니다. 제한값은 계획 결과를 확인하지 않고 임의로 높이지 마세요.

## WEISS IEG 55-020 읽기 전용 통신 시험

`weiss_gripper_read_state`는 UR3 컨트롤러에서 실행 중인 GRIPKIT XML-RPC daemon에
접속하여 `GetState`와 `GetPos`만 호출합니다. `Reference`, `Release`, `Grip`은 호출하지
않으므로 이 단계에서는 그리퍼의 의도적인 움직임이 없어야 합니다.

```bash
ros2 run ur3_moveit_examples weiss_gripper_read_state
```

현재 기본 연결 정보는 다음과 같습니다.

```text
URL:       http://192.168.1.11:44221/RPC2
device ID: IEG 55-020 SN:000234
timeout:   3.0 s
```

주소나 장치가 변경된 경우에만 옵션으로 덮어씁니다.

```bash
ros2 run ur3_moveit_examples weiss_gripper_read_state \
  --url http://192.168.1.11:44221/RPC2 \
  --device-id "IEG 55-020 SN:000234" \
  --timeout 3.0
```

프로그램은 각 응답의 Python 타입, `repr`, 문자열 표현을 출력합니다. 이 실제 반환값을
확인하기 전에는 상태 코드 의미를 가정하거나 동작 명령을 실행하지 않습니다.

확보한 GRIPKIT 1.2.0 URScript에서 확인된 상태값은 다음과 같습니다.

```text
-1 = COMMUNICATION_FAULT
 0 = NOT_REFERENCED
 1 = IDLE
 2 = RELEASED
 4 = NO_PART
 8 = HOLDING
```

### Reference 시험

기본 실행은 현재 상태만 확인하며 그리퍼를 움직이지 않습니다.

```bash
ros2 run ur3_moveit_examples weiss_gripper_reference
```

출력에서 `NOT_REFERENCED`를 확인하고 그리퍼 핑거 주변을 완전히 비운 뒤에만 Reference를
실행합니다. Reference 과정에서 그리퍼가 움직일 수 있습니다.

```bash
ros2 run ur3_moveit_examples weiss_gripper_reference \
  --execute \
  --operation-timeout 15.0
```

프로그램은 `Reference`를 한 번만 호출하고 `GetState`와 `GetPos`를 polling합니다. 이미
참조된 상태라면 다시 Reference하지 않으며, 통신 장애 또는 timeout 발생 시 실패로
종료합니다.

### Release 시험

Reference 완료 후 기본 실행으로 현재 상태만 확인합니다.

```bash
ros2 run ur3_moveit_examples weiss_gripper_release
```

핑거가 열리는 전체 공간을 비운 후 Grip Configuration 1(index 0)의 Release를 실행합니다.

```bash
ros2 run ur3_moveit_examples weiss_gripper_release \
  --execute \
  --index 0 \
  --operation-timeout 10.0
```

프로그램은 `Release`를 한 번만 호출하고 상태가 `RELEASED(2)`가 될 때까지 polling합니다.
`NOT_REFERENCED(0)`, `COMMUNICATION_FAULT(-1)` 또는 timeout은 실패로 처리하며, 이미
`RELEASED` 상태이면 동작 명령을 다시 보내지 않습니다.

### Grip 빈 공간 시험

첫 Grip 시험은 핑거 사이를 완전히 비우고 `RELEASED(2)`에서 시작합니다. 기본 기대
결과는 `NO_PART(4)`입니다.

```bash
ros2 run ur3_moveit_examples weiss_gripper_grip
```

상태 확인 후 실제로 핑거를 닫습니다.

```bash
ros2 run ur3_moveit_examples weiss_gripper_grip \
  --execute \
  --index 0 \
  --expect no-part \
  --operation-timeout 10.0
```

프로그램은 시작 상태가 `RELEASED(2)`가 아니면 Grip을 차단합니다. `Grip`을 한 번만
호출하고 `NO_PART(4)` 또는 `HOLDING(8)`까지 polling하며, 실제 결과가 `--expect`와
다르면 실패로 종료합니다. 물체를 사용한 시험은 빈 공간 시험이 완료된 이후
`--expect holding`으로 별도 수행합니다.

### 통신 구조와 실제 검증 결과

그리퍼는 ROS2 PC의 USB에 직접 연결되어 있지 않습니다. Python 프로그램이 LAN으로 UR3
컨트롤러 내부의 WEISS GRIPKIT daemon에 XML-RPC 요청을 보내며, daemon이 USB와
DC-IOLINK를 거쳐 IEG 55-020을 제어합니다.

```text
ROS2 PC (192.168.1.12)
  └─ Ethernet / XML-RPC
       └─ UR3 Controller (192.168.1.11:44221)
            └─ WEISS GRIPKIT daemon
                 └─ USB
                      └─ DC-IOLINK
                           └─ IO-Link
                                └─ IEG 55-020 SN:000234
```

현재 그리퍼 시험 프로그램은 `ros2 run`으로 실행되지만 실제 장치 통신에는 ROS2
Topic/Service가 아닌 Python 표준 `xmlrpc.client`를 사용합니다. 추후 검증된 XML-RPC
클라이언트를 ROS2 Service와 상태 Topic으로 감쌀 예정입니다.

실제 장비에서 다음 결과를 확인했습니다.

| 시험 | 상태 변화 | 위치 변화 | 결과 |
|---|---|---|---|
| 읽기 | `NOT_REFERENCED(0)` | `-1.9mm` | `GetState=int`, `GetPos=float` |
| Reference | `0 → IDLE(1)` | `-1.9 → 5.7 → 21.1mm` | 성공 |
| Release | `1 → RELEASED(2)` | `21.1 → 20.0mm` | 성공 |
| 빈 Grip | `2 → NO_PART(4)` | `20.0 → 1.0mm` | 예상 결과로 성공 |
| 물체 Grip | `2 → NO_PART(4)` | `20.0 → 1.0mm` | `HOLDING(8)` 미검출 |

물체 Grip 시험 및 티치펜던트 Grip에서 한쪽 핑거만 움직이는 현상이 확인되었습니다.
이는 ROS2/XML-RPC와 무관한 기계적 간섭, 핑거 체결 또는 그리퍼 내부 기구 문제일 수
있으므로 원인을 점검하기 전까지 물체 파지 및 로봇팔과의 통합 실행을 중단합니다.
Force 증가나 No Part Limit 변경으로 우회하지 않습니다.
