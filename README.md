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
```

기존 `ros2 run` 명령도 동일하게 유지되며, 내부적으로 이 클래스를 호출합니다.
