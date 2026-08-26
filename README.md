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
```

계획이나 controller 실행에 실패하면 오류를 출력하고 종료 코드 `1`로 종료합니다.

## 주요 옵션

```text
--execute                       실제 trajectory 실행
--velocity-scaling VALUE        속도 배율, 기본값 0.1
--acceleration-scaling VALUE    가속도 배율, 기본값 0.1
--planning-time VALUE           최대 계획 시간, 기본값 5.0초
--planning-attempts VALUE       계획 시도 횟수, 기본값 5
--position-tolerance VALUE      위치 허용 오차, 기본값 0.005m
--orientation-tolerance VALUE   자세 허용 오차, 기본값 0.01rad
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

기본 위치 허용 오차는 5mm, orientation 허용 오차는 0.01rad입니다. 목표 pose는
`base_link` 좌표계에서 표현한 `tool0`의 pose입니다.
