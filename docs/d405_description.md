# UR3 끝단 D405 — 임시 Description

> 2026-09-11: 기본 장착값을 시험용 PARK 보정값으로 변경했습니다.
> 아래 사진 기반 위치 설명은 이전 배치 기록입니다. 현재 값과 복원 방법은
> [시험용 적용 기록](handeye_application.md)을 우선 참고하세요.

## 체커보드 검출 미리보기 (로봇 이동 없음)

가로 9칸 × 세로 6칸, 실측 한 칸 20 mm 보드용입니다.
내부 교차점은 8 × 5입니다. 먼저 패키지를 빌드하고 환경을 source합니다.

```bash
cd /home/unieye/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select ur3_moveit_examples
source install/setup.bash
ros2 run ur3_moveit_examples checkerboard_live
```

RealSense 카메라 노드는 별도 터미널에서 실행되어 있어야 합니다.
입력은 `/camera/camera/color/image_raw`, `/camera/camera/color/camera_info`입니다.
보드 전체가 보이면 교차점과 XYZ 축(빨강/초록/파랑)을 표시합니다.
표시되는 xyz는 카메라 optical frame 기준 보드의 첫 내부 교차점 위치(m),
reproj는 교차점 재투영 RMS 오차(pixel)입니다. 보드 중심 좌표가 아닙니다.
원점 방향이 뒤집히는지 확인하세요. 작은 reproj 값만으로 정확도가 보장되지 않습니다.
이 노드는 데이터 수집/Hand-eye 계산/TF 발행/로봇 제어를 하지 않습니다.
보드 좌표계 방향의 일관성 검증 전에는 출력값을 로봇 제어에 사용하지 마세요.
구현 참고: https://docs.opencv.org/4.13.0/dc/d43/tutorial_camera_calibration_square_chess.html

창에서 q 또는 ESC, 터미널에서 Ctrl+C로 종료합니다. 창이 없는 환경에서는
`--no-window`를 사용하고 `/checkerboard/image`를 RViz Image로 확인합니다.
raw 영상 전용이며 rectified 영상에는 그대로 사용하지 않습니다.

그리퍼 없는 현재 구성입니다. 공식 UR Description에 공식 RealSense D405
메시와 충돌 박스를 추가하며, 설치된 공식 패키지는 수정하지 않습니다.

## 기준과 한계

- 카메라 하우징 중심을 플랜지 중심에서 카메라의 위쪽으로 52.5 mm,
  좌우 편차 없이 임시 배치합니다. 카메라 앞면은 플랜지 끝단면에 맞춥니다.
  공식 visual 앞면 오프셋 3.8 mm를 보상한 뒤, RViz 화면을 기준으로
  총 2 mm 더 뒤로 미세 조정하여 tool0 Z를 -5.8 mm로 둡니다.
  사진 기반 임시 조정이며 실측값이나 캘리브레이션 결과는 아닙니다.
  공식 충돌 박스 중심을 하우징 중심의 근사로 사용합니다.
  `camera_link`는 하우징 중심과 다르므로 그 차이를 보상한 tool0 기준 위치는
  `-0.0480961272 -0.0228913204 -0.0058` m입니다.
- 카메라 본체 +X(보는 방향)를 tool0 +Z 방향으로 가정합니다.
  사진만으로 장착 회전을 확정한 값은 아닙니다.
- 임시 RPY는 `0.6140009093 -1.5707963267948966 0`입니다.
  사용자가 제공한 TCP quaternion `(0.682646, 0.158293, -0.660869, -0.268681)`에서
  base_link +Z를 위로 가정하고, 카메라 +Z를 그 방향에 최대한 맞췄습니다.
  기존 배치에서 광축 주위 약 35.18도 보정이며 실측 장착각은 아닙니다.
  고정 변환이므로 다른 로봇 자세에서는 카메라도 로봇과 함께 회전합니다.
- `urdf/ur3_d405.urdf.xacro`의 `camera_xyz`, `camera_rpy` 기본값이
  tool0 → camera_link 변환입니다. 단위는 m, rad입니다.
- Hand-eye calibration은 위치와 회전을 모두 추정합니다. 결과가 optical frame
  기준이면 camera_link 기준으로 변환해서 적용해야 합니다.
- 보정값은 이 고정 joint에 반영합니다. 같은 camera_link에 별도 static TF를
  동시에 발행하지 마세요.
- 테이프가 움직이면 보정값이 무효가 됩니다. 안정적인 고정이 선행되어야 합니다.
- 임시 위치에서는 카메라 충돌 박스가 손목과 겹칠 수 있습니다. 충돌 검사를
  끄지 말고 실제 장착 위치를 반영하세요. 테이프·케이블 형상은 모델에 없습니다.
  현재 모델은 시각화/TF 연결 준비용이며 실제 이동 안전성을 보증하지 않습니다.

## 적용

실행 중인 이동 작업을 끝내고 로봇이 정지한 상태에서 기존 driver/MoveIt launch를
종료한 뒤 교체합니다. 중복 실행하지 마세요. 아래 명령은 각각 별도 터미널입니다.
먼저 워크스페이스에서 빌드합니다.

```bash
cd /home/unieye/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select ur3_moveit_examples
source install/setup.bash
```

각 터미널에서 환경을 source한 뒤 실행합니다. `description_package`는
`ur_description` 그대로 두고 `description_file`만 절대 경로로 교체합니다.
따라서 기존 UR 관절 제한 및 하드웨어 설정 경로는 유지됩니다.

```bash
source /opt/ros/humble/setup.bash
source /home/unieye/ros2_ws/install/setup.bash
ros2 launch ur_robot_driver ur_control.launch.py \
  ur_type:=ur3 robot_ip:=192.168.1.11 \
  kinematics_params_file:=$HOME/ur3_calibration.yaml \
  description_file:=/home/unieye/ros2_ws/src/ur3_moveit_examples/urdf/ur3_d405.urdf.xacro \
  launch_rviz:=false
```

```bash
source /opt/ros/humble/setup.bash
source /home/unieye/ros2_ws/install/setup.bash
ros2 launch ur3_moveit_examples ur3_d405_moveit.launch.py \
  ur_type:=ur3 robot_ip:=192.168.1.11 \
  kinematics_params_file:=$HOME/ur3_calibration.yaml \
  description_file:=/home/unieye/ros2_ws/src/ur3_moveit_examples/urdf/ur3_d405.urdf.xacro \
  launch_rviz:=true
```

기존 공식 MoveIt launch 대신 위 프로젝트 launch를 사용해야 driver와 같은
실기 보정 YAML을 읽습니다. `kinematics_params_file`은 필수 인자이며 자동으로
기본 로봇 모델에 대체되지 않습니다. 이 launch는 설치된 Humble 공식 launch를
기반으로 하며 원본 라이선스를 유지합니다. 패키지 업그레이드 시 비교가 필요합니다.

```bash
source /opt/ros/humble/setup.bash
ros2 launch realsense2_camera rs_launch.py \
  camera_name:=camera camera_namespace:=camera \
  enable_color:=true enable_depth:=true publish_tf:=true
```

RealSense Viewer나 다른 카메라 노드와 동시에 장치를 열지 마세요.

## 확인 (로봇 이동 없음)

```bash
ros2 run tf2_ros tf2_echo tool0 camera_link
ros2 run tf2_ros tf2_echo base_link camera_color_optical_frame
```

첫 변환의 translation은 임시값 `[-0.04810, -0.02289, -0.0058]` m이어야 합니다. RViz Fixed Frame은
`base_link`로 두고 RobotModel과 TF를 확인합니다. 표시되는 큰 회전 링은
MoveIt interactive marker일 수 있으므로, 정확한 축은 TF 표시에서 확인하세요.

TF 발행 책임:

```text
robot_state_publisher: ... → tool0 → d405_mount → camera_bottom_screw_frame → camera_link
RealSense driver:     camera_link → camera_*_frame → camera_*_optical_frame
```

기존 launch 명령에서 `description_file`을 빼면 원래 카메라 없는 모델로 돌아갑니다.
