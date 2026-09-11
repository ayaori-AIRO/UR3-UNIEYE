# 수동 Hand-eye 후보 샘플 수집

이 단계는 계산/자동 이동/보정 적용을 하지 않습니다. 보드와 카메라 장착을
움직이지 않게 고정합니다. 9×6칸, 내부 교차점 8×5, 실측 20mm 전용입니다.

## 1. 빌드 및 로봇 모델 통일

```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select ur3_moveit_examples
source install/setup.bash
```

로봇 정지 후 기존 driver/MoveIt을 종료하고 [D405 안내](d405_description.md)의
새 실행 명령으로 교체합니다. driver는 기존 `ur_control.launch.py`, MoveIt은
프로젝트의 `ur3_d405_moveit.launch.py`를 사용하며 둘 다
`kinematics_params_file:=$HOME/ur3_calibration.yaml`을 지정합니다.
기존 공식 `ur_moveit.launch.py`를 그대로 사용하면 통일되지 않습니다.
보정 파일이 실제 연결된 로봇에서 추출된 것인지도 확인하세요.

## 2. 카메라와 수집 노드

RealSense 노드는 기존처럼 실행합니다. `checkerboard_live`는 종료하세요.

```bash
ros2 run ur3_moveit_examples handeye_capture
```

- 로봇 자세는 작업자가 안전하게 변경합니다. 카메라/케이블/보드 충돌에 주의합니다.
- 각 자세에서 최소 1초 이상 정지하고, 보드 전체와 같은 원점/축 방향을 확인합니다.
- 화면에 포커스를 두고 **s**를 눌러 저장합니다. **q/ESC**는 종료입니다.
- GUI가 없으면 `--no-window`와 아래 서비스를 사용합니다.

```bash
ros2 service call /handeye_capture/save std_srvs/srv/Trigger '{}'
```

저장 위치는 `~/handeye_samples/<세션시각>/<영상시각>/`입니다.
`--output /원하는/폴더`로 변경할 수 있습니다. 저장물은 원본 `image.png`와
`sample.json`이며, JSON이 없는 디렉터리는 저장 실패한 불완전 샘플입니다.
이미지와 보정행렬, 검출점, 재투영 오차, 보드 자세, 촬영 시각의 TF,
정지 판단에 사용한 관절 이력을 기록합니다. 기존 파일은 덮어쓰지 않습니다.

변환 정의:
- `base_T_tool`: tool0 좌표를 base_link로 변환 (영상 timestamp로 TF 조회).
- `camera_T_board`: 보드 좌표를 camera optical frame으로 변환.
- 보드 원점은 첫 내부 교차점입니다. 임시 URDF 카메라 위치는 계산 입력으로 쓰지 않습니다.

저장 거부 조건: 영상 0.5초 초과/미래 시각, 보드 미검출, 재투영 RMS >0.5px,
1초 정지 이력 부족, 관절 변화 >0.001rad, 관절 시각 불일치/누락, 해당 시각 TF 없음.
이 임계값은 수집 품질 확인용이며 안전 정지 감지 장치가 아닙니다.

**중요:** 평면 PnP 모호성/교차점 순서 뒤집힘/보드 흔들림을 자동으로 검증하지 않습니다.
저장 데이터는 후보일 뿐입니다. 한 자세만 반복하지 말고 서로 다른 방향의 회전을
포함해 수집해야 하며, 이후 계산 단계에서 자세 다양성·이상치·별도 검증 데이터를
검사해야 합니다. 현재 노드는 캘리브레이션 완료나 로봇 이동 안전성을 보증하지 않습니다.
