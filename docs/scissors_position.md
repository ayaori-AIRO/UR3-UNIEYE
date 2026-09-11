# 가위 표면 3D 위치 확인 (로봇 이동 없음)

## 자동 프리뷰 (클릭 없음)

### 현재 기본: 첫 유효 검출 / 반복 재시도

`--auto` 기본 정책은 이제 `--auto-policy first-valid`입니다.
이 모드에서는 `--confidence`(현재 기본 0.4)를 사용하며 8회/5회 투표를 하지 않습니다.
검출 실패, 다수 물체, 깊이/표면/TF 검사 실패 시 계속 재시도합니다.
새 영상만 처리하며 시도 시작 간격은 최소 0.5초입니다. q/ESC 또는 Ctrl+C로 종료합니다.
표면 선택/깊이/시각/TF 검사는 그대로 유지합니다. 한 번의 오검출도 통과할 수 있습니다.

관찰 자세에서 자동 노드를 실행한 뒤, 다른 터미널에서 첫 새 후보점의 계획만 확인:

```bash
ros2 run ur3_moveit_examples scissors_position --auto --confidence 0.4
# 다른 터미널:
ros2 run ur3_moveit_examples eye_in_hand_pick_and_place \
  --stage approach --candidate-source auto --candidate-timeout 0 \
  --approach-height 0.20 --velocity-scaling 0.03 --acceleration-scaling 0.03
```

접근은 첫 유효 신규 후보만 고정하고 구독을 종료합니다. 대기만 무기한이며,
IK/계획/실행 실패는 중단합니다. 검출 노드는 계속 표시하지만 이동 목표는 갱신하지 않습니다.
위 접근 명령에 `--execute`를 추가하면 **후보 수신 후 별도 확인 없이 재계획·검사 후 이동**합니다.
관찰 경로까지 자동 연결한 것은 아닙니다. 실제 경로와 카메라/작업대 여유를 먼저 확인하세요.

아래 누적 규칙은 `--auto-policy stable`을 선택할 때만 적용됩니다.

```bash
ros2 run ur3_moveit_examples scissors_position --auto
```

기존 클릭 노드를 종료한 뒤 실행합니다. 로봇과 가위를 고정하세요.
RViz Marker 토픽은 `/scissors/auto_preview_marker`입니다.
좌표는 `/scissors/auto_preview_point`로 발행합니다. 기존 approach가 구독하는
`/scissors/candidate_point`에는 **발행하지 않으므로 이동과 연결되지 않습니다.**

stable 정책에서는 --confidence 대신 고정된 누적 규칙을 사용합니다:
점수 0.30 이상을 후보로 받고, 최근 8회 시도 중 10초 이내 유효 표본 5개 이상,
그중 0.50 이상 2개 이상, 중앙값에서 각 점 거리 8mm 이내를 요구합니다.
따라서 0.4 한 번이나 잠깐 미검출은 즉시 전체 실패가 되지 않지만,
계속 0.4만 나오는 검출은 확정하지 않습니다. 동일 영상은 중복 누적하지 않습니다.
실패한 현재 프레임에는 과거 좌표를 다시 발행하지 않습니다.

GrabCut 전경의 내부 여유가 큰 픽셀을 자동 선택하고 기존 깊이 검사를 적용합니다.
다수 가위는 거부하며 큰 위치 변화는 누적을 초기화합니다. 이는 추적 ID 기반
다중 물체 추적이 아니며 의미론적 표면 분할도 아닙니다. 배경 오선택이 안정적으로
반복돼도 통과할 수 있으므로 영상의 빨간 점과 포인트클라우드를 직접 확인하세요.
가위 중심/집기점을 보장하지 않습니다. 자동 로봇 이동은 아직 구현하지 않습니다.
RGB/깊이/TF 동기화 검사와 허용 오차는 변경하지 않았습니다.

관찰 자세에서 로봇과 물체를 정지시키고 사용합니다. 기존 YOLO 라이브 노드는
종료해 CPU 중복 추론을 피하세요. Driver/보정된 TF는 유지합니다.
이 노드는 MoveIt, 그리퍼, 로봇 이동 명령을 사용하지 않습니다.

기존 카메라 launch만 종료한 후 정렬 깊이를 켜서 다시 실행:

```bash
ros2 launch realsense2_camera rs_launch.py \
  camera_name:=camera camera_namespace:=camera \
  enable_color:=true enable_depth:=true \
  align_depth.enable:=true enable_sync:=true publish_tf:=true
```

정렬 깊이 옵션/토픽은 [RealSense ROS 공식 문서](https://github.com/realsenseai/realsense-ros)를 참조합니다.

다른 터미널:

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 run ur3_moveit_examples scissors_position
```

1. `s`: 현재 컬러/정렬 깊이/CameraInfo와 촬영 시점 TF를 확보하고 YOLO 추론.
2. 고정된 화면의 검출 박스 안에서 **실제 가위 표면**을 클릭합니다.
   손잡이 구멍, 배경, 얇은 경계, 반사로 깊이가 사라진 부분은 피하세요.
3. 터미널에 camera/base_link 좌표가 m 단위로 표시됩니다.
4. RViz Fixed Frame을 `base_link`로 설정하고 Marker를 추가하여
   `/scissors/candidate_marker`를 선택합니다. 10 mm 크기 구가 3초 표시됩니다.
5. `r`: 라이브로 복귀. 다시 `s`로 새 표본을 얻습니다. `q`/ESC: 종료.

`/scissors/candidate_point`는 PointStamped이며 시각은 원본 RGB 촬영 시각입니다.
화면은 최대 10초 후 해제됩니다. 표시점은 자동 이동 입력이나 집기 자세가 아닙니다.
정지 상태에서도 추론 중 물체가 움직이면 사용하지 마세요.

## 거부 조건과 한계

- RGB 0.5초 초과 지연, RGB/깊이 시각 차이 30 ms 초과,
  CameraInfo 1초 초과 차이, 프레임/해상도 불일치, 촬영 시각 TF 부재를 거부합니다.
  최신 TF로 대신 변환하지 않습니다.
- 깊이 `16UC1`은 mm, `32FC1`은 m로 해석합니다. 0.07–0.50 m 범위에서
  클릭 중심 깊이가 유효하고 5×5 패치 중 20개 이상 유효하며 깊이 범위가
  6 mm 이내여야 합니다. 이는 휴리스틱이지 정확도 보증이 아닙니다.
- 원본 컬러 CameraInfo의 K/D로 픽셀을 역투영합니다.
  plumb_bob/rational_polynomial 외 왜곡 모델과 binning/ROI는 거부합니다.
- 검출 박스만으로 물체 표면과 배경을 확정할 수 없으므로 이번 단계는
  **사용자가 표면을 선택하는 반자동 검증**입니다. 배경을 클릭해도 평탄하면
  통과할 수 있습니다. 자동 표면 선택/추적/안정성 검증은 아직 구현하지 않았습니다.
- 캘리브레이션 잔차 외 깊이/검출/클릭 오차가 더해집니다. 여러 관찰 자세에서
  동일한 물리적 표면점을 측정하여 좌표 일관성을 확인하세요.
- tool0 목표 이동이나 가위 집기는 포함하지 않습니다.
