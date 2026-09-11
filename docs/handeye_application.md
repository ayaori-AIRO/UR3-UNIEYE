# 2026-09-11 시험용 보정값 파일 적용

사용자 요청으로 PARK 후보를 적용했습니다. 노드 재시작/로봇 이동은 하지 않았습니다.
독립 검증 RMS 3.75mm/0.73도, 최대 5.21mm/1.23도입니다. 정밀도 보증은 없으며
분석 JSON의 미검증 상태도 유지합니다.

입력: calibration_results/handeye_163205_analysis.json methods.PARK.tool_T_camera.
실제로 읽은 camera_link_T_camera_color_optical_frame:

```text
translation[m]: [-9.969236089091282e-06, 9.921733180817682e-06, 1.0058383850264363e-05]
quaternion[xyzw]: [-0.4986040748332797, 0.5019557312219748, -0.4990204460501724, 0.5004128444364202]
```

tool0_T_camera_link = PARK * inverse(위 내부 TF).

```text
xyz[m]: [-0.0497590312201, -0.0278310918365, 0.00148158371381]
rpy[rad]: [-1.13302251758, -1.56644916181, 1.73958781696]
```

pitch가 -pi/2 근처라 개별 RPY 숫자 변화보다는 회전행렬로 비교해야 합니다.
사진에 맞추려고 보정값을 다시 덮어쓰지 마세요.

이전 파일 전체 백업: urdf/ur3_d405_visual_backup.urdf.xacro.
복원이 필요하면 이 파일의 내용을 원본에 복원하고 정지 상태에서 재시작합니다.

로봇 정지 및 대기 이동 명령이 없는 것을 확인한 후 driver/MoveIt을 기존 D405
명령으로 재시작해야 런타임에 반영됩니다. 카메라 내부 TF는 그대로 유지합니다.
`ros2 run tf2_ros tf2_echo tool0 camera_link`로 위 xyz를 확인하고
로봇 이동 없이 RViz와 보드 좌표부터 확인합니다.
