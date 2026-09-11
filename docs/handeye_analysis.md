# Hand-eye 후보 계산 (오프라인, 적용 안 함)

## 독립 검증

보드와 카메라 장착을 훈련 수집 이후 움직이지 않은 상태에서
`handeye_capture`를 다시 실행해 새 세션에 5~8개 다른 정지 자세를 저장합니다.
보드를 옮겼다면 기존 보드 위치와 직접 비교하는 아래 검증은 사용할 수 없습니다.
아래 NEW_SESSION은 실제 새 세션 폴더로 바꿉니다.

```bash
cd ~/ros2_ws/src/ur3_moveit_examples
/usr/bin/python3 scripts/validate_handeye.py \
  calibration_results/handeye_163205_analysis.json \
  /home/unieye/handeye_samples/NEW_SESSION \
  --report /tmp/handeye_independent_validation.json
```

기존 PARK 변환과 기존 훈련 데이터의 평균 보드 위치를 고정합니다.
새 샘플로 재학습하지 않습니다. 메타데이터/이미지/투영을 검사하고,
훈련 및 이전 검증 자세와 유사한지 표시합니다. 유사 샘플도 보고서에는 유지합니다.
자동 합격/TF 적용은 없으며 보드 좌표계 뒤집힘, 고정 상태 및 허용 오차는
별도로 검토해야 합니다. 새로운 독립 자세가 부족하면 더 수집합니다.

```bash
cd ~/ros2_ws/src/ur3_moveit_examples
/usr/bin/python3 scripts/analyze_handeye.py \
  /home/unieye/handeye_samples/20260910_163205_009978 \
  --report /tmp/handeye_report_new.json
```

기존 보고서 경로는 덮어쓰지 않습니다. 원본 샘플은 변경하지 않습니다.
OpenCV calibrateHandEye 입력은 base_T_tool, camera_T_board이며 출력은
tool0_T_camera_color_optical_frame입니다. camera_link 변환과 다르므로
URDF camera_xyz/rpy에 직접 복사하지 마세요.
참고: https://docs.opencv.org/4.5.2/d9/d0c/group__calib3d.html

검사: 이미지 읽기/크기, 보드 및 카메라 메타데이터 일치, 유효 강체 변환,
저장된 교차점 재투영 RMS <=0.5px 및 양의 깊이.
현재 검사는 이미지에서 교차점을 다시 검출하거나 원점 뒤집힘을 자동 수정하지 않습니다.
관절 정지 판정은 수집 노드에서 수행한 것을 사용하며 여기서 재검증하지 않습니다.

유사 자세 기준은 위치차 <5mm AND 회전차 <2도이며 낮은 재투영 오차부터
대표로 선택합니다. 제외 ID와 대표 ID는 보고서에 남깁니다. 원본 삭제는 없습니다.
TSAI/PARK/HORAUD/ANDREFF/DANIILIDIS를 모두 비교하며 큰 잔차 샘플도
임의로 제거하지 않습니다. 회전 변화의 SVD 값은 다양성 진단용입니다.

## 20260910_163205_009978 결과

- 18개 파일/투영 검사 통과, 유사 자세 3개 제외, 15개 사용.
- PARK: 고정 보드 자세의 평균 대비 위치 RMS 2.432mm, 회전 RMS 0.483도.
- PARK leave-one-out: 위치 RMS 2.834mm, 회전 RMS 0.536도.
- PARK leave-one-out 최대: 위치 4.764mm, 회전 0.923도.
- 다른 방법도 위치 fit RMS 2.4~2.6mm로 비슷함.
- 로컬 상세 보고서: `calibration_results/handeye_163205_analysis.json` (Git 제외).

`base_T_tool * tool_T_camera * camera_T_board`가 모든 관측에서 같은
고정 보드 자세가 되는지 평가합니다. Leave-one-out은 한 관측을 빼고 계산한
변환으로 그 관측을 평가하며, 별도 수집한 독립 검증 데이터는 아닙니다.
이 수치는 절대 위치 정확도, TCP 정확도, 집기 성공 또는 안전성을 보장하지 않습니다.
별도 자세로 새 샘플을 수집하고, 보드 고정/카메라 고정/로봇 모델 일치를 확인한 뒤
적합성을 판단해야 합니다. 상태는 NOT_VALIDATED_DO_NOT_APPLY이며 TF 변경은 없습니다.
