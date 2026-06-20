# 모듈 1 구현 플랜 — 센서 통합 & 오도메트리 + EKF

> 포트폴리오 프로젝트 "ROS2 기반 실내 자율주행 로봇"의 5개 모듈 중 1단계.
> 측정은 **실제 로봇 우선**, 진행 순서는 **1→5 순차**. 핵심 난점: EKF 센서융합 + 측정 신뢰성.
> 상태: **계획 확정 / 미착수** (코드 미수정)

## 현황 (코드 확인 결과)

| 항목 | 상태 |
|------|------|
| dynamixel 모터 제어 | ✓ `pinky_bringup/bringup.py` + `dynamixel_driver.py` |
| 휠 엔코더 오도메트리 | ✓ `bringup.py` (30Hz 단일 루프, `odom→base_footprint` TF 직접 발행 `:157`) |
| BNO055 IMU | ⚠ 패키지 존재(`pinky_imu_bno055`, exec `main_node`, 100Hz, `imu_raw`@`imu_link`)하나 **어떤 런치에도 미포함** |
| robot_localization EKF | ✗ 없음 (패키지는 `/opt/ros/jazzy`에 설치됨) |
| TF 체인 | ✓ `base_footprint → base_link → imu_link` (imu 장착 rpy=0 → gyro z = 로봇 yaw rate 직결) |
| odom 메시지 공분산 | ✗ 미설정 (EKF가 "완벽 신뢰"로 오해) |
| 휠 파라미터 | `wheel_radius=0.027`, `wheel_separation=0.0961` (캘리브레이션 대상) |

## 목표 (기획서 수치 = 완료 판정 기준)

| 측정 | 목표 | 검증 방법 |
|------|------|----------|
| 5m 직진 거리오차 | < 2% | 바닥 줄자, 10회 |
| 제자리 360° 회전오차 | ±3% | 각도자, 10회 |
| EKF 정지 yaw 드리프트(1분) | ±1° | EKF yaw 로깅 |
| odom/EKF 발행 주기 | 30~50Hz | `ros2 topic hz` |
| TF extrinsic | 완료 | `view_frames` 트리 |
| **산출물** | EKF config + TF 다이어그램 + **전/후 드리프트 그래프** | README |

---

## Step 0 — 베이스라인 고정 (git 규율)
`src/`로 옮긴 워크스페이스 재구성이 아직 커밋 안 됨. 작업 시작 전 **현재 동작 상태를 먼저 커밋**해서 되돌아갈 지점을 만든다.
- [ ] `git add -A && git commit` — "chore: restructure into colcon src workspace (baseline)"
- [ ] 새 작업은 브랜치에서: `feature/module1-ekf`
- **검증**: `colcon build` 통과, 현재 bringup 정상 동작 확인 후 커밋

## Step 1 — IMU를 런치에 올리고 신뢰성 검증 (EKF 전제조건)
EKF가 IMU를 융합하려면 먼저 `imu_raw`가 떠야 함. **gyro z 부호**가 yaw 융합의 생사를 가름.
- [ ] `bringup_robot.launch.xml`에 IMU 노드 추가:
  `<node pkg="pinky_imu_bno055" exec="main_node" name="imu"><param name="frame_id" value="imu_link"/></node>`
- **검증 (Step 1의 핵심)**:
  - `ros2 topic hz /imu_raw` → ~100Hz 확인
  - 로봇을 **반시계로 손으로 돌릴 때 `angular_velocity.z`가 +** 인지 (REP-103). 부호 반대면 EKF 발산 → IMU 노드에서 부호 보정
  - `ros2 run tf2_tools view_frames`로 `base_link→imu_link` static TF 존재 확인
- ⚠ 리스크: BNO055가 실제 하드웨어에서 안 뜰 수 있음(I2C/캘리브레이션). 여기서 처음 실측되는 부분.

## Step 2 — bringup을 EKF-ready로 (TF 소유권 이양 준비)
파일: `src/pinky_bringup/pinky_bringup/bringup.py`
- [ ] **`publish_tf` 파라미터 추가** (기본 True = 하위호환). `_publish_tf` 호출을 `if self.publish_tf:`로 게이트 (`:157`)
- [ ] **odom 메시지에 twist 공분산 설정** (`_publish_odometry`, `:174`) — `twist.covariance[0]`(vx), `[35]`(vyaw)에 현실적 값. 지금은 0이라 EKF가 "완벽 신뢰"로 오해
- **검증**: 재빌드 후 `publish_tf:=true`로 기존과 동일 동작(회귀 없음) 확인

## Step 3 — EKF config + 런치 배선 (TF 충돌 0)
- [ ] 신규 `src/pinky_bringup/config/ekf.yaml` (robot_localization):
  - `two_d_mode: true`, `frequency: 30`, `world_frame: odom`, `odom_frame: odom`, `base_link_frame: base_footprint`
  - `odom0: /odom` → **vx, vyaw만** 융합 (절대 x/y 금지)
  - `imu0: /imu_raw` → **vyaw만** 융합 (지자기 yaw는 실내에서 불신 → 절대 orientation 미융합)
- [ ] `bringup_robot.launch.xml`에 `use_ekf` arg 추가:
  - `use_ekf:=true` → `ekf_node` 시작 **AND** bringup `publish_tf:=false` (정확히 한 노드만 `odom→base_footprint` 발행)
- **검증 (모듈 1의 정확성 게이트)**:
  - `view_frames`에서 `odom→base_footprint` 발행자가 **EKF 하나뿐** (충돌/덜덜거림 없음)
  - `ros2 topic hz /odometry/filtered` ~30Hz
  - RViz에서 odom 프레임 안정(튐 없음)

## Step 4 — 캘리브레이션 & 측정 (포트폴리오 본질)
- [ ] **휠 오도메트리 캘리브레이션**: 5m 직진 → 거리오차로 `wheel_radius` 보정, 360°×N 회전 → 각도오차로 `wheel_separation` 보정 (UMBmark 방식). 목표 <2% / ±3% 달성까지 반복
- [ ] **측정 스크립트** 신규 `scripts/measure_odom.py`: `/odom`(raw)와 `/odometry/filtered`(EKF) 동시 CSV 로깅 + 패턴 주행
- [ ] **EKF 정지 드리프트**: 1분 정지, EKF yaw 로깅 → ±1° 확인
- [ ] **전/후 비교 그래프** (README 산출물): 같은 주행을 ①raw 휠 ②raw IMU ③융합 3개 오버레이 → `plot_drift.py`
- [ ] 산출물 저장: `doc/portfolio/module1/` (CSV + 그래프 + `view_frames` PNG)

---

## 식별된 불확실성 (코딩 전 인지)
1. **gyro z 부호** — Step 1에서 실측 검증 (틀리면 EKF 발산)
2. **BNO055 실동작** — 런치에 없었으므로 첫 실측
3. **IMU 공분산 0.01 하드코딩**(`pinky_imu_bno055/src/main_node.cpp:103,109,115`) — 융합 결과 보고 튜닝 필요할 수 있음
4. **30Hz 직렬 루프** — 더 올리면 1Mbaud 직렬이 병목일 수 있음 (현재도 30~50Hz 목표 충족이라 보류)

## 측정 신뢰성 원칙 (전 모듈 공통)
- **반복정밀도 ≠ 절대정확도**: 헤드라인 수치 대부분은 반복정밀도 → 줄자로도 신뢰성 확보 가능(퍼짐/std 측정)
- 측정 대상과 측정 도구 분리 (5m odom→줄자, EKF 정지 드리프트→진실값 0)
- 항상 N≥20, 평균/표준편차/최댓값 (단일 숫자 금지)
- raw 증거 레포에 보관 (CSV + 사진 + 측정 스크립트)
- 측정 자동화로 재현성 = 신뢰성
