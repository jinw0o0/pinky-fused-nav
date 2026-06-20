# 🤖 pinky-fused-nav

**RGB-D 카메라 + 2D LiDAR를 융합**해 바닥 위 **1–25 cm 저(低)장애물**까지 점유 격자에 담고,
그 맵 위에서 **자율주행**하는 ROS 2 (Jazzy) 프로젝트.

> Pinky 모바일 로봇(Raspberry Pi 5) 플랫폼 기반. 센서 융합 인지 · SLAM · 자율주행 · 센서 캘리브레이션을 직접 구현했습니다.

---

## 🎯 풀려는 문제

2D LiDAR는 **고정된 한 평면(바닥 ~12.5 cm 높이)만** 스캔합니다.
그래서 그 아래에 있는 **상자 · 문턱 · 낮은 가구** 같은 저장애물을 "비어 있음"으로 취급 → 충돌 위험.

**접근:** RGB-D 깊이 카메라로 바닥~25 cm 영역을 관측하고, 이를 LiDAR와 **융합**해서
저장애물까지 점유 셀로 만든 뒤, 그 맵 위에서 안전하게 주행합니다.

---

## 🧩 아키텍처

```
 Astra Pro (depth) ─┐
                    ├─►  scan_fuser (C++)  ─►  /scan_fused  ─┬─►  slam_toolbox  ─►  map
 RPLidar C1 (/scan) ─┘          │                            └─►  Nav2 (Smac2D + MPPI)
                                │
 wheel odom + BNO055 IMU  ─►  robot_localization EKF  ─►  odom → base_footprint TF
```

**핵심 노드 — `scan_fuser`** (`pinky_perception`, C++):

1. depth 포인트클라우드를 `base_footprint` 좌표계로 변환
2. **RANSAC 바닥 평면 제거 + 2단 높이 게이트** — grazing 바닥 오인식 억제
3. LiDAR `/scan`과 **per-bin 최소거리 융합** (π-yaw 보정) → 단일 `/scan_fused` (720 bins / 0.5°, `base_footprint`)
4. `slam_toolbox`(매핑)와 Nav2 global costmap이 `/scan_fused`를 소비

---

## 🤖 하드웨어

| 구성 | 부품 |
|---|---|
| 컴퓨트 | Raspberry Pi 5 — Ubuntu / **ROS 2 Jazzy** |
| Depth | Orbbec **Astra Pro** (structured-light) |
| LiDAR | Slamtec **RPLidar C1** |
| IMU | Bosch **BNO055** (software i2c, GPIO0/1) |
| 구동 | Dynamixel 휠 모터 ×2 |

---

## 📦 주요 패키지

| 패키지 | 역할 |
|---|---|
| **`pinky_perception`** | `scan_fuser` — RGB-D + LiDAR 융합 → `/scan_fused` |
| **`pinky_navigation`** | `slam_toolbox` · Nav2 파라미터/런치, 검증 하니스 |
| **`pinky_bringup`** | 모터 구동 · 휠 오돔 · `cmd_vel` watchdog |
| **`pinky_description`** | URDF · 센서 마운트 TF |
| **`pinky_imu_bno055`** | BNO055 IMU 드라이버 |
| `ros2_astra_camera`, `sllidar_ros2` | 카메라 · 라이다 드라이버 (vendored) |

---

## 🚀 빌드 & 실행

```bash
git clone git@github.com:jinw0o0/pinky-fused-nav.git
cd pinky-fused-nav
colcon build --symlink-install
source install/setup.bash
```

전체 스택 기동:

```bash
ros2 launch pinky_bringup bringup_robot.launch.xml
```

**의존성:** ROS 2 Jazzy · PCL · `robot_localization` · `slam_toolbox` · `nav2` · `spatio-temporal-voxel-layer` · wiringPi(IMU)

### ⚙️ Pi 셋업 — BNO055 소프트웨어 i2c (필수)

BNO055는 라즈베리파이 하드웨어 i2c에서 **클록 스트레칭 락업**이 발생합니다.
GPIO 0/1에 bit-bang i2c를 구성해 회피 — `/boot/firmware/config.txt`:

```ini
#dtparam=i2c_vc=on
dtoverlay=i2c-gpio,bus=10,i2c_gpio_sda=0,i2c_gpio_scl=1,i2c_gpio_delay_us=2
```

→ IMU는 `/dev/i2c-10`을 사용. 적용에는 **콜드부팅**(전원 재인가)이 필요합니다.

---

## 🛠️ 엔지니어링 하이라이트

- **센서 융합 파이프라인** — RANSAC 바닥 제거 + 높이 게이트로 바닥 오인식을 억제하고, π-yaw 보정 per-bin 최소거리 융합으로 LiDAR·depth를 단일 LaserScan으로 결합.
- **Stage A 카메라 캘리브레이션** — 바닥 grazing(스침각)으로 floor-fit이 불안정한 문제를 **평벽 normal 방식**으로 우회, depth pitch 잔차를 **0.2° 이내**로 보정.
- **BNO055 i2c 락업 정복** — device-tree에서 IMU 핀(GPIO0/1) 추적 → 소프트웨어 i2c 구성 + 드라이버 `RST_SYS` 리셋 제거 + 모드 readback 검증/재시도로 **100 Hz 안정화**.
- **Phase 1a 온로봇 검증** — 자이로 단위(deg/s→rad/s) · 휠 조인트 TF · `cmd_vel` watchdog을 실로봇에서 검증.

---

## 🗺️ 상태 & 로드맵

- [x] `scan_fuser` 구현 + 온로봇 `/scan_fused` 동작 검증
- [x] Stage A 카메라 pitch 캘리브레이션
- [x] Phase 1a 하드웨어 검증 · IMU 안정화
- [ ] 바닥-제거 정량 검증 → mapper를 `/scan_fused`로 승격 (Phase 3)
- [ ] Nav2 MPPI 튜닝 · 자율주행 (Phase 4)
- [ ] robot_localization EKF 오프라인 튜닝 (golden rosbag)

---

## 🙏 크레딧

[Pinky](https://github.com/pinklab-art/pinky_pro) 모바일 로봇 플랫폼(pinklab-art) 기반.
융합 인지 · SLAM · 자율주행 · 센서 캘리브레이션 · IMU 드라이버 수정은 본인 작업입니다.
