# Pinky 융합 점유격자(Fused Occupancy Map) 설계 문서
2D LiDAR + RGB-D depth → 1–25cm 장애물 점유 맵 → 자율주행 (ROS 2 Jazzy / Raspberry Pi 5 8GB)

> 작성: 다각도 설계(3안) + Jazzy 툴 검증 + 기하 계산 합성. 모든 정량값은 **Phase 2 Stage A 실측 전까지 잠정치** — on-robot RViz 재측정 없이 hard-code 금지.

---

## 1. 요약 (TL;DR)

**추천 아키텍처 (한 줄):** "전용 custom C++ 노드(`scan_fuser`)가 depth cloud를 `base_footprint`에서 RANSAC 바닥제거 + height-gate 후 가상 scan으로 변환하고, 그 자리에서 LiDAR `/scan`과 per-bin 최근접 융합하여 `/scan_fused` 단일 LaserScan을 발행" — Approach B 골격 + Approach C 바닥제거/검증 + Approach A 통합지점을 이식한 **hybrid(B-core)**.

핵심 판단 3가지:
- **stock 툴 체인(Approach A)은 packaging이 깨진다.** `ira_laser_tools`는 ROS 1 전용(ROS Index 확인). Jazzy 대안은 source-only `dual_laser_merger` 뿐. 그리고 stock `pointcloud_to_laserscan`은 평면 box gate만 지원 → 이 카메라의 make-or-break인 바닥 false-positive를 막을 수 없다.
- **이 문제의 본질은 "병합"이 아니라 "바닥제거"다.** 기하 분석(2절)이 정량적으로 증명: +8° up-tilt에도 FOV 하단 1/3이 바닥을 향하고, 바닥 return의 계산 높이가 0.58m 부근에서 [0.01, 0.25] gate 밴드를 통과한다. RANSAC ground-plane fit + min-points-per-bin이 필수인데, custom 노드 안에서만 깔끔히 가능.
- **Pi5 비용은 custom 노드가 더 싸다.** ptl + merger 2-노드 체인은 307k-point cloud를 TF 변환하고 두 번째 LaserScan을 토픽으로 hop. custom 노드는 한 프로세스에서 voxel-downsample 후 처리 → 추정 ~10–20% of one A76 core.

**기본 운영 정책(보수적): Phase 2에서는 `/scan_fused`를 *costmap*에만 먹이고 slam_toolbox mapper는 LiDAR `/scan` 유지. 바닥 검증 통과 후(Phase 3) `mapper_params.yaml:16`을 `/scan_fused`로 승격.** (근거: 8절 loop-closure 리스크)

---

## 2. 기하 현실 체크 — 카메라는 바닥/밴드를 실제로 어디서 보는가

검증된 TF(`pinky.urdf.xacro` 실측 합산):
- `rplidar_link` z = 0.028 + 0.067 + 0.030 = **0.125 m**, 그리고 `rplidar_mount`→`rplidar_link`에 **`rpy=(0,0,π)`** (yaw 180°) — 융합 시 load-bearing, 4(b)/8 참조.
- `camera_link` z = 0.028 + 0.067 + 0.080 = **0.175 m**, pitch = `depth_cam_tilt_deg=-8.0` → 광축 **위로 8°**.
- optical 내부 프레임은 astra 드라이버가 `publish_tf=true`로 발행(REP-103 광학회전 포함).

**광축 +8°, V-FOV 49.5°(±24.75°) → 광선은 +32.75°(상단)부터 −16.75°(하단)까지.** 즉 up-tilt에도 **FOV의 33.8%(16.75°)가 수평 아래(바닥)를 향한다.**

높이 h=0.175m 기준 바닥 교차(높이 = h + r·sin α):

| 수평거리 X | 하단광선 높이 | 상단광선 높이 | 바닥(≤0) FOV 내? | 1–25cm 밴드 포함? |
|---|---|---|---|---|
| 0.6 m | **−0.6 cm** (≈바닥 직격) | +56 cm | YES (간신히) | yes |
| 1.0 m | −12.6 cm | +82 cm | YES | yes |
| 1.5 m | −27.6 cm | +114 cm | YES | yes |
| 2.0 m | −42.7 cm | +146 cm | YES | yes |

**결론(정량 확정):** 하단광선은 range_min(slant 0.58m, 지상 X≈0.58m)에서 바닥높이 **+0.8cm**에 닿는다 — 사용 가능한 최소거리에서 바닥이 정확히 1cm gate 위에 걸린다. X>0.58m의 모든 거리에서 하단부 광선들이 바닥을 직접 imaging하며 그 return은 in-range다. 민감도: **X=1.0m에서 1° pitch 오차당 바닥높이 ~19mm 이동.** 0.5–1° mount/calib 오차나 수 cm depth noise면 바닥 return이 [0.01, 0.25] 밴드로 떠올라 맵 전체에 **phantom wall이 smear된다.**

긍정 사실: 1.0m의 2cm 물체 top(z=0.02m)을 보려면 하단광선이 8.8° 하향이면 충분(가용 16.75° 안). **작은 물체는 기하적으로 관측 가능** — 위험은 "물체를 못 본다"가 아니라 "바닥을 본다"다.

**그러므로 floor-rejection이 설계의 전부다. tilt 튜닝으로는 해결 불가** — 0.6m에서 1cm 물체를 잡으려면 하단광선이 바닥을 grazing(=바닥 노출), 바닥을 FOV 밖으로 빼려면(+20° 이상) 12–22cm 이하 물체를 못 본다. 두 요구는 상호배타적.

---

## 3. 추천 파이프라인 — 노드/토픽/프레임 그래프

```
sllidar_node ── /scan ───────────────────────────┐  (rplidar_link, BEST_EFFORT, rpy yaw=π!)
 (NOT vendored; system dep)                       │
                                                  ▼
astra driver ── /camera/depth/points ──►  ┌──────────────────────────────┐
 (camera_depth_optical_frame,             │      scan_fuser (custom C++)  │
  640x480, point_cloud_qos=default        │  scanCb: cache /scan + π-yaw  │
  =RELIABLE, depth_fps=30)                │         LUT resample → bins   │
                                          │  cloudCb: TF→base_footprint   │── /scan_fused
                                          │     voxel(min3) → RANSAC floor│   (frame: base_footprint,
                                          │     → SOR → height-gate       │    720 bins/0.5°, RELIABLE)
                                          │     → depth_bins (nearest)    │        │
                                          │  timer(10–15Hz): fuse=min,    │        │
                                          │     stale-gate, publish       │        │
                                          └──────────────────────────────┘        │
                                                                                   ▼
                                              ┌─────────────────────────┬──────────────────────┐
                                              ▼ (Phase 3)               ▼ (Phase 2 default)     ▼
                                   slam_toolbox.scan_topic       global costmap          (선택) /scan_depth
                                   = /scan_fused  → saved map     obstacle_layer          디버그 RViz
                                                                  = /scan_fused
   local costmap: 변경 없음 — STVL(stvl_layer)가 /camera/depth/points 직접 소비(이중계산 방지)
```

- **scan_fuser (신규, 유일한 custom 컴포넌트):** depth→바닥제거→가상scan, LiDAR와 per-bin 융합, 단일 LaserScan 발행. 바닥방어·π-yaw 보정·staleness fallback이 한 곳에 → 두-파일 drift 구조적으로 없음.
- **slam_toolbox:** `/scan_fused` 또는 `/scan` 하나만 소비(Phase에 따라).
- **costmaps:** local은 STVL로 depth 직접(기억형, 0.58m 사각 보완) — 그대로. global obstacle_layer만 `/scan`→`/scan_fused`로 전환.

---

## 4. 단계별 상세

### (a) depth → 가상 scan + height-gate (`scan_fuser` cloudCb 내부)

**핵심 원칙: gate는 반드시 중력정렬 프레임 `base_footprint`(z=0=바닥)에서.** optical 프레임에서 gate하면 8° tilt 때문에 gate가 거리종속이 되는 고전 버그.

처리 순서(싼 것부터):
```
/camera/depth/points
 → (구조적 다운샘플: u,v 픽셀 step=2 → ~77k pts)         # Pi5 비용 절감
 → PCL VoxelGrid leaf=0.02 m, setMinimumPointsNumberPerVoxel(3)  # 다운샘플 + 단일픽셀 flyer 제거
 → PCL SACSegmentation (SACMODEL_PERPENDICULAR_PLANE, axis=+Z, eps_angle≈10°, dist_thresh=0.02, z<0.10만) + ExtractIndices(outliers)
      ※ guard: 평면 법선 z성분 > 0.95 일 때만 제거(근접 대형물체 오인 방지). 아니면 그 프레임은 gate에만 의존.
 → PCL StatisticalOutlierRemoval (mean_k=10, stddev_mul=1.0)     # 잔여 speckle 제거
 → TF: base_footprint ← camera_depth_optical_frame @ cloud.stamp
 → height-gate (two-tier) + range-gate
 → per-bin nearest range → depth_bins[]
```

**height-gate 권장값 (RANSAC 바닥제거 *후* 적용, two-tier — stock ptl로는 불가):**

| param | 값 | 근거 |
|---|---|---|
| `min_height` (near, range ≤ 1.2 m) | **0.02 m** | 1.0m 3σ 바닥 spread ~1.1cm → 2cm가 안전한 하한 |
| `min_height` (far, range > 1.2 m) | **0.05 m** | 2m 3σ ~3.4cm, 3m ~7cm → far는 5cm |
| `max_height` | **0.25 m** | 1–25cm 밴드. ※ 로봇 자체 lamp/screen 자기점 들어오면 footprint mask 추가 |
| `range_min` | **0.58 m** | depth HW 최소거리 |
| `range_max` | **3.0 m** | noise가 거리²로 증가; 0.6–2.0m를 certified zone, 그 이상 LiDAR 위임 |
| `angle_min / max` | **±30° (∓0.5236)** | 실제 H-FOV 60° 밖으로 beam 금지 |

> **정직한 spec:** "≤1.2m 내 1–25cm 물체는 occupied, ~2m까지 5cm 이상 occupied." 노이즈 많은 up-tilt mid-range 카메라로 "어디서나 1cm"는 over-claim. k=0.002(σ_z≈k·z²), two-tier 임계값은 Stage A 실측 후 확정 — **on-robot RViz re-measure 필수.**

### (b) `/scan` + depth → `/scan_fused` 병합

**방식: 별도 merger 노드 없이 `scan_fuser` 내부에서 직접 융합.** 이유:
1. `ira_laser_tools`는 Jazzy에 없다(ROS 1 전용).
2. 외부 merger(`dual_laser_merger`)는 두 센서를 한 sensor 프레임으로 모으는데, LaserScan은 단일 원점 가정 → 높이 다른 두 센서(0.125 vs 0.175m)를 한 프레임에 합치면 기하 부정확. 게다가 vendored source 추가 필요.
3. custom 노드는 이미 cloud를 `base_footprint`로 변환 → LiDAR도 거기서 합치면 모든 range가 footprint 원점 기준 XY 수평거리 = slam/costmap이 기대하는 바로 그것.

**출력 grid (slam_toolbox self-consistent):**

| 필드 | 값 | 근거 |
|---|---|---|
| `frame_id` | **base_footprint** | identity TF, 두 센서 원점 일관 |
| `angle_min/max/increment` | −π / (π−inc) / 2π/N, **N=720 (0.5°)** | slam_toolbox는 `ranges` 길이가 헤더 산술과 정확히 일치할 것을 strict 검증. 720으로 정확히, 반올림 금지 |
| `range_min/max` | 0.12 / 8.0 | max = slam `scan_buffer_maximum_scan_distance` |
| 빈 bin | **+inf** | range_max로 채우면 slam이 phantom arc를 그림 |

**π-yaw gotcha (필수 처리):** `rplidar_link`에 `rpy=(0,0,π)`. LiDAR `ranges[i]`를 그대로 fused bin i에 복사하면 모든 장애물이 **180° 회전**. LUT에서 `th_fused = lidar_angle_min + i·inc + π (mod 2π)`로 회전. (LiDAR x-offset 1.7cm는 5cm 해상도 이하라 무시.)

**per-bin 융합 = 무조건 `min`(최근접 우선).** 두 센서 모두 바닥제거 후 실제 장애물만 보고 → 가장 가까운 장애물이 이긴다 = mapping·costmap·safety 모두에 옳다. 12.5cm LiDAR 평면 아래 낮은 물체를 depth가 더 가깝게 보고 → depth 승.

**LiDAR 자기-차폐 마스크 (융합 *전* 필수).** 로봇 본체(카메라 마운트 지지대·스크린·램프 등)가 라이다 평면을 가리는 **특정 각도 sector**에서 라이다가 자기 자신을 장애물로 오인한다. 융합 전에 그 sector의 `ranges`를 **+inf로 마스킹**해야 한다(안 하면 맵·costmap에 로봇을 둘러싼 고정 phantom ring이 박힘). param `lidar_ignore_sectors: [[θmin,θmax], ...]`(base_footprint 각도, rad). **실제 각도는 on-robot RViz로 `/scan`을 보며 확정** — 자기점이 뜨는 각도 구간을 기록. scanCb의 π-yaw LUT 단계에서 해당 bins를 drop. (대안: stock `laser_filters`의 LaserScanAngularBoundsFilterInPlace를 별도 노드로 쓸 수 있으나, 우리는 노드 일원화 원칙상 scan_fuser 내부 처리.)

### (c) slam_toolbox 연결

- **Phase 3 승격:** `mapper_params.yaml:16` `/scan` → `/scan_fused` 한 줄.
- `base_frame: base_footprint`(line 15) — fused scan frame과 일치(identity TF).
- `transform_timeout: 0.4`(line 26)가 depth+RANSAC 지연(<100ms 목표) 커버.
- **global costmap obstacle_layer**(`nav2_params.yaml:270–272`)도 `/scan`→`/scan_fused`. **`nav2_params_smac_mppi.yaml` 쌍둥이도 동일(sync 필수).** local costmap은 STVL가 depth 직접 소비 → **변경 금지**(이중계산).

---

## 5. Floor false-positive 방어 + 캘리브레이션

**Defense-in-depth (직렬 4중):**
1. **Stage A — extrinsic/pitch 캘리브레이션 (다른 튜닝 전에 먼저).** 브래킷 8° 신뢰 금지. 1.0m에서 0.6° 오차 = 1cm 바닥오차 = gate 무력화.
2. **Stage B — `base_footprint` 보수적 z-gate** (4(a) two-tier).
3. **Stage C — RANSAC ground-plane 제거 (진짜 바닥 killer).** calib drift에도 매 프레임 실제 바닥 재탐색. 법선-수직 제약으로 벽 오삭제 방지.
4. **Stage D — VoxelGrid min-3-pts + SOR + min-points-per-bin.** 단일 flyer 차단. STVL `voxel_min_points: 2` 철학과 일치.
- (선택) Stage E 시간 persistence: slam 확률 업데이트가 1–2프레임 flicker를 씻으므로 기본 OFF.

**Stage A 절차 (오프라인, 평탄 바닥, 전방 3m 비움):**
1. `ros2 bag record /camera/depth/points /tf /tf_static` 10s.
2. RViz fixed=base_footprint, color-by-Z. 바닥이 z=0 평면이어야(tilt 오류면 ramp).
3. open3d 평면 fit → residual pitch, floor z0:
```python
model, inl = pcd.segment_plane(distance_threshold=0.01, ransac_n=3, num_iterations=2000)
a,b,c,d = model
pitch = np.degrees(np.arctan2(a, c)); z0 = -d/c
```
4. residual만큼 `depth_cam_tilt_deg`(xacro default `-8.0`) 보정, **|pitch|<0.3° & |floor z|<0.5cm** 까지 반복.
5. 수렴 시: 3m에서 systematic 바닥오차 ≤1.6cm로 cap → two-tier margin이 흡수. 측정된 per-range floor std가 4(a) 임계값 확정.

---

## 6. QoS / timestamp / TF-time 정합

- **depth cloud sub:** astra `point_cloud_qos` 기본 = `"default"` = RELIABLE/VOLATILE/depth5 (SENSOR_DATA 아님). 권장: astra launch `point_cloud_qos:=SENSOR_DATA` + 노드 **BEST_EFFORT** 구독(307k-point reliable 재전송 회피). **launch·노드 양쪽 일관 필수 — 불일치 시 silent no-data.**
- **lidar sub:** `/scan` BEST_EFFORT → SENSOR_DATA/BEST_EFFORT/depth5.
- **`/scan_fused` pub:** RELIABLE/KEEP_LAST/depth5/VOLATILE (720 floats, reliable 거의 무비용).
- **timestamp:** cloudCb는 cloud stamp로 TF lookup(`canTransform` guard, 실패 시 drop — 블록 금지). 출력 stamp = **LiDAR stamp**.
- **staleness gate:** `now − last_depth_stamp > 0.2s`면 depth_bins 무시 → fused = pure LiDAR. depth 정지가 mapping을 절대 오염/중단 안 함.
- **clock 단일화:** 모든 노드 `use_sim_time` 일관(Pi5 False). 혼합 = 즉시 TF-extrapolation 실패.
- **timer 발행:** LiDAR ~10Hz vs depth 15–30Hz → 고정 10–15Hz timer로 slam에 일정 cadence + depth 드롭 시 LiDAR 항상 존재.

---

## 7. 검증 방법론 — occupied/free pass/fail (rosbag 오프라인)

**목표: (a) 알려진 1–25cm 물체 → saved map occupied, (b) 맨바닥 → free/unknown, 절대 occupied 아님.**

**7.1 통제 bag 2종 (실로봇, 평탄 바닥):**
- **Bag FLOOR:** 빈 바닥 3×3m 루프 60s. `/camera/depth/points /scan /tf /tf_static /odom /imu`.
- **Bag OBJECT:** 동일 + 캘리브 물체(2cm 블록, 10cm 박스, 24cm 박스)를 0.8/1.5/2.5m 알려진 map 좌표에 배치(ground-truth (x,y) 기록).

**7.2 오프라인 재생 (노트북, 결정적):**
```bash
ros2 bag play bag_floor --clock          # use_sim_time true 전역
# launch: scan_fuser + slam_toolbox(mapping) + RViz
ros2 run nav2_map_server map_saver_cli -f /tmp/map_floor
```

**7.3 RViz 오버레이:** fixed=map. depth cloud(color-Z, 바닥 평탄 확인), `/scan`(흰) + `/scan_depth`(빨강 디버그) — **빨강 beam이 물체 위에만, 빈 바닥엔 절대 안 뿌려져야.**

**7.4 정량 pass/fail (.pgm 스코어링):**
- **FLOOR:** swept empty 내 false-occupied ≤ **0.5%** AND **≥3셀(≈15cm) 연속 blob 0개**.
- **OBJECT:** ≤1.5m 물체 100% + 2.5m 중 ≥1개 occupied. **2cm@0.8m 반드시 PASS**, 2cm@2.5m 실패 허용(문서화된 한계).
- **cross-check:** 동일 bag을 depth-off(LiDAR-only) 재생 → 10cm 박스는 LiDAR-only **부재**, fused **존재** → depth 가치 직접 증명.

**7.5 회귀 하니스:** `src/pinky_navigation/test/verify_floor_rejection.sh` — bag 재생→map 저장→스코어→PASS/FAIL. bag+스크립트 커밋.

---

## 8. slam 영향 (좁은 60° depth scan 주입) + 완화

`mapper_params.yaml` 기준:
1. **depth가 correlative match 지배 방지.** `use_scan_barycenter: true`(line 33) → 60° dense arc가 barycenter를 전방 편향 → yaw 관측성 훼손. 완화: depth `angle_increment`를 LiDAR보다 촘촘히 금지(소수파 유지).
2. **겹침 range 불일치 → match 싸움.** 완화: **min-range** 채택 → matcher가 근접 구조에 lock.
3. **loop-closure(`do_loop_closing: true` line 41).** 전방 arc가 viewpoint 민감 → signature 분산↑. 완화: arc 17%로 작고 Stage A–D 후 안정. `loop_match_minimum_response_fine: 0.45` 느슨하게 금지.
4. **timing:** merged stamp 단일 일관, `minimum_travel_distance: 0.5`가 throttle. RANSAC>100ms면 `transform_tolerance`/`transform_timeout` 상향.

**핵심 완화 = Phase 정책(1절):** 바닥 검증 전엔 LiDAR `/scan`을 mapper, `/scan_fused`는 costmap에만 → 맵 기하 품질을 depth noise에서 분리하면서 낮은 장애물은 costmap에. 7.4 통과 시 fused-to-mapper 승격.

---

## 9. 대안/트레이드오프 요약표

| 기준 | A (stock: ptl + merger) | **B-core hybrid (추천)** | 순수 B |
|---|---|---|---|
| 코드량 | 0줄 | ~300–400줄 C++ | ~300–400줄 |
| 바닥제거 | ❌ box gate만 | ✅ RANSAC+SOR+min-pts+two-tier | ✅ |
| Jazzy packaging | ❌ ira 없음 | ✅ merger 불필요 | ✅ |
| 병합 기하 | ⚠️ 단일 sensor 프레임 오류 | ✅ base_footprint 단일 원점 | ✅ |
| Pi5 CPU | ⚠️ 2노드 hop | ✅ ~10–20% one core | ✅ |
| 두-파일 drift | ⚠️ 3노드 분산 | ✅ 한 곳 | ✅ |
| 유지보수 | ✅ battle-tested | ⚠️ bespoke | ⚠️ bespoke |
| 1–25cm spec 정직성 | ❌ 바닥 smear | ✅ 검증됨 | ✅ |

**결정: B-core hybrid.**

---

## 10. 지금 결정해야 할 열린 질문

1. **mapper 입력을 처음부터 fused? 아니면 costmap-only 후 승격?** → 추천: **승격 방식**(Phase 2 costmap-only → Phase 3 fused-to-mapper). 되돌리기 쉽고 loop-closure 리스크 차단.
2. **scan_fuser 언어 C++ vs Python?** → 추천: **C++(rclcpp + PCL).** 307k-point @15–30Hz는 Python 처리량 위험.
3. **신규 패키지 위치 — pinky_navigation에 노드 추가 vs 신규 pinky_perception?** → 추천: **신규 `pinky_perception`.** PCL/tf2 무거운 의존 분리.
4. **depth 처리율 full vs throttle?** → 추천: **cloudCb 내부 5–8Hz throttle.** STVL는 raw 직접 받으니 무관.
5. **`/scan_fused`를 local costmap에도?** → 추천: **아니오.** STVL 시간감쇠 "기억"이 0.58m 사각 보완을 담당, fused는 복제 못 함. local=STVL+voxel, global=fused.

**on-robot RViz 실측 필수(추측 금지):** (i) 실제 camera pitch/높이, (ii) 평탄 바닥 per-range z std(→k, 임계값), (iii) C1 DenseBoost 실제 beam 수(→N), (iv) 자기점(lamp/screen)의 max_height 침범 여부.

---

## 11. 구현 순서 (Phase 2 → 3 체크리스트)

**Phase 2 — 인지/융합 파이프라인 + costmap-only 검증**
- [ ] **Stage A 캘리브레이션 먼저** (5절): bag→open3d 평면fit→`depth_cam_tilt_deg` 보정→|pitch|<0.3°, |floor z|<0.5cm. per-range z std 기록.
- [ ] `sllidar_ros2` src/에 vendor (현재 부재).
- [ ] 신규 `pinky_perception` + `scan_fuser` C++ (rclcpp, tf2_ros, tf2_sensor_msgs, pcl_ros, pcl_conversions). 4(a)(b) 구현.
- [ ] QoS 일관화: astra `point_cloud_qos:=SENSOR_DATA`, 노드 BEST_EFFORT, `/scan_fused` RELIABLE.
- [ ] `scan_fuser.launch.xml` 작성 + map/bringup launch에서 include.
- [ ] **global costmap만** `/scan`→`/scan_fused`: `nav2_params.yaml:272` + `nav2_params_smac_mppi.yaml` 쌍둥이(sync). local STVL 그대로.
- [ ] `mapper_params.yaml:16`은 **아직 `/scan` 유지.**
- [ ] 검증 하니스(7절) `src/pinky_navigation/test/`에 커밋.

**Phase 3 — fused-to-mapper 승격 (검증 통과 조건)**
- [ ] 7.4 PASS: floor false-occ ≤0.5% & blob 0개, 2cm@0.8m occupied, LiDAR-only cross-check.
- [ ] loop-closure count 회귀 없음(8절).
- [ ] 통과 시 `mapper_params.yaml:16` → `/scan_fused`. saved map에 1–25cm 포함 재확인.
- [ ] 두 nav param 파일 최종 sync, 회귀 하니스 재실행으로 baseline 고정.

> 모든 정량 임계값(two-tier 0.02/0.05, k=0.002, range_max 3.0, N=720)은 Phase 2 Stage A 실측으로 확정 전까지 **잠정치**.
