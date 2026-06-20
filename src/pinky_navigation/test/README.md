# Fused-scan verification harness (Phase 2 → 3 gate)

Proves the `scan_fuser` pipeline (1) does **not** smear the floor into the map and
(2) **does** put real 1–25 cm objects into the map. This is the gate that must
pass before promoting the mapper to `/scan_fused` (Phase 3,
`mapper_params.yaml:16`). See `../docs/scan_fusion_design.md` §7.

## Files
- `score_map.py` — map scorer (✅ unit-tested on dev with synthetic maps).
- `verify_floor_rejection.sh` — driver: bag → fused map + lidar map → score.
  *(orchestration, run on the robot/Pi5; not runnable on the x86 dev box.)*
- `objects.example.csv` — template for the OBJECT-bag known objects.

## Core idea
A floor false-positive = a cell **occupied in the FUSED map but FREE in the
LiDAR-only map** (the lidar plane at 12.5 cm cannot see the floor, so anything it
confirmed free that fused marks occupied is depth's mistake). The harness builds
both maps from the **same bag** and diffs them in world coordinates.

## Record the two bags (on the robot, flat floor)
```bash
# Required topics (golden-bag subset):
ros2 bag record -o bag_floor  /scan /camera/depth/points /tf /tf_static /odom imu_raw
#   FLOOR : empty 3x3 m area, drive a slow loop ~60 s (nothing in front).
#   OBJECT: same loop with 2cm / 10cm / 24cm objects at measured (x,y); fill objects.csv.
```
The bag must carry the dynamic TF (odom→base_footprint etc.) but **not** a baked
`map→odom` (slam regenerates it). All consumers replay with `use_sim_time:=true`.

## Run
```bash
# floor rejection (needs the FLOOR bag)
./verify_floor_rejection.sh /path/to/bag_floor floor

# object detection (needs the OBJECT bag + filled objects.csv)
./verify_floor_rejection.sh /path/to/bag_object object objects.csv
```
Exit code: `0 = PASS`, `1 = FAIL`. Maps are written to `$OUTDIR` (default
`/tmp/pinky_verify`): inspect `fused.pgm` / `lidar.pgm` in an image viewer or RViz.

## PASS criteria (defaults, tune via flags)
- **floor**: depth-added/swept-free ≤ `--max-ratio` (0.5%) **and** no depth-added
  connected blob ≥ `--blob-cells` (3 ≈ 15 cm, i.e. big enough to block a path).
- **object**: every `required` object has an occupied cell within `--window`
  (0.10 m) in the fused map. `depth_unique=Y` (fused sees it, lidar does not)
  is the proof that depth added a sub-lidar-plane obstacle.

## Score a pre-made pair of maps directly
```bash
python3 score_map.py floor  --fused fused.yaml --lidar lidar.yaml
python3 score_map.py object --fused fused.yaml --lidar lidar.yaml --objects objects.csv
```

> Thresholds (`max-ratio`, `blob-cells`, `window`) and the depth gate values are
> **provisional** until Stage A calibration on the robot. Re-run this harness
> after any tilt/gate change to catch regressions.
