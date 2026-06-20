#!/usr/bin/env bash
# =============================================================================
# Pinky fused-scan verification harness (design doc §7).
#
# Builds TWO occupancy maps from ONE rosbag and scores depth-introduced
# false obstacles:
#   - FUSED map : slam_toolbox(mapping) on /scan_fused  (scan_fuser running)
#   - LIDAR map : slam_toolbox(mapping) on /scan         (raw lidar only)
# then runs score_map.py to compare them.
#
# USAGE:
#   verify_floor_rejection.sh <bag_path> floor
#   verify_floor_rejection.sh <bag_path> object <objects.csv>
#
# The bag MUST contain: /scan  /camera/depth/points  /tf  /tf_static
#   (and NOT a baked map->odom; slam regenerates it). Record with publish_tf
#   handled per the golden-bag plan. See README.md.
#
# NOTE: run on the robot/Pi5 (or any machine that can replay the bag with
#       slam_toolbox + scan_fuser). NOT tested on the x86 dev box — the
#       SCORER (score_map.py) is unit-tested; this orchestration is best-effort.
# =============================================================================
set -uo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_ROOT="$(cd "$HERE/../../.." && pwd)"          # .../pinky_pro
MAPPER_PARAMS="$HERE/../params/mapper_params.yaml"
SCORER="$HERE/score_map.py"
OUTDIR="${OUTDIR:-/tmp/pinky_verify}"
SETTLE="${SETTLE:-4}"                            # s to let slam finish after bag ends

BAG="${1:-}"
MODE="${2:-}"
OBJECTS="${3:-}"

if [[ -z "$BAG" || -z "$MODE" ]]; then
  echo "usage: $0 <bag_path> floor|object [objects.csv]" >&2
  exit 2
fi
[[ "$MODE" == "object" && -z "$OBJECTS" ]] && { echo "object mode needs objects.csv" >&2; exit 2; }

# --- environment ---
source /opt/ros/jazzy/setup.bash
[[ -f "$WS_ROOT/install/setup.bash" ]] && source "$WS_ROOT/install/setup.bash"
mkdir -p "$OUTDIR"

PIDS=()
cleanup() { for p in "${PIDS[@]:-}"; do kill "$p" 2>/dev/null || true; done; }
trap cleanup EXIT

# build_map <scan_topic> <out_basename> <with_fuser:0|1>
build_map() {
  local scan_topic="$1" out="$2" with_fuser="$3"
  echo ">>> building map: scan_topic=$scan_topic fuser=$with_fuser -> $out"
  PIDS=()

  ros2 run slam_toolbox sync_slam_toolbox_node --ros-args \
      --params-file "$MAPPER_PARAMS" \
      -p use_sim_time:=true \
      -p scan_topic:="$scan_topic" &
  PIDS+=($!)

  if [[ "$with_fuser" == "1" ]]; then
    ros2 launch pinky_perception scan_fuser.launch.xml use_sim_time:=True &
    PIDS+=($!)
  fi

  sleep 3                                        # let nodes come up before replay
  echo ">>> replaying bag (blocks until done)..."
  ros2 bag play "$BAG" --clock                   # blocks until bag ends
  echo ">>> bag done; settling ${SETTLE}s for slam..."
  sleep "$SETTLE"

  ros2 run nav2_map_server map_saver_cli -f "$OUTDIR/$out" \
      --ros-args -p use_sim_time:=true
  cleanup
  PIDS=()
  sleep 1
}

build_map "/scan_fused" "fused" 1
build_map "/scan"       "lidar" 0

echo
echo "############ SCORING ############"
if [[ "$MODE" == "floor" ]]; then
  python3 "$SCORER" floor --fused "$OUTDIR/fused.yaml" --lidar "$OUTDIR/lidar.yaml"
else
  python3 "$SCORER" object --fused "$OUTDIR/fused.yaml" --lidar "$OUTDIR/lidar.yaml" \
      --objects "$OBJECTS"
fi
RC=$?
echo "############ exit $RC (0=PASS, 1=FAIL) ############"
exit $RC
