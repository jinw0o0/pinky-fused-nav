#!/usr/bin/env bash
# =============================================================================
# stop_pinky.sh — cleanly stop the whole Pinky ROS stack so you can relaunch fresh.
#
# Order is SAFE-BY-DESIGN for the motors:
#   1) stop NAV  -> /cmd_vel halts -> bringup watchdog (0.5s) zeros the motors
#   2) stop bringup + ekf (bringup also zeroes motors on shutdown)
#   3) SIGKILL any stragglers (sensors, orphans), reset the daemon
#   4) close local RViz on this laptop
#
# Run it on the LAPTOP (it SSHes into the robot).
#   ./stop_pinky.sh                       # uses PI below
#   PI=pinky@192.168.4.1 ./stop_pinky.sh  # override target
#
# Pattern note: every kill pattern has its first char bracketed (e.g. [r]os2) —
# the classic pgrep/pkill self-match guard so the kill never targets itself.
# =============================================================================
set -uo pipefail
PI="${PI:-pinky@192.168.4.1}"

echo "[stop] robot = $PI"
ssh -o ConnectTimeout=8 "$PI" '
  source /opt/ros/jazzy/setup.bash 2>/dev/null
  echo "  [pi] 1) stopping NAV (cmd_vel halts -> watchdog zeros motors)..."
  pkill -INT -f "[n]avigation_launch" 2>/dev/null
  pkill -INT -f "[m]ap_building"      2>/dev/null
  sleep 2
  echo "  [pi] 2) stopping bringup + ekf (motors zeroed)..."
  pkill -INT -f "[b]ringup_robot" 2>/dev/null
  pkill -INT -f "[e]kf_node"      2>/dev/null
  sleep 2
  echo "  [pi] 3) SIGKILL stragglers + reset daemon..."
  # NOTE: deliberately NO "ros2" pattern here — this script text contains
  # "ros2 daemon stop", and a [r]os2 pattern would match THIS shell cmdline and
  # SIGKILL itself mid-run. Launch parents are caught by [p]inky_ instead. Every
  # alternative is first-char-bracketed so the pattern never matches its own text.
  pkill -9 -f "[s]llidar|[a]stra_camera|[s]can_fuser|[p]inky_|[s]lam_toolbox|[n]av2|[e]kf_node|[e]kf_filter_node|[r]obot_state_pub|[j]oint_state_pub|[c]ontroller_server|[p]lanner_server|[b]t_navigator|[b]ehavior_server|[w]aypoint_follower|[v]elocity_smoother|[s]moother_server|[l]ifecycle_manager|[c]omponent_container|[b]attery_publisher|[m]ain_node" 2>/dev/null
  ros2 daemon stop >/dev/null 2>&1
  sleep 1
  left=$(pgrep -cf "[s]lam_toolbox|[c]ontroller_server|[p]inky_bringup|[s]llidar|[e]kf_filter" 2>/dev/null)
  if [ "${left:-0}" = "0" ]; then echo "  [pi] clean ✓"; else echo "  [pi] !! ${left} procs survived — run again"; fi
'

echo "[stop] 4) laptop: closing rviz2 (for a fresh start)..."
pkill -INT -f "[r]viz2" 2>/dev/null && echo "  [laptop] rviz2 closed" || echo "  [laptop] no rviz2 running"

echo "[stop] clean slate ✓"
echo "[stop] relaunch:  bringup -> slam -> nav (3 Pi terminals)  +  rviz with ROS_DOMAIN_ID=0 (laptop)"
