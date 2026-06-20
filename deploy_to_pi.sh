#!/usr/bin/env bash
# =============================================================================
# Deploy Pinky dev tree <-> Pi5.  (rsync = scp-override done safely:
#   no dir-nesting, only changed files, excludes x86 build artifacts.)
#
# Set the target once:   export PI=pinky@192.168.0.42   (or pass as 2nd arg)
#
#   ./deploy_to_pi.sh push           # dev -> Pi5 : code + vault (CODE is dev-authoritative)
#   ./deploy_to_pi.sh push-all       # dev -> Pi5 : also overwrite config/urdf (DANGER: clobbers
#                                    #              robot-measured calibration; use only first time)
#   ./deploy_to_pi.sh pull-config    # Pi5 -> dev : bring robot-tuned params/urdf back
#   ./deploy_to_pi.sh diff           # dry-run of `push` (shows what WOULD change, transfers nothing)
#
# Why two dirs only: build/ install/ log/ live OUTSIDE src/, so they never ride along.
# =============================================================================
set -euo pipefail

LOCAL="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"      # .../pinky_pro
PI="${PI:-${2:-}}"
REMOTE="${REMOTE:-pinky_pro}"                              # ~/pinky_pro on the Pi5
MODE="${1:-}"

[[ -z "$PI" ]] && { echo "set PI=user@host (env or 2nd arg)"; exit 2; }

# Files the ROBOT owns (measured on-robot) — pushed only by push-all, pulled by pull-config.
CALIB=(
  "src/pinky_perception/config/scan_fuser.yaml"
  "src/pinky_description/urdf/pinky.urdf.xacro"
)
# Exclude by BASENAME (robust to rsync path anchoring; these names are unique).
CALIB_EXCLUDES=(--exclude='scan_fuser.yaml' --exclude='pinky.urdf.xacro')

RSYNC=(rsync -av --exclude build --exclude install --exclude log
       --exclude '__pycache__' --exclude '.git')

case "$MODE" in
  push)
    # Code + vault, but DO NOT overwrite robot-tuned calibration files.
    "${RSYNC[@]}" "${CALIB_EXCLUDES[@]}" \
      "$LOCAL/src" "$LOCAL/개인프로젝트" "$PI:~/$REMOTE/"
    echo "[push] code+vault sent (calibration files left untouched on Pi5)."
    ;;
  push-all)
    echo "!! push-all OVERWRITES robot-measured calibration on the Pi5. Ctrl-C to abort."
    read -r -p "continue? [y/N] " a; [[ "$a" == "y" ]] || exit 1
    "${RSYNC[@]}" "$LOCAL/src" "$LOCAL/개인프로젝트" "$PI:~/$REMOTE/"
    ;;
  pull-config)
    for f in "${CALIB[@]}"; do
      rsync -av "$PI:~/$REMOTE/$f" "$LOCAL/$f"
    done
    echo "[pull-config] robot-tuned calibration copied back to dev."
    ;;
  diff)
    "${RSYNC[@]}" -n "${CALIB_EXCLUDES[@]}" \
      "$LOCAL/src" "$LOCAL/개인프로젝트" "$PI:~/$REMOTE/"
    echo "[diff] dry-run only — nothing transferred."
    ;;
  *)
    echo "usage: PI=user@host $0 {push|push-all|pull-config|diff}"; exit 2;;
esac
