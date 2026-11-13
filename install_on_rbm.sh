#!/usr/bin/env bash
set -Eeuo pipefail
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
TARGET="${PATROL_TARGET:-/src/patrol_classroom}"
SOURCE="${1:-${PATROL_SOURCE:-}}"
UPSTREAM_DIR="${PATROL_UPSTREAM_DIR:-/src/patrol_source_robomarv26}"
is_source() {
  [[ -f "$1/pack/scripts/route_manager.py" && -f "$1/pack/scripts/leg_detector.py" && -f "$1/robot_recognition/card_recognizer.py" ]]
}
if [[ ! -d /src ]]; then
  echo 'ERROR: /src not found. Run inside the RBM container named ros.' >&2
  exit 2
fi
if [[ -e "$TARGET" ]]; then
  echo "ERROR: $TARGET already exists; not overwriting. Set PATROL_TARGET=/src/patrol_classroom_2 if required." >&2
  exit 3
fi
if [[ -n "$SOURCE" ]] && ! is_source "$SOURCE"; then
  echo "ERROR: given source is not robomarv_26: $SOURCE" >&2
  exit 4
fi
if [[ -z "$SOURCE" ]]; then
  for candidate in /src/scripts /src/robomarv_26 /src/patrol_source_robomarv26 /src/patrol_original; do
    if is_source "$candidate"; then SOURCE="$candidate"; break; fi
  done
fi
if [[ -z "$SOURCE" ]]; then
  echo "No existing robomarv_26 found; cloning original read-only into $UPSTREAM_DIR"
  if ! command -v git >/dev/null 2>&1; then echo 'ERROR: no git'; exit 4; fi
  if [[ -e "$UPSTREAM_DIR" ]]; then echo "ERROR: $UPSTREAM_DIR already exists but is not recognized as source"; exit 4; fi
  git clone --depth 1 https://github.com/MatveyMakiy/robomarv_26.git "$UPSTREAM_DIR"
  SOURCE="$UPSTREAM_DIR"
fi
if ! is_source "$SOURCE"; then echo "ERROR: incomplete source $SOURCE"; exit 4; fi
printf 'Original: %s\nAdd-on: %s\nInstall target: %s\n' "$SOURCE" "$HERE" "$TARGET"
"$HERE/prepare_project.sh" "$SOURCE" "$TARGET"
# RBM v6 already provides the NCNN YOLO model; link only when source copy has no model.
MODEL="$TARGET/pack/models/your_model_ncnn_model"
if [[ ! -e "$MODEL" && -d /root/weights/yolo11n_ncnn_model ]]; then
  mkdir -p "$TARGET/pack/models"
  ln -s /root/weights/yolo11n_ncnn_model "$MODEL"
  echo "Linked RBM NCNN model in separate project: $MODEL"
fi
cat <<EOF

INSTALL COMPLETE. No running system services were changed.
Next, inside the SAME container:
  cd "$TARGET/pack"
  ./preflight.sh
  ./start_classroom.sh

Lichtblick layout file on your laptop:
  lichtblick/PATROL_QR_Classroom.json (in the downloaded ZIP)
EOF
