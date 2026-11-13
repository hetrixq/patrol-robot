#!/usr/bin/env bash
set -Eeuo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SOURCE="${1:-}"
TARGET="${2:-}"
if [[ -z "$SOURCE" || -z "$TARGET" ]]; then
  echo 'Использование: ./prepare_project.sh /путь/к/исходному/robomarv_26 /путь/к/новому/патрулю' >&2
  exit 2
fi
SOURCE="$(realpath "$SOURCE")"
if [[ ! -d "$SOURCE/pack" || ! -d "$SOURCE/robot_recognition" || ! -f "$SOURCE/pack/scripts/route_manager.py" ]]; then
  echo "ОШИБКА: не найден оригинальный robomarv_26: $SOURCE" >&2
  exit 2
fi
if [[ -e "$TARGET" ]]; then
  echo "ОШИБКА: $TARGET уже существует. Не перезаписываю." >&2
  exit 3
fi
PARENT="$(dirname "$TARGET")"
mkdir -p "$PARENT"
PARENT="$(realpath "$PARENT")"
TARGET="$PARENT/$(basename "$TARGET")"
STAGE="$(mktemp -d "$PARENT/.patrol_stage_XXXXXXXX")"
cleanup() { [[ -e "$STAGE" ]] && rm -rf "$STAGE"; }
trap cleanup EXIT INT TERM
cp -a "$SOURCE/pack" "$STAGE/pack"
cp -a "$SOURCE/robot_recognition" "$STAGE/robot_recognition"
"$ROOT/install.sh" "$STAGE"
mv "$STAGE" "$TARGET"
trap - EXIT INT TERM
printf '\nГотово, отдельный проект: %s\n' "$TARGET"
printf 'Внутри Docker: cd "%s/pack" && ./preflight.sh && ./start_classroom.sh\n' "$TARGET"
