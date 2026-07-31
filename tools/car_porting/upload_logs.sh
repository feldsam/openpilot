#!/usr/bin/env bash
# Send route logs from the device to a host for offline analysis.
#
# Runs ON the comma, over SSH. Sends rlogs only by default, since the camera files are
# roughly ten times larger and are not what the analysis tools read. Directory structure
# is preserved so dump_lat_traces.py can read the result as a route.
#
#   ./upload_logs.sh                          # newest route
#   ./upload_logs.sh 00000003--a1b2c3d4e5     # a specific route
#   ./upload_logs.sh 00000003--a1b2c3d4e5 4 5 6   # only those segments
#   ./upload_logs.sh -a <route>               # include qlogs and camera files
#   ./upload_logs.sh -d <route>               # dry run, just report what would go
#   DEST=user@host:/path ./upload_logs.sh     # somewhere else
#
# The default host is an IP on purpose: web0.feldhost.cz resolves to a different machine.

set -euo pipefail

REALDATA="${REALDATA:-/data/media/0/realdata}"
DEST="${DEST:-feldsam@185.174.169.39:~/prelude-logs}"

names=(rlog.zst rlog.bz2 rlog)
dry_run=0

while getopts "adh" opt; do
  case "$opt" in
    a) names+=(qlog.zst qlog.bz2 qlog qcamera.ts fcamera.hevc ecamera.hevc dcamera.hevc) ;;
    d) dry_run=1 ;;
    h) sed -n '2,20p' "$0"; exit 0 ;;
    *) exit 1 ;;
  esac
done
shift $((OPTIND - 1))

[ -d "$REALDATA" ] || { echo "no $REALDATA (run this on the device, or set REALDATA)" >&2; exit 1; }
cd "$REALDATA"

route="${1:-}"
if [ -z "$route" ]; then
  # newest segment directory wins; strip the trailing --N to get the route
  newest=$(ls -1dt -- */ 2>/dev/null | head -1) || true
  [ -n "$newest" ] || { echo "no routes in $REALDATA" >&2; exit 1; }
  route=$(basename "$newest" | sed 's/--[0-9]*$//')
  echo "# no route given, using newest: $route"
fi
shift || true

# remaining args are segment numbers; default to every segment of the route
if [ "$#" -gt 0 ]; then
  segments=("$@")
else
  segments=()
  for d in "$route"--*/; do
    [ -d "$d" ] || continue
    segments+=("$(basename "$d" | sed "s/^${route}--//")")
  done
fi
[ "${#segments[@]}" -gt 0 ] || { echo "no segments found for $route" >&2; exit 1; }

files=()
for seg in "${segments[@]}"; do
  for name in "${names[@]}"; do
    [ -f "$route--$seg/$name" ] && files+=("$route--$seg/$name")
  done
done
[ "${#files[@]}" -gt 0 ] || { echo "no matching files for $route" >&2; exit 1; }

size=$(du -ch "${files[@]}" | tail -1 | cut -f1)
echo "# $route: ${#files[@]} files across ${#segments[@]} segments, $size"
echo "# -> $DEST"

if [ "$dry_run" -eq 1 ]; then
  printf '%s\n' "${files[@]}"
  exit 0
fi

# tar over ssh rather than scp: it keeps the segment directories, and rsync is not
# installed on either end. Not resumable, so send fewer segments if the link is flaky.
remote_host="${DEST%%:*}"
remote_path="${DEST#*:}"
tar cf - "${files[@]}" | \
  ssh "$remote_host" "mkdir -p $remote_path && tar xf - -C $remote_path"

echo "# done"
