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
#   ./upload_logs.sh -s                       # send every segment the destination lacks
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
sync_all=0

while getopts "adsh" opt; do
  case "$opt" in
    a) names+=(qlog.zst qlog.bz2 qlog qcamera.ts fcamera.hevc ecamera.hevc dcamera.hevc) ;;
    d) dry_run=1 ;;
    s) sync_all=1 ;;
    h) sed -n '2,22p' "$0"; exit 0 ;;
    *) exit 1 ;;
  esac
done
shift $((OPTIND - 1))

[ -d "$REALDATA" ] || { echo "no $REALDATA (run this on the device, or set REALDATA)" >&2; exit 1; }
cd "$REALDATA"

case "$DEST" in
  *:*) remote_host="${DEST%%:*}"; remote_path="${DEST#*:}" ;;
  *)   remote_host=""; remote_path="$DEST" ;;   # local path, for testing
esac

list_dest() {
  if [ -n "$remote_host" ]; then
    ssh "$remote_host" "ls -1 $remote_path 2>/dev/null" 2>/dev/null || true
  else
    ls -1 "$remote_path" 2>/dev/null || true
  fi
}

if [ "$sync_all" -eq 1 ]; then
  have=$(list_dest)
  files=()
  segs=0
  for d in $(ls -1d -- */ 2>/dev/null | sed 's:/$::' | grep -E -- '--[0-9]+$' | sort); do
    printf '%s\n' "$have" | grep -qxF "$d" && continue
    added=0
    for name in "${names[@]}"; do
      [ -f "$d/$name" ] && { files+=("$d/$name"); added=1; }
    done
    [ "$added" -eq 1 ] && segs=$((segs + 1))
  done
  [ "${#files[@]}" -gt 0 ] || { echo "# destination already has every segment"; exit 0; }
  size=$(du -ch "${files[@]}" | tail -1 | cut -f1)
  echo "# sync: ${#files[@]} files across $segs new segments, $size"
  echo "# -> $DEST"
  if [ "$dry_run" -eq 1 ]; then printf '%s\n' "${files[@]}"; exit 0; fi
  if [ -n "$remote_host" ]; then
    tar cf - "${files[@]}" | ssh "$remote_host" "mkdir -p $remote_path && tar xf - -C $remote_path"
  else
    mkdir -p "$remote_path" && tar cf - "${files[@]}" | tar xf - -C "$remote_path"
  fi
  echo "# done"
  exit 0
fi

route="${1:-}"
if [ -z "$route" ]; then
  # newest segment directory wins; strip the trailing --N to get the route.
  # the log root also holds boot/ and crash/, so match only <route>--<segment>.
  newest=$(ls -1dt -- */ 2>/dev/null | sed 's:/$::' | grep -E -- '--[0-9]+$' | head -1) || true
  [ -n "$newest" ] || { echo "no route segments in $REALDATA" >&2; exit 1; }
  route=${newest%--*}
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
if [ -n "$remote_host" ]; then
  tar cf - "${files[@]}" | ssh "$remote_host" "mkdir -p $remote_path && tar xf - -C $remote_path"
else
  mkdir -p "$remote_path" && tar cf - "${files[@]}" | tar xf - -C "$remote_path"
fi

echo "# done"
