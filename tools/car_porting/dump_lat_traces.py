#!/usr/bin/env python3
"""
Summarize what limited lateral control in each corner of a route.

Two different things make a car run wide, and they need opposite fixes:

  1. the curvature request is clipped before it ever reaches the steering.
     clip_curvature() in selfdrive/controls/lib/drive_helpers.py caps lateral
     acceleration at 3.0 m/s^2 and lateral jerk at 5 m/s^3, so the model can ask
     for a corner the controller refuses to pass on. Nothing in a car port
     changes this.
  2. the request survives, but the steering never achieves it. That is the
     lateral tune (PID gains, feedforward, torque limits) under-delivering, and
     it is fixable per car.

modelV2.action.desiredCurvature is the raw request and controlsState.desiredCurvature
is what survived the clip, so the two cases are directly distinguishable.

Run it on the device over SSH, on the newest route:

    python3 dump_lat_traces.py

or point it at routes, segments or files (on device, or on a laptop with logs copied over):

    python3 dump_lat_traces.py 00000042--a1b2c3d4e5
    python3 dump_lat_traces.py /data/media/0/realdata/*--3
    python3 dump_lat_traces.py --csv corners.csv <path>
"""

import argparse
import bz2
import os
import subprocess
import sys
from dataclasses import dataclass, field

# make `cereal`/`openpilot` importable when run directly over SSH, where the launch
# script's PYTHONPATH isn't set (python only adds the *script's* dir, not the repo root)
REPO_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
sys.path.insert(0, REPO_ROOT)

REALDATA = os.environ.get("REALDATA", "/data/media/0/realdata")

# mirrors selfdrive/controls/lib/drive_helpers.py, so the reported headroom matches what ran
MAX_LATERAL_ACCEL_NO_ROLL = 3.0  # m/s^2
MAX_LATERAL_JERK = 5.0  # m/s^3
MAX_CURVATURE = 0.2  # 1/m
ACCELERATION_DUE_TO_GRAVITY = 9.81

# the saturation alert ("Turn Exceeds Steering Limit") is gated on this in latcontrol.py
SAT_CHECK_MIN_SPEED = 10.0  # m/s

CLIP_EPS = 2e-4  # 1/m, ignore float noise when comparing requested against used curvature
CORNER_MIN_CURVATURE = 0.01  # 1/m, ~100 m radius
CORNER_MIN_DURATION = 0.8  # s
CORNER_MERGE_GAP = 0.5  # s


def load_log_schema():
  """cereal moved around across openpilot/sunnypilot versions; fall back to the raw schema."""
  for mod in ("openpilot.cereal", "cereal"):
    try:
      return __import__(mod, fromlist=["log"]).log
    except ImportError:
      pass

  import capnp
  capnp.remove_import_hook()
  # log.capnp pulls in car.capnp from opendbc and c++.capnp from pycapnp itself
  imports = [os.path.dirname(capnp.__file__),
             os.path.join(REPO_ROOT, "opendbc_repo", "opendbc", "car"),
             os.path.join(REPO_ROOT, "opendbc", "car")]
  for d in (os.path.join(REPO_ROOT, "openpilot", "cereal"), os.path.join(REPO_ROOT, "cereal")):
    if os.path.exists(os.path.join(d, "log.capnp")):
      return capnp.load(os.path.join(d, "log.capnp"), imports=[d] + [i for i in imports if os.path.isdir(i)])
  raise SystemExit("could not import cereal or find log.capnp")


def decompress_zst(dat: bytes) -> bytes:
  try:
    import zstandard
    with zstandard.ZstdDecompressor().stream_reader(dat) as reader:
      return reader.read()
  except ImportError:
    pass
  try:
    return subprocess.run(["zstd", "-dc"], input=dat, capture_output=True, check=True).stdout
  except (OSError, subprocess.CalledProcessError) as e:
    raise SystemExit(f"need the zstandard module or the zstd binary to read .zst logs: {e}") from e


def read_log(path: str):
  log = load_log_schema()
  with open(path, "rb") as f:
    dat = f.read()

  if dat.startswith(b"BZh9"):
    dat = bz2.decompress(dat)
  elif dat.startswith(b"\x28\xb5\x2f\xfd"):
    dat = decompress_zst(dat)

  return log.Event.read_multiple_bytes(dat)


def find_logs(paths: list[str]) -> list[str]:
  """Accept rlog files, segment dirs, route names, or nothing (newest route)."""
  if not paths:
    if not os.path.isdir(REALDATA):
      raise SystemExit(f"no paths given and {REALDATA} does not exist")
    segs = [os.path.join(REALDATA, d) for d in os.listdir(REALDATA)]
    segs = [s for s in segs if os.path.isdir(s)]
    if not segs:
      raise SystemExit(f"no routes found in {REALDATA}")
    newest = max(segs, key=os.path.getmtime)
    route = os.path.basename(newest).rsplit("--", 1)[0]
    print(f"# no path given, using newest route: {route}", file=sys.stderr)
    paths = [route]

  out: list[str] = []
  for p in paths:
    if os.path.isfile(p):
      out.append(p)
      continue

    # a segment dir, or a route name to expand into its segments
    dirs = [p] if os.path.isdir(p) else []
    if not dirs:
      parent, name = os.path.split(p.rstrip("/"))
      for d in ([parent] if parent else [".", REALDATA]):
        if os.path.isdir(d):
          dirs = sorted(os.path.join(d, s) for s in os.listdir(d)
                        if s.startswith(name) and os.path.isdir(os.path.join(d, s)))
        if dirs:
          break

    for d in dirs:
      for fn in ("rlog.zst", "rlog.bz2", "rlog"):
        if os.path.exists(os.path.join(d, fn)):
          out.append(os.path.join(d, fn))
          break

  if not out:
    raise SystemExit(f"no rlogs found for {paths} (qlogs are too decimated for this)")
  return sorted(out)


@dataclass
class Row:
  t: float = 0.0
  v_ego: float = 0.0
  lat_active: bool = False
  curv_req: float = 0.0  # model request, before clip_curvature
  curv_used: float = 0.0  # what the lateral controller was given
  curv_actual: float = 0.0  # measured, from the vehicle model
  angle_des: float = 0.0
  angle_act: float = 0.0
  output: float = 0.0
  saturated: bool = False
  pressed: bool = False
  driver_torque: float = 0.0
  roll: float = 0.0
  steer_ratio: float = 0.0
  confidence: float = 1.0

  @property
  def clipped(self) -> bool:
    return abs(self.curv_req) - abs(self.curv_used) > CLIP_EPS


def extract_rows(paths: list[str]) -> list[Row]:
  """controlsState runs at 100 Hz, so emit one row per controlsState and carry the rest forward."""
  rows: list[Row] = []
  cur = Row()
  t0 = None

  for path in paths:
    for evt in read_log(path):
      which = evt.which()

      if which == "carState":
        cs = evt.carState
        cur.v_ego = cs.vEgo
        cur.pressed = cs.steeringPressed
        cur.driver_torque = cs.steeringTorque
      elif which == "carControl":
        cur.lat_active = evt.carControl.latActive
      elif which == "liveParameters":
        cur.roll = evt.liveParameters.roll
        cur.steer_ratio = evt.liveParameters.steerRatio
      elif which == "modelV2":
        model = evt.modelV2
        cur.curv_req = model.action.desiredCurvature
        preds = model.meta.disengagePredictions
        brake = max(preds.brakeDisengageProbs) if len(preds.brakeDisengageProbs) else 1.0
        steer = max(preds.steerOverrideProbs) if len(preds.steerOverrideProbs) else 1.0
        cur.confidence = (1 - brake) * (1 - steer)
      elif which == "controlsState":
        c = evt.controlsState
        cur.curv_used = c.desiredCurvature
        cur.curv_actual = c.curvature

        lat = c.lateralControlState
        state = getattr(lat, lat.which())
        cur.angle_des = getattr(state, "steeringAngleDesiredDeg", 0.0)
        cur.angle_act = getattr(state, "steeringAngleDeg", 0.0)
        cur.output = getattr(state, "output", 0.0)
        cur.saturated = getattr(state, "saturated", False)

        t = evt.logMonoTime / 1e9
        t0 = t if t0 is None else t0
        cur.t = t - t0
        rows.append(Row(**cur.__dict__))

  return rows


@dataclass
class Corner:
  rows: list[Row] = field(default_factory=list)

  @property
  def t_start(self) -> float:
    return self.rows[0].t

  @property
  def duration(self) -> float:
    return self.rows[-1].t - self.rows[0].t

  @property
  def peak(self) -> Row:
    return max(self.rows, key=lambda r: abs(r.curv_req))

  def frac(self, pred) -> float:
    return sum(1 for r in self.rows if pred(r)) / len(self.rows)

  @property
  def tracking(self) -> float | None:
    """mean achieved angle over mean commanded angle, through the meat of the corner"""
    peak_des = max(abs(r.angle_des) for r in self.rows)
    meat = [r for r in self.rows if abs(r.angle_des) > 0.5 * peak_des and not r.pressed]
    if not meat or peak_des < 1.0:
      return None
    des = sum(abs(r.angle_des) for r in meat)
    return sum(abs(r.angle_act) for r in meat) / des if des > 0 else None

  def verdicts(self) -> list[str]:
    out = []
    clip_frac = self.frac(lambda r: r.clipped)
    track = self.tracking
    if clip_frac > 0.25:
      out.append(f"REQUEST CLIPPED {clip_frac * 100:.0f}% of corner")
    if track is not None and track < 0.90:
      out.append(f"STEERING UNDER-DELIVERS (achieved {track * 100:.0f}% of commanded angle)")
    if self.frac(lambda r: abs(r.output) > 0.99) > 0.1:
      out.append("TORQUE OUTPUT AT LIMIT")
    if self.frac(lambda r: r.pressed) > 0.5:
      out.append("driver was steering, ignore")
    return out or ["ok"]


def find_corners(rows: list[Row], min_curvature: float) -> list[Corner]:
  corners: list[Corner] = []
  cur: list[Row] = []
  last_hot = None

  for r in rows:
    hot = r.lat_active and abs(r.curv_req) >= min_curvature
    if hot:
      if last_hot is not None and r.t - last_hot > CORNER_MERGE_GAP and cur:
        corners.append(Corner(cur))
        cur = []
      cur.append(r)
      last_hot = r.t
    elif cur and last_hot is not None and r.t - last_hot > CORNER_MERGE_GAP:
      corners.append(Corner(cur))
      cur, last_hot = [], None

  if cur:
    corners.append(Corner(cur))
  return [c for c in corners if c.duration >= CORNER_MIN_DURATION]


def report(corners: list[Corner]) -> None:
  print(f"\n{len(corners)} corners\n")
  hdr = f"{'t':>7}  {'dur':>5}  {'speed':>11}  {'radius':>7}  {'lat accel':>17}  {'angle des/act':>15}  verdict"
  print(hdr)
  print("-" * (len(hdr) + 30))

  for c in corners:
    p = c.peak
    v = p.v_ego
    req_accel = abs(p.curv_req) * v ** 2
    used_accel = abs(p.curv_used) * v ** 2
    radius = 1 / abs(p.curv_req) if abs(p.curv_req) > 1e-6 else float("inf")
    track = c.tracking

    speed = f"{v * 3.6:.0f}->{min(r.v_ego for r in c.rows) * 3.6:.0f} km/h"
    accel = f"{req_accel:.1f} -> {used_accel:.1f} m/s2"
    angles = f"{p.angle_des:6.1f} /{p.angle_act:6.1f}"
    verdict = '; '.join(c.verdicts())
    print(f"{c.t_start:7.1f}  {c.duration:5.1f}  {speed:>11}  {radius:6.0f}m  {accel:>17}  {angles:>15}  {verdict}")
    if track is not None:
      err = max(abs(r.angle_des - r.angle_act) for r in c.rows)
      conf = min(r.confidence for r in c.rows)
      print(f"{'':>7}  tracking {track * 100:.0f}%, peak angle error {err:.1f} deg, min confidence {conf:.2f}")

  clipped = [c for c in corners if c.frac(lambda r: r.clipped) > 0.25]
  under = [c for c in corners if (c.tracking or 1.0) < 0.90]
  limits = f"{MAX_LATERAL_ACCEL_NO_ROLL} m/s2 / {MAX_LATERAL_JERK} m/s3"
  print(f"\n# {len(clipped)}/{len(corners)} corners had the request clipped by the {limits} limits")
  print(f"# {len(under)}/{len(corners)} corners had the steering achieve <90% of the commanded angle")

  # the saturation alert is speed gated, so clipping below this is silent
  silent = [c for c in clipped if c.peak.v_ego < SAT_CHECK_MIN_SPEED]
  if silent:
    gate = f"below {SAT_CHECK_MIN_SPEED * 3.6:.0f} km/h, where the 'Turn Exceeds Steering Limit' alert is suppressed"
    print(f"# {len(silent)}/{len(clipped)} clipped corners were {gate}")


def write_csv(path: str, rows: list[Row]) -> None:
  import csv
  cols = list(Row().__dict__.keys()) + ["clipped"]
  with open(path, "w", newline="") as f:
    w = csv.writer(f)
    w.writerow(cols)
    for r in rows:
      w.writerow([getattr(r, c) for c in cols])
  print(f"\n# wrote {len(rows)} rows to {path}")


def main() -> None:
  p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
  p.add_argument("paths", nargs="*", help="rlog files, segment dirs, or a route name (default: newest route)")
  p.add_argument("--csv", help="also dump every sample to this CSV")
  p.add_argument("--min-curvature", type=float, default=CORNER_MIN_CURVATURE,
                 help=f"corner detection threshold in 1/m (default {CORNER_MIN_CURVATURE}, ~100 m radius)")
  args = p.parse_args()

  paths = find_logs(args.paths)
  print(f"# reading {len(paths)} rlog(s)", file=sys.stderr)
  rows = extract_rows(paths)
  if not rows:
    raise SystemExit("no controlsState in these logs")

  engaged = sum(1 for r in rows if r.lat_active)
  print(f"# {len(rows)} samples, {engaged / 100:.0f}s with lateral control active")

  corners = find_corners(rows, args.min_curvature)
  if corners:
    report(corners)
  else:
    print(f"\nno corners above {args.min_curvature} 1/m while engaged")

  if args.csv:
    write_csv(args.csv, rows)


if __name__ == "__main__":
  main()
