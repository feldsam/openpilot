#!/usr/bin/env python3
"""
Dump a car's queried ECU firmware versions (and learned lateral params) from a
comma device, formatted for pasting into opendbc/car/<brand>/fingerprints.py.

Run this ON the device (comma three/four), over SSH, after having driven the car
at least once:

    python3 dump_car_fw.py

It reads the persisted params written by card/paramsd, so it works even when the
car was force-fingerprinted as a different platform: the FW query always runs
against the real car before any fingerprint override is applied.

Optionally pass a platform name to label the output:

    python3 dump_car_fw.py HONDA_PRELUDE_6G
"""

import os
import sys

PARAMS_DIR = os.environ.get("PARAMS_DIR", "/data/params/d")


def _read_struct(struct, raw: bytes):
  # capnp's from_bytes is a context manager; the reader stays valid as long as `raw` is alive
  with struct.from_bytes(raw, traversal_limit_in_words=2**48) as msg:
    return msg


def load_car_params(raw: bytes):
  """CarParams has lived in a few places across openpilot/sunnypilot versions."""
  errors = []
  for mod, attr in (("opendbc.car.structs", "CarParams"), ("cereal", "car")):
    try:
      imported = __import__(mod, fromlist=[attr])
      struct = getattr(imported, attr)
      if mod == "cereal":
        struct = struct.CarParams
      return _read_struct(struct, raw)
    except Exception as e:  # noqa: BLE001
      errors.append(f"{mod}: {e}")
  raise SystemExit("Could not parse CarParams:\n  " + "\n  ".join(errors))


def read_param(name: str) -> bytes | None:
  path = os.path.join(PARAMS_DIR, name)
  if not os.path.exists(path):
    return None
  with open(path, "rb") as f:
    return f.read()


def main() -> None:
  platform = sys.argv[1] if len(sys.argv) > 1 else None

  raw = read_param("CarParamsPersistent") or read_param("CarParams")
  if raw is None:
    raise SystemExit(f"No CarParamsPersistent/CarParams in {PARAMS_DIR}. Drive the car once first, "
                     f"or set PARAMS_DIR.")

  CP = load_car_params(raw)

  print(f"# fingerprinted as : {CP.carFingerprint}")
  print(f"# fingerprintSource: {CP.fingerprintSource}")
  print(f"# VIN              : {CP.carVin}")
  print()

  # group by (ecu, address, subAddress), skipping data-collection-only responses
  by_ecu: dict[tuple[str, int, int | None], list[bytes]] = {}
  for fw in CP.carFw:
    if getattr(fw, "logging", False):
      continue
    key = (str(fw.ecu), fw.address, None if fw.subAddress == 0 else fw.subAddress)
    by_ecu.setdefault(key, [])
    if bytes(fw.fwVersion) not in by_ecu[key]:
      by_ecu[key].append(bytes(fw.fwVersion))

  if not by_ecu:
    print("# WARNING: no non-logging FW responses found.")

  name = platform or CP.carFingerprint
  print(f"  CAR.{name}: {{")
  for (ecu, addr, sub_addr), versions in sorted(by_ecu.items(), key=lambda kv: kv[0][1]):
    print(f"    (Ecu.{ecu}, {hex(addr)}, {sub_addr}): [")
    for v in sorted(versions):
      print(f"      {v!r},")
    print("    ],")
  print("  },")

  # learned lateral params are the best source for steerRatio / tire stiffness
  lp_raw = read_param("LiveParametersV2")
  if lp_raw is not None:
    try:
      from cereal import log
      lp = _read_struct(log.Event, lp_raw).liveParameters
      print()
      print(f"# learned steerRatio      : {lp.steerRatio:.3f} (valid={lp.steerRatioValid})")
      print(f"# learned stiffnessFactor : {lp.stiffnessFactor:.3f} (valid={lp.stiffnessFactorValid})")
      print(f"# learned angleOffsetAvg  : {lp.angleOffsetAverageDeg:.3f} deg")
    except Exception as e:  # noqa: BLE001
      print(f"\n# could not read LiveParametersV2: {e}")


if __name__ == "__main__":
  main()
