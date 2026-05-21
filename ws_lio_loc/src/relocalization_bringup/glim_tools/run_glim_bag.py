#!/usr/bin/env python3
"""Single-shot GLIM mapping over a whole ROS 2 bag.

Spawns three child processes in order:
  1. livox_custom_to_pc2 relay (Mid360 CustomMsg -> PointCloud2)
  2. glim_rosnode (mapping)
  3. ros2 bag play <bag> --clock

When the bag finishes playing, the script SIGINTs glim so it can write its
dump (graph.bin + submaps + per-submap PCDs) to --output-dir.

This is the no-chunking variant: if the bag is too long for your GPU, use
run_glim_chunked.py instead. The interface intentionally mirrors that script
for muscle-memory consistency.

Assumes the calling shell has already sourced:
  /opt/ros/humble/setup.bash
  ~/livox_driver_ws/install/setup.bash
  ~/glim_ws/install/setup.bash

Usage:
  ros2 run relocalization_bringup run_glim_bag \\
      --bag /path/to/bag --output-dir /path/to/dump
"""

import argparse
import os
import signal
import subprocess
import sys
import time
from pathlib import Path

try:
    from ament_index_python.packages import get_package_share_directory
    _SHARE = Path(get_package_share_directory("relocalization_bringup"))
except Exception:
    _SHARE = None

# Viewer variants are sibling config dirs. --headless / --rviz-only just
# pick a different one; the only difference is the extension_modules list
# in config_ros.json.
CONFIG_VARIANTS = {
    "default": "glim_mid360",        # iridescence GUI + RViz topics
    "rviz_only": "glim_mid360_rviz", # RViz topics only
    "headless": "glim_mid360_headless",
}

DEFAULT_CONFIG = (_SHARE / "config" / CONFIG_VARIANTS["default"]) if _SHARE else None
DEFAULT_RELAY = Path(__file__).resolve().parent / "livox_custom_to_pc2.py"


def log(msg):
    sys.stdout.write(f"[run_glim_bag] {msg}\n")
    sys.stdout.flush()


def spawn(cmd):
    """Run cmd (list[str]) in its own process group so we can SIGINT just it."""
    return subprocess.Popen(cmd, preexec_fn=os.setsid)


def stop(p, sig=signal.SIGINT, timeout=30):
    if p is None or p.poll() is not None:
        return
    try:
        os.killpg(os.getpgid(p.pid), sig)
    except ProcessLookupError:
        return
    try:
        p.wait(timeout=timeout)
    except subprocess.TimeoutExpired:
        log(f"pid {p.pid} did not exit on {sig.name}, escalating to SIGKILL")
        try:
            os.killpg(os.getpgid(p.pid), signal.SIGKILL)
        except ProcessLookupError:
            pass
        p.wait(timeout=5)


def main():
    ap = argparse.ArgumentParser(
        formatter_class=argparse.RawDescriptionHelpFormatter, description=__doc__
    )
    ap.add_argument("--bag", required=True, type=Path,
                    help="ROS 2 bag directory (containing metadata.yaml)")
    ap.add_argument("--output-dir", required=True, type=Path,
                    help="where GLIM should dump its session")
    ap.add_argument("--config", type=Path, default=DEFAULT_CONFIG,
                    help=f"GLIM config_path (default {DEFAULT_CONFIG})")
    ap.add_argument("--relay", type=Path, default=DEFAULT_RELAY,
                    help="path to livox CustomMsg -> PointCloud2 relay script")
    ap.add_argument("--no-relay", action="store_true",
                    help="skip the relay (use if the bag already has PointCloud2 topics)")
    ap.add_argument("--ramp-up", type=float, default=4.0,
                    help="seconds to wait after launching relay+glim before starting the bag")
    ap.add_argument("--drain", type=float, default=30.0,
                    help="seconds to wait after bag ends before signaling glim to save")
    ap.add_argument("--save-timeout", type=float, default=300.0,
                    help="max seconds to wait for glim to write its dump")
    ap.add_argument("--rate", type=float, default=1.0,
                    help="ros2 bag play rate (default 1.0)")
    viewer = ap.add_mutually_exclusive_group()
    viewer.add_argument("--headless", action="store_true",
                        help="Use the headless config variant (no Iridescence GUI, "
                             "no RViz publisher). Implied by --config.")
    viewer.add_argument("--rviz-only", action="store_true",
                        help="Use the rviz-only config variant (no Iridescence GUI, "
                             "keeps RViz publisher). Implied by --config.")
    args = ap.parse_args()

    # Resolve viewer variant unless the caller explicitly passed --config.
    if args.config == DEFAULT_CONFIG and _SHARE is not None:
        if args.headless:
            args.config = _SHARE / "config" / CONFIG_VARIANTS["headless"]
        elif args.rviz_only:
            args.config = _SHARE / "config" / CONFIG_VARIANTS["rviz_only"]

    if args.config is None or not (args.config / "config.json").exists():
        sys.exit(f"glim config dir missing config.json: {args.config}")
    if not args.bag.exists():
        sys.exit(f"bag not found: {args.bag}")
    if not args.no_relay and not args.relay.exists():
        sys.exit(f"relay script not found: {args.relay}")

    args.output_dir.mkdir(parents=True, exist_ok=True)
    log(f"using config: {args.config}")

    relay = None
    glim = None
    bag_p = None
    try:
        if not args.no_relay:
            relay = spawn(["/usr/bin/env", "python3", str(args.relay)])
        glim = spawn([
            "ros2", "run", "glim_ros", "glim_rosnode", "--ros-args",
            "-p", f"config_path:={args.config}",
            "-p", f"dump_path:={args.output_dir}",
        ])

        log(f"waiting {args.ramp_up}s for relay+glim to subscribe")
        time.sleep(args.ramp_up)
        if not args.no_relay and relay.poll() is not None:
            raise RuntimeError(f"relay died before bag play (exit={relay.returncode})")
        if glim.poll() is not None:
            raise RuntimeError(f"glim died before bag play (exit={glim.returncode})")

        log(f"playing bag: {args.bag}")
        bag_cmd = [
            "ros2", "bag", "play", str(args.bag), "--clock",
            "--rate", str(args.rate),
            "--disable-keyboard-controls",
        ]
        bag_p = spawn(bag_cmd)
        bag_p.wait()
        log("bag finished")

        log(f"draining for {args.drain}s so global_mapping can finish queued submaps")
        time.sleep(args.drain)

        log(f"signaling glim to save (SIGINT), waiting up to {args.save_timeout}s")
        stop(glim, signal.SIGINT, timeout=args.save_timeout)

    finally:
        stop(bag_p, signal.SIGINT, timeout=5)
        stop(glim, signal.SIGINT, timeout=10)
        stop(relay, signal.SIGTERM, timeout=5)

    if not (args.output_dir / "graph.bin").exists():
        log(f"WARNING: {args.output_dir} has no graph.bin -- glim may have crashed before saving")
        return 1
    log(f"done. Dump at {args.output_dir}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
