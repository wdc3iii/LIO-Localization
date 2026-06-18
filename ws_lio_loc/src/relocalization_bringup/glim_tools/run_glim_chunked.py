#!/usr/bin/env python3
"""Chunked GLIM mapping driver.

Plays a long ROS 2 bag in time-bounded chunks and runs GLIM on each chunk
separately, saving each chunk's dump to its own directory. Cross-chunk loop
closures are recovered offline via `ros2 run glim_ros offline_viewer`
(see GLIM's docs/merge.md).

Why: keeps the global_mapping factor graph bounded per run, so a long bag
fits in VRAM even at GPU settings.

Tested sweet spot for a 4080 + Mid360: ~15 min chunks with 30-60 s overlap
keeps VRAM under 50 %% peak while still allowing within-chunk loop closures.

Resume: if a chunk's dump already has graph.bin, that chunk is skipped. To
force a re-run, delete the chunk directory first.

Assumes the calling shell has already sourced:
  /opt/ros/humble/setup.bash
  ~/livox_driver_ws/install/setup.bash
  ~/glim_ws/install/setup.bash

Usage:
  ros2 run relocalization_bringup run_glim_chunked \\
      --bag /path/to/bag \\
      --output-dir /path/to/chunks \\
      --chunk-duration 900 --overlap 60
"""

import argparse
import os
import signal
import subprocess
import sys
import time
from pathlib import Path

import yaml

try:
    from ament_index_python.packages import get_package_share_directory
    _SHARE = Path(get_package_share_directory("relocalization_bringup"))
except Exception:
    _SHARE = None

CONFIG_VARIANTS = {
    "default": "glim_mid360",
    "rviz_only": "glim_mid360_rviz",
    "headless": "glim_mid360_headless",
    "indoor": "glim_mid360_indoor",
}

DEFAULT_CONFIG = (_SHARE / "config" / CONFIG_VARIANTS["default"]) if _SHARE else None
DEFAULT_RELAY = Path(__file__).resolve().parent / "livox_custom_to_pc2.py"


def log(msg):
    sys.stdout.write(f"[run_glim_chunked] {msg}\n")
    sys.stdout.flush()


def bag_duration_seconds(bag_dir: Path) -> float:
    with open(bag_dir / "metadata.yaml") as f:
        m = yaml.safe_load(f)
    return m["rosbag2_bagfile_information"]["duration"]["nanoseconds"] * 1e-9


def spawn(cmd):
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


def plan_chunks(total: float, chunk: float, overlap: float):
    """Return [(idx, start_sec, duration_sec), ...]."""
    if overlap >= chunk:
        raise ValueError(f"overlap ({overlap}) must be < chunk-duration ({chunk})")
    step = chunk - overlap
    chunks = []
    t = 0.0
    idx = 0
    while t < total:
        dur = min(chunk, total - t)
        chunks.append((idx, t, dur))
        t += step
        idx += 1
        if total - t <= overlap and chunks and chunks[-1][1] + chunks[-1][2] >= total - 0.5:
            break
    return chunks


def run_chunk(bag, start, duration, dump_dir, config_path, relay_path, no_relay,
              ramp_up, drain, save_timeout):
    dump_dir.mkdir(parents=True, exist_ok=True)
    log(f"chunk -> {dump_dir.name}: start={start:.1f}s dur={duration:.1f}s")

    relay = None
    glim = None
    bag_p = None
    try:
        if not no_relay:
            relay = spawn(["/usr/bin/env", "python3", str(relay_path)])
        glim = spawn([
            "ros2", "run", "glim_ros", "glim_rosnode", "--ros-args",
            "-p", f"config_path:={config_path}",
            "-p", f"dump_path:={dump_dir}",
        ])

        log(f"waiting {ramp_up}s for relay+glim to subscribe")
        time.sleep(ramp_up)
        if not no_relay and relay.poll() is not None:
            raise RuntimeError(f"relay died before bag play (exit={relay.returncode})")
        if glim.poll() is not None:
            raise RuntimeError(f"glim died before bag play (exit={glim.returncode})")

        log(f"starting bag at offset {start:.1f}s, will stop after {duration:.1f}s")
        bag_p = spawn([
            "ros2", "bag", "play", str(bag), "--clock",
            "--start-offset", str(start),
            "--disable-keyboard-controls",
        ])

        try:
            bag_p.wait(timeout=duration)
            log("bag exited naturally (probably end of bag)")
        except subprocess.TimeoutExpired:
            log("reached chunk duration, stopping bag")
            stop(bag_p, signal.SIGINT, timeout=15)

        log(f"draining for {drain}s so global_mapping can finish queued submaps")
        time.sleep(drain)

        log(f"signaling glim to save (SIGINT), waiting up to {save_timeout}s")
        stop(glim, signal.SIGINT, timeout=save_timeout)

    finally:
        stop(bag_p, signal.SIGINT, timeout=5)
        stop(glim, signal.SIGINT, timeout=10)
        stop(relay, signal.SIGTERM, timeout=5)

    if not (dump_dir / "graph.bin").exists():
        log(f"WARNING: {dump_dir} has no graph.bin -- glim may have crashed before saving")


def main():
    ap = argparse.ArgumentParser(
        formatter_class=argparse.RawDescriptionHelpFormatter, description=__doc__
    )
    ap.add_argument("--bag", required=True, type=Path)
    ap.add_argument("--output-dir", required=True, type=Path,
                    help="parent directory; each chunk gets a subdir chunk_NNN/")
    ap.add_argument("--chunk-duration", type=float, default=900.0,
                    help="seconds of bag-time per chunk (default 900 = 15 min)")
    ap.add_argument("--overlap", type=float, default=60.0,
                    help="seconds of overlap between consecutive chunks (default 60)")
    ap.add_argument("--config", type=Path, default=DEFAULT_CONFIG,
                    help=f"GLIM config_path (default {DEFAULT_CONFIG})")
    ap.add_argument("--relay", type=Path, default=DEFAULT_RELAY,
                    help="path to livox CustomMsg -> PointCloud2 relay script")
    ap.add_argument("--no-relay", action="store_true",
                    help="skip the relay (use if the bag already has PointCloud2 topics)")
    ap.add_argument("--ramp-up", type=float, default=4.0)
    ap.add_argument("--drain", type=float, default=30.0)
    ap.add_argument("--save-timeout", type=float, default=180.0)
    ap.add_argument("--start-from", type=int, default=0,
                    help="skip chunks with index < N (resume support)")
    ap.add_argument("--dry-run", action="store_true",
                    help="print the chunk plan and exit")
    variant = ap.add_mutually_exclusive_group()
    variant.add_argument("--headless", action="store_true",
                         help="Use the headless config variant. Ignored when --config "
                              "is passed.")
    variant.add_argument("--rviz-only", action="store_true",
                         help="Use the rviz-only config variant. Ignored when --config "
                              "is passed.")
    variant.add_argument("--indoor", action="store_true",
                         help="Use the indoor config variant (short-range preprocessing, "
                              "finer voxels, tighter loop search). Ignored when --config "
                              "is passed.")
    args = ap.parse_args()

    if args.config == DEFAULT_CONFIG and _SHARE is not None:
        if args.headless:
            args.config = _SHARE / "config" / CONFIG_VARIANTS["headless"]
        elif args.rviz_only:
            args.config = _SHARE / "config" / CONFIG_VARIANTS["rviz_only"]
        elif args.indoor:
            args.config = _SHARE / "config" / CONFIG_VARIANTS["indoor"]
    log(f"using config: {args.config}")

    if args.config is None or not (args.config / "config.json").exists():
        sys.exit(f"glim config dir missing config.json: {args.config}")
    if not args.bag.exists():
        sys.exit(f"bag not found: {args.bag}")
    if not args.no_relay and not args.relay.exists():
        sys.exit(f"relay script not found: {args.relay}")

    total = bag_duration_seconds(args.bag)
    log(f"bag duration: {total:.1f}s ({total/60:.1f} min)")

    chunks = plan_chunks(total, args.chunk_duration, args.overlap)
    log(f"chunk plan ({len(chunks)} chunks):")
    for idx, start, dur in chunks:
        log(f"  chunk_{idx:03d}: [{start:7.1f}, {start+dur:7.1f}]  dur={dur:.1f}s")

    if args.dry_run:
        return 0

    args.output_dir.mkdir(parents=True, exist_ok=True)

    interrupted = False

    def handle_sigint(signum, frame):
        nonlocal interrupted
        if interrupted:
            log("second interrupt, exiting hard")
            sys.exit(130)
        log("interrupt received; will stop after current chunk finishes saving")
        interrupted = True

    signal.signal(signal.SIGINT, handle_sigint)

    for idx, start, dur in chunks:
        if idx < args.start_from:
            log(f"skipping chunk_{idx:03d} (--start-from {args.start_from})")
            continue
        dump = args.output_dir / f"chunk_{idx:03d}"
        if (dump / "graph.bin").exists():
            log(f"chunk_{idx:03d} already has graph.bin, skipping (delete to re-run)")
            continue
        run_chunk(
            bag=args.bag,
            start=start,
            duration=dur,
            dump_dir=dump,
            config_path=args.config,
            relay_path=args.relay,
            no_relay=args.no_relay,
            ramp_up=args.ramp_up,
            drain=args.drain,
            save_timeout=args.save_timeout,
        )
        if interrupted:
            log("interrupt acknowledged; stopping before next chunk")
            break
        time.sleep(2.0)

    log("all done")
    return 0


if __name__ == "__main__":
    sys.exit(main())
