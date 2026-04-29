#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import os
import shutil
import signal
import subprocess
import sys
import time
from dataclasses import dataclass
from datetime import datetime, timezone
from pathlib import Path


DEFAULT_TOPICS = [
    "/tf",
    "/tf_static",
    "/localization/vision_base_pose",
    "/localization/debug_image",
    "/odomWheel",
    "/odometry/filtered",
    "/bno055/imu",
]


@dataclass(frozen=True)
class CaptureConfig:
    duration_s: float
    output_dir: Path
    topics: list[str]
    include_hidden_topics: bool


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Record a short ROS 2 bag with TF and the most useful localization topics."
        )
    )
    parser.add_argument(
        "--duration",
        type=float,
        default=15.0,
        help="Recording duration in seconds. Default: 15",
    )
    parser.add_argument(
        "--output-dir",
        default="captures",
        help="Parent directory where the capture folder will be created. Default: captures",
    )
    parser.add_argument(
        "--name",
        default="localization_capture",
        help="Prefix used for the timestamped capture folder name.",
    )
    parser.add_argument(
        "--topic",
        action="append",
        dest="extra_topics",
        default=[],
        help="Additional topic to record. Can be passed multiple times.",
    )
    parser.add_argument(
        "--no-debug-image",
        action="store_true",
        help="Skip /localization/debug_image to keep bags smaller.",
    )
    parser.add_argument(
        "--include-hidden-topics",
        action="store_true",
        help="Pass --include-hidden-topics to ros2 bag record.",
    )
    return parser.parse_args(argv)


def build_topics(extra_topics: list[str], include_debug_image: bool) -> list[str]:
    topics = [topic for topic in DEFAULT_TOPICS if include_debug_image or topic != "/localization/debug_image"]
    for topic in extra_topics:
        if topic not in topics:
            topics.append(topic)
    return topics


def build_capture_config(args: argparse.Namespace) -> CaptureConfig:
    timestamp = datetime.now(timezone.utc).strftime("%Y%m%d_%H%M%S")
    output_dir = Path(args.output_dir).expanduser().resolve() / f"{args.name}_{timestamp}"
    topics = build_topics(
        extra_topics=args.extra_topics,
        include_debug_image=not args.no_debug_image,
    )
    return CaptureConfig(
        duration_s=float(args.duration),
        output_dir=output_dir,
        topics=topics,
        include_hidden_topics=bool(args.include_hidden_topics),
    )


def ensure_ros2_cli_available() -> None:
    if shutil.which("ros2") is None:
        raise RuntimeError("Could not find 'ros2' in PATH.")


def snapshot_command(command: list[str], output_file: Path) -> None:
    try:
        result = subprocess.run(
            command,
            check=False,
            capture_output=True,
            text=True,
        )
        output = result.stdout if result.stdout else result.stderr
    except Exception as exc:  # pragma: no cover - defensive
        output = f"Failed to run {' '.join(command)}: {exc}\n"

    output_file.write_text(output, encoding="utf-8")


def write_metadata(config: CaptureConfig, bag_dir: Path, started_at: float, ended_at: float) -> None:
    metadata = {
        "started_at_iso_utc": datetime.fromtimestamp(started_at, tz=timezone.utc).isoformat(),
        "ended_at_iso_utc": datetime.fromtimestamp(ended_at, tz=timezone.utc).isoformat(),
        "duration_s_requested": config.duration_s,
        "duration_s_actual": ended_at - started_at,
        "bag_directory": str(bag_dir),
        "topics": config.topics,
        "cwd": os.getcwd(),
    }
    (config.output_dir / "capture_metadata.json").write_text(
        json.dumps(metadata, indent=2),
        encoding="utf-8",
    )


def run_capture(config: CaptureConfig) -> int:
    ensure_ros2_cli_available()

    if config.duration_s <= 0.0:
        raise ValueError("Duration must be greater than zero.")

    config.output_dir.mkdir(parents=True, exist_ok=False)
    bag_dir = config.output_dir / "bag"

    snapshot_command(["ros2", "topic", "list"], config.output_dir / "topic_list.txt")
    snapshot_command(["ros2", "node", "list"], config.output_dir / "node_list.txt")

    command = ["ros2", "bag", "record", "-o", str(bag_dir)]
    if config.include_hidden_topics:
        command.append("--include-hidden-topics")
    command.extend(config.topics)

    print(f"Recording localization capture for {config.duration_s:.1f}s")
    print(f"Output folder: {config.output_dir}")
    print("Topics:")
    for topic in config.topics:
        print(f"  {topic}")

    started_at = time.time()
    process = subprocess.Popen(command)

    try:
        time.sleep(config.duration_s)
    except KeyboardInterrupt:
        print("Stopping early after keyboard interrupt...")
    finally:
        if process.poll() is None:
            process.send_signal(signal.SIGINT)
            try:
                process.wait(timeout=15.0)
            except subprocess.TimeoutExpired:
                process.terminate()
                process.wait(timeout=5.0)

    ended_at = time.time()
    write_metadata(config, bag_dir, started_at, ended_at)
    snapshot_command(
        ["ros2", "bag", "info", str(bag_dir)],
        config.output_dir / "bag_info.txt",
    )

    print("Capture complete.")
    print(f"Bag: {bag_dir}")
    print(f"Metadata: {config.output_dir / 'capture_metadata.json'}")
    return int(process.returncode or 0)


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv)
    config = build_capture_config(args)
    try:
        return run_capture(config)
    except Exception as exc:
        print(f"localization_capture failed: {exc}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
