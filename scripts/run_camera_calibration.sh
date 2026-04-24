#!/bin/bash
set -e

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
VENV_DIR="${CALIB_ENV_DIR:-$REPO_ROOT/calib_env}"

if [ ! -f "$VENV_DIR/bin/activate" ]; then
  echo "Calibration environment not found at $VENV_DIR"
  echo "Run ./scripts/setup_calibration_env.sh first."
  exit 1
fi

source "$VENV_DIR/bin/activate"
cd "$REPO_ROOT"
python src/em_robot/CameraCalibration/calibrate_image.py
