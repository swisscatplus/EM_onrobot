#!/bin/bash
set -e

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
VENV_DIR="${CALIB_ENV_DIR:-$REPO_ROOT/calib_env}"

echo "Setting up calibration environment at $VENV_DIR"

sudo apt update
sudo apt install -y python3-picamera2 python3-opencv python3-yaml python3-numpy

rm -rf "$VENV_DIR"
python3 -m venv --system-site-packages "$VENV_DIR"

echo
echo "Calibration environment ready."
echo "Activate it with:"
echo "  source \"$VENV_DIR/bin/activate\""
