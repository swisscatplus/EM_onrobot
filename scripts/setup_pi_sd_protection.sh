#!/bin/bash
# One-time host setup that reduces writes to the Raspberry Pi SD card.
# Run on the robot with: sudo ./scripts/setup_pi_sd_protection.sh
# Safe to run again. Every file it changes is backed up first as <file>.bak.<date>.
# Reboot afterwards so all changes apply.
set -e

if [ "$(id -u)" -ne 0 ]; then
  echo "Run this script with sudo." >&2
  exit 1
fi

STAMP="$(date +%Y%m%d-%H%M%S)"

backup() {
  if [ -f "$1" ]; then
    cp -a "$1" "$1.bak.$STAMP"
    echo "  backup: $1.bak.$STAMP"
  fi
}

echo "1. Mount the root filesystem with noatime (no write on every file read)"
if awk '$2 == "/" && $4 !~ /noatime/ {found=1} END {exit !found}' /etc/fstab; then
  backup /etc/fstab
  awk 'BEGIN {OFS="\t"} $2 == "/" && $4 !~ /noatime/ {$4 = $4 ",noatime"} {print}' \
    /etc/fstab > /etc/fstab.new
  mv /etc/fstab.new /etc/fstab
  echo "  added noatime to / in /etc/fstab"
else
  echo "  already set (or no / entry in /etc/fstab), skipping"
fi

echo "2. Keep the systemd journal in RAM (logs are lost on reboot)"
mkdir -p /etc/systemd/journald.conf.d
cat > /etc/systemd/journald.conf.d/90-sd-protection.conf <<'EOF'
[Journal]
Storage=volatile
RuntimeMaxUse=30M
EOF
systemctl restart systemd-journald
echo "  wrote /etc/systemd/journald.conf.d/90-sd-protection.conf"

echo "3. Disable swap on the SD card"
if command -v dphys-swapfile >/dev/null 2>&1; then
  dphys-swapfile swapoff || true
  systemctl disable --now dphys-swapfile
  echo "  disabled dphys-swapfile (Raspberry Pi OS)"
fi
swapoff -a || true
if grep -Eq '^[^#].*[[:space:]]swap[[:space:]]' /etc/fstab; then
  backup /etc/fstab
  sed -i -E 's|^([^#].*[[:space:]]swap[[:space:]].*)$|# \1  # disabled by setup_pi_sd_protection.sh|' /etc/fstab
  echo "  commented out swap entries in /etc/fstab"
fi

echo "4. Cap Docker container logs for every container"
DAEMON_JSON=/etc/docker/daemon.json
mkdir -p /etc/docker
backup "$DAEMON_JSON"
python3 - "$DAEMON_JSON" <<'EOF'
import json
import sys

path = sys.argv[1]
try:
    with open(path) as handle:
        config = json.load(handle)
except FileNotFoundError:
    config = {}

config["log-driver"] = "local"
config["log-opts"] = {"max-size": "10m", "max-file": "2"}

with open(path, "w") as handle:
    json.dump(config, handle, indent=2)
    handle.write("\n")
EOF
echo "  updated $DAEMON_JSON (other settings kept)"
echo "  Docker will be restarted: running containers restart with it."
systemctl restart docker

echo
echo "Done. Reboot the robot so noatime and the swap change fully apply: sudo reboot"
