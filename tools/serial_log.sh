#!/usr/bin/env bash
set -euo pipefail

device="${1:-/dev/ttyACM0}"
baud="${2:-115200}"
log_dir="${3:-/home/george-sleen/Documents/design-team/sailbot/com-module-firmware/logs}"

mkdir -p "$log_dir"
ts="$(date +%Y%m%d_%H%M%S)"
log_file="$log_dir/uart_log_${ts}.txt"

echo "Logging from ${device} at ${baud} baud to ${log_file}"
echo "Press Ctrl+C to stop."

stty -F "$device" "$baud" raw -echo -icanon min 1 time 0
stdbuf -oL cat "$device" | tee -a "$log_file"
