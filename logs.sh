#!/bin/zsh
# Live serial log of the Nano in the terminal. Uses only built-in macOS tools
# (stty + cat), nothing from Arduino.
#   ./logs.sh            - auto-detect the port (/dev/cu.usbmodem*)
#   ./logs.sh /dev/cu.X  - use a specific port
# Reconnects automatically after a reset / upload / unplug.
# Everything is also saved to logs/YYYY-MM-DD_HH-MM-SS.log next to this script.
# Type anything + Enter to make the board re-print its history. Ctrl-C to quit.

BAUD=115200
DIR="${0:A:h}"
mkdir -p "$DIR/logs"
LOG="$DIR/logs/$(date +%Y-%m-%d_%H-%M-%S).log"

cleanup() {
  trap - INT TERM
  echo; echo "log saved: $LOG"
  kill 0 2>/dev/null   # whole process group: reader, writer, this script
  exit 0
}
trap cleanup INT TERM

find_port() {
  if [[ -n "$1" ]]; then echo "$1"; return; fi
  ls /dev/cu.usbmodem* 2>/dev/null | head -1
}

echo "saving to $LOG"
state=""
while true; do
  PORT=$(find_port "$1")
  if [[ -z "$PORT" || ! -e "$PORT" ]]; then
    [[ "$state" != nodev ]] && echo "--- waiting for the board (plug it in)..."
    state=nodev; sleep 1; continue
  fi
  HOLDER=$(lsof -t "$PORT" 2>/dev/null | head -1)
  if [[ -n "$HOLDER" ]]; then
    [[ "$state" != busy ]] && echo "--- $PORT is busy: $(ps -o comm= -p $HOLDER) (pid $HOLDER) has it open"
    state=busy; sleep 1; continue
  fi
  state=""

  if ! exec 3<>"$PORT" 2>/dev/null; then sleep 1; continue; fi
  stty "$BAUD" raw -echo <&3 2>/dev/null
  echo "--- connected to $PORT" | tee -a "$LOG"

  { cat >&3 } <&0 2>/dev/null &      # keyboard -> board
  WRITER=$!
  cat <&3 2>/dev/null | tee -a "$LOG" &  # board -> screen + file (ends on reset/unplug)
  wait $!                                # in background so Ctrl-C / kill act at once
  kill $WRITER 2>/dev/null
  exec 3>&-

  echo "--- disconnected, reconnecting..." | tee -a "$LOG"
  sleep 1
done
