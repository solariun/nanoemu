#!/bin/sh
# Optional helper: compile the example sketches for the Arduino Nano with
# arduino-cli (Arduino manages its own toolchain) and drop .hex/.elf/.bin next
# to each sketch. nanoemu itself does not need any of this.
# Requires: arduino-cli + `arduino-cli core install arduino:avr`.
set -e
cd "$(dirname "$0")"
command -v arduino-cli >/dev/null || { echo "arduino-cli not found (optional; see README)"; exit 1; }
# Use a natively built toolchain (tools/build-avr-toolchain.sh) when present,
# otherwise whatever compiler Arduino installed. Override with AVR_GCC_BIN=dir.
AVR_GCC_BIN=${AVR_GCC_BIN:-$HOME/.local/avr/bin}
EXTRA=""
if [ -x "$AVR_GCC_BIN/avr-gcc" ]; then
  EXTRA="--build-property compiler.path=$AVR_GCC_BIN/"
  echo "using $AVR_GCC_BIN/avr-gcc ($("$AVR_GCC_BIN/avr-gcc" -dumpversion))"
fi
for s in */; do
  s=${s%/}
  [ -f "$s/$s.ino" ] || continue
  out=$(mktemp -d)
  # shellcheck disable=SC2086
  arduino-cli compile --fqbn arduino:avr:nano $EXTRA --output-dir "$out" "$s" >/dev/null
  cp "$out/$s.ino.hex" "$s/$s.hex"
  cp "$out/$s.ino.elf" "$s/$s.elf"
  [ -f "$out/$s.ino.bin" ] && cp "$out/$s.ino.bin" "$s/$s.bin"
  rm -rf "$out"
  echo "built examples/$s"
done
