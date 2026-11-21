#!/bin/sh
# Compile the example sketches for the Arduino Nano and drop .hex/.bin/.elf
# next to each sketch. Requires: arduino-cli + `arduino-cli core install arduino:avr`.
set -e
cd "$(dirname "$0")"
OBJCOPY=$(ls -d "$HOME"/Library/Arduino15/packages/arduino/tools/avr-gcc/*/bin/avr-objcopy \
               "$HOME"/.arduino15/packages/arduino/tools/avr-gcc/*/bin/avr-objcopy 2>/dev/null | head -1)
for s in */; do
  s=${s%/}
  [ -f "$s/$s.ino" ] || continue
  out=$(mktemp -d)
  arduino-cli compile --fqbn arduino:avr:nano --output-dir "$out" "$s" >/dev/null
  cp "$out/$s.ino.hex" "$s/$s.hex"
  cp "$out/$s.ino.elf" "$s/$s.elf"
  if [ -n "$OBJCOPY" ]; then "$OBJCOPY" -I ihex -O binary "$s/$s.hex" "$s/$s.bin"; fi
  rm -rf "$out"
  echo "built examples/$s"
done
