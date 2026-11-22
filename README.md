# nanoemu

An **Arduino Nano (ATmega328P @ 16 MHz) emulator** with a full terminal UI. It's a single C++11 file with no dependencies beyond ncurses.

```
┌─ CPU ──────────┐┌─ ARDUINO NANO  ATmega328P @ 16 MHz ─────────────────┐┌─ PINS ──────────────┐
│ PC / SP / SREG ││        50%            25%          RX   TX           ││ D0..D13, A0..A7     │
│ cycles, time   ││  D12 D11 D10 D9 D8 D7 D6 D5 D4 D3 D2 GND RST RX0 TX1 ││ mode, level, drive, │
│ real-time rate ││   ○   ○   ○  ●  ○  ○  ●  ○  ○  ○  ○  ■   ■   ●   ●   ││ analog value        │
├─ REGISTERS ────┤│ USB   L PWR      [ATMEGA328P]        (o)  ICSP      │├─ PERIPHERALS ───────┤
│ r0..r31, X Y Z ││   ●   ■   ■  ○  ○  ○  ○  ○  ○  ○  ○  ■   ■   ■   ■   ││ T0/T1/T2, UART, ADC │
├─ CODE ─────────┤│  D13 3V3 REF A0 A1 A2 A3 A4 A5 A6 A7 5V  RST GND VIN ││ EEPROM, WDT, IRQs   │
│ live disasm    │├─ SERIAL  115200 baud  pty /dev/ttys004 ──────────────┤│                     │
│                ││ serial console + input line                          ││                     │
└────────────────┘└──────────────────────────────────────────────────────┘└─────────────────────┘
┌─ LOGIC ANALYZER  (all 20 digital pins, freq + duty per pin, time cursor, trigger) ──────────────┐
│ D6  PWM  976.6Hz 25%  ┌┐   ┌┐   ┌┐   ┌┐   ┌┐   ┌┐   ┌┐   ┌┐   ┌┐   ┌┐   ┌┐   ┌┐   ┌┐   ┌┐       │
│                     ──┘└───┘└───┘└───┘└───┘└───┘└───┘└───┘└───┘└───┘└───┘└───┘└───┘└───       │
└────────────────────────────────────────────────────────────────────────────────────────────────┘
```

## Build

```bash
make            # macOS: links -lncurses, Linux: -lncursesw -lutil
```

Or build it directly:

```bash
c++ -std=c++11 -O2 -o nanoemu nanoemu.cpp -lncurses
```

## Run

```bash
./nanoemu examples/asmtest/asmtest.hex
./nanoemu sketch.ino.bin          # raw flash image
./nanoemu sketch.ino.elf          # ELF from avr-gcc / arduino-cli
./nanoemu -b loop sketch.ino.elf  # source view + stop when loop() is reached
./nanoemu -b main.ino:42 -b 0x1a4 firm.elf   # breakpoints by file:line or byte address
./nanoemu --serial firm.hex       # no TUI: real time, serial ↔ stdin/stdout, Ctrl-C stops
./nanoemu -c --speed 0 -H 5 firm.hex  # the same, as fast as possible, stop after 5 simulated s
./nanoemu -H 2 -i "ping" firm.hex # headless: 2 s simulated, serial → stdout (good for CI)
```

Use a terminal of at least 110×34. 180×50 or larger looks best.

### Keys

| Key | Action |
|---|---|
| `SPACE` | run / pause |
| `s` | single-step one instruction (pauses) |
| `r` | restart (hard reset, keeps flash and EEPROM) |
| `l` / `F5` | reload the firmware file from disk and reset |
| `f` | open another firmware file |
| `b` | toggle a breakpoint: `function`, `file:line`, `line` (in the current file) or `0xADDR`. Empty input clears all of them |
| `v` | show/hide the source panel |
| `a` | SOURCE panel: also show Arduino's internal code (core, libraries, avr-libc). Hidden by default |
| `+` / `-` | simulation speed: x0.001 … x10, or max |
| `TAB` | move keyboard focus between the PINS panel and the LOGIC ANALYZER (the focused panel's border lights up) |
| `↑` `↓` | select a pin, in either panel. The selection is shared |
| `ENTER` | (PINS) cycle the selected pin's external drive: float → HIGH → LOW |
| `←` `→` | (PINS) analog pins: change the ADC voltage. Digital pins: drive LOW/HIGH |
| `←` `→` | (LOGIC ANALYZER) move the time cursor; `ESC` hides it |
| `[` / `]` | logic analyzer timebase (1 µs … 100 ms per column) |
| `t` | trigger the logic analyzer on the selected pin's rising edge |
| `i` | type into the serial port (ENTER sends the line + `\n`, ESC leaves) |
| `c` | clear the serial console |
| `q` | quit |

### Virtual serial port

The UART also shows up as a pseudo-terminal. Its path, such as `/dev/ttys004`, is in the SERIAL panel title. You can connect any tool to it:

```bash
screen /dev/ttys004 115200
```

## Seeing the code that runs

Load the `.elf` that arduino-cli or the Arduino IDE produces (it sits next to the `.hex`), or load a `.hex` that has its `.elf` beside it. nanoemu then reads the symbols and the DWARF line tables, and shows:

- a **SOURCE** panel with the line of *your* code being executed, highlighted. While the CPU is inside Arduino's own code (the core, libraries, avr-libc), it stays on the last line of yours that ran, usually the call that led there, and the title says where execution actually is. Press `a` to follow Arduino's code too.
- under it, the **current values of the global and static variables** used on that line, read live from RAM. Locals and class members aren't shown.
- the **function** being executed in the CODE title (for example `MyThread::run()+0x22`), with function names on `call` targets and labels in the disassembly.

If a source file has moved, pass `-S DIR` to tell nanoemu where to find it.

## What is emulated

- **CPU**: the full AVRe+ instruction set of the ATmega328P, with cycle counts, interrupts and their latency, the one-instruction delay after `SEI`/`RETI`, `SLEEP`, `BREAK` (it pauses the emulator) and `WDR`.
- **Memory**: 32 KB flash, 2 KB SRAM, 1 KB EEPROM (with timed writes). EEPROM survives restart and reload.
- **GPIO**: ports B, C and D, with pull-ups, `PINx` write-to-toggle, external drive from the UI, and the `PUD` bit.
- **Timers**:
  - Timer0 and Timer2 (8-bit) and Timer1 (16-bit, with the TEMP register).
  - All modes: normal, CTC, fast PWM and phase-correct PWM.
  - Output-compare pins (OC0A/B, OC1A/B, OC2A/B), so `analogWrite()` works on D3, D5, D6, D9, D10 and D11.
- **USART0**: double-buffered TX with bit-accurate timing. The TX waveform appears on D1, so you can watch it on the scope. Also RX, the UDRE/RX/TX interrupts and U2X.
- **ADC**: 10-bit, AVCC or 1.1 V reference, ADLAR, free-running mode, plus the bandgap and temperature channels. You set the input voltages from the UI.
- **Interrupts**: INT0/INT1 (level and edge modes), PCINT0..2, watchdog (interrupt or reset), and EEPROM ready.
- **SPI / TWI**: minimal models. SPI transfers complete instantly and read `0xFF`. TWI is a bus with no devices, so every address gets a NACK and `Wire` fails cleanly.

## Getting firmware

nanoemu has **no compiler or toolchain dependency**. It only runs firmware that is already compiled. You compile with Arduino's own tools, which install and manage their own compiler:

- **Arduino IDE:** choose the board *Arduino Nano*, then *Sketch → Export Compiled Binary*. The `.hex` file appears in the sketch's `build/` folder. Load it with `./nanoemu path/to/sketch.ino.hex`.
- **arduino-cli:** `arduino-cli compile --fqbn arduino:avr:nano --output-dir out sketch/`, then `./nanoemu out/sketch.ino.hex`.

Use the `.hex` or `.elf` *without* the bootloader. The `*.with_bootloader.hex` file also works, but it boots straight into the sketch.

## Examples

| Folder | Contents |
|---|---|
| `examples/asmtest` | A self-test in plain AVR assembly. A prebuilt `asmtest.hex` is included, so it runs right away. |
| `examples/demo` | An Arduino sketch: blink, PWM fade on every PWM pin, ADC, an INT0 button counter, an EEPROM boot counter and serial echo. |
| `examples/blink` | The classic blink plus serial output. |

Compile the two `.ino` sketches as described in [Getting firmware](#getting-firmware). `make examples` is a shortcut that does it with `arduino-cli`.

### Native compiler on Apple Silicon (no Rosetta)

On macOS, the avr-gcc that Arduino ships is an x86_64 binary. If you don't want Rosetta, use [arduino-avr-arm64](https://github.com/solariun/arduino-avr-arm64) to build and install a native toolchain for arduino-cli and Arduino IDE 2. Or use the bundled script, which builds from the official GNU and avr-libc sources:

```bash
tools/build-avr-toolchain.sh        # binutils 2.43 + gcc 14.2 + avr-libc 2.2.1 → ~/.local/avr
```

The build takes about 20 minutes. When `~/.local/avr/bin/avr-gcc` exists, `make examples` uses it automatically. arduino-cli still provides the Arduino core and libraries. To use the native compiler for your own sketches:

```bash
arduino-cli compile --fqbn arduino:avr:nano \
  --build-property compiler.path=$HOME/.local/avr/bin/ --output-dir out mysketch/
```

## Layout of the code

`nanoemu.cpp` is split into sections:

1. **Constants**: I/O register map and pin mapping.
2. **`class Nano`**: the MCU. It contains:
   - memory and the firmware loader (bin/hex/elf)
   - GPIO and pin logging
   - timers, USART, ADC, EEPROM and TWI
   - I/O dispatch
   - the instruction decoder/executor
   - interrupts
   - the clock.
3. **Disassembler**: used by the CODE panel.
4. **`class App`**: the ncurses UI (board, panels, scope, serial and PTY), the input handling, the real-time pacing loop and headless mode.

## License

MIT
