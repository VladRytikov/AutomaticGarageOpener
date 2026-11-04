# Garage opener

A garage opener that fires **by itself when the car starts moving**.

The Arduino is powered from the car, so it boots when the engine starts. It then
watches its built-in accelerometer and, once the car actually begins to move, it
replays a captured 433.92 MHz remote signal for about a minute and shuts itself
down until the next time the car is started. No button press, no phone, no app.

The transmitted signal is a raw replay of a Flipper Zeros pre-recorded `.sub` capture of the
user's own gate remote (`garage_signal.h`), keyed out through a CC1101 module in
asynchronous OOK mode.

---

## Hardware

| Part | Notes |
|------|-------|
| Arduino Nano 33 IoT | SAMD21, 3.3 V logic, **built-in LSM6DS3 accelerometer** |
| CC1101 433 MHz module | 3.3 V — connects to the Nano directly, no level shifter |
| GY-GPS6MV2 Module | 5 V — connects to the Nano directly, no level shifter |

The accelerometer is on the board, so no external sensor is needed.

### Wiring (CC1101 → Nano 33 IoT)

| CC1101 | Nano pin | Purpose |
|--------|----------|---------|
| SCK    | D13 | SPI clock |
| MISO   | D12 | SPI |
| MOSI   | D11 | SPI |
| CSN    | D10 | SPI chip select |
| GDO0   | D6  | **driven by the Nano** to key the carrier on/off |
| VCC    | 3V3 | **3.3 V, not 5 V** |
| GND    | GND | |

---

## How it works, step by step

The sketch is a small state machine. Times and thresholds below are the
defaults at the top of `car_motion.ino`.

### 1. Boot
On power-up the Arduino:
- starts the serial log at 115200 baud **without** waiting for a host, so it
  runs identically whether or not a laptop is attached;
- logs the **reset cause** (power-on, brown-out, upload, …) — useful for
  spotting a car supply that dips while cranking;
- initialises the CC1101 and confirms it answers (`RADIO CC1101 OK`);
- initialises the LSM6DS3 accelerometer (104 Hz).

### 2. Settle — 3 s (`SETTLE_MS`)
Everything is ignored for the first 3 seconds. This throws away the engine
shake, doors closing, and people getting in and settling — none of which should
count as "the car is moving."

### 3. Calibrate — 2 s (`CALIB_MS`)
The firmware averages the low-pass-filtered acceleration into a **rest vector**:
the direction and magnitude of gravity in whatever orientation the board happens
to be mounted. Because "rest" is learned, **the board can be mounted at any
angle** — it does not need to be flat or aligned with the car.

It logs the rest vector and the total magnitude `|g|`. If `|g|` is far from
1000 mg it warns that the board was probably moving during calibration.

### 4. Armed — waiting for motion
Each accelerometer sample is low-pass filtered (`LP_ALPHA`, ≈0.2 s time
constant) to kill engine and road vibration. The firmware then measures the
**deviation** of the filtered acceleration from the rest vector.

- An accelerometer cannot sense constant speed — only acceleration. But a car
  that pulls away **always** accelerates, which shows up as a horizontal
  deviation of roughly 0.1–0.3 g plus some body pitch.
- Motion is confirmed only when the deviation stays above **`MOVE_THRESH_G`
  (100 mg)** continuously for **`MOVE_HOLD_MS` (600 ms)**. A short jolt — a door
  slam, someone shifting in a seat — is too brief and is rejected.
- Brief dips below the threshold shorter than `DROPOUT_MS` (150 ms) are
  tolerated, so a single noisy sample does not reset the timer.
- While the car is clearly at rest, the rest vector is allowed to track very
  slow drift (temperature, a passenger settling in) via `DRIFT_ALPHA`, but never
  while a motion candidate is building.

Once per second it logs a status line with the current deviation, the peak over
the last second, and the vibration level, so the threshold can be tuned from a
real drive.

### 5. Sending — ~1 minute (`SEND_WINDOW_MS`)
When motion is confirmed, the firmware replays the captured signal repeatedly
for 60 seconds:
- each replay of `garage_signal.h` takes about 7 s;
- replays are separated by a `SEND_GAP_MS` (2 s) pause;
- that yields roughly **7 transmissions** over the window.

Each transmission drives GDO0 HIGH/LOW for the exact microsecond timings from the
capture, with the CC1101 in raw async OOK mode (no sync word, CRC, whitening or
Manchester) — a faithful reproduction of what the original remote sends.

### 6. Done — idle
After the window the firmware logs `STOP` and does nothing until the next
power-up (i.e. the next time the car is started). It never transmits again on
its own within one power cycle.

---

## Configuration

All tunables are constants at the top of `car_motion.ino`:

| Constant | Default | Meaning |
|----------|---------|---------|
| `SETTLE_MS` | 3000 | quiet period after power-up |
| `CALIB_MS` | 2000 | rest-vector averaging window |
| `LP_ALPHA` | 0.05 | low-pass strength (vibration rejection) |
| `MOVE_THRESH_G` | 0.10 | deviation (g) that counts as moving |
| `MOVE_HOLD_MS` | 600 | how long motion must persist |
| `DROPOUT_MS` | 150 | tolerated dip below threshold |
| `DRIFT_ALPHA` | 0.0005 | slow rest-vector tracking at rest |
| `SEND_WINDOW_MS` | 60000 | how long to transmit once moving |
| `SEND_GAP_MS` | 2000 | pause between replays |
| `STATUS_MS` | 1000 | status-line period while armed |

> **Toggle gates:** many receivers treat each command as open / stop / close.
> Transmitting back-to-back can open the gate and then immediately stop or close
> it. If the gate misbehaves, increase `SEND_GAP_MS` or shorten
> `SEND_WINDOW_MS`. If one replay is not enough to open it, decrease the gap.

---

## Logging

Logs are plain text at **115200 baud**. Events are also kept in a small RAM ring
buffer and **re-printed in full whenever a serial monitor connects** (or you
send any key), so you can plug in a laptop *after* a drive and still see the
whole boot — opening the port does not reset a SAMD board.

A helper script reads the log with nothing but built-in macOS tools (no Arduino
IDE):

```bash
./logs.sh            # auto-detect the Nano's port
./logs.sh /dev/cu.X  # or name a port explicitly
```

It reconnects automatically after a reset, upload, or unplug, and saves a copy
to `logs/<date_time>.log`. Press Ctrl-C to quit.

Example session:

```
[    0.001] BOOT   car_motion, reset cause: power-on
[    0.001] CFG    thresh 100 mg, hold 600 ms, settle 3000 ms, calib 2000 ms
[    0.001] CFG    send window 60 s, gap 2000 ms
[    0.003] RADIO  CC1101 OK, 433.92 MHz
[    0.005] IMU    LSM6DS3 OK, 104 Hz
[    5.019] CALIB  rest (5, 22, 1014) mg, |g| 1014 mg, vibration 7 mg
[    5.029] ARMED  waiting for the car to move (thresh 100 mg)
[    9.090] .      ARMED  dev   2  max   3 / 100 mg   vib   3 mg
[  48.210] MOTION candidate #1: dev 128 mg
[  48.811] MOVING candidate #1 held 601 ms, peak 232 mg
[  48.812] SEND   moving -> transmitting for 60 s
[  48.820] TX     #1 at +0.008 s, took 7027 ms
...
[ 108.9  ] STOP   window over after 7 TX - idle until next power-up
```

---

## Powering it in the car (no laptop)

The firmware runs fully standalone — USB data is not required. Only the power
wiring matters:

- A car's 12 V (≈13.8 V running, with large transient spikes) **must not** go
  directly to the `3V3`, `5V`, or `VIN` pins.
- The simplest safe option is a **12 V → 5 V car USB charger**, feeding the
  Nano's USB connector or its `5V` pin. The charger absorbs the spikes.
- Wire the board to a supply that is **live only when the engine is running**
  (e.g. switched/ignition power), so each start is one clean power cycle.

If the log ever shows `brown-out 3.3V (supply dipped!)` as the reset cause, the
supply is sagging (often during cranking) and needs to be more stable.

---

## Build & flash (command line)

Uses the `arduino-cli` bundled with the Arduino IDE; adjust the path if yours
differs.

```bash
CLI="/Applications/Arduino IDE.app/Contents/Resources/app/lib/backend/resources/arduino-cli"
"$CLI" compile --fqbn arduino:samd:nano_33_iot .
"$CLI" upload  -p /dev/cu.usbmodem1201 --fqbn arduino:samd:nano_33_iot .
```

Dependencies:

```bash
"$CLI" lib install Arduino_LSM6DS3
# SmartRC-CC1101-Driver-Lib is already in this repo's libraries/ folder
```

Notes:
- Stop `logs.sh` (Ctrl-C) before flashing — it holds the serial port.
- If the SAMD bootloader is missed (`No device found`), double-tap **RESET** to
  force the board into the bootloader, then upload again.

---

## Files

- `car_motion.ino` — the firmware.
- `garage_signal.h` — the captured remote signal (rewritten in C/С++) 
- `transmitter.ino` — launching the transmission of 'garage_signal.h'
  `.sub` RAW capture; `+value` = carrier on, `-value` = carrier off, in µs).
- `logs.sh` — terminal serial-log viewer.
- `logs/` — saved log files.

---

## Safety / legal

This transmits on 433.92 MHz by replaying the user's **own** gate remote. Only
use it with a gate you are authorised to operate, and check your local rules on
transmitting in this band.
