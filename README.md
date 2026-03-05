# Santiago – Engine Ignition HIL System

A single-channel engine ignition controller for Arduino Mega 2560 with full
**Hardware-in-the-Loop (HIL)** instrumentation for laboratory validation.

---

## Table of Contents

1. [Overview](#overview)
2. [Repository Structure](#repository-structure)
3. [HIL Wiring](#hil-wiring)
4. [Configuration](#configuration)
5. [Building & Uploading](#building--uploading)
6. [HIL Test Scenarios](#hil-test-scenarios)
7. [Interpreting Oscilloscope Captures](#interpreting-oscilloscope-captures)
8. [Serial Monitor Output](#serial-monitor-output)
9. [Disabling HIL Instrumentation](#disabling-hil-instrumentation)

---

## Overview

The system reads a **36-1 missing-tooth crankshaft trigger wheel** via the
Timer1 Input Capture unit (ICP1, pin 49 on Mega) or an external interrupt
(INT0, pin 2).  It detects the gap tooth to establish engine synchronisation,
calculates RPM from consecutive tooth intervals, looks up the ignition advance
angle from a table, and drives an ignition coil output with a fixed dwell time.

Additional safety features:
- **Rev-limiter** – cuts ignition above `RPM_REVLIMIT` (default 7 500 RPM).
- **Safe mode** – cuts ignition and asserts a sync-loss flag after
  `SYNC_TIMEOUT_TEETH` consecutive teeth with no gap detected.

All timing-critical operations happen inside the ISR.  Three **debug pins**
are toggled at ISR entry/exit and coil events so that an oscilloscope can
measure WCET, jitter, dwell time, and spark timing in real-time.

---

## Repository Structure

```
engine_ignition_hil/
├── engine_ignition_hil.ino   # Main Arduino sketch
├── config.h                  # All tunable parameters and pin assignments
└── hil_debug.h               # HIL debug-pin macros and Serial helpers
README.md
```

---

## HIL Wiring

```
┌─────────────────────────────────────────┐
│         HIL PULSE GENERATOR             │
│  (signal generator, second Arduino,    │
│   or FPGA board)                        │
│                                         │
│  • Frequency  →  simulated RPM          │
│    f = RPM × TEETH_TOTAL / 60           │
│    e.g. 1 500 RPM × 36 / 60 = 900 Hz   │
│  • Duty cycle →  tooth width            │
│  • Include a 1.8× longer gap pulse for  │
│    each revolution to simulate the      │
│    missing tooth                        │
└──────────────┬──────────────────────────┘
               │  Square wave (0–5 V)
               │
┌──────────────▼──────────────────────────┐
│          ARDUINO MEGA 2560              │
│                                         │
│  Pin 49 (ICP1)  ← Crank sensor input   │
│  Pin  2 (INT0)  ← Alt. crank input     │
│                                         │
│  Pin  3         → Coil drive (IGBT)     │
│  Pin  4         → OSC CH-A  (ISR time) │
│  Pin  5         → OSC CH-B  (coil cmd) │
│  Pin  6         → OSC CH-C  (sync OK)  │
│                                         │
│  TX0 (pin 1)    → PC Serial monitor    │
└─────────────────────────────────────────┘
```

**Oscilloscope connections:**

| Channel | Pin | Signal | What to measure |
|---------|-----|--------|-----------------|
| CH-A    | 4   | `PIN_DEBUG_ISR`  | Pulse width = ISR execution time (WCET); period jitter |
| CH-B    | 5   | `PIN_DEBUG_COIL` | HIGH during dwell, falling edge = spark event |
| CH-C    | 6   | `PIN_DEBUG_SYNC` | HIGH = synchronised, LOW = sync lost / safe mode |

---

## Configuration

All parameters are in `engine_ignition_hil/config.h`:

| Parameter | Default | Description |
|-----------|---------|-------------|
| `TEETH_TOTAL` | 36 | Total teeth including gap |
| `TEETH_MISSING` | 1 | Missing teeth in the gap |
| `GAP_DETECT_RATIO` | 1.8 | Gap detection threshold (× normal interval) |
| `GAP_OFFSET_DEG` | 66 | Gap centre offset before TDC (degrees) |
| `RPM_REVLIMIT` | 7500 | Rev-limiter cut-off (RPM) |
| `RPM_MIN` | 100 | Minimum RPM to enable coil |
| `DWELL_MS` | 4 | Coil dwell time (ms) |
| `SYNC_TIMEOUT_TEETH` | 4 | Revolutions without gap before safe mode |
| `USE_ICP` | 1 | 1 = ICP1 (pin 49), 0 = INT0 (pin 2) |
| `SERIAL_PRINT_EVERY` | 35 | Print every N teeth (≈ once/revolution) |

The ignition advance table (`ADVANCE_RPM` / `ADVANCE_DEG`) maps RPM to
advance degrees BTDC and is linearly interpolated at runtime.

---

## Building & Uploading

### Arduino IDE
1. Open `engine_ignition_hil/engine_ignition_hil.ino`.
2. Select **Tools → Board → Arduino Mega 2560**.
3. Select the correct serial port.
4. Click **Upload**.

### PlatformIO
```ini
[env:megaatmega2560]
platform  = atmelavr
board     = megaatmega2560
framework = arduino
src_dir   = engine_ignition_hil
```
```bash
pio run --target upload
```

---

## HIL Test Scenarios

| # | Scenario | Generator setting | Expected result |
|---|----------|-------------------|-----------------|
| 1 | **Startup / crank** | Ramp 0 → 400 RPM | CH-C goes HIGH after first gap; advance = 5° |
| 2 | **Acceleration** | Sweep 400 → 6 000 RPM | Advance increases per table; coil fires each revolution |
| 3 | **Rev-limiter** | Hold at ≥ 7 500 RPM | CH-B stays LOW (ignition cut); CH-C stays HIGH |
| 4 | **Noise injection** | Add random pulses between teeth | No false sync acquisition; Serial shows no RPM spikes |
| 5 | **Signal loss** | Stop generator | CH-C goes LOW within 4 revolutions; coil stops firing |
| 6 | **Re-synchronisation** | Restart generator | CH-C returns HIGH after first gap; ignition resumes |

### Procedure

1. Connect the HIL pulse generator to the crank sensor input pin.
2. Connect oscilloscope channels to pins 4, 5, and 6.
3. Open the Serial monitor at 115 200 baud.
4. Run each scenario in order, adjusting generator frequency to match the RPM
   formula: `f [Hz] = RPM × TEETH_TOTAL / 60`.
5. Capture oscilloscope waveforms and note any deviations from expected
   behaviour.
6. Document WCET (widest CH-A pulse), jitter (CH-A period variation), and
   spark timing error (CH-B falling edge vs. expected advance angle).

---

## Interpreting Oscilloscope Captures

```
CH-A (ISR time):
  ┌─┐   ┌─┐   ┌─┐
──┘ └───┘ └───┘ └──
  ↑ ↑
  │ └─ ISR exit  (LOW)
  └─── ISR entry (HIGH)
  Pulse width = ISR execution time
  Period = tooth interval = 1 / (RPM × TEETH_TOTAL / 60)

CH-B (coil):
           ┌──────┐
───────────┘      └─
           ↑      ↑
           dwell  spark (falling edge)
  HIGH duration = DWELL_MS = 4 ms

CH-C (sync):
  LOW during crank/no signal
  ─────────────────────────────── HIGH (synced)
```

---

## Serial Monitor Output

```
=== Engine Ignition HIL System ===
Teeth: 36-1
Rev limit: 7500 RPM
Input: ICP1 (pin 49)
HIL debug pins: ISR=4, COIL=5, SYNC=6
Waiting for crank signal...

[HIL] RPM=1500 TOOTH=12 ADV=14deg SYNC=OK   REVLIM=NO
[HIL] RPM=1503 TOOTH=12 ADV=14deg SYNC=OK   REVLIM=NO
[HIL] RPM=7512 TOOTH=12 ADV=34deg SYNC=OK   REVLIM=YES
[HIL] RPM=0    TOOTH= 0 ADV= 5deg SYNC=LOST REVLIM=NO
```

---

## Disabling HIL Instrumentation

To remove all debug overhead for a production build, add `#define HIL_DISABLE`
before the `#include "hil_debug.h"` line in the sketch, or pass the compiler
flag:

```ini
# PlatformIO
build_flags = -DHIL_DISABLE
```

All `HIL_*` macros compile to empty `do {} while (0)` stubs and the debug
pins are never initialised or driven, adding zero overhead to the ISR.
