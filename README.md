# Santiago – DIS Ignition System (Speeduino-style, Arduino Mega 2560)

## Overview

This project implements a **Direct Ignition System (DIS)** with wasted-spark
firing, modelled on the Speeduino open-source ECU architecture, targeting the
**Arduino Mega 2560** (ATmega2560, 16 MHz).

Spark scheduling is performed entirely in hardware-timer interrupts using
**Timer1 Input Capture (ICP1)** for crank-reference signal acquisition and
**Timer1 Output Compare A (OCR1A)** for coil event dispatching. This approach
guarantees deterministic, jitter-free spark timing across the full RPM range.

---

## Key Features

| Feature | Detail |
|---|---|
| Crank reference | ICP1 (Pin 49) – rising-edge capture, noise-canceler enabled |
| Spark scheduler | OCR1A output-compare interrupt – fires the earliest pending coil event |
| Coil model | Wasted-spark, up to 4 coils (Pins 2–5) |
| Dwell | Fixed 3 ms; configurable via `DWELL_TICKS` |
| Advance | Fixed 10° BTDC; configurable via `FIXED_ADVANCE_DEG` |
| Rev-limit | 8 000 RPM – all coils shut off immediately |
| Signal timeout | 500 ms without a reference pulse → safe mode (all coils off) |
| Timer wrap | Handled with signed 32-bit delta arithmetic (`int32_t`) |
| ISR discipline | **No** `Serial`, `Wire`, or other blocking calls inside any ISR |
| Display | 128×64 SSD1306 OLED updated at ≤ 10 Hz from `loop()` |
| Trigger wheel | Configurable (default: 60-2, 58 active teeth) |

---

## Hardware Connections

```
Arduino Mega 2560
  Pin 49  (ICP1 / PB4)  ──── Crank trigger signal (rising edge)
  Pin  2                ──── Coil 1 low-side driver (IGBT/MOSFET gate)
  Pin  3                ──── Coil 2 low-side driver
  Pin  4                ──── Coil 3 low-side driver
  Pin  5                ──── Coil 4 low-side driver
  Pin 20  (SDA)         ──── OLED SDA
  Pin 21  (SCL)         ──── OLED SCL
  GND                   ──── Common ground
```

> **Important:** The crank-trigger input **must** be conditioned to 0–5 V
> logic levels before connecting to ICP1. Never connect a VR sensor directly.

---

## Timer1 Configuration

| Parameter | Value |
|---|---|
| Mode | Normal (free-running 16-bit, TOP = 0xFFFF) |
| Prescaler | 8 → **0.5 µs per tick** at 16 MHz |
| ICP edge | Rising |
| Noise canceler | Enabled (4-cycle filter ≈ 250 ns) |
| OCR1A | Loaded with absolute tick of next coil event |

---

## Software Architecture

```
loop()
 ├─ Signal-timeout watchdog  (read g_lastIcpMillis vs millis())
 ├─ Rev-limit check          (read g_rpm)
 ├─ Coil event precomputation (write g_coilQueue[] with charge/fire ticks)
 ├─ scheduleNextEvent()       (scan queues, load soonest tick into OCR1A)
 └─ OLED refresh at 10 Hz

ISR(TIMER1_CAPT_vect)   — ICP1 handler
 ├─ Capture ICR1 (crank tooth tick)
 ├─ Compute period (unsigned 16-bit subtraction, correct across wrap)
 ├─ Update g_rpm, g_toothPeriodTicks, g_lastIcpMillis
 └─ Clear g_faultNoSignal

ISR(TIMER1_COMPA_vect)  — OCR1A handler
 ├─ Execute pending coil event (digitalWrite HIGH or LOW)
 ├─ Clear event from queue
 └─ Call scheduleNextEvent() to re-arm OCR1A for the next event
```

### Signed-Delta Wrap Handling

All comparisons between Timer1 ticks use **signed 32-bit subtraction**:

```cpp
int32_t delta = (int32_t)(eventTick - TCNT1);
```

Because `uint16_t` subtraction wraps at 65 535, casting the result to
`int32_t` gives a value in [−32768, +32767], which correctly represents
"past" (negative) vs "future" (positive) events even across a counter
overflow.

---

## Dependencies

Install via the Arduino Library Manager:

- **Adafruit SSD1306** (≥ 2.5)
- **Adafruit GFX Library** (≥ 1.11)

---

## Building & Flashing

1. Open `DIS_Ignition/DIS_Ignition.ino` in the Arduino IDE (≥ 2.0) or
   PlatformIO.
2. Select board **Arduino Mega or Mega 2560**.
3. Install the required libraries listed above.
4. Compile and upload.

---

## Validation & Next Steps

- [ ] Bench-test with a pulse generator on ICP1 (e.g. 100–500 Hz for
      100–5 000 RPM equivalent).
- [ ] Measure ISR WCET and jitter with a logic analyser on coil output pins.
- [ ] Adjust `DWELL_TICKS` and `FIXED_ADVANCE_DEG` to match coil spec and
      engine requirements.
- [ ] Replace fixed advance with a 2-D map (RPM × MAP) for production use.
- [ ] HIL (Hardware-in-the-Loop) validation with a crank-simulator board.

---

## License

This project is released for educational and engineering purposes.
See [LICENSE](LICENSE) if present, or contact the repository owner.
