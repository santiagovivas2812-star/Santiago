# Santiago – DIS Ignition System (Speeduino-style)

Arduino-based **Direct Ignition System (DIS)** for 4-cylinder engines,
architecturally equivalent to [Speeduino](https://speeduino.com/) and
validated against its official codebase.

---

## Architecture overview

| Feature | Speeduino | Santiago |
|---------|-----------|---------|
| Timer & scheduling | Timer1 OCR1A + ICP1 | Timer1 OCR1A + ICP1 |
| Heavy maths in ISR | Avoided (pre-computed) | Avoided (pre-computed) |
| Modular wrap / OCR1A | uint16 natural wrap | uint16 natural wrap |
| Safe state / rev-limit | Coils off + disarm | Coils off + disarm |
| Sync / gap detection | goodGapCount + cam | goodGapCount + cam |
| Diagnostics counters | Yes | Yes (teeth, sync-loss, rev-limit, sparks) |
| OLED display | No (external) | **Yes – native SSD1306 128×64** |

---

## Files

```
engine_ignition_hil/
├── config.h                  # Compile-time constants (no runtime float/divide)
└── engine_ignition_hil.ino   # Main sketch
```

### `config.h`
All floating-point and heavyweight arithmetic is resolved at **compile time**
as `#define` macros.  The ISRs only reference pre-evaluated integer constants.

Key settings:

| Macro | Default | Purpose |
|-------|---------|---------|
| `TRIGGER_TEETH` | 36 | Trigger-wheel tooth count |
| `MISSING_TEETH` | 1 | Missing teeth in gap |
| `DWELL_TICKS` | 6000 | Dwell duration in Timer1 ticks (3 ms @ 2 MHz) |
| `DEFAULT_ADVANCE_DEG` | 15 | Ignition advance (degrees BTDC) |
| `REV_LIMIT_RPM` | 7000 | Rev-limit cut-off |
| `SYNC_MIN_GOOD_GAPS` | 3 | Gaps required before sync declared |
| `GAP_RATIO_NUM/DEN` | 18/10 | Missing-tooth ratio threshold (1.8×) |
| `CRANK_TIMEOUT_MS` | 500 | Stall-detection timeout |

### `engine_ignition_hil.ino`

**`TIMER1_CAPT_vect` – ICP1 ISR (crank-tooth capture)**
- Captures `ICR1` on each rising edge of the crank sensor.
- Computes inter-tooth period; detects missing-tooth gap with the integer
  ratio `(period × GAP_DEN) ≥ (prevPeriod × GAP_NUM)`.
- Increments `goodGapCount`; declares sync after `SYNC_MIN_GOOD_GAPS`
  consecutive valid gaps.
- Rev-limit cut: if period < `REV_LIMIT_MIN_TICKS`, disables all coils.
- At the trigger tooth for each coil pair, arms `OCR1A` using pre-computed
  offsets (no multiply or divide inside the ISR).

**`TIMER1_COMPA_vect` – OCR1A ISR (dwell / spark)**
- `IGN_DWELL` state: energises the coil, advances `OCR1A` by `dwellTicks`.
- `IGN_SPARK`  state: de-energises the coil (spark fires), disarms interrupt.
- `OCR1A += dwellTicks` uses natural uint16_t wrap – correct for any
  Timer1 roll-over, identical to Speeduino's scheduling model.

**`precomputeEvents()` – called from `loop()`**
- Computes `triggerTooth` and `dwellOffsetTicks` for each coil pair:
  ```
  rawTicks  = tdcTooth × tpt − (advTicks + dwellTicks)
  normalise to [0, revTicks) via modular wrap
  triggerTooth      = rawTicks / tpt
  dwellOffsetTicks  = rawTicks % tpt
  ```
- Uses `noInterrupts()` / `interrupts()` for the atomic struct write.

**OLED display** – refreshed every `OLED_UPDATE_MS` (250 ms):
- Line 1: RPM
- Line 2: Advance (degrees BTDC)
- Line 3: Sync state + rev-limit flag
- Line 4: Sparks fired counter
- Line 5: Sync-loss counter

**Serial diagnostics** – printed every 1 s:
```
RPM=2500 ADV=15deg SYNC=Y TEETH=12345 SPK=567 SLOSS=0 RLIM=0
```

---

## Wiring (Arduino Uno / Nano)

| Signal | Pin | Notes |
|--------|-----|-------|
| Crank sensor (ICP1) | 8 | Rising edge, INPUT_PULLUP |
| CAM / phase sensor | 2 | INT0, falling edge (optional) |
| Coil A (cyl 1 & 4) | 4 | Active HIGH = dwell |
| Coil B (cyl 2 & 3) | 5 | Active HIGH = dwell |
| Coil C (reserved) | 6 | Sequential mode |
| Coil D (reserved) | 7 | Sequential mode |
| OLED SDA | A4 | I²C |
| OLED SCL | A5 | I²C |

---

## Technical comparison with Speeduino

The Santiago DIS ignition follows the same architectural foundations as
Speeduino with functional equivalence across all real-time, modularity and
automotive robustness requirements:

1. **Timer & Scheduling** – both use Timer1 OCR1A + ICP1, same ISR pattern.
2. **Multiply/divide avoidance in ISR** – pre-computed in `loop()` in both.
3. **Modular wrap** – signed-delta / uint16 wrap identical in both.
4. **Safe state & rev-limit** – coil disable + OCR1A disarm in both.
5. **Sync robustness** – gap ratio + `goodGapCount` + cam sync in both.
6. **Diagnostics** – counters for teeth, sync-loss, rev-limit, sparks.
7. **OLED display** – native SSD1306 display (addition over Speeduino base).

---

## Dependencies

- [Adafruit SSD1306](https://github.com/adafruit/Adafruit_SSD1306)
- [Adafruit GFX Library](https://github.com/adafruit/Adafruit-GFX-Library)

Install via **Arduino IDE → Sketch → Include Library → Manage Libraries**.
