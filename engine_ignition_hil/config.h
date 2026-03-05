#pragma once

/*
 * config.h  –  Santiago DIS Ignition System
 *
 * All floating-point and heavyweight arithmetic is resolved here at
 * compile time.  Nothing in this file needs to be evaluated at runtime,
 * so the ISRs can remain arithmetic-free and deterministic.
 *
 * Architecture: Speeduino-style
 *   • Timer1 ICP1  (TIMER1_CAPT_vect)  – crank-tooth capture + sync
 *   • Timer1 OCR1A (TIMER1_COMPA_vect) – dwell-start / spark-fire
 *   • Events pre-computed in loop()    – no multiply/divide in ISR
 *   • Modular uint16_t wrap for OCR1A scheduling
 */

// ─── Trigger Wheel ────────────────────────────────────────────────────────────
#define TRIGGER_TEETH       36          // Total tooth positions on wheel
#define MISSING_TEETH        1          // Number of missing teeth (gap)
#define ACTUAL_TEETH        (TRIGGER_TEETH - MISSING_TEETH)  // 35 real teeth
#define DEGREES_PER_TOOTH   (360 / TRIGGER_TEETH)            // 10 deg (integer)

// ─── Engine / DIS Configuration ──────────────────────────────────────────────
#define NUM_CYLINDERS        4

// Tooth numbers where each cylinder pair reaches TDC
// (counted from tooth 0 = first tooth after the missing-tooth gap, 0-based)
// Standard 4-cylinder inline, firing order 1-3-4-2 (waste-spark DIS):
//   Pair A  →  cyl 1 & 4  →  TDC every revolution at CYL_A_TDC_TOOTH
//   Pair B  →  cyl 2 & 3  →  TDC every revolution at CYL_B_TDC_TOOTH
#define CYL_A_TDC_TOOTH      7          // 70° after TDC reference (missing-tooth edge)
#define CYL_B_TDC_TOOTH     25          // 250° after TDC reference (missing-tooth edge)

// ─── Pin Assignments ──────────────────────────────────────────────────────────
// ICP1 is hardware-fixed at PB0 → Arduino Uno/Nano pin 8
#define ICP_PIN              8

// Coil outputs (active-HIGH = dwell / energise, LOW = idle / spark-fired)
// All four on PORTD so safeState() can clear them with a single register write.
#define COIL1_PIN            4          // Waste-spark pair A  (cyl 1 & 4)
#define COIL2_PIN            5          // Waste-spark pair B  (cyl 2 & 3)
#define COIL3_PIN            6          // Reserved – sequential mode
#define COIL4_PIN            7          // Reserved – sequential mode

// Bitmask for simultaneously clearing all coil pins via PORTD
#define COIL_PORT_MASK  ((1 << COIL1_PIN) | (1 << COIL2_PIN) | \
                         (1 << COIL3_PIN) | (1 << COIL4_PIN))

// CAM / phase sensor – INT0 (Arduino pin 2) – optional for sequential ignition
#define CAM_PIN              2

// ─── Timer1 ───────────────────────────────────────────────────────────────────
// Prescaler  = 8  →  tick period = 0.5 µs  at 16 MHz
// Prescaler  = 8  →  tick rate   = 2 000 000 Hz
#define TIMER1_PRESCALER     8
#define TIMER1_TICK_HZ       ((uint32_t)F_CPU / TIMER1_PRESCALER)   // 2 000 000

// ─── Dwell (compile-time constant) ───────────────────────────────────────────
#define DWELL_US            3000UL
// Ticks = dwell_us × (tick_hz / 1 000 000)  ← no runtime division needed
#define DWELL_TICKS         ((uint16_t)(DWELL_US * (TIMER1_TICK_HZ / 1000000UL)))

// ─── Ignition Advance ─────────────────────────────────────────────────────────
#define DEFAULT_ADVANCE_DEG  15         // Degrees BTDC (adjustable at runtime)

// ─── Rev Limit ────────────────────────────────────────────────────────────────
#define REV_LIMIT_RPM        7000UL
// Minimum ticks-per-tooth at the rev limit – used in ISR as an integer compare.
// tpt_min = TIMER1_TICK_HZ × 60 / (RPM_limit × TRIGGER_TEETH)
#define REV_LIMIT_MIN_TICKS \
    ((uint16_t)((uint32_t)TIMER1_TICK_HZ * 60UL / \
                ((uint32_t)REV_LIMIT_RPM * TRIGGER_TEETH)))

// ─── Sync / Gap Detection ────────────────────────────────────────────────────
// Minimum consecutive correct-gap events before declaring crank sync
#define SYNC_MIN_GOOD_GAPS   3

// Missing-tooth threshold (integer ratio, avoids float in ISR):
//   gap detected  when  current_period × GAP_DEN  ≥  prev_period × GAP_NUM
//   i.e. current ≥ 1.8 × previous
#define GAP_RATIO_NUM       18
#define GAP_RATIO_DEN       10

// Crank-stall timeout: no tooth for this many ms → lose sync
#define CRANK_TIMEOUT_MS    500UL

// ─── OLED Display ────────────────────────────────────────────────────────────
#define USE_OLED             1
#define OLED_I2C_ADDR        0x3C
#define OLED_WIDTH          128
#define OLED_HEIGHT          64
#define OLED_UPDATE_MS      250UL       // Display refresh period
