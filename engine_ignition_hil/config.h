/**
 * @file config.h
 * @brief Configurable parameters for the Engine Ignition HIL system.
 *
 * All physical constants, pin assignments, and tuning tables live here so
 * that the rest of the sketch stays hardware-agnostic and easy to adapt to
 * different engines or Arduino boards.
 *
 * Target board: Arduino Mega 2560 (ATmega2560 @ 16 MHz)
 */

#ifndef CONFIG_H
#define CONFIG_H

/* =========================================================================
 * CRANKSHAFT TRIGGER WHEEL
 * =========================================================================
 * Standard 36-1 missing-tooth wheel:
 *   - 35 physical teeth + 1 gap (missing tooth position)
 *   - Gap is located at a fixed offset before TDC cylinder #1
 * ========================================================================= */
#define TEETH_TOTAL          36    /**< Nominal tooth count including the gap */
#define TEETH_MISSING         1    /**< Number of missing teeth in the gap    */
#define TEETH_PHYSICAL       (TEETH_TOTAL - TEETH_MISSING)  /* 35 */

/**
 * The gap tooth interval is approximately (TEETH_MISSING + 1) times the
 * normal tooth interval.  A ratio above GAP_DETECT_RATIO flags a gap.
 */
#define GAP_DETECT_RATIO     1.8f  /**< Gap detection threshold (×normal)    */

/** Degrees before TDC where the gap centre sits (engine-specific).           */
#define GAP_OFFSET_DEG       66    /**< Gap centre offset from TDC (degrees)  */

/**
 * Pre-calculated integer version of GAP_DETECT_RATIO × 10 used in the ISR to
 * avoid floating-point arithmetic on every tooth event.
 * (GAP_DETECT_RATIO * 10.0f = 1.8 * 10 = 18)
 */
#define GAP_DETECT_RATIO_SCALED  18U

/**
 * Pre-calculated sync-timeout limit in tooth counts.
 * When g_sync_timeout exceeds this value without a gap, sync is declared lost.
 */
#define SYNC_TIMEOUT_LIMIT  (TEETH_TOTAL * SYNC_TIMEOUT_TEETH)  /* 36 × 4 = 144 */

/**
 * Degrees per tooth (compile-time constant used for advance tooth calculation).
 */
#define DEG_PER_TOOTH       (360U / TEETH_TOTAL)   /* 360 / 36 = 10 degrees  */

/* =========================================================================
 * TIMING / RPM
 * ========================================================================= */
#define TIMER_PRESCALER      8     /**< Timer1 prescaler value                */
#define F_CPU_MHZ            16UL  /**< CPU frequency in MHz                  */

/** Microseconds per timer tick (with prescaler 8 @ 16 MHz → 0.5 µs/tick).   */
#define TICKS_PER_US         (F_CPU_MHZ / TIMER_PRESCALER)  /* 2 ticks/µs */

/** Maximum plausible RPM before inputs are considered noise.                 */
#define RPM_MAX              8000U
/** Minimum RPM to consider the engine as running (not cranking / stalled).  */
#define RPM_MIN               100U
/** RPM above which the rev-limiter cuts ignition.                            */
#define RPM_REVLIMIT          7500U

/** Number of tooth intervals with no pulse before declaring loss of sync.   */
#define SYNC_TIMEOUT_TEETH    4

/* =========================================================================
 * IGNITION ADVANCE TABLE  (RPM vs. advance degrees BTDC)
 *
 * Linear interpolation is performed between adjacent entries.
 * Both arrays must have the same length (ADVANCE_TABLE_LEN entries).
 * ========================================================================= */
#define ADVANCE_TABLE_LEN    8

static const uint16_t ADVANCE_RPM[ADVANCE_TABLE_LEN] = {
     500,  1000,  1500,  2000,  3000,  4000,  5500,  7000
};

static const uint8_t ADVANCE_DEG[ADVANCE_TABLE_LEN] = {
       5,    10,    14,    18,    24,    28,    32,    34
};

/* =========================================================================
 * COIL DWELL
 * ========================================================================= */
/** Fixed coil dwell time in milliseconds (charge time before firing).       */
#define DWELL_MS             4U

/* =========================================================================
 * PIN ASSIGNMENTS  (Arduino Mega 2560)
 * =========================================================================
 *
 *  ICP1  → Pin 49  : Crankshaft sensor input (Input Capture, Timer1)
 *                    Connect HIL pulse generator here.
 *
 *  INT0  → Pin  2  : Alternative interrupt input (if ICP1 is unavailable)
 *
 *  PIN_COIL        : Ignition coil drive output (active-HIGH → IGBT gate)
 *  PIN_DEBUG_ISR   : Toggled at the start/end of the crank ISR – connect to
 *                    oscilloscope channel A to measure WCET/jitter.
 *  PIN_DEBUG_COIL  : Mirrors the coil command – oscilloscope channel B.
 *  PIN_DEBUG_SYNC  : Asserted (HIGH) while the engine is synchronised.
 * ========================================================================= */
#define PIN_CRANK_ICP        49    /**< ICP1 – primary crank sensor input     */
#define PIN_CRANK_INT        2     /**< INT0 – fallback crank sensor input    */

#define PIN_COIL             3     /**< Ignition coil output                  */
#define PIN_DEBUG_ISR        4     /**< Debug: ISR entry/exit toggle          */
#define PIN_DEBUG_COIL       5     /**< Debug: coil command mirror            */
#define PIN_DEBUG_SYNC       6     /**< Debug: sync-valid flag                */

/* Set to 1 to use Input Capture (ICP1), 0 to use external interrupt (INT0). */
#define USE_ICP              1

/* =========================================================================
 * SERIAL / MONITORING
 * ========================================================================= */
#define SERIAL_BAUD          115200UL
/** Print a status line to Serial every N tooth events (0 = disabled).       */
#define SERIAL_PRINT_EVERY   35    /* roughly once per revolution */

#endif /* CONFIG_H */
