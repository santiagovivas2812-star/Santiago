/**
 * @file engine_ignition_hil.ino
 * @brief Engine ignition system with HIL (Hardware-in-the-Loop) instrumentation.
 *
 * ============================================================================
 * OVERVIEW
 * ============================================================================
 * This sketch implements a single-channel engine ignition controller for a
 * 36-1 missing-tooth crankshaft trigger wheel, with full HIL instrumentation
 * for laboratory validation on an Arduino Mega 2560.
 *
 * Key features
 * ------------
 *   • Input-Capture (ICP1 / Timer1) or external interrupt (INT0) for the
 *     crankshaft sensor — selectable via USE_ICP in config.h.
 *   • Missing-tooth (gap) detection to establish engine synchronisation.
 *   • RPM calculation from consecutive tooth intervals.
 *   • RPM-based ignition advance look-up table with linear interpolation.
 *   • Fixed dwell-time coil control (charge + fire).
 *   • Rev-limiter (ignition cut above RPM_REVLIMIT).
 *   • Safe-mode: ignition cut when sync is lost for > SYNC_TIMEOUT_TEETH
 *     consecutive tooth intervals.
 *   • HIL debug pins toggled at ISR entry/exit and coil events for
 *     WCET / jitter measurement with an oscilloscope.
 *   • Periodic Serial status prints for PC-side monitoring.
 *
 * ============================================================================
 * HIL WIRING DIAGRAM
 * ============================================================================
 *
 *  ┌─────────────────────────────────────────┐
 *  │         HIL PULSE GENERATOR             │
 *  │  (signal gen, Arduino, or FPGA board)   │
 *  └──────────────┬──────────────────────────┘
 *                 │  Square wave  (0–5 V, adjustable freq/duty)
 *                 │
 *  ┌──────────────▼──────────────────────────┐
 *  │          ARDUINO MEGA 2560              │
 *  │                                         │
 *  │  Pin 49 (ICP1)  ← Crank sensor input   │
 *  │  Pin  2 (INT0)  ← Alt. crank input     │
 *  │                                         │
 *  │  Pin  3         → Coil drive (IGBT)     │
 *  │  Pin  4         → OSC CH-A  (ISR time) │
 *  │  Pin  5         → OSC CH-B  (coil cmd) │
 *  │  Pin  6         → OSC CH-C  (sync OK)  │
 *  │                                         │
 *  │  TX0 (pin 1)    → PC Serial monitor    │
 *  └─────────────────────────────────────────┘
 *
 *  Oscilloscope measurements:
 *    CH-A pulse width  → ISR WCET (worst-case execution time)
 *    CH-A period jitter → Timing repeatability
 *    CH-B duty cycle   → Dwell time and spark timing
 *    CH-C level        → Sync-valid flag (HIGH = synced)
 *
 * ============================================================================
 * HIL TEST SCENARIOS
 * ============================================================================
 *  1. STARTUP / CRANK  : Ramp generator from 0 → 400 RPM.
 *                        Verify sync acquisition after first gap detected.
 *  2. ACCELERATION     : Sweep 400 → 6000 RPM; verify advance follows table.
 *  3. EXTREME RPM      : Hold at RPM_REVLIMIT; verify rev-limiter cuts fire.
 *  4. NOISE INJECTION  : Add random glitch pulses; verify no false sync.
 *  5. SIGNAL LOSS      : Remove generator signal; verify safe-mode activation
 *                        and CH-C goes LOW within SYNC_TIMEOUT_TEETH teeth.
 *  6. RE-SYNC          : Restore signal; verify re-synchronisation.
 *
 * ============================================================================
 * BUILD / UPLOAD
 * ============================================================================
 *  Board   : Arduino Mega 2560 (or Mega ADK)
 *  IDE     : Arduino IDE ≥ 1.8 or PlatformIO
 *  Monitor : 115200 baud, no line ending
 *
 *  To disable HIL instrumentation for production:
 *    Add  #define HIL_DISABLE  before  #include "hil_debug.h"
 *    or pass  -DHIL_DISABLE  in your build flags.
 *
 * @author  Santiago
 * @date    2026
 */

#include <Arduino.h>
#include "config.h"
#include "hil_debug.h"

/* ============================================================================
 * GLOBAL STATE  (accessed from both ISR and main loop – use volatile)
 * ========================================================================== */

/** Timer1 count captured at the previous tooth rising edge.                  */
static volatile uint16_t g_prev_capture  = 0;

/** Measured interval (Timer1 ticks) for the last tooth.                      */
static volatile uint16_t g_tooth_interval = 0;

/** Interval of the tooth just before the detected gap (reference interval).  */
static volatile uint16_t g_ref_interval  = 0;

/** Current tooth counter within the current engine cycle (0-based).          */
static volatile uint8_t  g_tooth_count   = 0;

/** TRUE once the gap has been found and the engine is synchronised.          */
static volatile bool     g_synced        = false;

/** Consecutive teeth seen without a gap since last sync loss.                */
static volatile uint8_t  g_sync_timeout  = 0;

/** Most recently calculated RPM.                                             */
static volatile uint16_t g_rpm           = 0;

/** Most recently calculated ignition advance in degrees BTDC.                */
static volatile uint8_t  g_advance_deg   = 0;

/** TRUE when the rev-limiter is active.                                      */
static volatile bool     g_rev_limit     = false;

/** Tooth counter used for periodic Serial prints.                            */
static volatile uint16_t g_print_counter = 0;

/** Flag set inside the ISR to request a Serial print from the main loop.    */
static volatile bool     g_print_request = false;

/* Snapshot of state variables for the main-loop print (avoid long Serial in
 * an ISR which would break timing).                                          */
static volatile uint16_t g_snap_rpm      = 0;
static volatile uint8_t  g_snap_tooth    = 0;
static volatile uint8_t  g_snap_advance  = 0;
static volatile bool     g_snap_synced   = false;
static volatile bool     g_snap_revlim   = false;

/* ============================================================================
 * HELPER: IGNITION ADVANCE INTERPOLATION
 * ========================================================================== */

/**
 * @brief Look up the ignition advance for a given RPM.
 *
 * Performs linear interpolation between entries in ADVANCE_RPM / ADVANCE_DEG.
 *
 * @param rpm  Current engine speed in RPM.
 * @return     Advance angle in degrees BTDC.
 */
static uint8_t advance_lookup(uint16_t rpm)
{
    /* Below first table entry → return minimum advance. */
    if (rpm <= ADVANCE_RPM[0]) {
        return ADVANCE_DEG[0];
    }
    /* Above last table entry → return maximum advance. */
    if (rpm >= ADVANCE_RPM[ADVANCE_TABLE_LEN - 1]) {
        return ADVANCE_DEG[ADVANCE_TABLE_LEN - 1];
    }
    /* Linear interpolation between bracketing entries. */
    for (uint8_t i = 0; i < (ADVANCE_TABLE_LEN - 1); ++i) {
        if (rpm < ADVANCE_RPM[i + 1]) {
            uint16_t rpm_lo  = ADVANCE_RPM[i];
            uint16_t rpm_hi  = ADVANCE_RPM[i + 1];
            uint8_t  adv_lo  = ADVANCE_DEG[i];
            uint8_t  adv_hi  = ADVANCE_DEG[i + 1];
            /* Avoid division by zero (table entries must be strictly ascending). */
            if (rpm_hi == rpm_lo) {
                return adv_lo;
            }
            uint8_t adv = adv_lo +
                (uint8_t)(((uint32_t)(rpm - rpm_lo) * (adv_hi - adv_lo)) /
                          (rpm_hi - rpm_lo));
            return adv;
        }
    }
    return ADVANCE_DEG[ADVANCE_TABLE_LEN - 1];
}

/* ============================================================================
 * HELPER: RPM CALCULATION
 * ========================================================================== */

/**
 * @brief Calculate RPM from a single tooth interval.
 *
 * Each tooth spans (360 / TEETH_TOTAL) degrees.
 * One full revolution = TEETH_TOTAL tooth intervals.
 *
 * interval [ticks] → period_rev [ticks] = interval × TEETH_TOTAL
 * RPM = 60 / period_rev_seconds
 *     = 60 × F_TICK / period_rev_ticks
 *
 * F_TICK = F_CPU / TIMER_PRESCALER = 16e6 / 8 = 2e6 ticks/s
 *
 * @param ticks  Timer1 tick count for one tooth interval.
 * @return       RPM (0 if ticks == 0 to avoid division by zero).
 */
static uint16_t ticks_to_rpm(uint16_t ticks)
{
    if (ticks == 0) {
        return 0;
    }
    /* F_TICK in ticks/s = (F_CPU_MHZ * 1e6) / TIMER_PRESCALER               */
    /* RPM = 60 * F_TICK / (ticks * TEETH_TOTAL)                              */
    uint32_t f_tick = (uint32_t)F_CPU_MHZ * 1000000UL / TIMER_PRESCALER;
    uint32_t rpm32  = (60UL * f_tick) / ((uint32_t)ticks * TEETH_TOTAL);
    if (rpm32 > 65535UL) {
        return 65535U;   /* cap to uint16 */
    }
    return (uint16_t)rpm32;
}

/* ============================================================================
 * COIL CONTROL
 * ========================================================================== */

/**
 * @brief Trigger one ignition event: dwell then fire.
 *
 * This is called from the main loop (NOT from the ISR) to keep the ISR short.
 * It blocks for DWELL_MS milliseconds – acceptable in a single-cylinder demo;
 * a production system would use Timer2 interrupts for non-blocking dwell.
 */
static void coil_fire_blocking(void)
{
    /* Start dwell (charge coil). */
    HIL_COIL_DWELL();
    digitalWrite(PIN_COIL, HIGH);
    delay(DWELL_MS);

    /* Fire (collapse field → spark). */
    HIL_COIL_FIRE();
    digitalWrite(PIN_COIL, LOW);
}

/* ============================================================================
 * ISR: INPUT CAPTURE (ICP1 / Timer1)
 * ========================================================================== */

#if USE_ICP

/**
 * @brief Timer1 Input Capture ISR.
 *
 * Triggered on every rising edge of the crankshaft sensor signal captured
 * by the ICP1 hardware (pin 49 on Mega).  Timer1 runs with prescaler 8,
 * giving 0.5 µs resolution at 16 MHz.
 *
 * Responsibilities:
 *   1. Measure tooth interval.
 *   2. Detect missing-tooth gap.
 *   3. Maintain tooth counter and sync state.
 *   4. Update RPM and advance angle.
 *   5. Toggle HIL debug pin to mark ISR WCET.
 */
ISR(TIMER1_CAPT_vect)
{
    HIL_ISR_ENTER();    /* ← oscilloscope CH-A goes HIGH */

    uint16_t capture = ICR1;   /* Read captured timer value */

    uint16_t interval = capture - g_prev_capture;
    g_prev_capture    = capture;

    /* ------------------------------------------------------------------ */
    /* Gap detection                                                       */
    /* ------------------------------------------------------------------ */
    bool gap_detected = false;

    if (g_ref_interval > 0) {
        /* The gap interval is roughly (TEETH_MISSING + 1) × normal.
         * Use the pre-scaled integer constant to avoid float in ISR.         */
        uint32_t threshold = (uint32_t)g_ref_interval * GAP_DETECT_RATIO_SCALED / 10U;
        if ((uint32_t)interval >= threshold) {
            gap_detected = true;
        }
    }

    if (gap_detected) {
        /* Re-synchronise: tooth 0 follows the gap. */
        g_tooth_count   = 0;
        g_synced        = true;
        g_sync_timeout  = 0;
        HIL_SYNC_VALID();   /* CH-C HIGH */
    } else {
        /* Normal tooth: increment counter and check for sync timeout. */
        if (g_synced) {
            ++g_tooth_count;
            if (g_tooth_count >= TEETH_PHYSICAL) {
                /* Wrapped without seeing a gap → sync lost. */
                g_synced       = false;
                g_sync_timeout = 0;
                HIL_SYNC_LOST();    /* CH-C LOW */
            }
        }
        /* Count consecutive teeth without a gap for timeout detection.  */
        ++g_sync_timeout;
        if (g_sync_timeout > SYNC_TIMEOUT_LIMIT) {
            g_synced       = false;
            g_sync_timeout = 0;
            HIL_SYNC_LOST();
        }
    }

    /* Store reference interval for next gap detection (use non-gap teeth). */
    if (!gap_detected) {
        g_ref_interval = interval;
    }
    g_tooth_interval = interval;

    /* ------------------------------------------------------------------ */
    /* RPM and advance update                                              */
    /* ------------------------------------------------------------------ */
    uint16_t rpm = ticks_to_rpm(interval);

    /* Clamp to plausible range. */
    if (rpm > RPM_MAX) {
        rpm = 0;    /* likely noise */
    }
    g_rpm        = rpm;
    g_rev_limit  = (rpm >= RPM_REVLIMIT);
    g_advance_deg = advance_lookup(rpm);

    /* ------------------------------------------------------------------ */
    /* Serial print request (every N teeth, handled in loop())            */
    /* ------------------------------------------------------------------ */
#if SERIAL_PRINT_EVERY > 0
    ++g_print_counter;
    if (g_print_counter >= SERIAL_PRINT_EVERY) {
        g_print_counter  = 0;
        g_snap_rpm      = g_rpm;
        g_snap_tooth    = g_tooth_count;
        g_snap_advance  = g_advance_deg;
        g_snap_synced   = g_synced;
        g_snap_revlim   = g_rev_limit;
        g_print_request = true;
    }
#endif

    HIL_ISR_EXIT();     /* ← oscilloscope CH-A goes LOW */
}

#else /* USE_ICP == 0  →  external interrupt on INT0 (pin 2) */

/**
 * @brief External interrupt ISR (INT0, pin 2) – rising edge.
 *
 * Identical logic to the ICP1 version but uses micros() for timing instead
 * of the hardware capture register.  micros() has ~4 µs granularity and is
 * less accurate than ICP but sufficient for demonstration purposes.
 */
static volatile uint32_t g_prev_micros = 0;

void crank_int_isr(void)
{
    HIL_ISR_ENTER();

    uint32_t now      = micros();
    uint32_t interval_us = now - g_prev_micros;
    g_prev_micros     = now;

    /* Convert µs interval to Timer1-equivalent ticks for shared helpers.  */
    uint16_t interval = (uint16_t)(interval_us * TICKS_PER_US);

    bool gap_detected = false;
    if (g_ref_interval > 0) {
        uint32_t threshold = (uint32_t)g_ref_interval * GAP_DETECT_RATIO_SCALED / 10U;
        if ((uint32_t)interval >= threshold) {
            gap_detected = true;
        }
    }

    if (gap_detected) {
        g_tooth_count  = 0;
        g_synced       = true;
        g_sync_timeout = 0;
        HIL_SYNC_VALID();
    } else {
        if (g_synced) {
            ++g_tooth_count;
            if (g_tooth_count >= TEETH_PHYSICAL) {
                g_synced       = false;
                g_sync_timeout = 0;
                HIL_SYNC_LOST();
            }
        }
        ++g_sync_timeout;
        if (g_sync_timeout > SYNC_TIMEOUT_LIMIT) {
            g_synced       = false;
            g_sync_timeout = 0;
            HIL_SYNC_LOST();
        }
    }

    if (!gap_detected) {
        g_ref_interval = interval;
    }
    g_tooth_interval = interval;

    uint16_t rpm = ticks_to_rpm(interval);
    if (rpm > RPM_MAX) {
        rpm = 0;
    }
    g_rpm         = rpm;
    g_rev_limit   = (rpm >= RPM_REVLIMIT);
    g_advance_deg = advance_lookup(rpm);

#if SERIAL_PRINT_EVERY > 0
    ++g_print_counter;
    if (g_print_counter >= SERIAL_PRINT_EVERY) {
        g_print_counter  = 0;
        g_snap_rpm      = g_rpm;
        g_snap_tooth    = g_tooth_count;
        g_snap_advance  = g_advance_deg;
        g_snap_synced   = g_synced;
        g_snap_revlim   = g_rev_limit;
        g_print_request = true;
    }
#endif

    HIL_ISR_EXIT();
}

#endif /* USE_ICP */

/* ============================================================================
 * SETUP
 * ========================================================================== */

void setup(void)
{
    /* ----- Serial monitor ------------------------------------------------ */
    Serial.begin(SERIAL_BAUD);
    Serial.println(F("=== Engine Ignition HIL System ==="));
    Serial.print(F("Teeth: "));   Serial.print(TEETH_TOTAL);
    Serial.print(F("-"));         Serial.println(TEETH_MISSING);
    Serial.print(F("Rev limit: ")); Serial.print(RPM_REVLIMIT);
    Serial.println(F(" RPM"));
#if USE_ICP
    Serial.println(F("Input: ICP1 (pin 49)"));
#else
    Serial.println(F("Input: INT0 (pin 2)"));
#endif
    Serial.println(F("HIL debug pins: ISR=4, COIL=5, SYNC=6"));
    Serial.println(F("Waiting for crank signal..."));
    Serial.println();

    /* ----- HIL debug pins ------------------------------------------------ */
    hil_debug_init();

    /* ----- Coil output --------------------------------------------------- */
    pinMode(PIN_COIL, OUTPUT);
    digitalWrite(PIN_COIL, LOW);

    /* ----- Crank input --------------------------------------------------- */
#if USE_ICP
    /* Configure ICP1: Timer1 in Normal mode, prescaler 8, ICP on rising edge */
    pinMode(PIN_CRANK_ICP, INPUT);      /* pin 49 */

    TCCR1A = 0;                         /* Normal mode */
    TCCR1B = (1 << ICES1)              /* Input capture on rising edge */
           | (1 << CS11);              /* Prescaler /8 */
    TIMSK1 = (1 << ICIE1);             /* Enable input capture interrupt */
    TCNT1  = 0;

#else
    /* Configure INT0 on pin 2 (rising edge) */
    pinMode(PIN_CRANK_INT, INPUT);
    attachInterrupt(digitalPinToInterrupt(PIN_CRANK_INT),
                    crank_int_isr, RISING);
#endif
}

/* ============================================================================
 * MAIN LOOP
 * ========================================================================== */

/**
 * @brief Main loop – coil control and Serial monitoring.
 *
 * The ISR handles all timing-critical operations.  The main loop:
 *   1. Decides whether to fire the coil (using snapshotted state to avoid
 *      long critical sections).
 *   2. Prints periodic HIL status to Serial.
 *
 * Coil firing strategy (simplified, single cylinder):
 *   Fire on every tooth that corresponds to the advance angle before TDC.
 *   TDC tooth = GAP_OFFSET_DEG / (360 / TEETH_TOTAL)
 *   Fire tooth = TDC tooth - advance_tooth
 *
 * In a real system this would be handled with a Timer2 compare match
 * interrupt; here we use a simple tooth-count comparison for clarity.
 */
void loop(void)
{
    /* ---------------------------------------------------------------------- */
    /* Snapshot volatile state with interrupts briefly disabled.             */
    /* ---------------------------------------------------------------------- */
    noInterrupts();
    bool     synced      = g_synced;
    bool     rev_limit   = g_rev_limit;
    uint8_t  tooth       = g_tooth_count;
    uint8_t  advance     = g_advance_deg;
    uint16_t rpm         = g_rpm;
    interrupts();

    /* ---------------------------------------------------------------------- */
    /* Coil fire decision                                                     */
    /* ---------------------------------------------------------------------- */
    /* Compile-time constants: avoid repeated division in the hot path.      */
    const uint8_t TDC_TOOTH     = GAP_OFFSET_DEG / DEG_PER_TOOTH;   /* = 6  */

    if (synced && !rev_limit && rpm >= RPM_MIN) {
        /* Calculate which tooth should trigger ignition.
         * TDC tooth index (from gap) = GAP_OFFSET_DEG / DEG_PER_TOOTH
         * Advance tooth offset       = advance_deg    / DEG_PER_TOOTH
         */
        uint8_t advance_off = advance / DEG_PER_TOOTH;
        uint8_t fire_tooth  = (TDC_TOOTH > advance_off)
                              ? (TDC_TOOTH - advance_off)
                              : 0U;

        if (tooth == fire_tooth) {
            coil_fire_blocking();
        }
    }

    /* ---------------------------------------------------------------------- */
    /* Serial status print (requested by ISR every SERIAL_PRINT_EVERY teeth) */
    /* ---------------------------------------------------------------------- */
#if SERIAL_PRINT_EVERY > 0
    if (g_print_request) {
        g_print_request = false;   /* clear before read to avoid race */
        hil_serial_status(g_snap_rpm,
                          g_snap_tooth,
                          g_snap_advance,
                          g_snap_synced,
                          g_snap_revlim);
    }
#endif
}
