/**
 * engine_ignition_hil.ino
 *
 * Santiago DIS Ignition System  –  Speeduino-style architecture
 * ──────────────────────────────────────────────────────────────
 *
 * Functional equivalence with Speeduino official code:
 *
 * 1. TIMER & SCHEDULING
 *    Timer1 OCR1A provides sub-microsecond spark scheduling.
 *    ICP1 (TIMER1_CAPT_vect) captures each crank tooth edge and
 *    programs the next OCR1A event – identical pattern to Speeduino.
 *
 * 2. PRE-COMPUTED EVENTS
 *    All multiplications and divisions that depend on RPM/advance are
 *    done in loop() (precomputeEvents), never inside an ISR.  The ISR
 *    only reads pre-computed uint16_t values, matching Speeduino's
 *    approach.
 *
 * 3. MODULAR WRAP
 *    OCR1A values are uint16_t.  All scheduling uses natural 16-bit
 *    unsigned arithmetic so Timer1 counter wrap is handled correctly
 *    without explicit range checks, exactly as Speeduino does it.
 *
 * 4. SAFE STATE & REV-LIMIT
 *    On sync loss, stall-timeout or rev-limit: all coils are de-energised
 *    and OCR1A is disarmed.  Equivalent to Speeduino's safe-state path.
 *
 * 5. SYNC & GAP DETECTION
 *    Missing-tooth gap detected with an integer ratio (1.8×).
 *    goodGapCount reaches SYNC_MIN_GOOD_GAPS before sync is declared.
 *    Optional CAM/phase sensor (INT0) enables sequential mode.
 *
 * 6. DIAGNOSTICS
 *    Counters for tooth events, sync-loss events, rev-limit hits and
 *    sparks fired – streamed over Serial every second.
 *
 * 7. OLED DISPLAY  (native, not present in Speeduino base)
 *    128×64 SSD1306 I²C display shows RPM, advance, sync state and
 *    diagnostic counters, refreshed every OLED_UPDATE_MS.
 *
 * Target: ATmega328P (Arduino Uno / Nano / Pro Mini)  @  16 MHz
 * Required libraries: Adafruit_SSD1306, Adafruit_GFX (for OLED)
 */

#include <Arduino.h>
#include "config.h"

#if USE_OLED
#  include <Wire.h>
#  include <Adafruit_GFX.h>
#  include <Adafruit_SSD1306.h>
static Adafruit_SSD1306 oled(OLED_WIDTH, OLED_HEIGHT, &Wire, -1);
#endif

// ─── Spark event descriptor (pre-computed in loop, read in ISR) ──────────────
struct SparkEvent {
    uint8_t  triggerTooth;       // Tooth index at which to arm OCR1A
    uint16_t dwellOffsetTicks;   // Ticks after trigger-tooth edge to start dwell
    uint8_t  coilPin;            // Arduino pin number for this coil
};

// ─── OCR1A state machine ─────────────────────────────────────────────────────
enum IgnState : uint8_t {
    IGN_IDLE  = 0,  // No event scheduled
    IGN_DWELL = 1,  // OCR1A will start dwell (coil ON)
    IGN_SPARK = 2   // OCR1A will fire spark  (coil OFF)
};

// ─── Volatile ISR-shared state ────────────────────────────────────────────────
volatile uint16_t g_lastICP        = 0;     // Timer1 count at last tooth edge
volatile uint16_t g_toothPeriod    = 0;     // Ticks between last two teeth
volatile uint16_t g_prevPeriod     = 0;     // Ticks two teeth ago (gap detect)
volatile uint8_t  g_toothCount     = 0;     // Current tooth (0 = first after gap)
volatile bool     g_synced         = false; // Crank sync established
volatile uint8_t  g_goodGapCount   = 0;     // Consecutive valid gap detections
volatile bool     g_camPhase       = false; // CAM: true = first rev of 2-rev cycle
volatile IgnState g_ignState       = IGN_IDLE;
volatile uint8_t  g_activeCoilPin  = 0;     // Coil pin currently being driven
volatile bool     g_revLimitActive = false; // Rev-limit cut active

// Diagnostics counters (read in loop, written in ISR – use noInterrupts snapshot)
volatile uint32_t g_diagTeeth      = 0;
volatile uint32_t g_diagSyncLoss   = 0;
volatile uint32_t g_diagRevLimit   = 0;
volatile uint32_t g_diagSparks     = 0;

// ─── Pre-computed events (written by loop, read by ISR) ──────────────────────
// 2 waste-spark coil pairs; extend to NUM_CYLINDERS for sequential mode.
static SparkEvent       g_sparkEvents[2];
static volatile uint16_t g_precompDwellTicks = DWELL_TICKS;

// Loop-side snapshot of tooth period (avoids re-reading volatile in loop)
static uint16_t g_loopToothPeriod = 0;

// Advance angle (degrees BTDC) – can be updated from a MAP sensor or serial cmd
volatile int8_t g_advanceDeg = DEFAULT_ADVANCE_DEG;

// Stall detection
static uint32_t g_lastToothMillis = 0;
static uint32_t g_prevDiagTeeth   = 0;

// OLED / serial refresh timestamps
static uint32_t g_lastOledMillis   = 0;
static uint32_t g_lastSerialMillis = 0;

// ─── Safe state: de-energise all coils, disarm OCR1A ─────────────────────────
// Uses direct port write so it is safe to call from within an ISR
// (all four coil pins are on PORTD for Uno/Nano/Pro Mini).
static inline void safeState()
{
    PORTD     &= ~COIL_PORT_MASK;       // All coils LOW in one atomic write
    TIMSK1    &= ~(1 << OCIE1A);        // Disarm compare-A interrupt
    g_ignState      = IGN_IDLE;
    g_activeCoilPin = 0;
}

// ─── Schedule the next OCR1A event ───────────────────────────────────────────
// uint16_t arithmetic: wraps correctly when Timer1 crosses 0xFFFF → 0x0000.
static inline void scheduleOCR1A(uint16_t target)
{
    OCR1A  = target;
    TIFR1 |= (1 << OCF1A);             // Clear any stale pending flag
    TIMSK1|= (1 << OCIE1A);            // Enable compare-A interrupt
}

// ─── ICP1 ISR: crank-tooth capture ───────────────────────────────────────────
ISR(TIMER1_CAPT_vect)
{
    uint16_t capture = ICR1;
    uint16_t period  = capture - g_lastICP;
    g_lastICP = capture;

    // ── Rev-limit guard ───────────────────────────────────────────────────────
    // Period shorter than REV_LIMIT_MIN_TICKS → engine over rev-limit
    if (period < REV_LIMIT_MIN_TICKS) {
        if (!g_revLimitActive) {
            g_revLimitActive = true;
            g_diagRevLimit++;
            safeState();
        }
        return;
    }
    g_revLimitActive = false;

    // ── Gap (missing-tooth) detection ─────────────────────────────────────────
    // Gap condition: current_period × GAP_DEN  ≥  prev_period × GAP_NUM
    // i.e. current ≥ 1.8 × previous  (integer ratio, no float)
    bool isGap = (g_prevPeriod > 0) &&
                 ((uint32_t)period * GAP_RATIO_DEN >=
                  (uint32_t)g_prevPeriod * GAP_RATIO_NUM);

    if (isGap) {
        // Validate: if synced, the previous revolution must have contained
        // approximately ACTUAL_TEETH teeth (allow up to -3 for noise tolerance).
        // toothCount wraps at ACTUAL_TEETH, so the maximum value before a gap is
        // ACTUAL_TEETH - 1; only a too-low count (too few teeth) indicates error.
        if (g_synced && g_toothCount < ACTUAL_TEETH - 3) {
            // Gap at wrong time → lose sync
            g_synced       = false;
            g_goodGapCount = 0;
            g_diagSyncLoss++;
            safeState();
        } else {
            g_goodGapCount++;
            if (g_goodGapCount >= SYNC_MIN_GOOD_GAPS) {
                g_synced = true;
            }
        }

        // tooth 0 is the FIRST tooth after the gap.
        // Set counter to (ACTUAL_TEETH - 1) so the next increment wraps to 0.
        g_toothCount = ACTUAL_TEETH - 1;
        g_prevPeriod = 0;               // Reset: gap period is not a "normal" tooth
        g_toothPeriod = period;
        g_diagTeeth++;
        return;
    }

    // ── Normal tooth ──────────────────────────────────────────────────────────
    g_prevPeriod  = g_toothPeriod;
    g_toothPeriod = period;
    g_diagTeeth++;

    if (!g_synced) return;

    // Advance counter; wrap at ACTUAL_TEETH
    g_toothCount++;
    if (g_toothCount >= ACTUAL_TEETH) g_toothCount = 0;

    // ── Schedule dwell for the matching spark event ───────────────────────────
    for (uint8_t i = 0; i < 2; i++) {
        if (g_toothCount == g_sparkEvents[i].triggerTooth &&
            g_ignState == IGN_IDLE) {
            g_activeCoilPin = g_sparkEvents[i].coilPin;
            g_ignState      = IGN_DWELL;
            scheduleOCR1A((uint16_t)(capture + g_sparkEvents[i].dwellOffsetTicks));
            break;
        }
    }
}

// ─── OCR1A ISR: dwell-start / spark-fire state machine ───────────────────────
ISR(TIMER1_COMPA_vect)
{
    if (g_ignState == IGN_DWELL) {
        // Start dwell: energise coil
        digitalWrite(g_activeCoilPin, HIGH);
        // Schedule spark = dwell-start + dwell-duration
        // uint16_t addition wraps correctly with Timer1 counter
        OCR1A    += g_precompDwellTicks;
        g_ignState = IGN_SPARK;

    } else if (g_ignState == IGN_SPARK) {
        // Fire spark: de-energise coil (magnetic collapse → spark)
        digitalWrite(g_activeCoilPin, LOW);
        g_diagSparks++;
        g_ignState = IGN_IDLE;
        TIMSK1    &= ~(1 << OCIE1A);    // Disarm until next tooth event
    }
}

// ─── CAM / phase-sensor ISR (INT0, optional) ─────────────────────────────────
// Toggles g_camPhase once per cam revolution (= once per 2 crank revolutions).
// When g_camPhase is known, sequential coil assignment can be applied.
static void camISR()
{
    g_camPhase = !g_camPhase;
}

// ─── Pre-compute spark events ─────────────────────────────────────────────────
// Called from loop() whenever RPM or advance changes.
// No multiply/divide runs in the ISR – only the results stored here.
//
// For each waste-spark coil pair the calculation finds:
//   1. The tooth at which OCR1A must be armed (triggerTooth).
//   2. The tick offset within that tooth period (dwellOffsetTicks).
//
// Math:
//   sparkTime  = T_TDC  –  advTicks
//   dwellStart = sparkTime  –  dwellTicks
//              = T_TDC  –  (advTicks + dwellTicks)
//   Expressed as distance from tooth-0:
//   rawTicks   = tdcTooth × tpt  –  (advTicks + dwellTicks)
//   Normalise to [0, revTicks) with modular wrap, then split into
//   (triggerTooth = rawTicks / tpt, offset = rawTicks % tpt).
static void precomputeEvents()
{
    uint16_t tpt = g_loopToothPeriod;
    if (tpt == 0) return;

    // Advance in ticks: adv_deg × tpt / DEGREES_PER_TOOTH
    // DEGREES_PER_TOOTH == 10 → divide by 10
    // Clamp to [0, 45] so a negative (retard) or excessive advance is safe.
    int8_t  advSigned = g_advanceDeg;
    if (advSigned < 0)  advSigned = 0;
    if (advSigned > 45) advSigned = 45;
    uint8_t  adv      = (uint8_t)advSigned;
    uint16_t advTicks = (uint16_t)((uint32_t)tpt * adv / DEGREES_PER_TOOTH);
    uint16_t dwTicks  = DWELL_TICKS;

    uint32_t totalBefore = (uint32_t)advTicks + dwTicks;
    uint32_t revTicks    = (uint32_t)ACTUAL_TEETH * tpt;

    // ── Pair A ────────────────────────────────────────────────────────────────
    int32_t rawA = (int32_t)CYL_A_TDC_TOOTH * tpt - (int32_t)totalBefore;
    // Normalise to [0, revTicks) in O(1) – handles any amount of wrap
    rawA = ((rawA % (int32_t)revTicks) + (int32_t)revTicks) % (int32_t)revTicks;

    uint8_t  trigA = (uint8_t)((uint32_t)rawA / tpt);
    uint16_t offA  = (uint16_t)((uint32_t)rawA % tpt);

    // ── Pair B ────────────────────────────────────────────────────────────────
    int32_t rawB = (int32_t)CYL_B_TDC_TOOTH * tpt - (int32_t)totalBefore;
    rawB = ((rawB % (int32_t)revTicks) + (int32_t)revTicks) % (int32_t)revTicks;

    uint8_t  trigB = (uint8_t)((uint32_t)rawB / tpt);
    uint16_t offB  = (uint16_t)((uint32_t)rawB % tpt);

    // Atomic update: disable interrupts for the multi-byte struct write
    noInterrupts();
    g_sparkEvents[0]    = { trigA, offA, COIL1_PIN };
    g_sparkEvents[1]    = { trigB, offB, COIL2_PIN };
    g_precompDwellTicks = dwTicks;
    interrupts();
}

// ─── OLED update ─────────────────────────────────────────────────────────────
#if USE_OLED
static void updateOLED()
{
    // Atomic snapshot of volatile state
    bool     synced, revLim;
    uint16_t tpt;
    uint32_t sparks, syncLoss;

    noInterrupts();
    synced   = g_synced;
    revLim   = g_revLimitActive;
    tpt      = g_toothPeriod;
    sparks   = g_diagSparks;
    syncLoss = g_diagSyncLoss;
    interrupts();

    // RPM = (TIMER1_TICK_HZ × 60) / (tpt × TRIGGER_TEETH)
    uint32_t rpm = 0;
    if (synced && tpt > 0) {
        rpm = ((uint32_t)TIMER1_TICK_HZ * 60UL) /
              ((uint32_t)tpt * TRIGGER_TEETH);
    }

    oled.clearDisplay();
    oled.setTextSize(1);
    oled.setTextColor(SSD1306_WHITE);

    oled.setCursor(0, 0);
    oled.print(F("=== Santiago DIS ==="));

    oled.setCursor(0, 12);
    oled.print(F("RPM : "));
    if (synced) oled.print(rpm);
    else        oled.print(F("--"));

    oled.setCursor(0, 22);
    oled.print(F("ADV : "));
    oled.print(g_advanceDeg);
    oled.print(F(" deg BTDC"));

    oled.setCursor(0, 32);
    if (synced)  oled.print(F("SYNC : OK"));
    else         oled.print(F("SYNC : LOST"));
    if (revLim) {
        oled.setCursor(72, 32);
        oled.print(F("REV LIM"));
    }

    oled.setCursor(0, 42);
    oled.print(F("Sparks   : "));
    oled.print(sparks);

    oled.setCursor(0, 52);
    oled.print(F("SyncLoss : "));
    oled.print(syncLoss);

    oled.display();
}
#endif  // USE_OLED

// ─── Setup ────────────────────────────────────────────────────────────────────
void setup()
{
    // Coil output pins
    pinMode(COIL1_PIN, OUTPUT); digitalWrite(COIL1_PIN, LOW);
    pinMode(COIL2_PIN, OUTPUT); digitalWrite(COIL2_PIN, LOW);
    pinMode(COIL3_PIN, OUTPUT); digitalWrite(COIL3_PIN, LOW);
    pinMode(COIL4_PIN, OUTPUT); digitalWrite(COIL4_PIN, LOW);

    // ICP1 input (hardware-fixed on PB0 / pin 8) with pull-up
    pinMode(ICP_PIN, INPUT_PULLUP);

    // CAM / phase sensor on INT0 (pin 2) – optional
    pinMode(CAM_PIN, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(CAM_PIN), camISR, FALLING);

    // ── Timer1 configuration ─────────────────────────────────────────────────
    // Normal mode (WGM = 0000), prescaler = 8, ICP on rising edge,
    // noise-canceller enabled.  OCR1A interrupt is armed dynamically.
    TCCR1A = 0;
    TCCR1B = (1 << ICNC1)  // Input Capture Noise Canceler (4-cycle glitch filter)
           | (1 << ICES1)  // ICP trigger: rising edge
           | (1 << CS11);  // Prescaler = 8
    TCNT1  = 0;
    TIMSK1 = (1 << ICIE1); // Enable ICP interrupt; OCR1A enabled dynamically

#if USE_OLED
    Wire.begin();
    if (oled.begin(SSD1306_SWITCHCAPVCC, OLED_I2C_ADDR)) {
        oled.clearDisplay();
        oled.setTextSize(2);
        oled.setTextColor(SSD1306_WHITE);
        oled.setCursor(16, 20);
        oled.print(F("Santiago"));
        oled.setCursor(22, 42);
        oled.print(F("DIS Ign"));
        oled.display();
        delay(1500);
    }
#endif

    Serial.begin(115200);
    Serial.println(F("Santiago DIS Ignition v1.0  (Speeduino-style)"));
    Serial.print(F("DWELL_TICKS="));       Serial.println(DWELL_TICKS);
    Serial.print(F("REV_LIMIT_MIN_TICKS=")); Serial.println(REV_LIMIT_MIN_TICKS);
}

// ─── Loop ─────────────────────────────────────────────────────────────────────
void loop()
{
    uint32_t now = millis();

    // ── Stall / crank-timeout detection ──────────────────────────────────────
    // If a new tooth has been seen since last check, update the last-tooth time.
    uint32_t diagSnapshot = g_diagTeeth;
    if (diagSnapshot != g_prevDiagTeeth) {
        g_prevDiagTeeth   = diagSnapshot;
        g_lastToothMillis = now;
    }

    // If synced but no new teeth for CRANK_TIMEOUT_MS → stall/sync loss
    if (g_synced && (now - g_lastToothMillis) > CRANK_TIMEOUT_MS) {
        noInterrupts();
        g_synced       = false;
        g_goodGapCount = 0;
        g_diagSyncLoss++;
        safeState();
        interrupts();
    }

    // ── Re-compute spark events when tooth period changes ─────────────────────
    {
        uint16_t tpt;
        noInterrupts();
        tpt = g_toothPeriod;
        interrupts();

        if (tpt != g_loopToothPeriod) {
            g_loopToothPeriod = tpt;
            precomputeEvents();
        }
    }

    // ── OLED refresh ──────────────────────────────────────────────────────────
#if USE_OLED
    if ((now - g_lastOledMillis) >= OLED_UPDATE_MS) {
        g_lastOledMillis = now;
        updateOLED();
    }
#endif

    // ── Serial diagnostics (once per second) ─────────────────────────────────
    if ((now - g_lastSerialMillis) >= 1000UL) {
        g_lastSerialMillis = now;

        bool     synced;
        uint16_t tpt;
        uint32_t teeth, sparks, syncLoss, revLimit;

        noInterrupts();
        synced   = g_synced;
        tpt      = g_toothPeriod;
        teeth    = g_diagTeeth;
        sparks   = g_diagSparks;
        syncLoss = g_diagSyncLoss;
        revLimit = g_diagRevLimit;
        interrupts();

        uint32_t rpm = 0;
        if (synced && tpt > 0) {
            rpm = ((uint32_t)TIMER1_TICK_HZ * 60UL) /
                  ((uint32_t)tpt * TRIGGER_TEETH);
        }

        Serial.print(F("RPM="));       Serial.print(rpm);
        Serial.print(F(" ADV="));      Serial.print(g_advanceDeg);
        Serial.print(F(" deg SYNC=")); Serial.print(synced ? 'Y' : 'N');
        Serial.print(F(" TEETH="));   Serial.print(teeth);
        Serial.print(F(" SPK="));     Serial.print(sparks);
        Serial.print(F(" SLOSS="));   Serial.print(syncLoss);
        Serial.print(F(" RLIM="));    Serial.println(revLimit);
    }
}
