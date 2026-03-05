/**
 * @file    DIS_Ignition.ino
 * @brief   DIS (Direct Ignition System) – Speeduino-style spark scheduler
 *          targeting Arduino Mega 2560.
 *
 * Architecture summary
 * ====================
 *  - Timer1 Input Capture (ICP1 / PB4, pin 49 on Mega) captures every rising
 *    edge of the crank-reference trigger wheel tooth. From consecutive
 *    captures the period (and therefore RPM) is computed inside the ISR with
 *    only integer arithmetic.
 *
 *  - Timer1 Output Compare A (OCR1A) fires the next coil event. Before
 *    writing to OCR1A the loop() precomputes the absolute Timer1 tick at
 *    which each coil should charge (dwell start) and fire (spark). These
 *    precomputed values are posted to per-coil event queues that the OCR1A
 *    ISR drains.
 *
 *  - Wasted-spark is implemented with COIL_COUNT independent coils. Each
 *    coil has a two-slot queue: [0] = charge event, [1] = fire event.
 *
 *  - Signed-integer delta arithmetic (int32_t) is used throughout so that
 *    timer wrap-around (16-bit overflow of Timer1) never causes missed or
 *    spurious events.
 *
 *  - Safe mode: if no reference pulse arrives within SIGNAL_TIMEOUT_MS the
 *    system shuts all coils off and raises a fault flag. Rev-limit also shuts
 *    all coils off immediately.
 *
 *  - No Serial writes or I/O calls are made inside any ISR.
 *    OLED display and RPM telemetry are refreshed only from loop().
 *
 * Hardware connections (Arduino Mega 2560)
 * =========================================
 *  ICP1 (crank trigger)  → Pin 49 (PB4 / ICP1)
 *  Coil 1 (low-side)     → Pin 2
 *  Coil 2 (low-side)     → Pin 3
 *  Coil 3 (low-side)     → Pin 4
 *  Coil 4 (low-side)     → Pin 5
 *  SDA (OLED)            → Pin 20
 *  SCL (OLED)            → Pin 21
 *
 * Timer1 configuration
 * ====================
 *  Prescaler  : 8  → tick = 0.5 µs @ 16 MHz
 *  Mode       : Normal (free-running 16-bit counter, TOP = 0xFFFF)
 *  ICP edge   : Rising
 *  Noise canceler: enabled
 */

#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// ---------------------------------------------------------------------------
// Hardware / timing constants
// ---------------------------------------------------------------------------
static const uint8_t  COIL_COUNT         = 4;
static const uint8_t  COIL_PINS[COIL_COUNT] = {2, 3, 4, 5};

// Timer1 prescaler = 8 → 0.5 µs per tick at 16 MHz
static const uint32_t TICKS_PER_US       = 2UL;          // ticks per microsecond
static const uint32_t TICKS_PER_MS       = 2000UL;       // ticks per millisecond

// Ignition parameters (all in Timer1 ticks)
static const uint32_t DWELL_TICKS        = 3UL * TICKS_PER_MS;  // 3 ms dwell
static const int16_t  FIXED_ADVANCE_DEG  = 10;           // degrees BTDC (constant map)
static const uint16_t RPM_MIN            = 50;
static const uint16_t RPM_MAX            = 8000;          // rev-limit
static const uint32_t SIGNAL_TIMEOUT_MS  = 500UL;         // 0.5 s without ref pulse → fault

// Degrees per tooth for a 60-2 wheel (360°/58 teeth)
// expressed as a fraction to avoid floating-point: TOOTH_DEG_NUM/TOOTH_DEG_DEN
static const uint16_t TOOTH_DEG_NUM      = 360;
static const uint16_t TOOTH_DEG_DEN      = 58;

// OLED
static const uint8_t  OLED_WIDTH         = 128;
static const uint8_t  OLED_HEIGHT        = 64;
static const int8_t   OLED_RESET         = -1;

// ---------------------------------------------------------------------------
// Coil event queue
// Each coil holds exactly two pending events: [0]=charge, [1]=fire.
// ---------------------------------------------------------------------------
struct CoilEvent {
    uint16_t tick;   // absolute Timer1 tick (16-bit, wraps at 0xFFFF)
    bool     active; // true = event is pending
};

struct CoilQueue {
    CoilEvent charge; // dwell start (coil ON)
    CoilEvent fire;   // spark       (coil OFF)
};

// ---------------------------------------------------------------------------
// Shared state (written by ISRs, read by loop – all volatile)
// ---------------------------------------------------------------------------
static volatile uint16_t  g_lastIcpTick      = 0;
static volatile uint16_t  g_toothPeriodTicks = 0;   // Timer1 ticks between last two teeth
static volatile uint32_t  g_lastIcpMillis    = 0;   // millis() at last ICP (for timeout)
static volatile uint16_t  g_rpm              = 0;
static volatile bool      g_faultNoSignal    = true;
static volatile bool      g_faultRevLimit    = false;
static volatile CoilQueue g_coilQueue[COIL_COUNT];

// ---------------------------------------------------------------------------
// Next scheduled OCR1A tick (written only from loop or ISR that owns OCR1A)
// ---------------------------------------------------------------------------
static volatile uint16_t  g_nextOcrTick      = 0;
static volatile uint8_t   g_nextCoilIndex    = 0;   // which coil the next event belongs to
static volatile bool      g_nextEventIsCharge= true; // true=charge, false=fire

// ---------------------------------------------------------------------------
// OLED instance
// ---------------------------------------------------------------------------
static Adafruit_SSD1306 display(OLED_WIDTH, OLED_HEIGHT, &Wire, OLED_RESET);

// ---------------------------------------------------------------------------
// Forward declarations
// ---------------------------------------------------------------------------
static void     coilsOff();
static void     scheduleNextEvent();
static uint16_t degreesToTicks(int16_t deg, uint16_t periodTicks);
static uint16_t rpm_from_period(uint16_t periodTicks);
static void     updateOled(uint16_t rpm, bool faultSig, bool faultRev);

// ===========================================================================
// setup()
// ===========================================================================
void setup() {
    // Coil output pins (active HIGH drives low-side MOSFET)
    for (uint8_t i = 0; i < COIL_COUNT; i++) {
        pinMode(COIL_PINS[i], OUTPUT);
        digitalWrite(COIL_PINS[i], LOW);
    }

    // Initialise coil queues
    for (uint8_t i = 0; i < COIL_COUNT; i++) {
        g_coilQueue[i].charge.active = false;
        g_coilQueue[i].fire.active   = false;
    }

    // OLED initialisation (failure is non-fatal)
    if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
        // OLED not found – continue without display
    } else {
        display.clearDisplay();
        display.setTextSize(1);
        display.setTextColor(SSD1306_WHITE);
        display.setCursor(0, 0);
        display.println(F("DIS Ignition"));
        display.println(F("Waiting for sync..."));
        display.display();
    }

    // ------------------------------------------------------------------
    // Timer1 configuration
    //   WGM13:0 = 0000  → Normal mode (free-running, TOP = 0xFFFF)
    //   CS12:0  = 010   → Prescaler = 8 (0.5 µs / tick @ 16 MHz)
    //   ICES1   = 1     → Capture on rising edge
    //   ICNC1   = 1     → Noise canceler ON (4 cycles ≈ 250 ns)
    // ------------------------------------------------------------------
    TCCR1A = 0x00;                     // Normal mode, no PWM outputs
    TCCR1B = _BV(ICNC1)               // Noise canceler
           | _BV(ICES1)               // Rising-edge capture
           | _BV(CS11);               // Prescaler = 8
    TCCR1C = 0x00;

    TCNT1  = 0;

    // Enable Input Capture interrupt; Output Compare A interrupt will be
    // enabled once the first valid period is available.
    TIMSK1 = _BV(ICIE1);

    sei();
}

// ===========================================================================
// loop()
// ===========================================================================
void loop() {
    static uint32_t lastOledMs = 0;

    // ------------------------------------------------------------------
    // 1. Check for signal timeout → fault
    // ------------------------------------------------------------------
    uint32_t nowMs = millis();
    {
        // Read shared state atomically
        uint8_t sreg = SREG;
        cli();
        uint32_t lastIcpMs = g_lastIcpMillis;
        SREG = sreg;

        if ((nowMs - lastIcpMs) > SIGNAL_TIMEOUT_MS) {
            // No reference pulse for too long → fault
            uint8_t s2 = SREG; cli();
            g_faultNoSignal = true;
            SREG = s2;
            coilsOff();
        }
    }

    // ------------------------------------------------------------------
    // 2. Read stable copies of shared state
    // ------------------------------------------------------------------
    uint8_t  sreg = SREG;
    cli();
    uint16_t periodTicks  = g_toothPeriodTicks;
    uint16_t lastIcpTick  = g_lastIcpTick;
    uint16_t localRpm     = g_rpm;
    bool     faultSig     = g_faultNoSignal;
    bool     faultRev     = g_faultRevLimit;
    SREG = sreg;

    // ------------------------------------------------------------------
    // 3. Check rev-limit
    // ------------------------------------------------------------------
    if (localRpm >= RPM_MAX) {
        uint8_t s3 = SREG; cli();
        g_faultRevLimit = true;
        SREG = s3;
        coilsOff();
        faultRev = true;
    } else {
        uint8_t s3 = SREG; cli();
        g_faultRevLimit = false;
        SREG = s3;
        faultRev = false;
    }

    // ------------------------------------------------------------------
    // 4. Precompute coil events when signal is healthy
    // ------------------------------------------------------------------
    if (!faultSig && !faultRev && periodTicks > 0 && localRpm >= RPM_MIN) {
        // Advance in Timer1 ticks before TDC
        uint16_t advanceTicks = degreesToTicks(FIXED_ADVANCE_DEG, periodTicks);

        // Distribute events across COIL_COUNT cylinders (wasted spark pairs)
        // Each pair fires every 360°/COIL_COUNT of crank rotation.
        //
        // For a 4-cylinder 4-stroke engine with wasted spark:
        //   Coil 0 → cylinders 1 & 4 (0° and 360° offset)
        //   Coil 1 → cylinders 2 & 3 (180° and 540°)
        //   ...
        // The reference tooth is at a known position; here we simplify by
        // computing the fire tick for coil i as:
        //   fireTick = lastIcpTick + offset_ticks[i] - advanceTicks
        // where offset_ticks[i] = degreesToTicks(360/COIL_COUNT * i, periodTicks)
        // = i * (360/COIL_COUNT) * periodTicks * TOOTH_DEG_DEN / TOOTH_DEG_NUM

        for (uint8_t i = 0; i < COIL_COUNT; i++) {
            // Precompute ticks with interrupts enabled (expensive arithmetic
            // must not hold off the ICP ISR for an entire crank tooth period).
            uint16_t degOffset  = (360U / COIL_COUNT) * i;
            // degreesToTicks correctly applies * TOOTH_DEG_DEN / TOOTH_DEG_NUM
            uint16_t offset     = degreesToTicks((int16_t)degOffset, periodTicks);

            uint16_t fireTick   = lastIcpTick + offset - advanceTicks;
            uint16_t chargeTick = (uint16_t)(fireTick - (uint16_t)DWELL_TICKS);

            // Atomically post to queue; interrupts disabled only for this
            // single coil's check-and-write to keep latency minimal.
            uint8_t s4 = SREG; cli();
            if (!g_coilQueue[i].charge.active && !g_coilQueue[i].fire.active) {
                g_coilQueue[i].charge.tick   = chargeTick;
                g_coilQueue[i].charge.active = true;
                g_coilQueue[i].fire.tick     = fireTick;
                g_coilQueue[i].fire.active   = true;
            }
            SREG = s4;
        }

        // Schedule the soonest pending event in OCR1A (must be atomic)
        uint8_t s5 = SREG; cli();
        scheduleNextEvent();
        SREG = s5;
    }

    // ------------------------------------------------------------------
    // 5. Refresh OLED at ~10 Hz
    // ------------------------------------------------------------------
    if ((nowMs - lastOledMs) >= 100UL) {
        lastOledMs = nowMs;
        updateOled(localRpm, faultSig, faultRev);
    }
}

// ===========================================================================
// ISR – Timer1 Input Capture (crank reference pulse)
// ===========================================================================
ISR(TIMER1_CAPT_vect) {
    uint16_t icpTick = ICR1;  // latched by hardware at capture moment

    uint16_t period = icpTick - g_lastIcpTick;  // wraps correctly (16-bit unsigned)
    g_lastIcpTick   = icpTick;

    // Reject obviously wrong periods (noise) – min 250 RPM = ~4000 ticks
    if (period < 200U) {
        return;
    }

    g_toothPeriodTicks = period;
    g_lastIcpMillis    = millis();  // millis() is safe to call from ISR on AVR
    g_faultNoSignal    = false;

    // Compute RPM: RPM = 60e6 / (period_us * teeth_per_rev)
    // period_us = period / TICKS_PER_US
    // teeth_per_rev = TOOTH_DEG_DEN (= 58 for 60-2 wheel)
    // RPM = 60e6 * TICKS_PER_US / (period * 58)
    uint32_t rpm_calc = (60000000UL * TICKS_PER_US) / ((uint32_t)period * TOOTH_DEG_DEN);
    if (rpm_calc > 65535UL) rpm_calc = 65535UL;
    g_rpm = (uint16_t)rpm_calc;
}

// ===========================================================================
// ISR – Timer1 Output Compare A (coil event dispatcher)
// ===========================================================================
ISR(TIMER1_COMPA_vect) {
    uint8_t  ci    = g_nextCoilIndex;
    bool     isChg = g_nextEventIsCharge;

    if (isChg) {
        // Charge event: switch coil ON (start dwell)
        if (ci < COIL_COUNT) {
            digitalWrite(COIL_PINS[ci], HIGH);
        }
        g_coilQueue[ci].charge.active = false;
    } else {
        // Fire event: switch coil OFF (spark discharge)
        if (ci < COIL_COUNT) {
            digitalWrite(COIL_PINS[ci], LOW);
        }
        g_coilQueue[ci].fire.active = false;
    }

    // Arm the next pending event (still inside ISR, re-arms OCR1A)
    scheduleNextEvent();
}

// ===========================================================================
// Helper – turn all coils off immediately (safe mode).
// May be called from loop() or from within an already-cli()'d context.
// ===========================================================================
static void coilsOff() {
    uint8_t sreg = SREG;
    cli();
    // Disable OCR1A interrupt first so no further coil events fire
    TIMSK1 &= ~_BV(OCIE1A);
    // Flush all pending events
    for (uint8_t i = 0; i < COIL_COUNT; i++) {
        g_coilQueue[i].charge.active = false;
        g_coilQueue[i].fire.active   = false;
    }
    SREG = sreg;
    // Drive all coil outputs LOW (safe – no current through coil primaries)
    for (uint8_t i = 0; i < COIL_COUNT; i++) {
        digitalWrite(COIL_PINS[i], LOW);
    }
}

// ===========================================================================
// Helper – scan queues and load the soonest upcoming event into OCR1A.
// Must be called with interrupts disabled (or from within an ISR).
// ===========================================================================
static void scheduleNextEvent() {
    uint16_t now    = TCNT1;
    bool     found  = false;
    int32_t  best   = INT32_MAX;
    uint8_t  bestCi = 0;
    bool     bestChg = true;

    for (uint8_t i = 0; i < COIL_COUNT; i++) {
        if (g_coilQueue[i].charge.active) {
            // Cast via int16_t so the 16-bit unsigned subtraction wraps to a
            // signed value in [-32768, +32767], giving correct past/future sense.
            int32_t delta = (int32_t)(int16_t)(g_coilQueue[i].charge.tick - now);
            if (delta < best) {
                best    = delta;
                bestCi  = i;
                bestChg = true;
                found   = true;
            }
        }
        if (g_coilQueue[i].fire.active) {
            int32_t delta = (int32_t)(int16_t)(g_coilQueue[i].fire.tick - now);
            if (delta < best) {
                best    = delta;
                bestCi  = i;
                bestChg = false;
                found   = true;
            }
        }
    }

    if (found) {
        g_nextCoilIndex     = bestCi;
        g_nextEventIsCharge = bestChg;
        g_nextOcrTick       = bestChg ? g_coilQueue[bestCi].charge.tick
                                      : g_coilQueue[bestCi].fire.tick;
        OCR1A  = g_nextOcrTick;
        TIFR1  = _BV(OCF1A);   // clear any stale flag before enabling
        TIMSK1 |= _BV(OCIE1A); // enable Output Compare A interrupt
    } else {
        // Nothing pending – disable OCR1A interrupt
        TIMSK1 &= ~_BV(OCIE1A);
    }
}

// ===========================================================================
// Helper – convert degrees of crank rotation to Timer1 ticks.
// Uses integer arithmetic; periodTicks is ticks per tooth.
// One tooth = TOOTH_DEG_NUM / TOOTH_DEG_DEN degrees.
// ===========================================================================
static uint16_t degreesToTicks(int16_t deg, uint16_t periodTicks) {
    // ticks_per_degree = periodTicks * TOOTH_DEG_DEN / TOOTH_DEG_NUM
    // ticks = deg * ticks_per_degree
    // Use uint64_t intermediate to avoid overflow at extreme values.
    uint64_t result = ((uint64_t)abs(deg) * (uint64_t)periodTicks
                       * TOOTH_DEG_DEN) / TOOTH_DEG_NUM;
    if (result > 0xFFFFUL) result = 0xFFFFUL;
    return (uint16_t)result;
}

// ===========================================================================
// Helper – compute RPM from a tooth period in Timer1 ticks.
// (Mirrors the ISR calculation; available for use in loop() or test harness.)
// ===========================================================================
static uint16_t __attribute__((unused)) rpm_from_period(uint16_t periodTicks) {
    if (periodTicks == 0) return 0;
    uint32_t r = (60000000UL * TICKS_PER_US) / ((uint32_t)periodTicks * TOOTH_DEG_DEN);
    if (r > 65535UL) r = 65535UL;
    return (uint16_t)r;
}

// ===========================================================================
// Helper – update OLED display (called only from loop(), never from ISR)
// ===========================================================================
static void updateOled(uint16_t rpm, bool faultSig, bool faultRev) {
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0, 0);
    display.print(F("DIS Ignition v1.0"));

    display.setCursor(0, 16);
    display.print(F("RPM: "));
    display.print(rpm);

    display.setCursor(0, 32);
    if (faultSig) {
        display.setTextColor(SSD1306_BLACK, SSD1306_WHITE); // inverted
        display.print(F("FAULT: No Signal"));
        display.setTextColor(SSD1306_WHITE);
    } else if (faultRev) {
        display.setTextColor(SSD1306_BLACK, SSD1306_WHITE);
        display.print(F("FAULT: Rev Limit"));
        display.setTextColor(SSD1306_WHITE);
    } else {
        display.print(F("Status: OK"));
    }

    display.setCursor(0, 48);
    display.print(F("Coils: "));
    for (uint8_t i = 0; i < COIL_COUNT; i++) {
        display.print(digitalRead(COIL_PINS[i]) ? '1' : '0');
    }

    display.display();
}
