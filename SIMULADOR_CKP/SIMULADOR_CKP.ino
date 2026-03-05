/*
 * SIMULADOR_CKP.ino
 *
 * CKP (Crankshaft Position Sensor) 60-2 Hall Simulator for Arduino Nano
 * Target : ATmega328P @ 16 MHz
 * Author : Santiago Vivas
 *
 * Architecture overview
 * ─────────────────────
 *  • PATRON_CKP[120]  – 60-tooth-×-2-half-period signal map stored in PROGMEM.
 *    Teeth 0-57 present → [1,0] (HIGH first half, LOW second half).
 *    Teeth 58-59 missing → [0,0] (gap, always LOW).
 *
 *  • Timer1 (16-bit, CTC)
 *    ISR(TIMER1_COMPA_vect) fires at (RPM×120/60) Hz, steps through
 *    PATRON_CKP and drives D10 HIGH or LOW accordingly.
 *    updateTimer()  selects the optimal prescaler/OCR1A pair with hysteresis.
 *    stopTimer1()   halts the timer and clears g_lastOCR / g_lastPrescaler → 0.
 *
 *  • Timer2 (8-bit, CTC)
 *    setupTimer2ForAdcTrigger() configures ~1 kHz COMPA interrupt.
 *    ISR(TIMER2_COMPA_vect) fires ADSC to start one ADC conversion.
 *
 *  • ADC
 *    setupADC() – AVcc reference, channel A0, interrupt enabled.
 *    ISR(ADC_vect) collects samples into g_adcWindow[].
 *    Uninitialized slots hold ADC_SENTINEL (0xFFFF); g_adcCount tracks
 *    how many valid readings have been written this cycle.
 *    g_adcUpdated is set only when the window is fully populated.
 *
 *  • loop()
 *    Atomically snapshots g_adcWindow when g_adcUpdated is set.
 *    Validates all slots (none may equal ADC_SENTINEL) before calling
 *    compute_trimmed_mean_window().
 *    Integer EMA smooths the RPM setpoint → updateTimer().
 *    LCD refreshed at ~4 Hz.
 *
 * Pin assignments
 * ───────────────
 *  D10  CKP signal output  (PB2 – driven directly via PORT register in ISR)
 *  A0   Analog RPM setpoint (0 V → RPM_MIN, 5 V → RPM_MAX)
 *  D8   LCD RS
 *  D9   LCD EN   (OC1A left disconnected: TCCR1A = 0)
 *  D7   LCD D4
 *  D6   LCD D5
 *  D5   LCD D6
 *  D4   LCD D7
 */

#include <Arduino.h>
#include <LiquidCrystal.h>
#include <avr/pgmspace.h>
#include <avr/interrupt.h>
#include <util/atomic.h>

// ─── Compile-time configuration ───────────────────────────────────────────────

#define F_CPU_HZ         16000000UL  ///< Target oscillator frequency

#define PATRON_SIZE      120U        ///< Entries in PATRON_CKP (60 teeth × 2 half-periods)
#define NUM_TEETH        60U         ///< Nominal tooth count (60-2 wheel)
#define NUM_MISSING      2U          ///< Number of missing teeth
#define NUM_PRESENT      (NUM_TEETH - NUM_MISSING)  ///< Actual teeth (58)

#define ADC_WINDOW_SIZE  16U         ///< ADC sample-window depth
#define ADC_SENTINEL     0xFFFFU     ///< Marks an uninitialized ADC window slot
                                     ///< (> 10-bit max of 1023, never a real reading)

#define CKP_PIN          10          ///< Digital output for CKP signal (PB2)
#define ADC_INPUT_CHAN   0           ///< ADC mux channel (A0)

#define RPM_MIN          100U        ///< Minimum simulated RPM
#define RPM_MAX          9000U       ///< Maximum simulated RPM
#define RPM_DEFAULT      600U        ///< Startup RPM

/** EMA alpha expressed as a fraction: alpha = EMA_ALPHA_NUM / EMA_ALPHA_DEN */
#define EMA_ALPHA_NUM    1U
#define EMA_ALPHA_DEN    8U

#define ADC_TRIG_HZ      1000UL      ///< Desired ADC trigger frequency [Hz]

#define LCD_UPDATE_MS    250UL       ///< LCD refresh period [ms]  (~4 Hz)

// ─── Static assertions ────────────────────────────────────────────────────────

static_assert(PATRON_SIZE   == 120,
              "PATRON_CKP must have exactly 120 entries (60 teeth x 2 half-periods)");
static_assert(ADC_WINDOW_SIZE >= 4,
              "ADC window must be at least 4 for trimmed-mean (drop 1 + 1)");
static_assert(ADC_SENTINEL  > 1023U,
              "ADC_SENTINEL must not be a valid 10-bit ADC value (0..1023)");
static_assert(RPM_MIN < RPM_MAX,
              "RPM_MIN must be strictly less than RPM_MAX");
static_assert(EMA_ALPHA_NUM > 0 && EMA_ALPHA_NUM < EMA_ALPHA_DEN,
              "EMA alpha must be in the open interval (0, 1)");
static_assert(ADC_TRIG_HZ > 0,
              "ADC trigger frequency must be positive");

// ─── 60-2 CKP pattern in PROGMEM ─────────────────────────────────────────────
/*
 * Layout (index i = tooth × 2 + half):
 *   PATRON_CKP[2*t  ] = 1 if tooth t is present, 0 if missing  (first  half-period)
 *   PATRON_CKP[2*t+1] = 0  always                              (second half-period)
 *
 * Teeth 0-57  (present) → [1, 0]
 * Teeth 58-59 (missing) → [0, 0]
 *
 * Expected counts: 58 × 1 = 58 HIGH entries, 62 LOW entries, total 120.
 */
static const uint8_t PATRON_CKP[PATRON_SIZE] PROGMEM = {
    /* tooth  0 */ 1, 0,
    /* tooth  1 */ 1, 0,
    /* tooth  2 */ 1, 0,
    /* tooth  3 */ 1, 0,
    /* tooth  4 */ 1, 0,
    /* tooth  5 */ 1, 0,
    /* tooth  6 */ 1, 0,
    /* tooth  7 */ 1, 0,
    /* tooth  8 */ 1, 0,
    /* tooth  9 */ 1, 0,
    /* tooth 10 */ 1, 0,
    /* tooth 11 */ 1, 0,
    /* tooth 12 */ 1, 0,
    /* tooth 13 */ 1, 0,
    /* tooth 14 */ 1, 0,
    /* tooth 15 */ 1, 0,
    /* tooth 16 */ 1, 0,
    /* tooth 17 */ 1, 0,
    /* tooth 18 */ 1, 0,
    /* tooth 19 */ 1, 0,
    /* tooth 20 */ 1, 0,
    /* tooth 21 */ 1, 0,
    /* tooth 22 */ 1, 0,
    /* tooth 23 */ 1, 0,
    /* tooth 24 */ 1, 0,
    /* tooth 25 */ 1, 0,
    /* tooth 26 */ 1, 0,
    /* tooth 27 */ 1, 0,
    /* tooth 28 */ 1, 0,
    /* tooth 29 */ 1, 0,
    /* tooth 30 */ 1, 0,
    /* tooth 31 */ 1, 0,
    /* tooth 32 */ 1, 0,
    /* tooth 33 */ 1, 0,
    /* tooth 34 */ 1, 0,
    /* tooth 35 */ 1, 0,
    /* tooth 36 */ 1, 0,
    /* tooth 37 */ 1, 0,
    /* tooth 38 */ 1, 0,
    /* tooth 39 */ 1, 0,
    /* tooth 40 */ 1, 0,
    /* tooth 41 */ 1, 0,
    /* tooth 42 */ 1, 0,
    /* tooth 43 */ 1, 0,
    /* tooth 44 */ 1, 0,
    /* tooth 45 */ 1, 0,
    /* tooth 46 */ 1, 0,
    /* tooth 47 */ 1, 0,
    /* tooth 48 */ 1, 0,
    /* tooth 49 */ 1, 0,
    /* tooth 50 */ 1, 0,
    /* tooth 51 */ 1, 0,
    /* tooth 52 */ 1, 0,
    /* tooth 53 */ 1, 0,
    /* tooth 54 */ 1, 0,
    /* tooth 55 */ 1, 0,
    /* tooth 56 */ 1, 0,
    /* tooth 57 */ 1, 0,   /* last present tooth */
    /* tooth 58 */ 0, 0,   /* MISSING – gap start */
    /* tooth 59 */ 0, 0    /* MISSING – gap end   */
};

// ─── Timer1 prescaler tables ──────────────────────────────────────────────────

/** Prescaler divisor values for Timer1 (in ascending order). */
static const uint16_t T1_PRESCALERS[] PROGMEM = { 1, 8, 64, 256, 1024 };

/**
 * CS1[2:0] bit patterns for each Timer1 prescaler entry.
 * Applied to the lower 3 bits of TCCR1B.
 */
static const uint8_t T1_CS_BITS[] PROGMEM = {
    (1 << CS10),                       /* /1    */
    (1 << CS11),                       /* /8    */
    (1 << CS11) | (1 << CS10),         /* /64   */
    (1 << CS12),                       /* /256  */
    (1 << CS12) | (1 << CS10)          /* /1024 */
};

#define T1_PRESC_COUNT  5U

// ─── Timer2 prescaler tables ──────────────────────────────────────────────────

static const uint16_t T2_PRESCALERS[] PROGMEM = { 1, 8, 32, 64, 128, 256, 1024 };
static const uint8_t  T2_CS_BITS[] PROGMEM = {
    (1 << CS20),                                  /* /1    */
    (1 << CS21),                                  /* /8    */
    (1 << CS21) | (1 << CS20),                    /* /32   */
    (1 << CS22),                                  /* /64   */
    (1 << CS22) | (1 << CS20),                    /* /128  */
    (1 << CS22) | (1 << CS21),                    /* /256  */
    (1 << CS22) | (1 << CS21) | (1 << CS20)       /* /1024 */
};

#define T2_PRESC_COUNT  7U

// ─── ISR-shared volatile state ────────────────────────────────────────────────

/** Current index into PATRON_CKP; written only by TIMER1_COMPA_vect. */
static volatile uint8_t  g_patronIdx    = 0;

/**
 * Timer1 parameters reflecting the *running* configuration.
 * Both are cleared to 0 when the timer is stopped; get_generated_freqHz_atomic()
 * returns 0 in that case because a prescaler value of 0 is never valid.
 */
static volatile uint16_t g_lastOCR      = 0;   ///< Last OCR1A written
static volatile uint16_t g_lastPrescaler = 0;  ///< Actual divisor (1/8/64/256/1024), 0 = stopped

/**
 * ADC sample window.
 * Each slot is pre-filled with ADC_SENTINEL (0xFFFF) in setup() and after
 * every completed batch.  ISR(ADC_vect) replaces slots with real readings
 * (0..1023), never with ADC_SENTINEL, so the sentinel unambiguously marks
 * an *uninitialized* slot even when a true ADC reading is 0.
 */
static volatile uint16_t g_adcWindow[ADC_WINDOW_SIZE];

/** Number of valid (non-sentinel) samples written in the current fill cycle. */
static volatile uint8_t  g_adcCount   = 0;

/** Set by ISR(ADC_vect) when the window is fully populated; cleared by loop(). */
static volatile bool     g_adcUpdated = false;

// ─── LCD ─────────────────────────────────────────────────────────────────────

/** LiquidCrystal(RS, EN, D4, D5, D6, D7) – all on non-conflicting pins. */
static LiquidCrystal lcd(8, 9, 7, 6, 5, 4);

// ─── Runtime pattern validation ───────────────────────────────────────────────

/**
 * check_patron_runtime()
 *
 * Reads every byte of PATRON_CKP from PROGMEM and verifies:
 *   1. Every value is either 0 or 1 (no other bit patterns).
 *   2. Exactly NUM_PRESENT (58) high-level entries are present.
 *
 * On failure: blinks the built-in LED in a distinctive pattern and halts.
 * Safe to call from setup() before interrupts are enabled.
 */
static void check_patron_runtime(void)
{
    uint8_t count_high = 0;
    uint8_t count_low  = 0;

    for (uint8_t i = 0; i < PATRON_SIZE; i++) {
        uint8_t v = pgm_read_byte(&PATRON_CKP[i]);
        if (v == 1) {
            count_high++;
        } else if (v == 0) {
            count_low++;
        } else {
            /* Invalid entry: halt with fast blink (10 Hz) */
            pinMode(LED_BUILTIN, OUTPUT);
            for (;;) {
                digitalWrite(LED_BUILTIN, HIGH); delay(50);
                digitalWrite(LED_BUILTIN, LOW);  delay(50);
            }
        }
    }

    /* 58 present teeth → 58 HIGH first-half-period entries */
    if (count_high != NUM_PRESENT || (count_high + count_low) != PATRON_SIZE) {
        /* Structural mismatch: halt with 3-blink pattern */
        pinMode(LED_BUILTIN, OUTPUT);
        for (;;) {
            for (uint8_t b = 0; b < 3; b++) {
                digitalWrite(LED_BUILTIN, HIGH); delay(150);
                digitalWrite(LED_BUILTIN, LOW);  delay(150);
            }
            delay(600);
        }
    }
}

// ─── ADC window helpers ───────────────────────────────────────────────────────

/**
 * validate_window_complete()
 *
 * Returns true if all @p count slots in @p snap are valid ADC readings
 * (i.e., none equals ADC_SENTINEL).  Must be called after snapshotting the
 * window from the ISR, before processing it.
 */
static bool validate_window_complete(const uint16_t *snap, uint8_t count)
{
    for (uint8_t i = 0; i < count; i++) {
        if (snap[i] == ADC_SENTINEL) return false;
    }
    return true;
}

// ─── Trimmed-mean window ──────────────────────────────────────────────────────

/**
 * compute_trimmed_mean_window()
 *
 * Pre-condition: all @p count entries in @p src are valid ADC readings
 * (0..1023); none may equal ADC_SENTINEL.  @p count must be >= 4.
 *
 * Copies src to a local buffer, insertion-sorts it (suitable for small N),
 * drops the minimum and maximum, and returns the integer average of the
 * remaining (count-2) values.
 *
 * Must NOT be called from an ISR.
 */
static uint16_t compute_trimmed_mean_window(const uint16_t *src, uint8_t count)
{
    uint16_t buf[ADC_WINDOW_SIZE];

    /* Copy */
    for (uint8_t i = 0; i < count; i++) {
        buf[i] = src[i];
    }

    /* Insertion sort – O(n²) acceptable for n ≤ 16 on AVR */
    for (uint8_t i = 1; i < count; i++) {
        uint16_t key      = buf[i];
        int8_t   sortIdx  = (int8_t)(i - 1);
        while (sortIdx >= 0 && buf[sortIdx] > key) {
            buf[sortIdx + 1] = buf[sortIdx];
            sortIdx--;
        }
        buf[sortIdx + 1] = key;
    }

    /* Average of inner (count-2) values, dropping min and max */
    uint32_t sum = 0;
    uint8_t  n   = (uint8_t)(count - 2U);
    for (uint8_t i = 1; i <= n; i++) {
        sum += buf[i];
    }
    return (uint16_t)(sum / n);
}

// ─── ADC → RPM mapping ────────────────────────────────────────────────────────

/**
 * Map a 10-bit ADC reading to RPM in [RPM_MIN, RPM_MAX].
 * Uses 32-bit arithmetic to avoid overflow.
 */
static uint16_t adc_to_rpm(uint16_t adc)
{
    return (uint16_t)(RPM_MIN +
           ((uint32_t)(RPM_MAX - RPM_MIN) * (uint32_t)adc) / 1023UL);
}

// ─── Integer EMA filter ───────────────────────────────────────────────────────

/**
 * One EMA step: new_ema = alpha × sample + (1-alpha) × ema
 * where alpha = EMA_ALPHA_NUM / EMA_ALPHA_DEN (integer arithmetic).
 * Avoids float/double entirely.
 */
static uint16_t ema_update(uint16_t ema, uint16_t sample)
{
    uint32_t result =
        (uint32_t)(EMA_ALPHA_DEN - EMA_ALPHA_NUM) * (uint32_t)ema
        + (uint32_t)EMA_ALPHA_NUM * (uint32_t)sample;
    return (uint16_t)(result / (uint32_t)EMA_ALPHA_DEN);
}

// ─── Timer1 prescaler selection ───────────────────────────────────────────────

/**
 * choose_timer1_params()
 *
 * For a desired ISR frequency @p freq_hz, iterates over T1_PRESCALERS in
 * ascending order and returns the first combination where OCR1A fits in
 * [0, 0xFFFF].
 *
 * Returns true on success and writes *presc_idx and *ocr.
 * Returns false if @p freq_hz is 0 or no prescaler yields a valid OCR.
 */
static bool choose_timer1_params(uint32_t freq_hz,
                                 uint8_t  *presc_idx,
                                 uint16_t *ocr)
{
    if (freq_hz == 0) return false;

    for (uint8_t i = 0; i < T1_PRESC_COUNT; i++) {
        uint16_t ps   = pgm_read_word(&T1_PRESCALERS[i]);
        uint32_t top  = F_CPU_HZ / ((uint32_t)ps * freq_hz);
        if (top == 0) continue;          /* prescaler too small: OCR would be -1 */
        top -= 1;
        if (top <= 0xFFFFUL) {
            *presc_idx = i;
            *ocr       = (uint16_t)top;
            return true;
        }
    }
    return false;
}

// ─── updateTimer() ────────────────────────────────────────────────────────────

/**
 * updateTimer()
 *
 * Recomputes Timer1 OCR1A and prescaler for @p target_rpm and applies the
 * new settings atomically *only if* they differ by more than the hysteresis
 * threshold (OCR changes > 1 count, or prescaler changes at all).
 *
 * Operates safely with interrupts running; the critical section that
 * modifies hardware registers and the volatile state mirrors is wrapped
 * in ATOMIC_BLOCK(ATOMIC_RESTORESTATE).
 */
static void updateTimer(uint16_t target_rpm)
{
    /* Clamp to operating range */
    if (target_rpm < RPM_MIN) target_rpm = RPM_MIN;
    if (target_rpm > RPM_MAX) target_rpm = RPM_MAX;

    /* ISR fires PATRON_SIZE times per revolution → freq = RPM/60 × PATRON_SIZE */
    uint32_t freq_hz =
        ((uint32_t)target_rpm * (uint32_t)PATRON_SIZE) / 60UL;

    uint8_t  new_pi  = 0;
    uint16_t new_ocr = 0;
    if (!choose_timer1_params(freq_hz, &new_pi, &new_ocr)) return;

    /* Read current settings atomically for hysteresis comparison */
    uint16_t cur_ocr;
    uint16_t cur_presc;
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        cur_ocr   = g_lastOCR;
        cur_presc = g_lastPrescaler;
    }

    uint16_t new_presc = pgm_read_word(&T1_PRESCALERS[new_pi]);

    /* Hysteresis: skip if prescaler unchanged and OCR differs by ≤ 1 count */
    if (new_presc == cur_presc) {
        int32_t delta = (int32_t)new_ocr - (int32_t)cur_ocr;
        if (delta < 0) delta = -delta;
        if (delta <= 1) return;
    }

    /* Apply new settings atomically */
    uint8_t cs_bits = pgm_read_byte(&T1_CS_BITS[new_pi]);
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        TCCR1B = (1 << WGM12);               /* CTC, clock stopped   */
        OCR1A  = new_ocr;
        TCNT1  = 0;
        TCCR1B = (1 << WGM12) | cs_bits;     /* CTC, clock restarted */
        g_lastOCR      = new_ocr;
        g_lastPrescaler = new_presc;
    }
}

// ─── stopTimer1() ────────────────────────────────────────────────────────────

/**
 * Halt Timer1 and clear the shared state mirrors so that
 * get_generated_freqHz_atomic() returns 0.
 */
static void stopTimer1(void)
{
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        TCCR1B      = (1 << WGM12);   /* CTC mode, no clock source */
        g_lastOCR      = 0;
        g_lastPrescaler = 0;           /* sentinel: 0 is not a valid divisor */
    }
}

// ─── get_generated_freqHz_atomic() ───────────────────────────────────────────

/**
 * Return the frequency [Hz] currently being generated by Timer1.
 * Returns 0 when the timer is stopped (g_lastPrescaler == 0).
 *
 * Safe to call from any context; reads the volatile mirrors under an
 * ATOMIC_BLOCK so the OCR and prescaler values are always consistent.
 */
static uint32_t get_generated_freqHz_atomic(void)
{
    uint16_t ocr;
    uint16_t presc;
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        ocr   = g_lastOCR;
        presc = g_lastPrescaler;
    }
    if (presc == 0) return 0;   /* timer stopped */
    return F_CPU_HZ / ((uint32_t)(ocr + 1U) * (uint32_t)presc);
}

/** Return the RPM currently being generated (0 if timer is stopped). */
static uint16_t get_generated_rpm_atomic(void)
{
    uint32_t f = get_generated_freqHz_atomic();
    if (f == 0) return 0;
    return (uint16_t)((f * 60UL) / (uint32_t)PATRON_SIZE);
}

// ─── setupTimer2ForAdcTrigger() ───────────────────────────────────────────────

/**
 * Configure Timer2 in CTC mode to fire COMPA interrupt at ADC_TRIG_HZ.
 *
 * Iterates T2_PRESCALERS to find the first prescaler giving OCR2A ∈ [0, 255].
 * Falls back to prescaler=128, OCR2A=124 (exact 1 kHz at 16 MHz) if the
 * loop finds nothing (should not happen for ADC_TRIG_HZ = 1000 Hz).
 */
static void setupTimer2ForAdcTrigger(void)
{
    TCCR2A = 0;
    TCCR2B = 0;
    TCNT2  = 0;

    for (uint8_t i = 0; i < T2_PRESC_COUNT; i++) {
        uint16_t ps      = pgm_read_word(&T2_PRESCALERS[i]);
        uint32_t top     = F_CPU_HZ / ((uint32_t)ps * ADC_TRIG_HZ);
        if (top == 0) continue;
        top -= 1;
        if (top <= 255UL) {
            OCR2A  = (uint8_t)top;
            TCCR2A = (1 << WGM21);                       /* CTC mode */
            TCCR2B = pgm_read_byte(&T2_CS_BITS[i]);
            TIMSK2 = (1 << OCIE2A);
            return;
        }
    }

    /* Fallback: prescaler 128 → OCR2A = 16 000 000/(128×1000) − 1 = 124 */
    OCR2A  = 124;
    TCCR2A = (1 << WGM21);
    TCCR2B = pgm_read_byte(&T2_CS_BITS[4]);   /* index 4 = /128 */
    TIMSK2 = (1 << OCIE2A);
}

// ─── setupADC() ───────────────────────────────────────────────────────────────

/**
 * Configure the ADC for free-start-on-demand operation:
 *   • AVcc voltage reference
 *   • Input channel A0
 *   • Conversion complete interrupt enabled
 *   • Clock prescaler /128  → ADC clock ≈ 125 kHz (13 cycles/conv ≈ 9.6 kHz max)
 *
 * Conversions are started individually by ISR(TIMER2_COMPA_vect).
 */
static void setupADC(void)
{
    ADMUX  = (1 << REFS0) | (ADC_INPUT_CHAN & 0x07);
    ADCSRA = (1 << ADEN)  | (1 << ADIE)
           | (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0);  /* /128 */
}

// ─── ISRs ─────────────────────────────────────────────────────────────────────

/**
 * Timer1 Compare-A ISR  –  CKP signal generation
 *
 * Duration estimate: ~12–18 AVR cycles (pgm_read_byte + port write + index wrap).
 * At 9000 RPM the period is 16 000 000 / (9000×120/60) = 889 cycles → <2 % load.
 *
 * Reads PATRON_CKP[g_patronIdx] from PROGMEM and drives D10 (PB2) HIGH or LOW.
 * Wraps g_patronIdx at PATRON_SIZE.
 *
 * NOTE: pgm_read_byte() is safe inside an ISR because LPM has no side-effects
 * and does not depend on any RAM state that another ISR could corrupt.
 */
ISR(TIMER1_COMPA_vect)
{
    uint8_t lvl = pgm_read_byte(&PATRON_CKP[g_patronIdx]);
    if (lvl) {
        PORTB |=  (1 << PB2);    /* D10 HIGH */
    } else {
        PORTB &= ~(1 << PB2);    /* D10 LOW  */
    }
    if (++g_patronIdx >= PATRON_SIZE) {
        g_patronIdx = 0;
    }
}

/**
 * Timer2 Compare-A ISR  –  ADC trigger
 *
 * Duration estimate: ~4–6 AVR cycles (single SBI instruction).
 * At 1 kHz this consumes < 0.04 % CPU time.
 */
ISR(TIMER2_COMPA_vect)
{
    ADCSRA |= (1 << ADSC);   /* start one ADC conversion */
}

/**
 * ADC Conversion Complete ISR  –  sample collection
 *
 * Duration estimate: ~20–30 AVR cycles.
 * At 1 kHz trigger rate this is well within budget.
 *
 * Behaviour:
 *   • If window is already full and loop() has not yet consumed it, the
 *     sample is silently discarded (prevents overwrite before processing).
 *   • Otherwise writes the 10-bit result into the next window slot and
 *     increments g_adcCount.
 *   • When the window is full (g_adcCount == ADC_WINDOW_SIZE), sets
 *     g_adcUpdated = true to signal loop().
 */
ISR(ADC_vect)
{
    /* Read ADCL first, then ADCH (required by hardware; the pair is
       captured automatically when ADCL is read, so just reading ADC suffices) */
    uint16_t raw = ADC;   /* 10-bit result, 0..1023 */

    if (g_adcCount >= ADC_WINDOW_SIZE) {
        return;   /* window full, not yet consumed – discard */
    }

    g_adcWindow[g_adcCount] = raw;   /* never writes ADC_SENTINEL (raw ≤ 1023) */
    g_adcCount++;

    if (g_adcCount >= ADC_WINDOW_SIZE) {
        g_adcUpdated = true;
    }
}

// ─── LCD helpers ─────────────────────────────────────────────────────────────

/**
 * Update the 16×2 LCD with current RPM values.
 * Called from loop() at ~4 Hz; never from an ISR.
 */
static void lcd_update(uint16_t target_rpm, uint16_t gen_rpm)
{
    lcd.setCursor(0, 0);
    lcd.print(F("TGT:"));
    lcd.print(target_rpm);
    lcd.print(F(" rpm    "));   /* trailing spaces erase old digits */

    lcd.setCursor(0, 1);
    lcd.print(F("GEN:"));
    lcd.print(gen_rpm);
    lcd.print(F(" rpm    "));
}

// ─── Persistent loop-scope state ──────────────────────────────────────────────

static uint16_t g_rpmTarget = RPM_DEFAULT;
static uint16_t g_rpmEMA    = RPM_DEFAULT;
static uint32_t g_lastLcdMs = 0;

// ─── setup() ─────────────────────────────────────────────────────────────────

void setup()
{
    /* CKP output pin (D10 = PB2) */
    pinMode(CKP_PIN, OUTPUT);
    PORTB &= ~(1 << PB2);   /* start LOW */

    /* LCD */
    lcd.begin(16, 2);
    lcd.print(F("SIMULADOR CKP"));
    lcd.setCursor(0, 1);
    lcd.print(F("60-2  Init..."));

    /* Runtime pattern integrity check (halts on error) */
    check_patron_runtime();

    /* Initialise ADC window with sentinel values */
    for (uint8_t i = 0; i < ADC_WINDOW_SIZE; i++) {
        g_adcWindow[i] = ADC_SENTINEL;
    }
    g_adcCount   = 0;
    g_adcUpdated = false;

    /* ADC */
    setupADC();

    /* Timer2 – ADC trigger at 1 kHz */
    setupTimer2ForAdcTrigger();

    /* Timer1 – CKP signal generator
     *   CTC mode: WGM13:0 = 0100 (WGM12 in TCCR1B)
     *   TCCR1A = 0 disconnects OC1A (D9) and OC1B (D10) from hardware output;
     *   we drive D10 manually in the ISR.
     */
    TCCR1A = 0;
    TCCR1B = 0;
    TCNT1  = 0;
    TIMSK1 = (1 << OCIE1A);   /* enable Compare-A interrupt */
    updateTimer(RPM_DEFAULT);

    sei();   /* global interrupt enable */

    delay(1000);
    lcd.clear();
}

// ─── loop() ──────────────────────────────────────────────────────────────────

void loop()
{
    /* ── 1. Process ADC window when fully populated ─────────────────────── */
    bool updated;
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        updated = g_adcUpdated;
    }

    if (updated) {
        /*
         * Atomically snapshot the window and reset for the next fill cycle.
         * Sentinel values are restored so the ISR can detect uninitialized
         * slots in the new cycle.
         */
        uint16_t snap[ADC_WINDOW_SIZE];
        uint8_t  cnt;

        ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
            cnt = g_adcCount;
            for (uint8_t i = 0; i < ADC_WINDOW_SIZE; i++) {
                snap[i] = g_adcWindow[i];
            }
            /* Reset window for next batch */
            g_adcCount   = 0;
            g_adcUpdated = false;
            for (uint8_t i = 0; i < ADC_WINDOW_SIZE; i++) {
                g_adcWindow[i] = ADC_SENTINEL;
            }
        }

        /*
         * Verify every slot was written by the ISR (none should remain as
         * ADC_SENTINEL) before calling compute_trimmed_mean_window().
         * This guards against the (unlikely) race where the ISR was starved.
         */
        if (validate_window_complete(snap, ADC_WINDOW_SIZE) && cnt >= ADC_WINDOW_SIZE) {
            uint16_t adc_mean = compute_trimmed_mean_window(snap, ADC_WINDOW_SIZE);

            if (adc_mean < 5U) {
                /*
                 * Potentiometer at (near) zero: stop the CKP output and
                 * reset the EMA so we ramp up cleanly when it is turned back.
                 * stopTimer1() clears g_lastOCR and g_lastPrescaler → 0, so
                 * get_generated_freqHz_atomic() correctly returns 0.
                 */
                g_rpmTarget = 0;
                g_rpmEMA    = RPM_DEFAULT;
                stopTimer1();
            } else {
                uint16_t rpm_raw = adc_to_rpm(adc_mean);
                g_rpmEMA    = ema_update(g_rpmEMA, rpm_raw);
                g_rpmTarget = g_rpmEMA;
                updateTimer(g_rpmTarget);
            }
        }
    }

    /* ── 2. LCD refresh at ~4 Hz ────────────────────────────────────────── */
    uint32_t now = millis();
    if ((now - g_lastLcdMs) >= LCD_UPDATE_MS) {
        g_lastLcdMs = now;
        uint16_t gen_rpm = get_generated_rpm_atomic();
        lcd_update(g_rpmTarget, gen_rpm);
    }
}
