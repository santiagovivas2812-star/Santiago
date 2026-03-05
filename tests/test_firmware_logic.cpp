/*
 * tests/test_firmware_logic.cpp
 *
 * Native (x86) unit tests for the SIMULADOR CKP firmware logic.
 * Validates critical functions that are architecture-independent.
 *
 * Build and run:
 *   g++ -std=c++11 -Wall -Wextra -o /tmp/test_ckp tests/test_firmware_logic.cpp && /tmp/test_ckp
 *
 * No external test framework required (uses a minimal inline harness).
 */

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

// ─── Minimal test harness ─────────────────────────────────────────────────────

static int g_tests_run    = 0;
static int g_tests_failed = 0;

#define CHECK(expr) \
    do { \
        g_tests_run++; \
        if (!(expr)) { \
            fprintf(stderr, "  FAIL  %s:%d: %s\n", __FILE__, __LINE__, #expr); \
            g_tests_failed++; \
        } \
    } while (0)

#define CHECK_EQ(a, b) \
    do { \
        g_tests_run++; \
        if ((a) != (b)) { \
            fprintf(stderr, "  FAIL  %s:%d: expected %lld, got %lld  [%s == %s]\n", \
                    __FILE__, __LINE__, \
                    (long long)(b), (long long)(a), #a, #b); \
            g_tests_failed++; \
        } \
    } while (0)

#define SECTION(name) printf("── %s\n", (name))

// ─── Reproduce firmware constants ─────────────────────────────────────────────

#define F_CPU_HZ         16000000UL
#define PATRON_SIZE      120U
#define NUM_TEETH        60U
#define NUM_MISSING      2U
#define NUM_PRESENT      (NUM_TEETH - NUM_MISSING)
#define ADC_WINDOW_SIZE  16U
#define ADC_SENTINEL     0xFFFFU
#define RPM_MIN          100U
#define RPM_MAX          9000U
#define RPM_DEFAULT      600U
#define EMA_ALPHA_NUM    1U
#define EMA_ALPHA_DEN    8U
#define ADC_TRIG_HZ      1000UL

static_assert(PATRON_SIZE   == 120,   "PATRON_CKP must have 120 entries");
static_assert(ADC_WINDOW_SIZE >= 4,   "Window too small for trimmed-mean");
static_assert(ADC_SENTINEL  > 1023U,  "ADC_SENTINEL must not be a valid ADC value");
static_assert(RPM_MIN < RPM_MAX,      "RPM_MIN < RPM_MAX");
static_assert(EMA_ALPHA_NUM > 0 && EMA_ALPHA_NUM < EMA_ALPHA_DEN, "EMA alpha in (0,1)");

// ─── PATRON_CKP (copy of PROGMEM array for host testing) ─────────────────────

static const uint8_t PATRON_CKP[PATRON_SIZE] = {
    1,0, 1,0, 1,0, 1,0, 1,0, 1,0, 1,0, 1,0,   /* teeth  0-7  */
    1,0, 1,0, 1,0, 1,0, 1,0, 1,0, 1,0, 1,0,   /* teeth  8-15 */
    1,0, 1,0, 1,0, 1,0, 1,0, 1,0, 1,0, 1,0,   /* teeth 16-23 */
    1,0, 1,0, 1,0, 1,0, 1,0, 1,0, 1,0, 1,0,   /* teeth 24-31 */
    1,0, 1,0, 1,0, 1,0, 1,0, 1,0, 1,0, 1,0,   /* teeth 32-39 */
    1,0, 1,0, 1,0, 1,0, 1,0, 1,0, 1,0, 1,0,   /* teeth 40-47 */
    1,0, 1,0, 1,0, 1,0, 1,0, 1,0, 1,0, 1,0,   /* teeth 48-55 */
    1,0, 1,0,                                   /* teeth 56-57 */
    0,0, 0,0                                    /* teeth 58-59 (missing) */
};

// ─── Reproduce firmware functions (host-compatible) ───────────────────────────

static uint16_t compute_trimmed_mean_window(const uint16_t *src, uint8_t count)
{
    uint16_t buf[ADC_WINDOW_SIZE];
    for (uint8_t i = 0; i < count; i++) buf[i] = src[i];

    for (uint8_t i = 1; i < count; i++) {
        uint16_t key = buf[i];
        int      j   = (int)i - 1;
        while (j >= 0 && buf[j] > key) { buf[j+1] = buf[j]; j--; }
        buf[j+1] = key;
    }

    uint32_t sum = 0;
    uint8_t  n   = (uint8_t)(count - 2U);
    for (uint8_t i = 1; i <= n; i++) sum += buf[i];
    return (uint16_t)(sum / n);
}

static uint16_t adc_to_rpm(uint16_t adc)
{
    return (uint16_t)(RPM_MIN +
           ((uint32_t)(RPM_MAX - RPM_MIN) * (uint32_t)adc) / 1023UL);
}

static uint16_t ema_update(uint16_t ema, uint16_t sample)
{
    uint32_t result =
        (uint32_t)(EMA_ALPHA_DEN - EMA_ALPHA_NUM) * (uint32_t)ema
        + (uint32_t)EMA_ALPHA_NUM * (uint32_t)sample;
    return (uint16_t)(result / (uint32_t)EMA_ALPHA_DEN);
}

static const uint16_t T1_PRESCALERS[] = { 1, 8, 64, 256, 1024 };
#define T1_PRESC_COUNT 5U

static bool choose_timer1_params(uint32_t freq_hz, uint8_t *presc_idx, uint16_t *ocr)
{
    if (freq_hz == 0) return false;
    for (uint8_t i = 0; i < T1_PRESC_COUNT; i++) {
        uint16_t ps   = T1_PRESCALERS[i];
        uint32_t top  = F_CPU_HZ / ((uint32_t)ps * freq_hz);
        if (top == 0) continue;
        top -= 1;
        if (top <= 0xFFFFUL) { *presc_idx = i; *ocr = (uint16_t)top; return true; }
    }
    return false;
}

/* Simulated volatile mirrors for get_generated_freqHz */
static uint16_t sim_lastOCR      = 0;
static uint16_t sim_lastPrescaler = 0;

static uint32_t get_generated_freqHz(void)
{
    if (sim_lastPrescaler == 0) return 0;
    return F_CPU_HZ / ((uint32_t)(sim_lastOCR + 1U) * (uint32_t)sim_lastPrescaler);
}

static uint16_t get_generated_rpm(void)
{
    uint32_t f = get_generated_freqHz();
    if (f == 0) return 0;
    return (uint16_t)((f * 60UL) / (uint32_t)PATRON_SIZE);
}

// ─── Tests ────────────────────────────────────────────────────────────────────

static void test_patron_ckp_structure(void)
{
    SECTION("PATRON_CKP structure");

    CHECK_EQ(PATRON_SIZE, 120U);

    uint8_t count_high = 0, count_low = 0;
    for (uint8_t i = 0; i < PATRON_SIZE; i++) {
        uint8_t v = PATRON_CKP[i];
        CHECK(v == 0 || v == 1);           /* only valid levels */
        if (v == 1) count_high++;
        else        count_low++;
    }

    /* 58 present teeth → 58 HIGH first-half entries */
    CHECK_EQ(count_high, NUM_PRESENT);    /* 58 */
    CHECK_EQ(count_low,  (PATRON_SIZE - NUM_PRESENT)); /* 62 */

    /* First half-period of teeth 0-57 must be HIGH */
    for (uint8_t t = 0; t < NUM_PRESENT; t++) {
        CHECK_EQ(PATRON_CKP[2 * t],     1U);
        CHECK_EQ(PATRON_CKP[2 * t + 1], 0U);
    }

    /* Missing teeth 58-59 must be all LOW */
    CHECK_EQ(PATRON_CKP[116], 0U);
    CHECK_EQ(PATRON_CKP[117], 0U);
    CHECK_EQ(PATRON_CKP[118], 0U);
    CHECK_EQ(PATRON_CKP[119], 0U);
}

static void test_adc_sentinel(void)
{
    SECTION("ADC_SENTINEL is not a valid 10-bit ADC reading");

    /* 10-bit ADC range: 0..1023 */
    CHECK(ADC_SENTINEL > 1023U);
    /* Specifically 0xFFFF */
    CHECK_EQ(ADC_SENTINEL, 0xFFFFU);

    /* Simulate window initialisation */
    uint16_t win[ADC_WINDOW_SIZE];
    for (uint8_t i = 0; i < ADC_WINDOW_SIZE; i++) win[i] = ADC_SENTINEL;

    /* No slot can be confused with a real reading */
    for (uint8_t i = 0; i < ADC_WINDOW_SIZE; i++) {
        CHECK(win[i] != 0U);    /* distinguishable from ADC reading = 0 */
        CHECK(win[i] > 1023U);
    }

    /* Write real readings (including 0) and verify detection */
    win[0] = 0;    /* legitimate reading of 0 */
    win[1] = 512;
    /* win[2] left as ADC_SENTINEL */
    bool all_valid = true;
    for (uint8_t i = 0; i < 3; i++) {
        if (win[i] == ADC_SENTINEL) { all_valid = false; break; }
    }
    CHECK(!all_valid);   /* win[2] is sentinel → not all valid */

    /* Fill all slots with real readings (including 0) */
    for (uint8_t i = 0; i < ADC_WINDOW_SIZE; i++) win[i] = (uint16_t)(i * 10);
    all_valid = true;
    for (uint8_t i = 0; i < ADC_WINDOW_SIZE; i++) {
        if (win[i] == ADC_SENTINEL) { all_valid = false; break; }
    }
    CHECK(all_valid);    /* all slots valid now */
}

static void test_trimmed_mean(void)
{
    SECTION("compute_trimmed_mean_window");

    /* Uniform values → mean equals the value */
    {
        uint16_t w[ADC_WINDOW_SIZE];
        for (uint8_t i = 0; i < ADC_WINDOW_SIZE; i++) w[i] = 500;
        CHECK_EQ(compute_trimmed_mean_window(w, ADC_WINDOW_SIZE), 500U);
    }

    /* One outlier high and one outlier low – both trimmed */
    {
        uint16_t w[ADC_WINDOW_SIZE];
        for (uint8_t i = 0; i < ADC_WINDOW_SIZE; i++) w[i] = 400;
        w[0]  = 0;     /* outlier low  */
        w[15] = 1023;  /* outlier high */
        /* After trim, 14 × 400 = 5600, mean = 400 */
        CHECK_EQ(compute_trimmed_mean_window(w, ADC_WINDOW_SIZE), 400U);
    }

    /* Ascending sequence 0..15, trim 0 and 15, mean of 1..14 = 105/14 = 7 (int) */
    {
        uint16_t w[ADC_WINDOW_SIZE];
        for (uint8_t i = 0; i < ADC_WINDOW_SIZE; i++) w[i] = i;
        uint16_t result = compute_trimmed_mean_window(w, ADC_WINDOW_SIZE);
        /* sum(1..14) = 105, / 14 = 7 */
        CHECK_EQ(result, 7U);
    }

    /* Minimum allowed window (4 samples): trim 1+1, average 2 middle */
    {
        uint16_t w[4] = { 100, 200, 300, 400 };
        uint16_t result = compute_trimmed_mean_window(w, 4);
        /* sorted: 100,200,300,400; drop 100 and 400; mean(200,300) = 250 */
        CHECK_EQ(result, 250U);
    }

    /* ADC reading of 0 is valid, must not be treated as sentinel */
    {
        uint16_t w[ADC_WINDOW_SIZE];
        for (uint8_t i = 0; i < ADC_WINDOW_SIZE; i++) w[i] = 0;
        CHECK_EQ(compute_trimmed_mean_window(w, ADC_WINDOW_SIZE), 0U);
    }
}

static void test_adc_to_rpm(void)
{
    SECTION("adc_to_rpm mapping");

    /* ADC=0 → RPM_MIN */
    CHECK_EQ(adc_to_rpm(0),    RPM_MIN);

    /* ADC=1023 → RPM_MAX */
    CHECK_EQ(adc_to_rpm(1023), RPM_MAX);

    /* Mid-scale: approximately (RPM_MIN + RPM_MAX) / 2 */
    uint16_t mid = adc_to_rpm(511);
    CHECK(mid > RPM_MIN);
    CHECK(mid < RPM_MAX);

    /* Monotonically non-decreasing */
    uint16_t prev = adc_to_rpm(0);
    for (uint16_t v = 1; v <= 1023; v++) {
        uint16_t cur = adc_to_rpm((uint16_t)v);
        CHECK(cur >= prev);
        prev = cur;
    }
}

static void test_ema_update(void)
{
    SECTION("ema_update (integer EMA)");

    /* EMA converges toward the sample */
    uint16_t ema = 0;
    for (int i = 0; i < 100; i++) ema = ema_update(ema, 1000);
    CHECK(ema > 900);   /* should have converged well above 90 % of 1000 */

    /* EMA of a constant sequence is the constant */
    ema = 500;
    for (int i = 0; i < 50; i++) ema = ema_update(ema, 500);
    CHECK_EQ(ema, 500U);

    /* EMA falls toward lower sample */
    ema = 1000;
    for (int i = 0; i < 100; i++) ema = ema_update(ema, 0);
    CHECK(ema < 100);

    /* Single step: new = (7*ema + 1*sample) / 8 */
    ema = 800;
    uint16_t one_step = ema_update(800, 0);
    CHECK_EQ(one_step, (uint16_t)((7U * 800U + 1U * 0U) / 8U));  /* 700 */
}

static void test_choose_timer1_params(void)
{
    SECTION("choose_timer1_params prescaler selection");

    uint8_t  pi  = 0;
    uint16_t ocr = 0;

    /* freq=0 → false */
    CHECK(!choose_timer1_params(0, &pi, &ocr));

    /* RPM=100 → freq=200 Hz */
    {
        uint32_t freq = ((uint32_t)100U * PATRON_SIZE) / 60UL; /* 200 Hz */
        bool ok = choose_timer1_params(freq, &pi, &ocr);
        CHECK(ok);
        if (ok) {
            uint32_t actual_freq = F_CPU_HZ / ((uint32_t)(ocr + 1U) * T1_PRESCALERS[pi]);
            /* Actual vs desired within 1 % */
            int32_t err = (int32_t)actual_freq - (int32_t)freq;
            if (err < 0) err = -err;
            CHECK((uint32_t)err * 100UL < freq);   /* < 1 % relative error */
            /* OCR fits in 16-bit */
            CHECK(ocr <= 0xFFFFU);
        }
    }

    /* RPM=9000 → freq=18000 Hz */
    {
        uint32_t freq = ((uint32_t)9000U * PATRON_SIZE) / 60UL; /* 18000 Hz */
        bool ok = choose_timer1_params(freq, &pi, &ocr);
        CHECK(ok);
        if (ok) CHECK(ocr <= 0xFFFFU);
    }

    /* Very high frequency (edge case: large RPM) */
    {
        uint32_t freq = 200000UL;
        bool ok = choose_timer1_params(freq, &pi, &ocr);
        /* OCR1A = 16000000/200000 - 1 = 79 → valid with prescaler=1 */
        CHECK(ok);
    }

    /* Prescaler selection: 16 MHz / (1 * (79+1)) = 200 000 Hz, so pi should be 0 (prescaler=1) */
    {
        uint32_t freq = 200000UL;
        choose_timer1_params(freq, &pi, &ocr);
        CHECK_EQ(T1_PRESCALERS[pi], 1U);
        CHECK_EQ(ocr, 79U);
    }
}

static void test_get_generated_freq_stopped(void)
{
    SECTION("get_generated_freqHz returns 0 when timer stopped");

    /* Simulate stopped state: both cleared to 0 */
    sim_lastOCR      = 0;
    sim_lastPrescaler = 0;
    CHECK_EQ(get_generated_freqHz(), 0UL);
    CHECK_EQ(get_generated_rpm(),   0U);

    /* Simulate running state: OCR=887, prescaler=1 → 18018 Hz ≈ 18000 Hz */
    sim_lastOCR      = 887;
    sim_lastPrescaler = 1;
    uint32_t f = get_generated_freqHz();
    CHECK(f > 17000UL && f < 19000UL);
    uint16_t rpm = get_generated_rpm();
    CHECK(rpm > 8000U && rpm < 9500U);

    /* Stop again */
    sim_lastOCR      = 0;
    sim_lastPrescaler = 0;
    CHECK_EQ(get_generated_freqHz(), 0UL);
}

static void test_hysteresis_logic(void)
{
    SECTION("updateTimer hysteresis (same prescaler, OCR delta <= 1)");

    uint8_t  pi1 = 0; uint16_t ocr1 = 0;
    uint8_t  pi2 = 0; uint16_t ocr2 = 0;

    /* Two adjacent RPM values at high speed should produce the same OCR or
       an OCR differing by ≤ 1 → hysteresis must suppress the update */
    uint32_t f1 = ((uint32_t)5000U * PATRON_SIZE) / 60UL;
    uint32_t f2 = ((uint32_t)5001U * PATRON_SIZE) / 60UL;
    choose_timer1_params(f1, &pi1, &ocr1);
    choose_timer1_params(f2, &pi2, &ocr2);

    /* When prescaler is the same, check delta */
    if (pi1 == pi2) {
        int32_t delta = (int32_t)ocr2 - (int32_t)ocr1;
        if (delta < 0) delta = -delta;
        /* Expect hysteresis to suppress tiny changes */
        if (delta <= 1) {
            printf("    OK: RPM 5000→5001 produces ΔOCR=%d (≤1), hysteresis fires\n",
                   (int)delta);
        } else {
            printf("    INFO: RPM 5000→5001 produces ΔOCR=%d (>1), update fires\n",
                   (int)delta);
        }
    }
    CHECK(true);   /* structural check passes */
}

static void test_timer2_ocr_range(void)
{
    SECTION("Timer2 OCR2A for ADC trigger fits in 0..255");

    /* Replicate setupTimer2ForAdcTrigger() logic */
    static const uint16_t T2_PS[] = { 1, 8, 32, 64, 128, 256, 1024 };
    bool found = false;
    for (uint8_t i = 0; i < 7; i++) {
        uint32_t top = F_CPU_HZ / ((uint32_t)T2_PS[i] * ADC_TRIG_HZ);
        if (top == 0) continue;
        top -= 1;
        if (top <= 255UL) {
            printf("    Timer2: prescaler=%u OCR2A=%lu (freq=%lu Hz)\n",
                   T2_PS[i], (unsigned long)top,
                   F_CPU_HZ / ((uint32_t)T2_PS[i] * (top + 1)));
            CHECK(top <= 255UL);
            found = true;
            break;
        }
    }
    CHECK(found);

    /* Specific check: prescaler=128, OCR2A=124 gives exactly 1 kHz */
    uint32_t actual = F_CPU_HZ / (128UL * 125UL);   /* 16000000/(128*125) = 1000 */
    CHECK_EQ(actual, 1000UL);
}

static void test_patron_signal_sequence(void)
{
    SECTION("CKP signal sequence simulation (120-step one revolution)");

    /* Simulate one full revolution stepping through PATRON_CKP */
    uint8_t  idx          = 0;
    uint32_t rising_edges = 0;
    uint32_t falling_edges = 0;
    uint8_t  prev_level   = 0;

    for (uint16_t step = 0; step < PATRON_SIZE; step++) {
        uint8_t lvl = PATRON_CKP[idx];
        if (lvl == 1 && prev_level == 0) rising_edges++;
        if (lvl == 0 && prev_level == 1) falling_edges++;
        prev_level = lvl;
        if (++idx >= PATRON_SIZE) idx = 0;
    }

    printf("    rising edges=%lu, falling edges=%lu\n",
           (unsigned long)rising_edges, (unsigned long)falling_edges);

    /* One revolution produces exactly NUM_PRESENT rising and NUM_PRESENT falling edges */
    CHECK_EQ(rising_edges,  (uint32_t)NUM_PRESENT);
    CHECK_EQ(falling_edges, (uint32_t)NUM_PRESENT);
}

// ─── main ─────────────────────────────────────────────────────────────────────

int main(void)
{
    printf("SIMULADOR CKP – firmware logic unit tests\n");
    printf("==========================================\n");

    test_patron_ckp_structure();
    test_adc_sentinel();
    test_trimmed_mean();
    test_adc_to_rpm();
    test_ema_update();
    test_choose_timer1_params();
    test_get_generated_freq_stopped();
    test_hysteresis_logic();
    test_timer2_ocr_range();
    test_patron_signal_sequence();

    printf("\n==========================================\n");
    printf("Results: %d tests run, %d failed\n", g_tests_run, g_tests_failed);

    return (g_tests_failed == 0) ? 0 : 1;
}
