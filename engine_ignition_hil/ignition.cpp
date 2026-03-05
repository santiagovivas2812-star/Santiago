/**
 * ignition.cpp — Implementación del módulo de encendido DIS
 *
 * Maneja:
 *  - Decodificación de la rueda 36-1 de cigüeñal
 *  - Filtrado de ruido y detección del diente de referencia
 *  - Cálculo de RPM con filtro de promedio móvil
 *  - Scheduling preciso de dwell e ignición usando Timer1 (OCR1A / OCR1B)
 *  - Protección contra desbordamiento de timer y wrap-around de micros()
 *  - Estado seguro y timeouts de bobina
 *
 * Notas de seguridad funcional:
 *  - Todo acceso a variables volátiles desde código no-ISR usa ATOMIC_BLOCK.
 *  - Todas las diferencias de tiempo usan uint32_t para ser wrap-safe.
 *  - Los pines HIL de alta frecuencia usan acceso directo a registro (PORTF).
 */

#include <Arduino.h>
#include "config.h"
#include "ignition.h"


/* ================================================================
 *  VARIABLES GLOBALES
 * ================================================================ */
CrankState_t           g_crankState;
IgnChannel_t           g_ignChannels[COIL_COUNT];
DiagCounters_t         g_diag;
volatile SystemState_t g_sysState = SYS_STALL;


/* ================================================================
 *  ESTADO INTERNO DEL FILTRO RPM
 * ================================================================ */
static uint16_t s_rpmHistory[RPM_FILTER_SAMPLES];
static uint8_t  s_rpmIdx = 0U;


/* ================================================================
 *  MÁQUINA DE ESTADOS DEL TIMER (dwell por canal)
 * ================================================================ */
typedef enum {
    DWELL_IDLE = 0,   /**< Sin actividad */
    DWELL_WAIT_ON,    /**< Esperando el inicio del dwell (OCR1A) */
    DWELL_CHARGING,   /**< Bobina cargando; esperando fin de dwell (OCR1B) */
} DwellState_t;

static volatile DwellState_t s_dwellState[COIL_COUNT];

/** Canal actualmente en servicio por el timer (0 o 1) */
static volatile uint8_t s_activeChannel = 0U;


/* ================================================================
 *  PROTOTIPOS INTERNOS
 * ================================================================ */
static void timer1_program(uint16_t ticksA, uint16_t ticksB);
static void coil_on(uint8_t ch);
static void coil_off(uint8_t ch);


/* ================================================================
 *  INICIALIZACIÓN
 * ================================================================ */
void ignition_init(void)
{
    /* Limpiar estado de cigüeñal */
    memset(&g_crankState, 0, sizeof(g_crankState));
    g_crankState.syncAcquired = false;
    g_crankState.syncLost     = true;

    /* Limpiar contadores de diagnóstico */
    memset(&g_diag, 0, sizeof(g_diag));

    /* Limpiar filtro RPM */
    memset(s_rpmHistory, 0, sizeof(s_rpmHistory));
    s_rpmIdx = 0U;

    /* ---- Canal 0: Bobina A (cilindros 1 y 4, wasted spark) ---- */
    g_ignChannels[0].outputPin     = PIN_COIL_A;
    g_ignChannels[0].debugPin      = PIN_HIL_COIL_A_DBG;
    g_ignChannels[0].dwellUs       = (uint16_t)DWELL_DEFAULT_US;
    g_ignChannels[0].advanceDegX10 = ADVANCE_DEFAULT_DEG_X10;
    g_ignChannels[0].coilCharging  = false;
    g_ignChannels[0].cylinderA     = 1U;
    g_ignChannels[0].cylinderB     = 4U;
    s_dwellState[0]                = DWELL_IDLE;

    /* ---- Canal 1: Bobina B (cilindros 2 y 3, wasted spark) ---- */
    g_ignChannels[1].outputPin     = PIN_COIL_B;
    g_ignChannels[1].debugPin      = PIN_HIL_COIL_B_DBG;
    g_ignChannels[1].dwellUs       = (uint16_t)DWELL_DEFAULT_US;
    g_ignChannels[1].advanceDegX10 = ADVANCE_DEFAULT_DEG_X10;
    g_ignChannels[1].coilCharging  = false;
    g_ignChannels[1].cylinderA     = 2U;
    g_ignChannels[1].cylinderB     = 3U;
    s_dwellState[1]                = DWELL_IDLE;

    s_activeChannel = 0U;

    /* ---- Configurar pines de salida ---- */
    for (uint8_t i = 0U; i < COIL_COUNT; i++) {
        pinMode(g_ignChannels[i].outputPin, OUTPUT);
        digitalWrite(g_ignChannels[i].outputPin, LOW);
    }

    /* ---- Timer1: Normal mode, prescaler 8, sin interrupts aún ---- */
    TCCR1A = 0U;
    /* Normal mode (WGM13:0 = 0000); prescaler 8 (CS11=1) */
    TCCR1B = (1U << CS11);
    TCNT1  = 0U;
    OCR1A  = TIMER1_OCR_MAX;
    OCR1B  = TIMER1_OCR_MAX;
    TIMSK1 = 0U;       /* Todas las interrupts de Timer1 deshabilitadas */
    TIFR1  = 0xFFU;    /* Limpiar flags de interrupción pendientes */

    g_sysState = SYS_STALL;
}


/* ================================================================
 *  CÁLCULO DE RPM CON FILTRO DE PROMEDIO MÓVIL
 * ================================================================ */
uint16_t ignition_calc_rpm(uint32_t periodUs)
{
    /*
     * Protección contra división por cero y períodos imposiblemente cortos.
     * MIN_TOOTH_PERIOD_US corresponde a ~8800 rpm, por encima de RPM_MAX.
     */
    if (periodUs < MIN_TOOTH_PERIOD_US) {
        return 0U;
    }

    /*
     * RPM = 60 000 000 / (periodUs × TEETH_PER_REV)
     *
     * Análisis de overflow uint32_t:
     *   Numerador: 60 000 000 — cabe en uint32_t (max 4 294 967 295).
     *   Denominador mín: 190 × 36 = 6 840 → resultado máx: ~8771 — cabe en uint16_t.
     *   Denominador máx: 200 000 × 36 = 7 200 000 → resultado mín: 8 — OK.
     */
    uint16_t rawRpm = (uint16_t)(60000000UL /
                                 ((uint32_t)periodUs * (uint32_t)TEETH_PER_REV));

    /* Actualizar filtro de promedio móvil */
    s_rpmHistory[s_rpmIdx] = rawRpm;
    s_rpmIdx = (uint8_t)((s_rpmIdx + 1U) % RPM_FILTER_SAMPLES);

    uint32_t sum = 0UL;
    for (uint8_t i = 0U; i < RPM_FILTER_SAMPLES; i++) {
        sum += s_rpmHistory[i];
    }
    return (uint16_t)(sum / RPM_FILTER_SAMPLES);
}


/* ================================================================
 *  CLAMP DE DWELL
 * ================================================================ */
uint16_t ignition_clamp_dwell(uint16_t requestedUs)
{
    if (requestedUs > (uint16_t)DWELL_MAX_US) {
        g_diag.dwellClampHigh++;
        return (uint16_t)DWELL_MAX_US;
    }
    if (requestedUs < (uint16_t)DWELL_MIN_US) {
        g_diag.dwellClampLow++;
        return (uint16_t)DWELL_MIN_US;
    }
    return requestedUs;
}


/* ================================================================
 *  HANDLER DE DIENTE (llamado desde ISR de cigüeñal)
 * ================================================================ */
void ignition_tooth_handler(uint32_t toothTimeUs)
{
    /*
     * ---- Filtro de ruido: período mínimo ----
     * La diferencia uint32_t es wrap-safe (ver elapsed_exceeds en ignition.h).
     */
    uint32_t period = (uint32_t)(toothTimeUs - g_crankState.lastToothTimeUs);

    if (period < MIN_TOOTH_PERIOD_US) {
        g_diag.noisePulses++;
        return;   /* Pulso rechazado */
    }

    /* ---- Detección de diente faltante (gap de referencia) ----
     *
     * Condición: period > 1,5 × período previo
     * Usando aritmética entera (×100) para evitar punto flotante en ISR.
     *
     * Protección contra overflow: ambos operandos son uint32_t ≤ 5555 µs
     * (a RPM_MIN); 5555 × 150 = 833 250, que cabe holgadamente en uint32_t.
     */
    bool isGap = false;
    if (g_crankState.toothPeriodUs > 0UL) {
        if (((uint32_t)period * 100UL) >
            ((uint32_t)g_crankState.toothPeriodUs * (uint32_t)MISSING_TOOTH_RATIO_X100)) {
            isGap = true;
            g_diag.gapDetections++;
        }
    }

    /* ---- Actualizar estado del cigüeñal ---- */
    g_crankState.toothPeriodUs   = period;
    g_crankState.lastToothTimeUs = toothTimeUs;

    if (isGap) {
        /* El hueco es el diente de referencia: reset del contador */
        g_crankState.toothNumber = 0U;
        g_crankState.gapsSeen++;

        if (g_crankState.gapsSeen >= CRANK_SYNC_MIN_GAPS) {
            if (!g_crankState.syncAcquired) {
                g_diag.syncEvents++;
                g_crankState.syncAcquired = true;
                g_crankState.syncLost     = false;
            }
            HIL_SYNC_SET();
        }
    } else {
        g_crankState.toothNumber++;

        /*
         * Si el conteo llega a EFFECTIVE_TEETH sin haber visto el hueco,
         * la sincronía es incorrecta: se declara pérdida de sync.
         */
        if (g_crankState.toothNumber >= (uint16_t)EFFECTIVE_TEETH) {
            if (g_crankState.syncAcquired) {
                g_crankState.syncAcquired = false;
                g_crankState.syncLost     = true;
                g_diag.syncLossEvents++;
                HIL_SYNC_CLR();
            }
            g_crankState.toothNumber = 0U;
            g_crankState.gapsSeen    = 0U;
        }
    }

    g_diag.toothCount++;

    /* ---- Actualizar RPM ---- */
    if (g_crankState.syncAcquired) {
        g_crankState.rpm = ignition_calc_rpm(period);
    }

    /* Toggle HIL tooth pulse (acceso directo a PORTF bit 0 = A0) */
    HIL_TOOTH_TOG();
}


/* ================================================================
 *  SCHEDULING DE ENCENDIDO
 * ================================================================ */

/**
 * Programa Timer1 para dos eventos:
 *  - OCR1A a `ticksA` ticks desde ahora → inicio de dwell (bobina ON).
 *  - OCR1B a `ticksA + ticksB` ticks desde ahora → fin de dwell (chispa).
 *
 * Se detiene el timer brevemente durante la reprogramación para evitar
 * la condición de carrera en la que TCNT1 supera OCR antes de habilitarlo.
 */
static void timer1_program(uint16_t ticksA, uint16_t ticksB)
{
    /* Detener timer (CS = 000) */
    TCCR1B &= ~((1U << CS12) | (1U << CS11) | (1U << CS10));

    /* Resetear contador y programar comparadores */
    TCNT1 = 0U;
    OCR1A = ticksA;

    /*
     * OCR1B = ticksA + ticksB (fin de dwell).
     *
     * Análisis de valores máximos reales desde el llamador:
     *   ticksA_max = TIMER1_MAX_DELAY_US × TIMER1_TICKS_PER_US
     *              = 32767 µs × 2 ticks/µs = 65534 ticks
     *   ticksB_max = DWELL_MAX_US × TIMER1_TICKS_PER_US
     *              = 5000 µs × 2 ticks/µs = 10000 ticks
     *   suma_max   = 65534 + 10000 = 75534 > TIMER1_OCR_MAX (65535)
     *
     * La suma se calcula en uint32_t y se recorta a TIMER1_OCR_MAX si
     * excede el rango del registro OCR1B de 16 bits.
     */
    uint32_t ocr1b_val = (uint32_t)ticksA + (uint32_t)ticksB;
    if (ocr1b_val > (uint32_t)TIMER1_OCR_MAX) {
        ocr1b_val = (uint32_t)TIMER1_OCR_MAX;
        g_diag.overflowGuards++;
    }
    OCR1B = (uint16_t)ocr1b_val;

    /* Limpiar flags de comparación pendientes */
    TIFR1 = (1U << OCF1A) | (1U << OCF1B);

    /* Habilitar interrupts de comparación */
    TIMSK1 |= (1U << OCIE1A) | (1U << OCIE1B);

    /* Reiniciar timer con prescaler 8 (CS11 = 1) */
    TCCR1B |= (1U << CS11);
}

bool ignition_schedule(uint8_t channel, uint32_t delayUs, uint16_t dwellUs)
{
    if (channel >= COIL_COUNT) {
        return false;
    }

    IgnChannel_t *ch = &g_ignChannels[channel];

    /*
     * Sanity check del retardo:
     * Si el retardo es mayor que una revolución a RPM_MIN (200 000 µs),
     * el evento es obsoleto y se rechaza.
     */
    if (delayUs > 200000UL) {
        return false;
    }

    /*
     * Convertir a ticks de Timer1.
     * Máximo: 200 000 µs × 2 = 400 000 ticks → no cabe en uint16_t.
     * Pero ya validamos delayUs ≤ TIMER1_MAX_DELAY_US antes de llamar.
     */
    if (delayUs > (uint32_t)TIMER1_MAX_DELAY_US) {
        g_diag.overflowGuards++;
        return false;
    }

    uint16_t dwellClamped = ignition_clamp_dwell(dwellUs);
    ch->dwellUs = dwellClamped;

    uint32_t dwellTicks = (uint32_t)dwellClamped * (uint32_t)TIMER1_TICKS_PER_US;
    if (dwellTicks > (uint32_t)TIMER1_OCR_MAX) {
        dwellTicks = (uint32_t)TIMER1_OCR_MAX;
        g_diag.overflowGuards++;
    }

    uint16_t delayTicks = (uint16_t)((uint32_t)delayUs * (uint32_t)TIMER1_TICKS_PER_US);

    /* Registro del canal activo (acceso atómico: se llama desde sección crítica) */
    s_activeChannel = channel;
    s_dwellState[channel] = DWELL_WAIT_ON;

    timer1_program(delayTicks, (uint16_t)dwellTicks);
    return true;
}


/* ================================================================
 *  HANDLER DEL TIMER1 (llamado desde ISR)
 * ================================================================ */
void ignition_timer_handler(bool isCompB)
{
    uint8_t ch = s_activeChannel;

    if (ch >= COIL_COUNT) {
        /* Canal inválido — apagar todo por seguridad */
        TIMSK1 &= ~((1U << OCIE1A) | (1U << OCIE1B));
        return;
    }

    if (!isCompB) {
        /*
         * Evento A: inicio de dwell.
         * Encender bobina si la máquina de estados lo permite.
         */
        if (s_dwellState[ch] == DWELL_WAIT_ON) {
            coil_on(ch);
            g_ignChannels[ch].chargeStartUs = micros();
            s_dwellState[ch] = DWELL_CHARGING;
        }
    } else {
        /*
         * Evento B: fin de dwell → disparo de chispa.
         * Apagar bobina (borde de bajada genera la chispa en el igniter).
         */
        if (s_dwellState[ch] == DWELL_CHARGING) {
            coil_off(ch);
            s_dwellState[ch] = DWELL_IDLE;
        }

        /* Deshabilitar ambas interrupts hasta el próximo schedule */
        TIMSK1 &= ~((1U << OCIE1A) | (1U << OCIE1B));
    }
}


/* ================================================================
 *  ESTADO SEGURO
 * ================================================================ */
void ignition_safe_state(void)
{
    /* 1. Detener timer de encendido */
    TIMSK1 &= ~((1U << OCIE1A) | (1U << OCIE1B));
    TCCR1B &= ~((1U << CS12) | (1U << CS11) | (1U << CS10));

    /* 2. Apagar todas las bobinas inmediatamente */
    for (uint8_t i = 0U; i < COIL_COUNT; i++) {
        coil_off(i);
        s_dwellState[i] = DWELL_IDLE;
    }

    /* 3. Actualizar estado y pines HIL */
    g_sysState = SYS_SAFE;
    g_diag.safeStateEntries++;
    HIL_STALL_SET();

    /* 4. Reiniciar timer (sin interrupts) para que quede listo para uso futuro */
    TCNT1  = 0U;
    OCR1A  = TIMER1_OCR_MAX;
    OCR1B  = TIMER1_OCR_MAX;
    TCCR1B = (1U << CS11);  /* Normal mode, prescaler 8 */
}


/* ================================================================
 *  HELPERS INTERNOS DE BOBINA (acceso directo a registro para mínima latencia ISR)
 * ================================================================ */
static void coil_on(uint8_t ch)
{
    if (ch >= COIL_COUNT) return;
    g_ignChannels[ch].coilCharging = true;
    if (ch == 0U) {
        COIL_A_ON();
        HIL_COIL_A_SET();
    } else {
        COIL_B_ON();
        HIL_COIL_B_SET();
    }
}

static void coil_off(uint8_t ch)
{
    if (ch >= COIL_COUNT) return;
    g_ignChannels[ch].coilCharging = false;
    if (ch == 0U) {
        COIL_A_OFF();
        HIL_COIL_A_CLR();
    } else {
        COIL_B_OFF();
        HIL_COIL_B_CLR();
    }
}

/* ================================================================
 *  CORTE DE EMERGENCIA DE BOBINA (API pública para uso desde loop)
 * ================================================================ */
void ignition_coil_emergency_off(uint8_t ch)
{
    coil_off(ch);
    g_diag.coilTimeoutEvents++;
}
