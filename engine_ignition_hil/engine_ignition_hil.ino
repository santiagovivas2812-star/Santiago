/**
 * engine_ignition_hil.ino — Sketch principal del sistema de encendido DIS
 *
 * Sistema de encendido distributorless (DIS) con chispa desperdiciada (wasted spark)
 * para motor de 4 cilindros. Basado en conceptos de Speeduino, adaptado para
 * pruebas HIL (Hardware-in-the-Loop), banco de pruebas y vehículo.
 *
 * Arquitectura de interrupciones:
 *  - ISR(INT0_vect)         : Sensor de cigüeñal (rising edge, pin 2)
 *  - ISR(TIMER1_COMPA_vect) : Inicio de dwell (encender bobina)
 *  - ISR(TIMER1_COMPB_vect) : Fin de dwell / disparo de chispa (apagar bobina)
 *
 * Ciclo principal (loop):
 *  1. Resetear Watchdog
 *  2. Detección de parada (stall)
 *  3. Protección sobre-revoluciones
 *  4. Timeout de seguridad de bobina (capa redundante)
 *  5. Scheduling de encendido (si hay sincronía y RPM válido)
 *  6. Reporte de diagnóstico por Serie cada 1 segundo
 *
 * Target : Arduino Mega 2560 (ATmega2560 @ 16 MHz)
 * IDE    : Arduino IDE 1.8.x / 2.x
 */

#include <Arduino.h>
#include <avr/wdt.h>
#include <util/atomic.h>
#include "config.h"
#include "ignition.h"


/* ================================================================
 *  PROTOTIPOS DE FUNCIONES LOCALES
 * ================================================================ */
static void check_stall(void);
static void check_overrev(void);
static void check_coil_timeout(void);
static void schedule_ignition(uint16_t rpm, uint16_t toothNumber,
                               uint32_t toothPeriodUs, uint32_t lastToothTimeUs);
static void print_diagnostics(void);


/* ================================================================
 *  ISR: SENSOR DE CIGÜEÑAL (INT0, pin 2, rising edge)
 * ================================================================ */
ISR(INT0_vect)
{
    /*
     * Captura del tiempo al inicio de la ISR.
     * micros() es seguro de llamar dentro de una ISR; las interrupts están
     * deshabilitadas automáticamente al entrar, así que no hay re-entrancia.
     */
    HIL_ISR_SET();              /* Toggle pin A5 para medición de latencia (scope) */
    uint32_t t = micros();
    ignition_tooth_handler(t);
    HIL_ISR_CLR();
}


/* ================================================================
 *  ISR: TIMER1 COMPARE MATCH A — inicio de dwell (bobina ON)
 * ================================================================ */
ISR(TIMER1_COMPA_vect)
{
    ignition_timer_handler(false);
}


/* ================================================================
 *  ISR: TIMER1 COMPARE MATCH B — fin de dwell / chispa (bobina OFF)
 * ================================================================ */
ISR(TIMER1_COMPB_vect)
{
    ignition_timer_handler(true);
}


/* ================================================================
 *  SETUP
 * ================================================================ */
void setup(void)
{
    /*
     * Deshabilitar WDT al inicio para evitar que un reset por WDT
     * loop infinitamente si el problema está en setup().
     */
    wdt_disable();

    /*
     * Leer y guardar la fuente del último reset ANTES de limpiar MCUSR.
     * MCUSR debe limpiarse antes de deshabilitar el WDT para asegurar
     * que el flag WDRF sea visible en el siguiente arranque.
     */
    uint8_t mcusr_snapshot = MCUSR;
    MCUSR = 0U;

    /* ---- Puerto serie (diagnóstico / HIL) ---- */
    Serial.begin(115200);
    while (!Serial) { /* Esperar a que el puerto esté listo (Leonardo/Mega USB) */ }

    Serial.println(F("============================================"));
    Serial.println(F("  DIS Ignition System — HIL / Bench Test"));
    Serial.println(F("  Target: Arduino Mega 2560  (Speeduino-based)"));
    Serial.println(F("============================================"));
    Serial.print(F("MCUSR reset flags: 0x"));
    Serial.println(mcusr_snapshot, HEX);

    /* ---- Pines HIL de diagnóstico ---- */
    /* Dirección OUTPUT + nivel inicial vía PORTF (A0-A5) */
    DDRF  |= (1U << 0) | (1U << 1) | (1U << 2) |
             (1U << 3) | (1U << 4) | (1U << 5);
    PORTF &= ~((1U << 0) | (1U << 1) | (1U << 2) |
               (1U << 3) | (1U << 4) | (1U << 5));
    /* STALL_FLAG empieza en ALTO (estado de parada al arranque) */
    HIL_STALL_SET();

    /* ---- Sensor de cigüeñal ---- */
    pinMode(PIN_CRANK_SENSOR, INPUT_PULLUP);

    /*
     * Configurar INT0 manualmente para mayor control:
     * EICRA bits ISC01=1, ISC00=1 → flanco de subida en INT0.
     * EIMSK bit INT0=1           → habilitar la interrupción.
     *
     * No se usa attachInterrupt() para evitar el overhead de la envoltura
     * de Arduino y tener acceso directo al vector ISR(INT0_vect).
     */
    EICRA |= (1U << ISC01) | (1U << ISC00);
    EIMSK |= (1U << INT0);

    /* ---- Inicializar módulo de encendido ---- */
    ignition_init();

    /*
     * Aplicar estado seguro inicial: bobinas apagadas, timer detenido.
     * Esto cubre el caso en que el MCU reinicia mientras una bobina
     * estaba energizada.
     */
    ignition_safe_state();
    g_sysState = SYS_STALL;  /* Sobrescribir SYS_SAFE de safe_state */

    /*
     * Procesar flag de reset por Watchdog DESPUÉS de ignition_safe_state()
     * para que el contador refleje resets reales, no el arranque limpio.
     */
    if (mcusr_snapshot & (1U << WDRF)) {
        Serial.println(F("ADVERTENCIA: Reset por Watchdog detectado"));
        /* g_diag.wdtResets se lee aquí — incrementar en setup, no en ISR */
        ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
            g_diag.wdtResets++;
        }
    }

    /* ---- Habilitar Watchdog de hardware ---- */
    wdt_enable(WATCHDOG_TIMEOUT);

    Serial.println(F("Setup OK. Esperando señal de cigüeñal..."));
    Serial.println();
}


/* ================================================================
 *  LOOP PRINCIPAL
 * ================================================================ */
void loop(void)
{
    /* 1. Resetear Watchdog — debe hacerse al inicio de cada iteración */
    wdt_reset();

    /* 2. Lectura atómica del estado del cigüeñal */
    uint16_t rpm;
    uint16_t toothNumber;
    uint32_t toothPeriodUs;
    uint32_t lastToothTimeUs;
    bool     syncAcquired;

    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        rpm             = g_crankState.rpm;
        toothNumber     = g_crankState.toothNumber;
        toothPeriodUs   = g_crankState.toothPeriodUs;
        lastToothTimeUs = g_crankState.lastToothTimeUs;
        syncAcquired    = g_crankState.syncAcquired;
    }

    /* 3. Detección de parada */
    check_stall();

    /* 4. Protección sobre-revoluciones */
    check_overrev();

    /* 5. Timeout de seguridad de bobina (capa redundante al timer ISR) */
    check_coil_timeout();

    /* 6. Scheduling de encendido */
    if (syncAcquired && (rpm >= RPM_MIN) && (rpm <= RPM_MAX)) {
        if (g_sysState == SYS_STALL || g_sysState == SYS_SAFE) {
            g_sysState = SYS_OK;
            HIL_STALL_CLR();
            Serial.println(F("Motor en marcha — encendido activo"));
        }
        schedule_ignition(rpm, toothNumber, toothPeriodUs, lastToothTimeUs);
    }

    /* 7. Reporte de diagnóstico periódico */
    static uint32_t s_lastDiagMs = 0UL;
    uint32_t nowMs = millis();
    if ((uint32_t)(nowMs - s_lastDiagMs) >= DIAG_REPORT_INTERVAL_MS) {
        s_lastDiagMs = nowMs;
        print_diagnostics();
    }
}


/* ================================================================
 *  DETECCIÓN DE PARADA (STALL)
 * ================================================================ */
static void check_stall(void)
{
    uint32_t lastTooth;
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        lastTooth = g_crankState.lastToothTimeUs;
    }

    /*
     * elapsed_exceeds usa sustracción uint32_t: segura ante el wrap-around
     * de micros() (~71 minutos). El intervalo de interés (20 ms) está muy por
     * debajo del período de wrap, por lo que la comparación es siempre válida.
     */
    if (elapsed_exceeds(micros(), lastTooth, STALL_TIMEOUT_US)) {
        if (g_sysState != SYS_STALL) {
            ignition_safe_state();
            g_sysState = SYS_STALL;

            ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
                g_crankState.syncAcquired  = false;
                g_crankState.syncLost      = true;
                g_crankState.rpm           = 0U;
                g_crankState.gapsSeen      = 0U;
                g_crankState.toothNumber   = 0U;
            }

            HIL_SYNC_CLR();
            HIL_STALL_SET();
            Serial.println(F("PARADA detectada — estado seguro aplicado"));
        }
    }
}


/* ================================================================
 *  PROTECCIÓN SOBRE-REVOLUCIONES
 * ================================================================ */
static void check_overrev(void)
{
    uint16_t rpm;
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        rpm = g_crankState.rpm;
    }

    if ((rpm > RPM_MAX) && (g_sysState != SYS_OVERREV)) {
        ignition_safe_state();
        g_sysState = SYS_OVERREV;
        Serial.print(F("SOBRE-REV: "));
        Serial.print(rpm);
        Serial.println(F(" rpm — corte de encendido"));
    }

    /* Recuperación automática al bajar de RPM_MAX */
    if ((g_sysState == SYS_OVERREV) && (rpm <= RPM_MAX)) {
        g_sysState = SYS_OK;
        Serial.println(F("Sobre-rev superado — encendido restablecido"));
    }
}


/* ================================================================
 *  TIMEOUT DE SEGURIDAD DE BOBINA (capa redundante en loop)
 * ================================================================ */
static void check_coil_timeout(void)
{
    uint32_t nowUs = micros();

    for (uint8_t i = 0U; i < COIL_COUNT; i++) {
        bool     charging;
        uint32_t startUs;

        ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
            charging = g_ignChannels[i].coilCharging;
            startUs  = g_ignChannels[i].chargeStartUs;
        }

        if (charging && elapsed_exceeds(nowUs, startUs, COIL_EMERGENCY_CUTOFF_US)) {
            /*
             * La bobina lleva demasiado tiempo cargando.
             * Esto indica un fallo en el scheduling del timer; se corta
             * desde el loop como medida de último recurso.
             * ignition_coil_emergency_off() usa acceso directo a registro
             * y actualiza g_diag.coilTimeoutEvents atómicamente.
             */
            ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
                ignition_coil_emergency_off(i);
            }
            Serial.print(F("TIMEOUT BOBINA canal "));
            Serial.println(i);
        }
    }
}


/* ================================================================
 *  SCHEDULING DE ENCENDIDO
 *
 * Implementación simplificada para pruebas HIL con avance fijo.
 * En una implementación completa se reemplazaría el avance fijo por
 * una tabla 2D (RPM × carga) interpolada fuera de la ISR.
 *
 * Modelo de timing para rueda 36-1, 4 cilindros, wasted spark:
 *   - Hay 4 eventos de ignición por vuelta de cigüeñal (cada 90°).
 *   - Coil A (cil. 1&4) dispara en los dientes 0 y 18 (0° y 180°).
 *   - Coil B (cil. 2&3) dispara en los dientes 9 y 27 (90° y 270°).
 *   - El dwell empieza `advanceDeg + dwell_deg` antes del punto de disparo.
 * ================================================================ */
static void schedule_ignition(uint16_t rpm, uint16_t toothNumber,
                               uint32_t toothPeriodUs, uint32_t lastToothTimeUs)
{
    /*
     * Puntos de disparo (diente en el que debe ocurrir la chispa),
     * expresados como número de diente desde el hueco (0…34).
     * Con avance fijo de 10° BTDC y dientes de 10°/cada uno:
     *   TDC cilindros 1&4 → diente 0 → disparo en diente 35 (= 0 − 1) = previo al hueco
     *   Para simplificar, se usa diente 34 (último antes del hueco) como referencia.
     * NOTA: En producción, calcular el diente exacto desde la tabla de avance.
     */
    static const uint8_t FIRE_TOOTH[COIL_COUNT][2] = {
        { 34U, 16U },   /* Coil A: aprox. TDC cil.1 (diente 34) y cil.4 (diente 16) */
        { 7U,  25U }    /* Coil B: aprox. TDC cil.2 (diente 7)  y cil.3 (diente 25) */
    };

    /* Tiempo estimado por diente (mismo que el período actual del último diente) */
    if (toothPeriodUs == 0UL) return;

    uint32_t nowUs = micros();

    for (uint8_t ch = 0U; ch < COIL_COUNT; ch++) {
        for (uint8_t ev = 0U; ev < 2U; ev++) {
            uint8_t fireTooth = FIRE_TOOTH[ch][ev];

            /* Dientes hasta el evento de disparo desde el diente actual */
            uint8_t toothsAhead;
            if (fireTooth >= toothNumber) {
                toothsAhead = (uint8_t)(fireTooth - toothNumber);
            } else {
                toothsAhead = (uint8_t)((uint16_t)EFFECTIVE_TEETH - toothNumber + fireTooth);
            }

            /* Retardo hasta el disparo (µs) */
            uint32_t fireDelayUs = (uint32_t)toothsAhead * toothPeriodUs;

            /*
             * Retardo hasta el inicio del dwell:
             * fireDelay − dwell − tiempo ya transcurrido desde el último diente.
             */
            uint32_t elapsedUs = (uint32_t)(nowUs - lastToothTimeUs);
            uint32_t dwellUs   = (uint32_t)g_ignChannels[ch].dwellUs;

            if (fireDelayUs <= (dwellUs + elapsedUs)) {
                /* Evento demasiado próximo o ya pasado — omitir */
                continue;
            }

            uint32_t dwellStartDelayUs = fireDelayUs - dwellUs - elapsedUs;

            /*
             * Solo programar si el retardo está dentro del rango del timer.
             * Si es demasiado largo, se re-evaluará en la próxima iteración del loop.
             */
            if (dwellStartDelayUs > (uint32_t)TIMER1_MAX_DELAY_US) {
                continue;
            }

            /* Solo programar el evento más próximo que no esté ya en progreso */
            if (toothsAhead <= 2U) {
                ignition_schedule(ch, dwellStartDelayUs, (uint16_t)dwellUs);
                break;   /* Un evento por canal por iteración del loop */
            }
        }
    }
}


/* ================================================================
 *  REPORTE DE DIAGNÓSTICO POR SERIE
 * ================================================================ */
static void print_diagnostics(void)
{
    /* Snapshot atómico de todo el estado en una sola sección crítica */
    DiagCounters_t diag;
    uint16_t       rpm;
    uint16_t       toothNum;
    bool           syncAcquired;
    SystemState_t  state;

    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        diag         = g_diag;
        rpm          = g_crankState.rpm;
        toothNum     = g_crankState.toothNumber;
        syncAcquired = g_crankState.syncAcquired;
        state        = g_sysState;
    }

    Serial.println(F("--- Diagnóstico DIS ---"));

    Serial.print(F("Estado: "));
    switch (state) {
        case SYS_OK:       Serial.println(F("OK"));          break;
        case SYS_STALL:    Serial.println(F("PARADA"));      break;
        case SYS_SYNC_ERR: Serial.println(F("ERROR_SYNC"));  break;
        case SYS_OVERREV:  Serial.println(F("SOBRE-REV"));   break;
        case SYS_COIL_ERR: Serial.println(F("FALLO_BOBINA")); break;
        case SYS_SAFE:     Serial.println(F("SEGURO"));      break;
        default:           Serial.println(F("DESCONOCIDO")); break;
    }

    Serial.print(F("RPM:               ")); Serial.println(rpm);
    Serial.print(F("Sincronía:         ")); Serial.println(syncAcquired ? F("SI") : F("NO"));
    Serial.print(F("Diente actual:     ")); Serial.println(toothNum);
    Serial.print(F("Dientes totales:   ")); Serial.println(diag.toothCount);
    Serial.print(F("Pulsos ruido:      ")); Serial.println(diag.noisePulses);
    Serial.print(F("Eventos sync:      ")); Serial.println(diag.syncEvents);
    Serial.print(F("Pérdidas sync:     ")); Serial.println(diag.syncLossEvents);
    Serial.print(F("Huecos detectados: ")); Serial.println(diag.gapDetections);
    Serial.print(F("Guardas overflow:  ")); Serial.println(diag.overflowGuards);
    Serial.print(F("Dwell clamp alto:  ")); Serial.println(diag.dwellClampHigh);
    Serial.print(F("Dwell clamp bajo:  ")); Serial.println(diag.dwellClampLow);
    Serial.print(F("Timeout bobina:    ")); Serial.println(diag.coilTimeoutEvents);
    Serial.print(F("Entradas seg.:     ")); Serial.println(diag.safeStateEntries);
    Serial.print(F("Resets WDT:        ")); Serial.println(diag.wdtResets);
    Serial.println();
}
