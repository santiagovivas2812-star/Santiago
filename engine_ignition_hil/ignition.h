/**
 * ignition.h — Declaraciones del módulo de encendido DIS
 *
 * Define los tipos de datos, contadores de diagnóstico, estado del sistema
 * y los prototipos de funciones del módulo de encendido.
 *
 * Convenciones de acceso seguro a variables volátiles:
 *   - Las variables compartidas entre ISR y loop() están marcadas `volatile`.
 *   - En el código no-ISR, se deben leer usando ATOMIC_BLOCK para garantizar
 *     consistencia en lecturas de tipos > 8 bits en AVR (arquitectura de 8 bits).
 */

#ifndef IGNITION_H
#define IGNITION_H

#include <stdint.h>
#include <stdbool.h>
#include "config.h"


/* ================================================================
 *  ESTADO DEL SISTEMA (máquina de estados de alto nivel)
 * ================================================================ */
typedef enum {
    SYS_OK        = 0,   /**< Funcionamiento normal */
    SYS_STALL     = 1,   /**< Motor parado / sin señal de cigüeñal */
    SYS_SYNC_ERR  = 2,   /**< No se puede adquirir sincronía */
    SYS_OVERREV   = 3,   /**< Protección por sobre-revoluciones activa */
    SYS_COIL_ERR  = 4,   /**< Fallo en driver de bobina */
    SYS_SAFE      = 5    /**< Estado seguro genérico */
} SystemState_t;


/* ================================================================
 *  ESTADO DEL SENSOR DE CIGÜEÑAL
 * ================================================================ */
typedef struct {
    /**
     * Tiempo (µs, resultado de micros()) del último diente válido.
     * Modificado en ISR → volatile. Leer con ATOMIC_BLOCK fuera de ISR.
     */
    volatile uint32_t lastToothTimeUs;

    /**
     * Período entre los dos últimos dientes válidos (µs).
     * Modificado en ISR → volatile.
     */
    volatile uint32_t toothPeriodUs;

    /**
     * Número de diente actual dentro de la vuelta (0 = justo después del hueco).
     * Rango: 0 … EFFECTIVE_TEETH-1.
     */
    volatile uint16_t toothNumber;

    /** RPM filtrado (promedio móvil). Modificado en ISR → volatile. */
    volatile uint16_t rpm;

    /** TRUE una vez que se ha detectado el hueco al menos CRANK_SYNC_MIN_GAPS veces. */
    volatile bool syncAcquired;

    /** TRUE si se perdió la sincronía después de haberla adquirido. */
    volatile bool syncLost;

    /** Contador de huecos detectados desde el último reset de sincronía. */
    volatile uint8_t gapsSeen;
} CrankState_t;


/* ================================================================
 *  ESTADO DE UN CANAL DE ENCENDIDO (bobina)
 * ================================================================ */
typedef struct {
    /** TRUE mientras la bobina está siendo cargada (dwell activo). */
    volatile bool     coilCharging;

    /**
     * Tiempo (µs) en que comenzó la carga de la bobina.
     * Usado para detectar timeouts. Modificado en ISR → volatile.
     */
    volatile uint32_t chargeStartUs;

    /** Tiempo de dwell configurado para este ciclo (µs). */
    uint16_t          dwellUs;

    /** Avance de encendido en grados × 10 (positivo = BTDC). */
    int16_t           advanceDegX10;

    /** Pin Arduino de la salida al driver de bobina. */
    uint8_t           outputPin;

    /**
     * Pin HIL de diagnóstico (espejo del outputPin para osciloscopio).
     * 0xFF = sin pin de debug asignado.
     */
    uint8_t           debugPin;

    /** Primer cilindro del par wasted-spark. */
    uint8_t           cylinderA;

    /** Segundo cilindro del par wasted-spark. */
    uint8_t           cylinderB;
} IgnChannel_t;


/* ================================================================
 *  CONTADORES DE DIAGNÓSTICO
 * ================================================================ */
typedef struct {
    /** Total de dientes válidos contados desde el arranque. */
    volatile uint32_t toothCount;

    /** Pulsos rechazados por el filtro de período mínimo (ruido). */
    volatile uint32_t noisePulses;

    /** Veces que se adquirió sincronía con la rueda fónica. */
    volatile uint32_t syncEvents;

    /** Veces que se perdió la sincronía tras haberla adquirido. */
    volatile uint32_t syncLossEvents;

    /** Detecciones del diente faltante (hueco de referencia). */
    volatile uint32_t gapDetections;

    /**
     * Veces que se activó la protección contra desbordamiento del timer:
     * el retardo calculado excedía TIMER1_MAX_DELAY_US o 65535 ticks.
     */
    volatile uint32_t overflowGuards;

    /** Veces que el dwell solicitado fue recortado al máximo (DWELL_MAX_US). */
    volatile uint32_t dwellClampHigh;

    /** Veces que el dwell solicitado fue elevado al mínimo (DWELL_MIN_US). */
    volatile uint32_t dwellClampLow;

    /** Veces que una bobina fue cortada por timeout de seguridad. */
    volatile uint32_t coilTimeoutEvents;

    /** Veces que se entró al estado seguro (ignition_safe_state). */
    volatile uint32_t safeStateEntries;

    /** Resets por Watchdog detectados en el registro MCUSR al arranque. */
    volatile uint32_t wdtResets;
} DiagCounters_t;


/* ================================================================
 *  VARIABLES GLOBALES (definidas en ignition.cpp)
 * ================================================================ */
extern CrankState_t           g_crankState;
extern IgnChannel_t           g_ignChannels[COIL_COUNT];
extern DiagCounters_t         g_diag;
extern volatile SystemState_t g_sysState;


/* ================================================================
 *  PROTOTIPOS DE FUNCIONES
 * ================================================================ */

/** Inicializa el módulo de encendido: pines, estructuras de datos, timers. */
void ignition_init(void);

/**
 * Debe llamarse desde la ISR del cigüeñal en cada transición válida.
 * @param toothTimeUs  Valor de micros() capturado al inicio de la ISR.
 */
void ignition_tooth_handler(uint32_t toothTimeUs);

/**
 * Programa el Timer1 para iniciar la carga de una bobina transcurridos
 * `delayUs` microsegundos desde ahora.
 *
 * @param channel   Índice del canal (0 … COIL_COUNT-1).
 * @param delayUs   Retardo hasta el inicio del dwell (µs).
 * @param dwellUs   Duración del dwell solicitada (será clampeada).
 * @return TRUE si el evento fue programado exitosamente; FALSE si fue rechazado
 *         (retardo fuera de rango, canal inválido, etc.).
 */
bool ignition_schedule(uint8_t channel, uint32_t delayUs, uint16_t dwellUs);

/**
 * Debe llamarse desde ISR(TIMER1_COMPA_vect) y ISR(TIMER1_COMPB_vect).
 * Gestiona la máquina de estados de dwell (inicio y fin de carga de bobina).
 * @param isCompB  TRUE si se llamó desde COMPB (fin de dwell), FALSE si COMPA (inicio).
 */
void ignition_timer_handler(bool isCompB);

/**
 * Aplica el estado seguro: desactiva todas las bobinas, deshabilita el timer
 * de encendido y actualiza los pines HIL. Puede llamarse desde ISR o desde
 * código normal.
 */
void ignition_safe_state(void);

/**
 * Corte de emergencia de una bobina individual (capa redundante de seguridad).
 * Apaga la bobina del canal indicado sin afectar los demás canales.
 * Es seguro llamar desde loop() o desde ISR.
 *
 * @param ch  Índice del canal (0 … COIL_COUNT-1).
 */
void ignition_coil_emergency_off(uint8_t ch);

/**
 * Calcula el RPM a partir del período de diente y actualiza el filtro
 * de promedio móvil interno.
 *
 * @param periodUs  Período del último diente (µs).
 * @return RPM filtrado.
 */
uint16_t ignition_calc_rpm(uint32_t periodUs);

/**
 * Restringe el dwell dentro de [DWELL_MIN_US, DWELL_MAX_US] y actualiza
 * los contadores de diagnóstico correspondientes.
 *
 * @param requestedUs  Dwell solicitado (µs).
 * @return Dwell efectivo clampeado (µs).
 */
uint16_t ignition_clamp_dwell(uint16_t requestedUs);


/* ================================================================
 *  UTILIDAD INLINE: comparación de tiempos con protección ante wrap-around
 * ================================================================ */
/**
 * Comprueba si el tiempo transcurrido desde `start` hasta `now` (ambos en µs,
 * obtenidos de micros()) supera `threshold`.
 *
 * La sustracción uint32_t es segura ante el wrap-around de micros() (~71 min)
 * porque el complemento a dos hace que (now - start) sea siempre el intervalo
 * correcto mientras el intervalo real no supere 2^32 µs ≈ 71 min.
 *
 * @param now        Tiempo actual (µs).
 * @param start      Tiempo de referencia (µs).
 * @param threshold  Umbral (µs).
 * @return TRUE si (now - start) >= threshold.
 */
static inline bool elapsed_exceeds(uint32_t now, uint32_t start, uint32_t threshold)
{
    return ((uint32_t)(now - start) >= threshold);
}


#endif /* IGNITION_H */
