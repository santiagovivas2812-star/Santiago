/**
 * config.h — Configuración del sistema de encendido DIS (Speeduino-based)
 *
 * Todas las constantes y macros de tiempo de compilación se definen aquí
 * para minimizar la latencia en las ISR. No se realiza ningún cálculo en
 * punto flotante dentro de las ISRs; todo se resuelve como macro o constante
 * entera en tiempo de compilación.
 *
 * Target: Arduino Mega 2560 (ATmega2560, 16 MHz)
 * Motor:  4 cilindros, chispa desperdiciada (wasted spark), rueda 36-1
 */

#ifndef CONFIG_H
#define CONFIG_H

/* ================================================================
 *  HARDWARE / CPU
 * ================================================================ */
#define CPU_FREQ_HZ             16000000UL   /**< Frecuencia del CPU (Hz) */

/** Prescaler de Timer1 usado para scheduling de encendido */
#define TIMER1_PRESCALER        8U

/**
 * Ticks de Timer1 por microsegundo.
 * Con F_CPU=16 MHz y prescaler=8: 16.000.000/8/1.000.000 = 2 ticks/µs
 */
#define TIMER1_TICKS_PER_US     ((uint16_t)(CPU_FREQ_HZ / TIMER1_PRESCALER / 1000000UL))

/** Valor máximo del registro OCR1x (16 bits) */
#define TIMER1_OCR_MAX          65535U

/** Máximo retardo programable en el timer (µs): 65535 / 2 = 32767 µs */
#define TIMER1_MAX_DELAY_US     (TIMER1_OCR_MAX / TIMER1_TICKS_PER_US)


/* ================================================================
 *  PARÁMETROS DEL MOTOR
 * ================================================================ */
#define CYLINDER_COUNT          4U           /**< Número de cilindros */
#define COIL_COUNT              2U           /**< Bobinas (wasted spark: 2 para 4 cil.) */

/** Dientes totales de la rueda fónica (incluyendo el/los faltantes) */
#define TEETH_PER_REV           36U
/** Dientes faltantes en la rueda (gap de referencia) */
#define MISSING_TEETH           1U
/** Dientes reales presentes en la rueda */
#define EFFECTIVE_TEETH         (TEETH_PER_REV - MISSING_TEETH)   /* 35 */

/**
 * Grados de cigüeñal por diente (x100 para evitar punto flotante).
 * 360° / 36 dientes = 10,00°/diente → representado como 1000 (= 10.00 × 100).
 */
#define DEGREES_PER_TOOTH_X100  (36000U / TEETH_PER_REV)          /* 1000 */


/* ================================================================
 *  LÍMITES DE RPM
 * ================================================================ */
#define RPM_MIN                 300U         /**< Por debajo: parada del motor */
#define RPM_MAX                 7000U        /**< Por encima: protección sobre-rev */
#define RPM_IDLE_NOMINAL        800U         /**< RPM de ralentí esperado */


/* ================================================================
 *  DETECCIÓN DE PARADA (STALL)
 * ================================================================ */
/**
 * Tiempo sin dientes antes de declarar parada (µs).
 * A RPM_MIN=300 rpm, período por diente = 60×10⁶ / (300×36) ≈ 5555 µs.
 * Se usa ×3 como margen: 20 000 µs ≈ 16 rpm mínimo real.
 */
#define STALL_TIMEOUT_US        20000UL


/* ================================================================
 *  CONTROL DE DWELL (tiempo de carga de bobina)
 * ================================================================ */
#define DWELL_DEFAULT_US        3500UL       /**< Dwell nominal (µs) */
#define DWELL_MIN_US            2000UL       /**< Mínimo dwell permitido */
#define DWELL_MAX_US            5000UL       /**< Máximo dwell permitido */

/**
 * Ticks de Timer1 para el dwell máximo.
 * Calculado en tiempo de compilación: 5000 µs × 2 ticks/µs = 10 000 ticks.
 */
#define DWELL_MAX_TICKS         ((uint32_t)DWELL_MAX_US * TIMER1_TICKS_PER_US)

/**
 * Timeout de seguridad para corte de emergencia de bobina (µs).
 * = DWELL_MAX + 1 ms de margen para tolerar jitter del scheduler.
 */
#define COIL_EMERGENCY_CUTOFF_US   (DWELL_MAX_US + 1000UL)


/* ================================================================
 *  AVANCE DE ENCENDIDO (fixed para esta versión HIL)
 * ================================================================ */
/** Avance por defecto en grados × 10 (10,0° BTDC) */
#define ADVANCE_DEFAULT_DEG_X10    100
/** Límite de retardo máximo en grados × 10 (−5,0° = 5° ATDC) */
#define ADVANCE_MIN_DEG_X10        (-50)
/** Límite de avance máximo en grados × 10 (40,0° BTDC) */
#define ADVANCE_MAX_DEG_X10        400


/* ================================================================
 *  FILTRO DE RUIDO (sensor de cigüeñal)
 * ================================================================ */
/**
 * Período mínimo válido entre dientes (µs).
 * A RPM_MAX=7000 rpm, 36 dientes: período = 60×10⁶/(7000×36) ≈ 238 µs.
 * Se usa el 80%: 190 µs. Pulsos más cortos se rechazan como ruido.
 */
#define MIN_TOOTH_PERIOD_US     190UL

/**
 * Umbral para detección del diente faltante (factor ×100).
 * En una rueda 36-1, el hueco mide ≈2× el período normal.
 * Se usa 1,5× como umbral con margen ante variación de velocidad.
 * Aritmética: si (período × 100) > (períodoPrev × 150) → es el hueco.
 */
#define MISSING_TOOTH_RATIO_X100   150U


/* ================================================================
 *  SINCRONÍA DE CIGÜEÑAL
 * ================================================================ */
/** Número mínimo de detecciones del hueco antes de declarar sincronía */
#define CRANK_SYNC_MIN_GAPS     2U


/* ================================================================
 *  ASIGNACIÓN DE PINES (Arduino Mega 2560)
 * ================================================================ */
/* Entradas */
#define PIN_CRANK_SENSOR        2    /**< INT0 — sensor de cigüeñal (VR / Hall) */
#define PIN_CAM_SENSOR          3    /**< INT1 — sensor de árbol de levas (opcional) */

/* Salidas de bobinas */
#define PIN_COIL_A              8    /**< Bobina A — cilindros 1 y 4 (wasted spark) */
#define PIN_COIL_B              9    /**< Bobina B — cilindros 2 y 3 (wasted spark) */

/* Pines HIL de diagnóstico / instrumentación (conector de pruebas) */
#define PIN_HIL_TOOTH_PULSE     A0   /**< Toggle en cada diente válido */
#define PIN_HIL_COIL_A_DBG      A1   /**< Espejo de bobina A */
#define PIN_HIL_COIL_B_DBG      A2   /**< Espejo de bobina B */
#define PIN_HIL_SYNC_STATE      A3   /**< ALTO = sincronía adquirida */
#define PIN_HIL_STALL_FLAG      A4   /**< ALTO = parada / estado seguro */
#define PIN_HIL_ISR_TIMING      A5   /**< Toggle en entrada/salida de ISR de cigüeñal */


/* ================================================================
 *  MACROS DE ACCESO DIRECTO A REGISTRO PARA PINES DE BOBINA
 *  (Arduino Mega 2560: D8 → PH5, D9 → PH6)
 *  Uso en las ISR de Timer1 para minimizar latencia.
 * ================================================================ */
#define COIL_A_ON()   (PORTH |=  (1U << 5))   /**< D8 (PH5) HIGH — bobina A ON  */
#define COIL_A_OFF()  (PORTH &= ~(1U << 5))   /**< D8 (PH5) LOW  — bobina A OFF */
#define COIL_B_ON()   (PORTH |=  (1U << 6))   /**< D9 (PH6) HIGH — bobina B ON  */
#define COIL_B_OFF()  (PORTH &= ~(1U << 6))   /**< D9 (PH6) LOW  — bobina B OFF */


/* ================================================================
 *  MACROS DE ACCESO DIRECTO A REGISTRO PARA PINES HIL
 *  (Arduino Mega 2560: A0-A7 → PORTF bits 0-7)
 *  Se usan dentro de la ISR para minimizar latencia evitando
 *  la sobrecarga de digitalWrite().
 * ================================================================ */
#define HIL_TOOTH_TOG()   (PORTF ^=  (1U << 0))   /**< Toggle A0 */
#define HIL_COIL_A_SET()  (PORTF |=  (1U << 1))   /**< Set A1 HIGH */
#define HIL_COIL_A_CLR()  (PORTF &= ~(1U << 1))   /**< Set A1 LOW  */
#define HIL_COIL_B_SET()  (PORTF |=  (1U << 2))   /**< Set A2 HIGH */
#define HIL_COIL_B_CLR()  (PORTF &= ~(1U << 2))   /**< Set A2 LOW  */
#define HIL_SYNC_SET()    (PORTF |=  (1U << 3))   /**< Sync HIGH */
#define HIL_SYNC_CLR()    (PORTF &= ~(1U << 3))   /**< Sync LOW  */
#define HIL_STALL_SET()   (PORTF |=  (1U << 4))   /**< Stall HIGH */
#define HIL_STALL_CLR()   (PORTF &= ~(1U << 4))   /**< Stall LOW  */
#define HIL_ISR_SET()     (PORTF |=  (1U << 5))   /**< ISR timing HIGH */
#define HIL_ISR_CLR()     (PORTF &= ~(1U << 5))   /**< ISR timing LOW  */


/* ================================================================
 *  WATCHDOG Y ESTADO SEGURO
 * ================================================================ */
/** Timeout del Watchdog de hardware (ver <avr/wdt.h> para valores válidos) */
#define WATCHDOG_TIMEOUT        WDTO_250MS


/* ================================================================
 *  DIAGNÓSTICO
 * ================================================================ */
/** Intervalo de reporte por puerto serie (ms) */
#define DIAG_REPORT_INTERVAL_MS   1000UL


/* ================================================================
 *  FILTRO RPM
 * ================================================================ */
/** Número de muestras para el promedio móvil de RPM */
#define RPM_FILTER_SAMPLES      4U


#endif /* CONFIG_H */
