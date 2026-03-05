/**
 * config.h — Configuración del sistema de encendido DIS optimizado
 *
 * Todas las operaciones de punto flotante y multiplicaciones se precalculan
 * como macros en tiempo de compilación para mantener la latencia de ISR mínima.
 *
 * Plataforma: Arduino Mega 2560 / ATmega2560
 * Sistema:    DIS (Direct Ignition System) — 4 cilindros
 */

#ifndef CONFIG_H
#define CONFIG_H

// ─────────────────────────────────────────────────
//  Pines de salida — bobinas de encendido DIS
// ─────────────────────────────────────────────────
#define PIN_COIL_1    2   // Cilindro 1 — par 1-4
#define PIN_COIL_2    3   // Cilindro 2 — par 2-3
#define PIN_COIL_3    4   // Cilindro 3 — par 2-3 (redundante DIS)
#define PIN_COIL_4    5   // Cilindro 4 — par 1-4 (redundante DIS)

// ─────────────────────────────────────────────────
//  Pines de entrada
// ─────────────────────────────────────────────────
#define PIN_CKP       18  // Sensor CKP (interrupción INT3)
#define PIN_CMP       19  // Sensor CMP / fase (interrupción INT2)
#define PIN_TPS       A0  // Posición del acelerador (TPS) — ADC
#define PIN_MAP       A1  // Presión múltiple de admisión (MAP) — ADC
#define PIN_CLT       A2  // Temperatura refrigerante (CLT) — ADC

// ─────────────────────────────────────────────────
//  Pines de diagnóstico HIL (Hardware-in-the-Loop)
// ─────────────────────────────────────────────────
#define PIN_DBG_ISR   22  // Toggle en cada ISR CKP — mide latencia
#define PIN_DBG_FIRE  23  // Toggle en disparo de bobina
#define PIN_DBG_SAFE  24  // Alto = safe state activo

// ─────────────────────────────────────────────────
//  Parámetros del motor y rueda fónica
// ─────────────────────────────────────────────────
#define TEETH_TOTAL         60    // Dientes totales rueda fónica (60-2)
#define TEETH_MISSING        2    // Dientes faltantes
#define TEETH_EFFECTIVE     (TEETH_TOTAL - TEETH_MISSING)   // 58
#define CYLINDERS            4
#define STROKES_PER_CYCLE    2    // Motor 4 tiempos

// Grados por diente (con flotante solo en compilación)
#define DEG_PER_TOOTH       (360.0 / TEETH_TOTAL)           // 6.0°
#define DEG_PER_TOOTH_INT   6                               // entero, sin división en runtime

// Evento de encendido cada 180° (4 cil, 2 tiempos)
#define FIRE_INTERVAL_DEG   180
#define FIRE_INTERVAL_TEETH (FIRE_INTERVAL_DEG / DEG_PER_TOOTH_INT)  // 30 dientes

// ─────────────────────────────────────────────────
//  Timing de encendido — límites y defaults
// ─────────────────────────────────────────────────
#define ADVANCE_MIN_DEG      0    // Mínimo avance permitido
#define ADVANCE_MAX_DEG     45    // Máximo avance permitido
#define ADVANCE_DEFAULT_DEG 10    // Avance seguro en safe state
#define DWELL_DEFAULT_US  3000    // Dwell por defecto [µs]
#define DWELL_MIN_US      1500    // Dwell mínimo [µs]
#define DWELL_MAX_US      4500    // Dwell máximo [µs]

// ─────────────────────────────────────────────────
//  Timer 1 — configuración
//  Prescaler 8 → resolución = 0.5 µs @ 16 MHz
// ─────────────────────────────────────────────────
#define TIMER1_PRESCALER      8
#define TIMER1_TICKS_PER_US   (F_CPU / TIMER1_PRESCALER / 1000000UL)  // 2
#define US_TO_TICKS(us)       ((us) * TIMER1_TICKS_PER_US)
#define TICKS_TO_US(t)        ((t) / TIMER1_TICKS_PER_US)

// Reducción de dwell por cada 1000 RPM sobre 4000 (10 % del dwell por defecto)
// 10% de DWELL_DEFAULT_TICKS = 6000 / 10 = 600 ticks = 300 µs por cada 1000 RPM
#define DWELL_REDUCTION_TICKS_PER_1K_RPM  (DWELL_DEFAULT_TICKS / 10)

#define DWELL_DEFAULT_TICKS   US_TO_TICKS(DWELL_DEFAULT_US)  // 6000 ticks
#define DWELL_MIN_TICKS       US_TO_TICKS(DWELL_MIN_US)
#define DWELL_MAX_TICKS       US_TO_TICKS(DWELL_MAX_US)

// ─────────────────────────────────────────────────
//  Detección de condición de seguridad
// ─────────────────────────────────────────────────
// Sin pulso CKP por más de este tiempo → safe state
#define RPM_STALL_TIMEOUT_MS  500
// Número de errores consecutivos antes de safe state
#define MAX_CONSECUTIVE_ERRORS 5
// Tiempo máximo dwell absoluto (bobina no puede quedar energizada)
#define COIL_MAX_ON_TICKS     US_TO_TICKS(6000)

// ADC — escalado precomputado (sin float en runtime)
// ─────────────────────────────────────────────────────────────────────────────
// TPS: 0–5 V → 0–100% → 0–1023 ADC
// MAP: 0–5 V → 10–110 kPa (sensor 1 bar, 10 kPa offset)
//   Fórmula exacta: kPa = ADC * 100 / 1023 + 10
//   Aproximación entera usada: kPa = ADC / 10 + 10
//   Error máximo: < 0.8 kPa (0.8%) — aceptable para scheduling de encendido.
//   Se eligió por evitar la división por 1023 (no potencia de 2) en cada ciclo.
#define ADC_TO_TPS_PCT(adc)   ((adc) / 10)          // 0-100 %
#define ADC_TO_MAP_KPA(adc)   ((adc) / 10 + 10)     // 10-112 kPa aprox.
// CLT: NTC 2.49 kΩ pull-up, tabla lookup (ver clt_table[] en .ino)

// ─────────────────────────────────────────────────
//  OLED — U8g2 128×64 I2C
// ─────────────────────────────────────────────────
#define OLED_UPDATE_PERIOD_MS  150   // Refresco OLED [ms]
#define OLED_I2C_ADDR         0x3C  // Dirección I2C display

// ─────────────────────────────────────────────────
//  Corrección modular (trim de avance por cilindro)
// ─────────────────────────────────────────────────
// Offset individual por cilindro en décimas de grado (±30 = ±3.0°)
// Se suma al avance global calculado
#define TRIM_CYL_1    0    // décimas de grado
#define TRIM_CYL_2    0
#define TRIM_CYL_3    0
#define TRIM_CYL_4    0
#define TRIM_SCALE    10   // divisor para pasar a grados

// ─────────────────────────────────────────────────
//  Orden de encendido
// ─────────────────────────────────────────────────
// Motor 4 cil — orden 1-3-4-2
static const uint8_t FIRE_ORDER[CYLINDERS] = {0, 2, 3, 1}; // índice bobina

#endif // CONFIG_H
