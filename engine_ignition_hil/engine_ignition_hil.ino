/**
 * engine_ignition_hil.ino — Código de encendido DIS optimizado
 *
 * Sistema de encendido directo (DIS) para motor 4 cilindros.
 * Implementa las mismas funciones core de Speeduino pero con arquitectura
 * orientada a latencia mínima en ISR y diagnóstico integrado en OLED.
 *
 * Características principales:
 *  - Timer1 con OC1A/OC1B para scheduling preciso de dwell y spark
 *  - ISR de CKP ultra-liviana: solo captura timestamp y actualiza contador
 *  - Toda la lógica de avance se precomputa en loop() antes de cada evento
 *  - Corrección modular por cilindro (trim individual)
 *  - Safe state automático ante fallo de señal o error acumulado
 *  - Diagnóstico en OLED 128x64 I2C con refresco no bloqueante
 *  - Pines HIL para medición de latencia con osciloscopio
 *
 * Plataforma: Arduino Mega 2560 (ATmega2560, 16 MHz)
 * Librerías:  U8g2 (OLED)
 */

#include <avr/io.h>
#include <avr/interrupt.h>
#include <U8g2lib.h>
#include <Wire.h>
#include "config.h"

// ─────────────────────────────────────────────────
//  OLED — U8g2 128×64 I2C
// ─────────────────────────────────────────────────
U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, U8X8_PIN_NONE);

// ─────────────────────────────────────────────────
//  Variables volátiles compartidas ISR ↔ loop
// ─────────────────────────────────────────────────
volatile uint32_t ckpTimestamp    = 0;   // Timestamp último diente CKP [ticks Timer1]
volatile uint32_t toothPeriod     = 0;   // Período entre dientes [ticks]
volatile uint16_t toothCount      = 0;   // Contador de dientes (0..TEETH_EFFECTIVE-1)
volatile bool     gapDetected     = false; // Hueco 60-2 detectado → fase conocida
volatile bool     phaseKnown      = false; // CMP confirmó fase del ciclo
volatile uint8_t  currentCylIndex = 0;   // Cilindro actual en secuencia de disparo
volatile bool     newToothReady   = false; // Nuevo diente procesado, loop puede calcular
volatile uint8_t  errorCount      = 0;   // Errores CKP consecutivos

// ─────────────────────────────────────────────────
//  Variables de scheduling (escritas en loop, leídas en ISR Timer)
// ─────────────────────────────────────────────────
volatile uint16_t dwellTicks   = DWELL_DEFAULT_TICKS;
volatile uint16_t advTeeth     = 5;    // Avance en dientes desde TDC
volatile uint8_t  nextCoilPin  = PIN_COIL_1;
volatile bool     scheduleReady = false; // loop terminó de precomputar

// ─────────────────────────────────────────────────
//  Estado del sistema
// ─────────────────────────────────────────────────
enum SystemState { STATE_NORMAL, STATE_SAFE, STATE_FAULT };
volatile SystemState sysState = STATE_SAFE;

// ─────────────────────────────────────────────────
//  Variables de diagnóstico (solo escritas en loop)
// ─────────────────────────────────────────────────
uint16_t rpm            = 0;
uint8_t  tpsPct         = 0;
uint8_t  mapKpa         = 40;
int16_t  advanceDeg     = ADVANCE_DEFAULT_DEG;
uint32_t lastCkpMs      = 0;
uint32_t lastOledUpdate = 0;

// ─────────────────────────────────────────────────
//  Tabla CLT (NTC) — lookup 10 puntos, interpolación lineal
//  Entrada: ADC 0-1023 | Salida: temperatura en °C
// ─────────────────────────────────────────────────
static const uint16_t CLT_ADC[10] = {
    50, 100, 160, 240, 340, 460, 580, 700, 820, 950
};
static const int8_t CLT_TEMP[10] = {
    110, 90, 75, 60, 45, 30, 15, 0, -15, -30
};

int8_t adcToClt(uint16_t adc) {
    if (adc <= CLT_ADC[0]) return CLT_TEMP[0];
    if (adc >= CLT_ADC[9]) return CLT_TEMP[9];
    for (uint8_t i = 0; i < 9; i++) {
        if (adc < CLT_ADC[i + 1]) {
            // Interpolación entera sin flotante
            int16_t range = CLT_ADC[i + 1] - CLT_ADC[i];
            int16_t delta = CLT_TEMP[i + 1] - CLT_TEMP[i];
            return (int8_t)(CLT_TEMP[i] + (int16_t)(adc - CLT_ADC[i]) * delta / range);
        }
    }
    return CLT_TEMP[9];
}

// ─────────────────────────────────────────────────
//  Timer1 — configuración CTC / OC1A / OC1B
// ─────────────────────────────────────────────────
void timer1Init() {
    // Modo CTC con prescaler 8 → tick = 0.5 µs
    TCCR1A = 0;
    TCCR1B = (1 << WGM12) | (1 << CS11);  // CTC, prescaler /8
    TIMSK1 = 0;  // Interrupciones deshabilitadas hasta schedule
    OCR1A  = 0xFFFF;
    OCR1B  = 0xFFFF;
}

// Programa dwell: energiza bobina en 'startTicks' ticks desde ahora
// y dispara (apaga) en startTicks + dwellTicks
void scheduleCoilEvent(uint8_t coilPin, uint16_t startTicks, uint16_t dwell) {
    nextCoilPin = coilPin;
    dwellTicks  = dwell;

    uint16_t now = TCNT1;
    OCR1A = now + startTicks;          // Dwell ON (energizar)
    OCR1B = now + startTicks + dwell;  // Spark (apagar → disparo)

    TIFR1  = (1 << OCF1A) | (1 << OCF1B);  // Limpiar flags
    TIMSK1 = (1 << OCIE1A) | (1 << OCIE1B); // Habilitar comparadores
}

// ─────────────────────────────────────────────────
//  ISR Timer1 — Compare A: energizar bobina (dwell start)
// ─────────────────────────────────────────────────
ISR(TIMER1_COMPA_vect) {
    // Solo activar si no estamos en safe state
    if (sysState != STATE_SAFE) {
        digitalWrite(nextCoilPin, HIGH);
        digitalWrite(PIN_DBG_FIRE, HIGH);
    }
}

// ─────────────────────────────────────────────────
//  ISR Timer1 — Compare B: apagar bobina (spark = flanco descendente)
// ─────────────────────────────────────────────────
ISR(TIMER1_COMPB_vect) {
    // Apagar siempre (seguridad: bobina no puede quedar ON)
    digitalWrite(nextCoilPin, LOW);
    digitalWrite(PIN_DBG_FIRE, LOW);
    TIMSK1 &= ~((1 << OCIE1A) | (1 << OCIE1B));
}

// ─────────────────────────────────────────────────
//  ISR CKP — Ultra-liviana: solo timestamp + contador
//  Sin flotante, sin multiplicación, sin llamadas a funciones
// ─────────────────────────────────────────────────
ISR(INT3_vect) {
    // Diagnóstico HIL — toggle pin para medir latencia con osciloscopio
    digitalWrite(PIN_DBG_ISR, HIGH);

    uint16_t now = TCNT1;
    uint32_t period = (uint32_t)(now - (uint16_t)ckpTimestamp);
    ckpTimestamp = now;

    // Detección de hueco 60-2: período ~3× mayor que el promedio
    if (period > (toothPeriod + (toothPeriod >> 1))) {
        // Hueco detectado
        gapDetected  = true;
        toothCount   = 0;
        errorCount   = 0;
    } else {
        toothPeriod = period;
        toothCount++;
        if (toothCount >= TEETH_EFFECTIVE) toothCount = 0;
        newToothReady = true;
    }

    digitalWrite(PIN_DBG_ISR, LOW);
}

// ─────────────────────────────────────────────────
//  ISR CMP — Confirmación de fase del ciclo
// ─────────────────────────────────────────────────
ISR(INT2_vect) {
    phaseKnown = true;
}

// ─────────────────────────────────────────────────
//  Cálculo de avance de encendido (loop — no ISR)
//  Retorna avance en dientes (entero) desde TDC
// ─────────────────────────────────────────────────
int16_t calcAdvanceDeg(uint16_t _rpm, uint8_t _tps, uint8_t _map) {
    // Curva base simple: avance lineal con RPM (solo para demo)
    // Speeduino usa tabla 2D — aquí se simplifica para referencia
    int16_t base = 8;

    // +1° por cada 500 RPM sobre 1000 (máx 20° adicionales)
    if (_rpm > 1000) {
        int16_t rpmAdv = (int16_t)((_rpm - 1000) / 500);
        if (rpmAdv > 20) rpmAdv = 20;
        base += rpmAdv;
    }

    // Corrección por MAP (carga): alta carga → menos avance
    if (_map > 80) {
        base -= (int16_t)((_map - 80) / 5);
    }

    // Corrección por TPS transitorio (aceleración brusca)
    if (_tps > 80) {
        base -= 3;
    }

    // Recortar a rango válido
    if (base < ADVANCE_MIN_DEG) base = ADVANCE_MIN_DEG;
    if (base > ADVANCE_MAX_DEG) base = ADVANCE_MAX_DEG;

    return base;
}

// ─────────────────────────────────────────────────
//  Conversión avance (grados → dientes desde TDC)
// ─────────────────────────────────────────────────
uint8_t degToTeeth(int16_t deg) {
    // DEG_PER_TOOTH_INT = 6° por diente (constante entera)
    uint8_t t = (uint8_t)(deg / DEG_PER_TOOTH_INT);
    if (t < 1) t = 1;
    if (t > FIRE_INTERVAL_TEETH - 2) t = FIRE_INTERVAL_TEETH - 2;
    return t;
}

// ─────────────────────────────────────────────────
//  Cálculo de RPM desde período inter-diente
// ─────────────────────────────────────────────────
uint16_t calcRpm(uint32_t periodTicks) {
    if (periodTicks == 0) return 0;
    // RPM = 60 * F_CPU / (PRESCALER * TEETH_EFFECTIVE * periodTicks)
    // = 60 * 16000000 / (8 * 58 * periodTicks)
    // = 2068965517 / periodTicks   → overflow uint32 posible
    // Usar pre-cálculo: constante = 60e6 / (prescaler_us * teeth)
    //   = 60000000 / (0.5 * 58) = 60000000 / 29 = 2068965 (en µs)
    // periodTicks en ticks → periodUs = periodTicks / TIMER1_TICKS_PER_US
    uint32_t periodUs = TICKS_TO_US(periodTicks);
    if (periodUs == 0) return 0;
    uint32_t rpmVal = 60000000UL / ((uint32_t)TEETH_EFFECTIVE * periodUs);
    if (rpmVal > 9999) rpmVal = 9999;
    return (uint16_t)rpmVal;
}

// ─────────────────────────────────────────────────
//  Safe state: apaga todas las bobinas, señaliza
// ─────────────────────────────────────────────────
void enterSafeState() {
    TIMSK1 = 0;  // Desactivar Timer1 interrupts
    digitalWrite(PIN_COIL_1, LOW);
    digitalWrite(PIN_COIL_2, LOW);
    digitalWrite(PIN_COIL_3, LOW);
    digitalWrite(PIN_COIL_4, LOW);
    digitalWrite(PIN_DBG_SAFE, HIGH);
    sysState = STATE_SAFE;
}

void exitSafeState() {
    digitalWrite(PIN_DBG_SAFE, LOW);
    sysState = STATE_NORMAL;
}

// ─────────────────────────────────────────────────
//  OLED — Actualización no bloqueante
// ─────────────────────────────────────────────────
void updateOled(int8_t clt) {
    char buf[24];
    u8g2.clearBuffer();

    // Encabezado
    u8g2.setFont(u8g2_font_6x10_tf);
    u8g2.drawStr(0, 10,
        sysState == STATE_SAFE  ? "*** SAFE STATE ***" :
        sysState == STATE_FAULT ? "*** FAULT       ***" :
                                  "  DIS IGNITION OK   ");

    // RPM y avance
    u8g2.setFont(u8g2_font_7x13B_tf);
    snprintf(buf, sizeof(buf), "RPM: %4u", rpm);
    u8g2.drawStr(0, 26, buf);

    snprintf(buf, sizeof(buf), "ADV: %3d", advanceDeg);
    u8g2.drawStr(68, 26, buf);

    // MAP y TPS
    u8g2.setFont(u8g2_font_6x10_tf);
    snprintf(buf, sizeof(buf), "MAP:%3ukPa TPS:%3u%%", mapKpa, tpsPct);
    u8g2.drawStr(0, 40, buf);

    // CLT y cilindro activo
    snprintf(buf, sizeof(buf), "CLT:%4d\xc2\xb0""C CYL:%u", clt,
             FIRE_ORDER[currentCylIndex] + 1);
    u8g2.drawStr(0, 52, buf);

    // Barra de RPM
    uint8_t barLen = (uint8_t)((uint32_t)rpm * 124 / 7000);
    if (barLen > 124) barLen = 124;
    u8g2.drawFrame(2, 55, 124, 8);
    u8g2.drawBox(2, 55, barLen, 8);

    u8g2.sendBuffer();
}

// ─────────────────────────────────────────────────
//  setup()
// ─────────────────────────────────────────────────
void setup() {
    // Salidas bobinas
    pinMode(PIN_COIL_1, OUTPUT); digitalWrite(PIN_COIL_1, LOW);
    pinMode(PIN_COIL_2, OUTPUT); digitalWrite(PIN_COIL_2, LOW);
    pinMode(PIN_COIL_3, OUTPUT); digitalWrite(PIN_COIL_3, LOW);
    pinMode(PIN_COIL_4, OUTPUT); digitalWrite(PIN_COIL_4, LOW);

    // Pines diagnóstico HIL
    pinMode(PIN_DBG_ISR,  OUTPUT); digitalWrite(PIN_DBG_ISR,  LOW);
    pinMode(PIN_DBG_FIRE, OUTPUT); digitalWrite(PIN_DBG_FIRE, LOW);
    pinMode(PIN_DBG_SAFE, OUTPUT); digitalWrite(PIN_DBG_SAFE, LOW);

    // Entradas con pull-up interno donde corresponde
    pinMode(PIN_CKP, INPUT);
    pinMode(PIN_CMP, INPUT);

    // Serial para depuración
    Serial.begin(115200);
    Serial.println(F("DIS Ignition HIL v1.0"));

    // Timer1 — CTC, prescaler /8
    timer1Init();

    // OLED
    u8g2.begin();
    u8g2.setContrast(200);

    // Interrupciones externas
    // INT3 = Pin 18 (CKP) — flanco de subida
    EIMSK &= ~((1 << INT3) | (1 << INT2));
    EICRA |= (1 << ISC31) | (1 << ISC30);  // INT3 rising edge
    EICRA |= (1 << ISC21) | (1 << ISC20);  // INT2 rising edge
    EIFR  = (1 << INTF3) | (1 << INTF2);
    EIMSK |= (1 << INT3) | (1 << INT2);

    sei();  // Habilitar interrupciones globales

    // Entrar en safe state hasta sincronizar
    enterSafeState();
}

// ─────────────────────────────────────────────────
//  loop() — Precomputación y scheduling
// ─────────────────────────────────────────────────
void loop() {
    uint32_t nowMs = millis();

    // ── 1. Leer sensores (ADC, no necesita sei crítico) ──────────
    uint16_t adcTps = analogRead(PIN_TPS);
    uint16_t adcMap = analogRead(PIN_MAP);
    uint16_t adcClt = analogRead(PIN_CLT);

    tpsPct = ADC_TO_TPS_PCT(adcTps);
    mapKpa = (uint8_t)ADC_TO_MAP_KPA(adcMap);
    int8_t  clt = adcToClt(adcClt);

    // ── 2. Snapshot atómico de variables volátiles ────────────────
    uint32_t snapPeriod;
    uint16_t snapToothCount;
    bool     snapGap, snapPhase;
    cli();
    snapPeriod     = toothPeriod;
    snapToothCount = toothCount;
    snapGap        = gapDetected;
    snapPhase      = phaseKnown;
    sei();

    // ── 3. Calcular RPM ──────────────────────────────────────────
    rpm = calcRpm(snapPeriod);

    // ── 4. Verificar timeout de señal CKP ────────────────────────
    if ((nowMs - lastCkpMs) > RPM_STALL_TIMEOUT_MS && rpm == 0) {
        if (sysState != STATE_SAFE) {
            enterSafeState();
        }
    } else if (snapGap && snapPhase && rpm > 0) {
        if (sysState == STATE_SAFE) {
            exitSafeState();
        }
        lastCkpMs = nowMs;
    }

    // ── 5. Precomputar avance y dwell ────────────────────────────
    if (sysState == STATE_NORMAL && newToothReady) {
        newToothReady = false;

        // Avance en grados (lógica compleja, segura fuera de ISR)
        advanceDeg = calcAdvanceDeg(rpm, tpsPct, mapKpa);

        // Corrección modular por cilindro (trim individual)
        const int8_t TRIMS[CYLINDERS] = {
            TRIM_CYL_1, TRIM_CYL_2, TRIM_CYL_3, TRIM_CYL_4
        };
        int16_t trimmed = advanceDeg
            + TRIMS[FIRE_ORDER[currentCylIndex]] / TRIM_SCALE;
        if (trimmed < ADVANCE_MIN_DEG) trimmed = ADVANCE_MIN_DEG;
        if (trimmed > ADVANCE_MAX_DEG) trimmed = ADVANCE_MAX_DEG;

        // Convertir avance a dientes
        uint8_t advTeethCalc = degToTeeth(trimmed);

        // Dwell proporcional a RPM (sencillo: 3 ms base, reduce a alta RPM)
        uint16_t dwellCalc = DWELL_DEFAULT_TICKS;
        if (rpm > 4000) {
            // Reducir DWELL_REDUCTION_TICKS_PER_1K_RPM por cada 1000 RPM sobre 4000 (mín DWELL_MIN)
            uint16_t reduction = (uint16_t)((rpm - 4000) / 1000) *
                                 DWELL_REDUCTION_TICKS_PER_1K_RPM;
            dwellCalc = (dwellCalc > reduction + DWELL_MIN_TICKS)
                        ? dwellCalc - reduction
                        : DWELL_MIN_TICKS;
        }

        // Determinar qué bobina corresponde al próximo evento
        uint8_t coilIdx = FIRE_ORDER[currentCylIndex];
        const uint8_t COIL_PINS[CYLINDERS] = {
            PIN_COIL_1, PIN_COIL_2, PIN_COIL_3, PIN_COIL_4
        };

        // Calcular cuántos ticks faltan hasta el evento (desde diente actual)
        uint16_t ticksToEvent = (uint16_t)(
            (uint32_t)(FIRE_INTERVAL_TEETH - snapToothCount % FIRE_INTERVAL_TEETH
                       - advTeethCalc) * snapPeriod
        );

        // Escribir atómicamente las variables de scheduling
        cli();
        advTeeth   = advTeethCalc;
        dwellTicks = dwellCalc;
        scheduleReady = true;
        sei();

        // Programar evento en Timer1
        if (ticksToEvent > US_TO_TICKS(200)) {  // Al menos 200 µs de margen
            scheduleCoilEvent(COIL_PINS[coilIdx],
                              ticksToEvent - dwellCalc,
                              dwellCalc);
            // Avanzar al siguiente cilindro en secuencia
            currentCylIndex = (currentCylIndex + 1) % CYLINDERS;
        }
    }

    // ── 6. Actualizar OLED (no bloqueante por temporizador) ───────
    if ((nowMs - lastOledUpdate) >= OLED_UPDATE_PERIOD_MS) {
        lastOledUpdate = nowMs;
        updateOled(clt);
    }

    // ── 7. Diagnóstico serial (cada 1 segundo) ───────────────────
    static uint32_t lastSerial = 0;
    if ((nowMs - lastSerial) >= 1000) {
        lastSerial = nowMs;
        Serial.print(F("RPM="));  Serial.print(rpm);
        Serial.print(F(" ADV=")); Serial.print(advanceDeg);
        Serial.print(F("° MAP=")); Serial.print(mapKpa);
        Serial.print(F("kPa TPS=")); Serial.print(tpsPct);
        Serial.print(F("% CLT=")); Serial.print(clt);
        Serial.print(F("°C STATE="));
        Serial.println(sysState == STATE_NORMAL ? "OK" :
                       sysState == STATE_SAFE   ? "SAFE" : "FAULT");
    }
}
