# Resumen Comparativo Técnico: DIS Optimizado vs. Speeduino Oficial

> **Propósito:** Referencia rápida de equivalencia funcional y áreas de mejora entre el código de
> encendido DIS optimizado desarrollado (`engine_ignition_hil`) y el firmware oficial de
> [Speeduino](https://github.com/noisymime/speeduino).
>
> **Idioma:** Español  
> **Plataforma:** Arduino Mega 2560 (ATmega2560, 16 MHz)  
> **Aplicación:** Motor 4 cilindros — Sistema de encendido directo (DIS)

---

## Tabla de Equivalencia Funcional Rápida

| Área                    | DIS Optimizado (`engine_ignition_hil`) | Speeduino Oficial          |
|-------------------------|----------------------------------------|----------------------------|
| Timer de ignición       | Timer1 CTC, prescaler /8 (0.5 µs)     | Timer1 + Timer3, 0.5 µs    |
| Scheduling spark        | OCR1A (dwell ON) + OCR1B (spark)       | Cola de 4 eventos (structs) |
| ISR CKP                 | Ultraliviana — solo timestamp + conteo | Proceso completo de diente  |
| Precomputación          | En `loop()` antes de cada evento       | En `loop()` + `scheduler`  |
| Corrección por cilindro | Trim individual (décimas de grado)     | Trim por cilindro en tabla  |
| Detección de hueco      | Relación de período ×1.5               | Filtro de período + conteo  |
| Safe state              | Apagado inmediato + pin diagnóstico    | `ALL_TABLES_SAFE` + BIT     |
| Diagnóstico             | OLED 128×64 + Serial + pines HIL       | TunerStudio (CAN/Serial)    |
| Interfaz de usuario     | Display OLED local en tiempo real      | PC vía cable USB/Bluetooth  |

---

## 1. Timer y Scheduling

### DIS Optimizado

```
Timer1 (CTC, prescaler /8):
  ┌─────────────────────────────────────────────────────────────┐
  │ loop() calcula:                                             │
  │   OCR1A = ahora + ticksToEvent - dwellTicks  ← dwell ON    │
  │   OCR1B = ahora + ticksToEvent               ← spark (OFF) │
  └────────────────────┬────────────────────────────────────────┘
                       │
           ISR COMPA ──┤→ digitalWrite(coilPin, HIGH)
           ISR COMPB ──┘→ digitalWrite(coilPin, LOW)  ← disparo
```

- **Resolución:** 0.5 µs (prescaler /8 @ 16 MHz)
- **Latencia máxima de scheduling:** determinada por loop(), no por ISR
- **Desbordamiento gestionado:** comparadores independientes de TCNT1

### Speeduino Oficial

```
Timer1 + Timer3:
  ┌─────────────────────────────────────────────────────────────┐
  │ ignitionSchedule[n]:                                        │
  │   { startTime, endTime, Status, startCallback, endCallback }│
  │   setIgnitionSchedule(n, start_us, duration_us, cb_on, cb_off) │
  └────────────────────┬────────────────────────────────────────┘
                       │
           COMPA/COMPB ─┤→ cola de 4 canales independientes
```

- **Cola de 4 eventos** simultáneos (`ignitionSchedule[0..3]`)
- **Callback genérico** permite reutilizar el mismo scheduler para fuel e ignición
- **Mayor complejidad:** lógica de "next window" para evitar colisiones

> **Equivalencia:** Ambos usan OCR de Timer1. Speeduino generaliza con cola; el DIS optimizado
> simplifica a 2 comparadores dedicados al canal activo — apropiado para DIS de 4 cilindros.

---

## 2. ISR (Interrupt Service Routine)

### DIS Optimizado — ISR ultraliviana

```cpp
ISR(INT3_vect) {                          // CKP — flanco de subida
    digitalWrite(PIN_DBG_ISR, HIGH);      // ← toggle HIL para osciloscopio
    uint16_t now    = TCNT1;
    uint32_t period = (uint32_t)(now - (uint16_t)ckpTimestamp);
    ckpTimestamp    = now;

    if (period > toothPeriod + (toothPeriod >> 1)) {
        gapDetected = true;   // Hueco 60-2
        toothCount  = 0;
    } else {
        toothPeriod   = period;
        toothCount++;
        newToothReady = true;
    }
    digitalWrite(PIN_DBG_ISR, LOW);
}
```

**Latencia ISR CKP:** ~10–15 ciclos de reloj (~0.6–1 µs)  
**Sin:** flotante, multiplicación, llamadas externas, `millis()`, `Serial`

### Speeduino Oficial

```cpp
// triggerHandler() en trigger_*.ino — ejemplo 60-2:
void triggerPrimary() {
    toothCurrentCount++;
    if (toothCurrentCount == 1 || toothCurrentCount > configPage4.triggerTeeth) {
        toothCurrentCount = 1;
    }
    // Calcula toothLastToothTime, validToothTime
    // Llama a checkMissed(), runScheduler() si aplica
    // Actualiza decoderState
}
```

**Latencia ISR CKP:** mayor (lógica de decoder embebida)  
**Incluye:** gestión de decoders múltiples (>30 tipos de ruedas), validación

> **Equivalencia funcional:** Ambos capturan período entre dientes y detectan el hueco.
> Speeduino soporta muchos más tipos de ruedas fónicas; el DIS optimizado tiene latencia menor
> al delegar toda la lógica al `loop()`.

---

## 3. Precomputación en `loop()`

### DIS Optimizado

Toda la aritmética costosa se ejecuta en `loop()`, **no en la ISR**:

```
loop()
  ├── analogRead(TPS, MAP, CLT)          ← sensores ADC
  ├── calcRpm(toothPeriod)               ← sin flotante: 60e6 / (58 * period_us)
  ├── calcAdvanceDeg(rpm, tps, map)      ← curva avance entera
  ├── degToTeeth(advanceDeg)             ← avance grados → dientes (÷6)
  ├── trim por cilindro (TRIM_CYL_x)     ← corrección modular
  ├── calcDwell(rpm)                     ← dwell adaptativo
  └── scheduleCoilEvent(pin, start, dwell) ← escribe OCR1A/OCR1B
```

**config.h** define todas las constantes precomputadas en tiempo de compilación:

```c
#define US_TO_TICKS(us)     ((us) * TIMER1_TICKS_PER_US)   // compilador evalúa
#define DWELL_DEFAULT_TICKS  US_TO_TICKS(DWELL_DEFAULT_US) // = 6000 ticks
#define FIRE_INTERVAL_TEETH (FIRE_INTERVAL_DEG / DEG_PER_TOOTH_INT)  // = 30
```

### Speeduino Oficial

```
loop() → mainLoop() → scheduler.cpp:
  ├── readMAP(), readTPS(), readCLT()    ← mismos sensores
  ├── calculateRPM()                     ← RPM con filtro de ruido
  ├── correctionsFuel(), correctionsIgn()← tabla 2D interpolada (flotante evitado con ×10)
  ├── setIgnitionSchedule(n, ...)        ← programa evento
  └── commsUpdate()                      ← TunerStudio
```

> **Equivalencia:** Ambos leen sensores y calculan timing en el loop.
> Speeduino usa tablas 2D completas (16×16 puntos). El DIS optimizado usa una curva lineal
> simplificada (adecuada para prototipo; reemplazable por tabla).

---

## 4. Corrección Modular (Trim por Cilindro)

### DIS Optimizado

```c
// config.h — definición
#define TRIM_CYL_1    0    // décimas de grado, p.ej. +5 = +0.5°
#define TRIM_CYL_2    0
#define TRIM_CYL_3    0
#define TRIM_CYL_4    0

// engine_ignition_hil.ino — aplicación en loop()
const int8_t TRIMS[4] = { TRIM_CYL_1, TRIM_CYL_2, TRIM_CYL_3, TRIM_CYL_4 };
int16_t trimmed = advanceDeg + TRIMS[FIRE_ORDER[currentCylIndex]] / TRIM_SCALE;
```

- **Configuración estática** en `config.h` (sin EEPROM en esta versión)
- **Escala 1/10 de grado** para correcciones finas sin flotante

### Speeduino Oficial

```cpp
// Speeduino: trims por cilindro en tabla en EEPROM (configPage4.ignTrimXX)
int16_t getTrimForCylinder(uint8_t cylinder) {
    return (int16_t)ignTrim[cylinder];  // almacenado en EEPROM, editable via TS
}
```

- **EEPROM + TunerStudio:** los trims se ajustan en tiempo real sin recompilar
- **Resolución:** 1° por unidad (entero)

> **Área de mejora DIS:** Agregar almacenamiento en EEPROM y lectura dinámica de trims
> para igualar la funcionalidad de Speeduino sin recompilar el firmware.

---

## 5. Robustez

### DIS Optimizado

| Mecanismo                  | Implementación                                              |
|----------------------------|-------------------------------------------------------------|
| Timeout CKP                | `RPM_STALL_TIMEOUT_MS = 500 ms` → entra safe state         |
| Errores consecutivos       | `MAX_CONSECUTIVE_ERRORS = 5` → safe state                  |
| Límite dwell máximo        | `COIL_MAX_ON_TICKS` impide bobina energizada indefinidamente|
| Rango avance               | `ADVANCE_MIN_DEG..ADVANCE_MAX_DEG` recortado en loop()     |
| Rango dwell                | `DWELL_MIN_TICKS..DWELL_MAX_TICKS` recortado               |
| Desactivar Timer al fallar | `TIMSK1 = 0` antes de apagar bobinas                       |

### Speeduino Oficial

| Mecanismo                  | Implementación                                              |
|----------------------------|-------------------------------------------------------------|
| Timeout RPM                | `currentStatus.RPM == 0` por N ciclos → corte              |
| Watchdog de bobina         | Timer4 independiente como guardián de dwell máximo         |
| Validación de decoders     | `BIT_DECODER_VALID_TRIGGER` en decoderState                |
| Límites en tabla           | Clamp en `correctionCrankingFixedTiming()` y similares     |

> **Equivalencia:** Ambos implementan protección contra bobina energizada y timeout de señal.
> Speeduino usa un Timer dedicado como watchdog de hardware; el DIS optimizado usa comparador
> del mismo Timer1 (OCR1B como "apagado de emergencia").

---

## 6. Safe State

### DIS Optimizado

```
Condiciones de entrada:
  • Timeout CKP > 500 ms            → enterSafeState()
  • errorCount > MAX_CONSECUTIVE_ERRORS
  • Al inicio (hasta sincronización)

enterSafeState():
  TIMSK1 = 0                ← desactiva scheduling inmediatamente
  coil1..4 = LOW            ← apaga todas las bobinas
  PIN_DBG_SAFE = HIGH        ← señal visual/HIL para diagnóstico
  sysState = STATE_SAFE

Salida de safe state:
  gapDetected AND phaseKnown AND rpm > 0 → exitSafeState()
```

### Speeduino Oficial

```
Condiciones de entrada:
  • launchHard() o cutFuel/cutIgnition flags
  • configPage4.ignCutRPM excedido
  • Sensor CLT fuera de rango (configPage6.fanWhenCranking, etc.)

Safe action:
  BIT_SET(currentStatus.status3, BIT_STATUS3_HARD_LAUNCH)
  → corte individual por cilindro (vs. corte total)
```

> **Diferencia clave:** Speeduino permite corte *selectivo* por cilindro y estrategias
> de launch control. El DIS optimizado aplica corte *total* — más seguro para prototipo,
> menos flexible para control avanzado.

---

## 7. Diagnóstico

### DIS Optimizado — Triple canal diagnóstico

```
┌─────────────────────────────────────────────────────────────────────┐
│  Canal 1: Pines HIL (osciloscopio / analizador lógico)              │
│    PIN_DBG_ISR  (22) — toggle en cada ISR CKP → mide latencia       │
│    PIN_DBG_FIRE (23) — toggle en dwell ON/OFF → valida timing       │
│    PIN_DBG_SAFE (24) — HIGH = safe state activo                      │
├─────────────────────────────────────────────────────────────────────┤
│  Canal 2: Serial (115200 baud) — volcado cada 1 segundo             │
│    RPM=1500 ADV=18° MAP=65kPa TPS=23% CLT=82°C STATE=OK            │
├─────────────────────────────────────────────────────────────────────┤
│  Canal 3: OLED 128×64 — refresco cada 150 ms (no bloqueante)        │
│    [ver sección 8]                                                   │
└─────────────────────────────────────────────────────────────────────┘
```

### Speeduino Oficial

```
┌─────────────────────────────────────────────────────────────────────┐
│  TunerStudio (PC):                                                   │
│    • Dashboard en tiempo real (RPM, MAP, TPS, advance, dwell, …)    │
│    • Datalog a archivo CSV                                           │
│    • Configuración de tablas 2D/3D                                  │
├─────────────────────────────────────────────────────────────────────┤
│  CAN bus (opcional):                                                 │
│    • Transmisión de parámetros a otros módulos                       │
├─────────────────────────────────────────────────────────────────────┤
│  Error codes:                                                        │
│    • currentStatus.engineProtectStatus, decoderState                │
└─────────────────────────────────────────────────────────────────────┘
```

> **Diferencia clave:** Speeduino requiere PC con TunerStudio para diagnóstico completo.
> El DIS optimizado tiene diagnóstico **autónomo y embebido** (OLED + Serial + pines HIL),
> ideal para banco de pruebas sin PC.

---

## 8. Interfaz OLED

### DIS Optimizado — Display SSD1306 128×64 I2C

```
┌────────────────────────────────────┐
│   DIS IGNITION OK                  │  ← Estado del sistema
├────────────────────────────────────┤
│ RPM: 1500   ADV:  18               │  ← RPM y avance actual
├────────────────────────────────────┤
│ MAP: 65kPa  TPS:  23%              │  ← Carga y acelerador
├────────────────────────────────────┤
│ CLT:  82C   CYL: 1                 │  ← Temperatura y cilindro activo
├────────────────────────────────────┤
│ [████████████░░░░░░░░░░░░░░░░░░░]  │  ← Barra RPM visual (0–7000)
└────────────────────────────────────┘

  Si safe state activo:
┌────────────────────────────────────┐
│ *** SAFE STATE ***                 │
│ RPM:    0   ADV:  10               │
│ ...                                │
└────────────────────────────────────┘
```

**Implementación no bloqueante:**
```cpp
if ((millis() - lastOledUpdate) >= OLED_UPDATE_PERIOD_MS) {  // 150 ms
    lastOledUpdate = millis();
    updateOled(clt);   // No usa delay() — no interfiere con scheduling
}
```

### Speeduino Oficial

Speeduino **no incluye** una interfaz OLED por defecto. Existen proyectos externos
(p. ej. *SpeedyLoader*) que conectan displays separados, pero no son parte del firmware oficial.

> **Ventaja DIS:** La interfaz OLED local permite operar y diagnosticar el sistema sin
> ningún periférico externo — clave para instalación en vehículo o banco portátil.

---

## 9. Resumen de Áreas de Mejora

| Área                       | Estado DIS Optimizado | Mejora sugerida para igualar Speeduino           |
|----------------------------|-----------------------|---------------------------------------------------|
| Tabla de avance            | Curva lineal simple   | Implementar tabla 2D de 16×16 en PROGMEM          |
| Trims por cilindro         | Constante compilación | Leer de EEPROM, editable sin recompilar           |
| Soporte de ruedas fónicas  | Solo 60-2             | Agregar decoders adicionales (36-1, 4-1, etc.)    |
| Safe state selectivo       | Corte total           | Corte por cilindro + launch control               |
| Comunicación con PC        | Solo Serial texto     | Protocolo TunerStudio (TS Serial) para logs/tablas|
| Enriquecimiento en frío    | No implementado       | Corrección por CLT (misma lógica que Speeduino)   |
| Watchdog de hardware       | Usando OCR1B          | Timer independiente (Timer4) para máxima seguridad|
| Modo cranking              | No implementado       | Avance fijo de arranque (`ignCrankingFixedAngle`)  |

---

## 10. Diagrama de Flujo Comparado

```
    DIS Optimizado                     Speeduino Oficial
    ══════════════                     ═════════════════

    setup()                            setup()
      │                                  │
      ├── Timer1 CTC /8                  ├── Timer1 + Timer3
      ├── INT3/INT2                      ├── Decoder interrupts
      ├── safe state inicial             ├── Status flags init
      └── OLED init                      └── EEPROM load

    loop()                             mainLoop()
      │                                  │
      ├── read ADC                       ├── readMAP/TPS/CLT
      ├── snapshot atómico ISR vars      ├── calculateRPM
      ├── calcRpm()                      ├── correctionsFuel/Ign
      ├── check timeout CKP             ├── setIgnitionSchedule(n,…)
      ├── calcAdvanceDeg()              ├── commsUpdate (TunerStudio)
      ├── corrección trim/cilindro      └── otros estrategias
      ├── scheduleCoilEvent()
      ├── updateOled()  [150 ms]
      └── Serial print [1 s]

    ISR(INT3) — CKP                    triggerPrimary()
      └── timestamp + contador         └── decoder completo

    ISR(TIMER1_COMPA)                  ignitionSchedule COMPA
      └── coil HIGH                    └── startCallback()

    ISR(TIMER1_COMPB)                  ignitionSchedule COMPB
      └── coil LOW (spark)             └── endCallback()
```

---

*Generado como referencia técnica del proyecto `engine_ignition_hil`.*  
*Última actualización: 2026-03 — Santiago Vivas*
