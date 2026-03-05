# Análisis y Mejoras del Sistema de Encendido DIS Optimizado

**Proyecto:** Sistema de encendido DIS basado en Speeduino
**Plataforma:** Arduino Mega 2560 (ATmega2560 @ 16 MHz)
**Motor objetivo:** 4 cilindros, 4 tiempos, chispa desperdiciada (wasted spark), rueda 36-1
**Fecha:** Marzo 2026
**Idioma:** Español

---

## 1. Descripción del Sistema

El sistema implementa un encendido DIS (*Distributorless Ignition System*) con chispa
desperdiciada para motor de 4 cilindros. Se basa en los principios de firmware de Speeduino
adaptados para pruebas en banco (HIL) y para instalación en vehículo.

### Componentes y arquitectura

| Elemento | Descripción |
|---|---|
| Rueda fónica | 36-1 (35 dientes + 1 faltante como referencia de sincronía) |
| Bobinas | 2 bobinas (Coil A: cil. 1 y 4, Coil B: cil. 2 y 3) |
| Sensor cigüeñal | VR o Hall; conectado a INT0 (pin 2) |
| Timer de scheduling | Timer1 de AVR, 16 bits, prescaler 8 (2 ticks/µs) |
| Watchdog | Hardware WDT, timeout 250 ms |
| Pines HIL | A0–A5 (PORTF 0–5), acceso directo a registro |

### Archivos del proyecto

```
engine_ignition_hil/
├── engine_ignition_hil.ino   Sketch principal: ISR, setup, loop, diagnóstico
├── config.h                  Configuración, macros compilados, límites de seguridad
├── ignition.h                Tipos de datos, prototipos, inline elapsed_exceeds
├── ignition.cpp              Decodificación de rueda, scheduling, estado seguro
└── ANALISIS_MEJORAS.md       Este documento
```

---

## 2. Riesgos Identificados y Mejoras Aplicadas

### 2.1 Overflow y Wrap-around de Temporizadores

**Riesgo original**
`micros()` devuelve `uint32_t` que hace wrap-around a los ~71 minutos. Comparar tiempos
con operadores `<` o `>` sin considerar el wrap puede producir resultados incorrectos
cuando el contador se reinicia, provocando fallos en la detección de parada, timeouts de
bobina y cálculo de retardos de scheduling.

**Mejoras aplicadas**

- Todas las diferencias de tiempo se calculan como `(uint32_t)(now - start)`, que es
  aritméticamente correcta ante el wrap por complemento a dos, siempre que el intervalo
  real sea menor que 2³² µs (~71 min), condición siempre satisfecha en este sistema.
- Se introdujo la función `elapsed_exceeds(now, start, threshold)` en `ignition.h`
  (inline) que encapsula explícitamente este patrón y documenta la justificación de
  seguridad.
- Ninguna comparación de tiempos emplea tipos con signo ni valores de punto flotante.

**Ejemplo de código resultante**
```c
// CORRECTO: sustracción uint32_t — segura ante wrap-around
if ((uint32_t)(now - start) >= threshold) { ... }

// INCORRECTO (no presente en el código): comparación directa
// if (now >= start + threshold) { ... }  // FALLA en wrap-around
```

---

### 2.2 Seguridad en ISR (Interrupt Service Routines)

**Riesgo original**
En AVR de 8 bits, la lectura o escritura de variables de 16 o 32 bits requiere múltiples
instrucciones de máquina. Sin protección, una interrupción entre instrucciones puede
corromper el valor leído o escrito (problema de "torn read/write").

**Mejoras aplicadas**

- Todas las variables compartidas entre ISRs y el `loop()` están declaradas `volatile`.
- Toda lectura de variables `volatile` de múltiples bytes desde el `loop()` se envuelve
  en `ATOMIC_BLOCK(ATOMIC_RESTORESTATE)` de `<util/atomic.h>`. Esto deshabilita
  interrupciones solo el tiempo mínimo necesario y restaura el estado previo (no asume
  que las interrupciones estaban habilitadas).
- Las ISRs son mínimas: capturan el timestamp con `micros()` y delegan inmediatamente
  a las funciones de módulo (`ignition_tooth_handler`, `ignition_timer_handler`).
- Los pines HIL de alta frecuencia (ISR timing, tooth pulse, sync, coils) usan acceso
  directo a registro (`PORTF |= bit`) en lugar de `digitalWrite()`, eliminando la
  latencia de unos ~5 µs por llamada que introduciría en la ISR.

**Fragmento de código**
```c
// En loop(): lectura segura de variables volatile de 32 bits
ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    rpm           = g_crankState.rpm;
    lastToothTime = g_crankState.lastToothTimeUs;
}
```

---

### 2.3 Filtros de Ruido en el Sensor de Cigüeñal

**Riesgo original**
Los sensores VR generan oscilaciones secundarias y los sensores de efecto Hall pueden
emitir pulsos espurios por ruido EMI. Sin filtrado, cada pulso espurio dispara un flanco
en INT0, corrompiendo el conteo de dientes y el cálculo de RPM.

**Mejoras aplicadas**

1. **Filtro de período mínimo:** Todo pulso cuyo período sea inferior a
   `MIN_TOOTH_PERIOD_US` (190 µs, equivalente a ≈8 800 rpm) se descarta. Los pulsos
   rechazados se cuentan en `g_diag.noisePulses`.

2. **Detección del diente faltante por razón de períodos:** Se usa comparación entera
   (sin punto flotante en ISR) para detectar el hueco:
   ```c
   if (period * 100UL > prevPeriod * MISSING_TOOTH_RATIO_X100) {
       // Es el diente faltante (gap de referencia)
   }
   ```
   El umbral `MISSING_TOOTH_RATIO_X100 = 150` (factor 1,5×) da margen ante variaciones
   de velocidad transitoria. Análisis de overflow: `5555 µs × 150 = 833 250`, que cabe
   en `uint32_t` con margen amplio.

3. **Mínimo de huecos para sincronía:** Se requiere detectar el hueco al menos
   `CRANK_SYNC_MIN_GAPS = 2` veces consecutivas antes de declarar sincronía adquirida,
   reduciendo falsas sincronías por pulsos de arranque o ruido transitorio.

4. **Filtro de RPM por promedio móvil:** Se promedian las últimas `RPM_FILTER_SAMPLES = 4`
   lecturas de RPM para suavizar variaciones instantáneas sin introducir latencia
   significativa.

---

### 2.4 Scheduling de Encendido y Desbordamiento del Timer

**Riesgo original**
Si el retardo calculado para el disparo de bobina excede el valor máximo del registro
`OCR1A` (16 bits = 65 535 ticks = 32 767 µs con prescaler 8), un truncamiento silencioso
a `uint16_t` provocaría un disparo prematuro incorrecto, potencialmente con el motor en
posición incorrecta. En el caso extremo, una bobina podría encenderse dos veces por vuelta
o dispararse fuera de tiempo, dañando el motor.

**Mejoras aplicadas**

- `ignition_schedule()` valida que `delayUs ≤ TIMER1_MAX_DELAY_US` (32 767 µs) antes de
  la conversión a ticks. Si excede, se registra en `g_diag.overflowGuards` y se cancela
  el scheduling.
- Se valida también que el retardo no supere `200 000 µs` (1 revolución a RPM mínimo),
  descartando schedulings obsoletos que pudieron quedar pendientes de un ciclo anterior.
- La función `timer1_program()` detiene el timer completamente (CS bits = 000) antes de
  reprogramar `TCNT1`, `OCR1A` y `OCR1B`, eliminando la condición de carrera en la que
  el contador podría haber superado el nuevo valor de OCR antes de habilitarlo.
- El flag de comparación pendiente se limpia explícitamente con `TIFR1 = (1<<OCF1A) | ...`
  antes de habilitar las interrupciones.

**Fragmento clave**
```c
// Detener timer — sin prescaler → sin conteo
TCCR1B &= ~((1<<CS12)|(1<<CS11)|(1<<CS10));
TCNT1 = 0;
OCR1A = ticksA;
OCR1B = ticksA + ticksB;  // con comprobación de overflow
TIFR1 = (1<<OCF1A)|(1<<OCF1B);   // limpiar flags pendientes
TIMSK1 |= (1<<OCIE1A)|(1<<OCIE1B);
TCCR1B |= (1<<CS11);   // reiniciar con prescaler 8
```

---

### 2.5 Control de Dwell — Protección de Bobinas

**Riesgo original**
Un fallo en el scheduling (desbordamiento del timer, pérdida de sincronía, cuelgue del
software) podría dejar una bobina energizada indefinidamente, causando sobrecalentamiento
del transistor de potencia (IGBT / Darlington) y posible destrucción de la bobina.

**Mejoras aplicadas**

1. **Clamp de dwell (`ignition_clamp_dwell`):** Restringe el dwell en el rango
   `[DWELL_MIN_US = 2 ms, DWELL_MAX_US = 5 ms]`. Cada vez que se activa el clamp se
   incrementa el contador de diagnóstico correspondiente (`dwellClampHigh` o
   `dwellClampLow`).

2. **Timeout en la ISR de Timer1:** Aunque el fin de dwell es manejado por `OCR1B`,
   si por algún motivo el evento B no dispara, el canal queda en estado `DWELL_CHARGING`
   indefinidamente. La capa de loop lo detecta (ver punto 3).

3. **Timeout redundante en el loop principal (`check_coil_timeout`):** Si una bobina
   sigue cargando más de `COIL_EMERGENCY_CUTOFF_US = DWELL_MAX_US + 1 ms` (6 ms),
   se la corta desde el `loop()` con acceso atómico, independientemente del timer.
   Esto actúa como segunda línea de defensa.

---

### 2.6 Detección de Parada y Estado Seguro Reforzado

**Riesgo original**
Al detener el motor, o si el sensor falla, el sistema podía quedar en estado indeterminado
con una bobina activa, el timer corriendo, y sin mecanismo de recuperación.

**Mejoras aplicadas**

1. **Detección de parada (`check_stall`):** Si no se recibe ningún diente en
   `STALL_TIMEOUT_US = 20 ms`, se invoca `ignition_safe_state()` y se resetea toda la
   información de sincronía. El umbral de 20 ms corresponde a ≈16 rpm, por debajo del
   cual se considera parada mecánica real.

2. **Estado seguro reforzado (`ignition_safe_state`):**
   - Deshabilita las interrupts de Timer1 (`TIMSK1 &= ~...`).
   - Detiene físicamente el timer (CS bits = 000).
   - Apaga todas las bobinas con `digitalWrite(pin, LOW)` y limpia los pines HIL.
   - Resetea los estados de la máquina de dwell.
   - Actualiza `g_sysState`, incrementa `safeStateEntries`, activa `HIL_STALL_FLAG`.
   - Es seguro de llamar tanto desde ISR como desde código normal (no usa malloc,
     no bloquea, completa en tiempo acotado).

3. **Watchdog Timer de hardware:** Habilitado con `wdt_enable(WDTO_250MS)` tras el setup.
   El `loop()` llama `wdt_reset()` al inicio. Si el software se bloquea por más de 250 ms,
   el hardware reinicia el MCU. En el próximo arranque, `setup()` lee `MCUSR` para detectar
   y registrar el reset en `g_diag.wdtResets`.

4. **Protección contra sobre-revoluciones (`check_overrev`):** Si `rpm > RPM_MAX = 7000`,
   se aplica el estado seguro (corte de encendido). Se restablece automáticamente cuando
   el RPM baja del umbral (lógica de histéresis implícita por el filtro de RPM).

---

### 2.7 Modularidad y Separación de Responsabilidades

**Situación identificada**
En implementaciones monolíticas de encendido sobre Arduino (incluyendo versiones tempranas
de Speeduino-para-Arduino), toda la lógica suele estar en el `.ino`, dificultando las
pruebas unitarias y la reutilización.

**Mejoras aplicadas**

- El código se divide en módulos con responsabilidades claras:
  - `config.h`: solo constantes y macros. Sin código ejecutable.
  - `ignition.h` + `ignition.cpp`: módulo autocontenido, potencialmente compilable en PC
    con mocks de `micros()` y `digitalWrite()` para test unitario.
  - `engine_ignition_hil.ino`: capa de integración y hardware: ISR vectors, setup/loop.
- Las funciones de alto impacto (`ignition_safe_state`, `ignition_clamp_dwell`) tienen
  responsabilidad única y son llamables de forma independiente.

---

## 3. Contadores de Diagnóstico

La estructura `DiagCounters_t` centraliza todos los eventos de interés. Se reportan por
puerto serie cada segundo y son accesibles en tiempo real vía monitor serie del IDE.

| Campo | Descripción | Umbral de alerta sugerido |
|---|---|---|
| `toothCount` | Dientes válidos contados (total) | — |
| `noisePulses` | Pulsos rechazados por ruido | > 5 % de `toothCount` |
| `syncEvents` | Adquisiciones de sincronía | > 1 por arranque es normal |
| `syncLossEvents` | Pérdidas de sincronía en marcha | > 0 en marcha estable |
| `gapDetections` | Huecos detectados (referencia) | ≈ rpm/60 eventos/s |
| `overflowGuards` | Schedulings rechazados por rango | > 0 indica bug o RPM extremo |
| `dwellClampHigh` | Dwell recortado al máximo | Consistente → ajustar dwell |
| `dwellClampLow` | Dwell elevado al mínimo | Consistente → ajustar dwell |
| `coilTimeoutEvents` | Corte de emergencia de bobina | > 0 es crítico |
| `safeStateEntries` | Entradas al estado seguro | > 0 en marcha estable |
| `wdtResets` | Resets por Watchdog | > 0 indica cuelgue de software |

---

## 4. Instrumentación HIL (Hardware-in-the-Loop)

### Pines de diagnóstico en el conector de pruebas (banco / osciloscopio)

| Pin Mega | Señal | Tipo | Descripción |
|---|---|---|---|
| A0 (PF0) | `TOOTH_PULSE` | Toggle por diente | Verificar patrón 35 dientes + hueco |
| A1 (PF1) | `COIL_A_DBG` | Espejo bobina A | Verificar dwell y borde de disparo |
| A2 (PF2) | `COIL_B_DBG` | Espejo bobina B | Ídem bobina B |
| A3 (PF3) | `SYNC_STATE` | Nivel ALTO/BAJO | ALTO = sincronía adquirida |
| A4 (PF4) | `STALL_FLAG` | Nivel ALTO/BAJO | ALTO = parada / estado seguro |
| A5 (PF5) | `ISR_TIMING` | Toggle ISR | Toggle en entrada y salida de ISR INT0 |

Todos los pines HIL usan acceso directo al registro PORTF para minimizar la perturbación
del sistema bajo prueba. En el osciloscopio, A5 permite medir con precisión la latencia
total de la ISR de cigüeñal (tiempo de alta del pin).

### Procedimiento de verificación en banco

1. **Señal de cigüeñal simulada:**
   Conectar generador de señal (o Arduino secundario) en pin 2.
   Frecuencia: `rpm × 36 / 60` Hz. Ejemplo: 1000 rpm → 600 Hz.
   Insertar un período de silencio de 2× el período normal para simular el hueco.

2. **Verificación de decodificación:**
   - A0 debe mostrar 35 pulsos regulares seguidos de una ausencia más larga.
   - A3 debe pasar a ALTO tras la segunda detección del hueco.
   - El monitor serie debe mostrar `Sincronía: SI` y un RPM correcto.

3. **Verificación de dwell y disparo:**
   - A1 (Coil A) debe mostrar pulsos positivos de ancho ≈ 3,5 ms (dwell por defecto).
   - El borde de bajada de A1 es el momento del disparo de chispa.
   - Verificar que el dwell no exceda 5 ms ni sea inferior a 2 ms.

4. **Medición de latencia ISR:**
   - Canal diferencial entre A5 y GND en el osciloscopio.
   - La latencia total (entrada de pulso → respuesta del sistema) debe ser < 15 µs.
   - El ancho de pulso de A5 indica el tiempo de ejecución de la ISR del cigüeñal.

5. **Prueba de filtrado de ruido:**
   - Inyectar pulsos cortos (< 190 µs) en el pin 2 entre dientes normales.
   - `noisePulses` debe incrementar. A0 NO debe mostrar toggles adicionales.
   - La sincronía (A3) debe permanecer estable.

6. **Prueba de detección de parada:**
   - Detener la señal de cigüeñal.
   - Tras ≈ 20 ms, A4 debe pasar a ALTO y A1/A2 deben estar en BAJO.
   - El monitor serie debe mostrar `PARADA detectada`.

7. **Prueba de Watchdog:**
   - Simulable conectando un pin de salida a RESET con un pulso controlado.
   - En el siguiente arranque, el monitor serie debe indicar
     `ADVERTENCIA: Reset por Watchdog detectado` y `Resets WDT: 1`.

---

## 5. Recomendaciones para Implementación en Producción / Vehículo

1. **Tabla de avance 2D (RPM × carga):**
   Reemplazar `ADVANCE_DEFAULT_DEG_X10` por una tabla bidimensional interpolada
   linealmente. La interpolación debe realizarse fuera de la ISR (en el `loop()`) y el
   resultado almacenarse en una variable `volatile` para uso en la ISR.

2. **Sensor de árbol de levas (INT1, pin 3):**
   Incorporar la ISR de INT1 para determinar la fase del motor (cilindro 1 en compresión
   vs. escape), permitiendo encendido secuencial y eliminando la necesidad de wasted spark.

3. **ADC para MAP/TPS sin bloqueo:**
   Usar el ADC en modo de interrupción (`ADCSRA |= (1<<ADIE)`) o DMA para no bloquear
   el `loop()` durante la conversión.

4. **Test unitario en host (PC):**
   Los módulos `ignition.cpp` y `config.h` pueden compilarse en x86 reemplazando
   `micros()`, `digitalWrite()` y los registros AVR con mocks de C. Esto permite
   validar la lógica de timing con Google Test o Unity antes de subir al hardware.

5. **CRC de parámetros en EEPROM:**
   Calcular CRC-16 de los parámetros configurables almacenados en EEPROM al arranque
   para detectar corrupción por resets inesperados durante escritura.

6. **Buffer circular de eventos para diagnóstico post-mortem:**
   Almacenar los últimos N eventos (tipo, timestamp, valor) en SRAM o EEPROM para
   análisis tras un fallo en carretera.

7. **Protección ESD y filtros hardware:**
   Para instalación en vehículo, agregar filtros RC (10 Ω + 100 nF) en las entradas
   de sensores y TVS en las líneas de bobina para proteger el ATmega de transitorios
   de la instalación eléctrica del vehículo.

---

*Documento generado como parte del análisis exhaustivo y fortalecimiento del sistema
de encendido DIS optimizado basado en Speeduino. Todos los hallazgos están implementados
en los archivos de código fuente del mismo directorio.*
