# Santiago — Sistema de Encendido DIS Optimizado

Proyecto de encendido directo (DIS) optimizado para motor 4 cilindros sobre Arduino Mega 2560,
con diagnóstico embebido en OLED y comparativa técnica con el firmware oficial Speeduino.

## Contenido

| Archivo / Carpeta | Descripción |
|---|---|
| [`engine_ignition_hil/`](engine_ignition_hil/) | Código Arduino DIS optimizado (sketch + config) |
| [`COMPARATIVO_DIS_SPEEDUINO.md`](COMPARATIVO_DIS_SPEEDUINO.md) | Resumen comparativo técnico en español |

## Comparativa rápida

Consulta [`COMPARATIVO_DIS_SPEEDUINO.md`](COMPARATIVO_DIS_SPEEDUINO.md) para ver la equivalencia
funcional y las áreas de mejora entre este código y Speeduino oficial, cubriendo:
timer/scheduling, ISR, precomputación en loop, corrección modular, robustez, safe state,
diagnóstico y la interfaz OLED.

## Hardware requerido

- Arduino Mega 2560
- Sensor CKP conectado a pin 18 (INT3)
- Sensor CMP conectado a pin 19 (INT2)
- 4 bobinas DIS en pines 2–5
- Display OLED SSD1306 128×64 I2C (dirección 0x3C)
- Sensores TPS (A0), MAP (A1), CLT (A2)
