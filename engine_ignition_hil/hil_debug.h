/**
 * @file hil_debug.h
 * @brief HIL (Hardware-in-the-Loop) debug helper macros and utilities.
 *
 * These macros wrap all debug instrumentation so that it can be compiled out
 * for production builds simply by defining HIL_DISABLE before including this
 * header (or via a build flag: -DHIL_DISABLE).
 *
 * Typical oscilloscope wiring:
 *   CH-A → PIN_DEBUG_ISR   : measures ISR entry/exit → WCET and jitter
 *   CH-B → PIN_DEBUG_COIL  : mirrors coil command    → dwell/fire timing
 *   CH-C → PIN_DEBUG_SYNC  : HIGH while synchronised  → sync loss events
 *
 * Usage inside an ISR:
 *   HIL_ISR_ENTER();
 *   // ... ISR body ...
 *   HIL_ISR_EXIT();
 *
 * The resulting square wave on CH-A has:
 *   pulse width  = ISR execution time  (WCET is the widest pulse)
 *   period jitter = variation in tooth arrival intervals
 */

#ifndef HIL_DEBUG_H
#define HIL_DEBUG_H

#include <Arduino.h>
#include "config.h"

/* =========================================================================
 * Compile-time enable/disable
 * ========================================================================= */
#ifndef HIL_DISABLE

/* --- Low-level direct port manipulation for minimal overhead in ISR ------- */
/** Set a digital pin HIGH using direct port write (no overhead).             */
#define HIL_PIN_HIGH(pin)   digitalWrite((pin), HIGH)
/** Set a digital pin LOW using direct port write (no overhead).              */
#define HIL_PIN_LOW(pin)    digitalWrite((pin), LOW)

/* --- ISR timing markers --------------------------------------------------- */
/** Call at the very first line of the crank/capture ISR.                     */
#define HIL_ISR_ENTER()     HIL_PIN_HIGH(PIN_DEBUG_ISR)
/** Call at the very last line of the crank/capture ISR.                      */
#define HIL_ISR_EXIT()      HIL_PIN_LOW(PIN_DEBUG_ISR)

/* --- Coil command markers ------------------------------------------------- */
/** Mirror the coil-dwell start on the debug pin.                             */
#define HIL_COIL_DWELL()    HIL_PIN_HIGH(PIN_DEBUG_COIL)
/** Mirror the coil-fire (spark) event on the debug pin.                      */
#define HIL_COIL_FIRE()     HIL_PIN_LOW(PIN_DEBUG_COIL)

/* --- Synchronisation status ---------------------------------------------- */
/** Assert sync-valid debug pin.                                              */
#define HIL_SYNC_VALID()    HIL_PIN_HIGH(PIN_DEBUG_SYNC)
/** Deassert sync-valid debug pin (lost sync).                                */
#define HIL_SYNC_LOST()     HIL_PIN_LOW(PIN_DEBUG_SYNC)

/* --- Initialise all debug pins as outputs --------------------------------- */
static inline void hil_debug_init(void)
{
    pinMode(PIN_DEBUG_ISR,   OUTPUT);
    pinMode(PIN_DEBUG_COIL,  OUTPUT);
    pinMode(PIN_DEBUG_SYNC,  OUTPUT);
    HIL_PIN_LOW(PIN_DEBUG_ISR);
    HIL_PIN_LOW(PIN_DEBUG_COIL);
    HIL_PIN_LOW(PIN_DEBUG_SYNC);
}

/* --- Serial helpers ------------------------------------------------------- */
/**
 * @brief Print a formatted HIL status line over Serial.
 *
 * Example output:
 *   [HIL] RPM=1500 TOOTH=12 ADV=18deg SYNC=OK REVLIM=NO
 */
static inline void hil_serial_status(uint16_t rpm,
                                     uint8_t  tooth,
                                     uint8_t  advance_deg,
                                     bool     synced,
                                     bool     rev_limit)
{
    Serial.print(F("[HIL] RPM="));
    Serial.print(rpm);
    Serial.print(F(" TOOTH="));
    Serial.print(tooth);
    Serial.print(F(" ADV="));
    Serial.print(advance_deg);
    Serial.print(F("deg SYNC="));
    Serial.print(synced   ? F("OK  ") : F("LOST"));
    Serial.print(F(" REVLIM="));
    Serial.println(rev_limit ? F("YES") : F("NO "));
}

/* =========================================================================
 * Disabled stubs (HIL_DISABLE defined)
 * ========================================================================= */
#else /* HIL_DISABLE */

#define HIL_ISR_ENTER()         do {} while (0)
#define HIL_ISR_EXIT()          do {} while (0)
#define HIL_COIL_DWELL()        do {} while (0)
#define HIL_COIL_FIRE()         do {} while (0)
#define HIL_SYNC_VALID()        do {} while (0)
#define HIL_SYNC_LOST()         do {} while (0)

static inline void hil_debug_init(void)                                  {}
static inline void hil_serial_status(uint16_t, uint8_t, uint8_t,
                                     bool, bool)                         {}

#endif /* HIL_DISABLE */

#endif /* HIL_DEBUG_H */
