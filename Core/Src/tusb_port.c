/*
 * TinyUSB Poortlaag voor STM32H5
 *
 * Dit bestand is de "brug" tussen TinyUSB (de USB-softwarebibliotheek) en de STM32 HAL.
 * TinyUSB verwacht bepaalde functies die platform-specifiek zijn:
 *   - tusb_hal_init(): initialiseer de hardware (hier niet nodig, HAL doet het al)
 *   - tusb_time_millis_api(): geef de huidige tijd in milliseconden terug
 *
 * TinyUSB gebruikt de tijd voor timeouts en interne timers.
 */

#include <stdint.h>
#include "tusb.h"

// Verklaring dat HAL_GetTick() bestaat in de HAL-bibliotheek (extern = elders gedefinieerd)
extern uint32_t HAL_GetTick(void);

//--------------------------------------------------------------------+
// Board Porting API - Functies die TinyUSB verwacht van het platform
//--------------------------------------------------------------------+

// Initialiseer de boardhardware voor TinyUSB
// Bij STM32 doet de HAL (MX_USB_PCD_Init) dit al in main.c, dus hier hoeven we niets te doen
void tusb_hal_init(void)
{
  // Niets nodig: USB-hardware wordt al geïnitialiseerd door MX_USB_PCD_Init() in main.c
}

// Geef de huidige tijd terug in milliseconden - TinyUSB gebruikt dit voor interne timers
// HAL_GetTick() telt elke milliseconde omhoog via de SysTick-timer (ingesteld door HAL_Init)
uint32_t tusb_time_millis_api(void)
{
  return HAL_GetTick();  // Lever de HAL-timer af als tijdsreferentie voor TinyUSB
}
