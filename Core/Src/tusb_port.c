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
#include "main.h"
#include "tusb.h"

// Verklaring dat HAL_GetTick() bestaat in de HAL-bibliotheek (extern = elders gedefinieerd)
extern uint32_t HAL_GetTick(void);

//--------------------------------------------------------------------+
// Board Porting API - Functies die TinyUSB verwacht van het platform
//--------------------------------------------------------------------+

// Initialiseer de boardhardware voor TinyUSB
// Hier zetten we alleen de USB clock, VDDUSB en interrupt-priority op;
// de USB device stack zelf wordt door TinyUSB beheerd.
void tusb_hal_init(void)
{
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_USB;
  PeriphClkInitStruct.UsbClockSelection = RCC_USBCLKSOURCE_HSI48;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  HAL_PWREx_EnableVddUSB();
  __HAL_RCC_USB_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  // USB FS data pins must be in alternate-function mode for enumeration to work.
  GPIO_InitStruct.Pin = GPIO_PIN_11 | GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF10_USB;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  HAL_NVIC_SetPriority(USB_DRD_FS_IRQn, 0, 0);
}

// Geef de huidige tijd terug in milliseconden - TinyUSB gebruikt dit voor interne timers
// HAL_GetTick() telt elke milliseconde omhoog via de SysTick-timer (ingesteld door HAL_Init)
uint32_t tusb_time_millis_api(void)
{
  return HAL_GetTick();  // Lever de HAL-timer af als tijdsreferentie voor TinyUSB
}
