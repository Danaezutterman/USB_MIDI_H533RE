/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "tusb.h"
#include <stdbool.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

// ================================================================
// MCP23S17 - SPI GPIO uitbreidingschip (16 extra GPIO pinnen via SPI)
// ================================================================

// Registeradressen van de MCP23S17
// Het IODIR-register bepaalt of een pin ingang (1) of uitgang (0) is
#define MCP23S17_IODIRA   0x00  // Richting register poort A (0=uitgang, 1=ingang)
#define MCP23S17_IODIRB   0x01  // Richting register poort B (0=uitgang, 1=ingang)
#define MCP23S17_GPPUA    0x0C  // Pull-up weerstand register poort A (1=aan, 0=uit)
#define MCP23S17_GPPUB    0x0D  // Pull-up weerstand register poort B (1=aan, 0=uit)
#define MCP23S17_GPIOA    0x12  // Data register poort A - lezen/schrijven van pin-waarden
#define MCP23S17_GPIOB    0x13  // Data register poort B - lezen/schrijven van pin-waarden

// SPI Control Byte: dit is het eerste byte dat je stuurt over SPI
// Opgebouwd als: 0100 A2 A1 A0 R/W  (A2-A0 = hardware adres, R/W = lezen of schrijven)
#define MCP23S17_ADDR     0x40  // Hardware adres: A0=A1=A2=GND, dus adres = 000 -> 0x40
#define MCP23S17_WRITE    0x00  // Bit 0 = 0 => schrijven naar de chip
#define MCP23S17_READ     0x01  // Bit 0 = 1 => lezen van de chip

// Chip Select (CS) pin - PC9
// CS LOW = chip geselecteerd (communicatie actief)
// CS HIGH = chip vrijgegeven (communicatie gestopt)
#define CS_PIN            GPIO_PIN_9
#define CS_PORT           GPIOC
#define CS_LOW()          HAL_GPIO_WritePin(CS_PORT, CS_PIN, GPIO_PIN_RESET)  // Start SPI-communicatie
#define CS_HIGH()         HAL_GPIO_WritePin(CS_PORT, CS_PIN, GPIO_PIN_SET)    // Stop SPI-communicatie

// ================================================================
// Knoppenmatrix configuratie (4x4 = 16 knoppen)
// ================================================================
// De matrix werkt met rijen en kolommen: kolom laag zetten, dan rijen lezen
// Als een knop ingedrukt is, verbindt hij de kolom met de rij -> rij gaat laag
#define MATRIX_ROWS       4          // 4 rijen (rij 0-3 via poort B bits 0-3)
#define MATRIX_COLS       4          // 4 kolommen (kolom 0-3 via poort A bits 0-3)
#define BASE_NOTE         60         // Startnoot = Middle C (MIDI nootnummer 60)

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

COM_InitTypeDef BspCOMInit;
ADC_HandleTypeDef hadc1;
DMA_NodeTypeDef Node_GPDMA1_Channel0;
DMA_QListTypeDef List_GPDMA1_Channel0;
DMA_HandleTypeDef handle_GPDMA1_Channel0;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim6;

PCD_HandleTypeDef hpcd_USB_DRD_FS;

/* USER CODE BEGIN PV */

// Vorige toestand van elke knop bijhouden (16 knoppen in 4x4 matrix)
// true = knop was ingedrukt, false = knop was losgelaten
// Hiermee detecteren we het MOMENT van indrukken/loslaten (flank-detectie)
static bool button_prev_state[16] = {false};

// ADC-buffers voor potentiometers (2 kanalen)
// PAO (ADC_CHANNEL_0) -> potentiometer 1 -> MIDI CC #10
// PA1 (ADC_CHANNEL_1) -> potentiometer 2 -> MIDI CC #11
static uint8_t adc_buffer[2] = {0, 0};
static uint8_t adc_prev[2] = {255, 255};

// Hysterese for potentiometers: waarde moet minstens 5 veranderen voor update
// Dit filtert kleine ruis uit
const uint8_t ADC_HYSTERESIS = 5;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_GPDMA1_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM6_Init(void);
static void MX_USB_PCD_Init(void);
static void MX_SPI1_Init(void);
/* USER CODE BEGIN PFP */
extern void tusb_hal_init(void);
void midi_task(void);
void mcp23s17_write_reg(uint8_t reg, uint8_t value);
uint8_t mcp23s17_read_reg(uint8_t reg);
void mcp23s17_init(void);
void scan_matrix(void);
void send_midi_note(uint8_t note, bool on);
void handle_potentiometers(void);
void send_midi_cc(uint8_t controller, uint8_t value);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_GPDMA1_Init();
  MX_ADC1_Init();
  MX_TIM6_Init();
  MX_SPI1_Init();
  /* USER CODE BEGIN 2 */

  // Voorkom USB-interrupts voordat TinyUSB volledig geïnitialiseerd is.
  HAL_NVIC_DisableIRQ(USB_DRD_FS_IRQn);

  // Initialiseer de TinyUSB-stack (softwarelaag die USB-MIDI regelt)
  tusb_init();
  tusb_hal_init();

  // TinyUSB is klaar: wis eventuele pending USB interrupt en zet IRQ terug aan.
  HAL_NVIC_ClearPendingIRQ(USB_DRD_FS_IRQn);
  HAL_NVIC_EnableIRQ(USB_DRD_FS_IRQn);

  // Wacht 1 seconde zodat de computer de USB-verbinding kan herkennen en configureren
  HAL_Delay(1000);
  
  // Initialiseer de MCP23S17: stel richtingen, pull-ups en beginwaarden in
  mcp23s17_init();

  // Start de ADC DMA conversie (voortdurend lezen van kanalen PA0 en PA1)
  HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc_buffer, 2);
  
  // Start Timer6: trigger voor ADC met vast interval (~1kHz stemupling)
  HAL_TIM_Base_Start(&htim6);

  /* USER CODE END 2 */

  /* Initialize USER push-button, will be used to trigger an interrupt each time it's pressed.*/
  BSP_PB_Init(BUTTON_USER, BUTTON_MODE_EXTI);

  /* Initialize COM1 port (115200, 8 bits (7-bit data + 1 stop bit), no parity */
  BspCOMInit.BaudRate   = 115200;
  BspCOMInit.WordLength = COM_WORDLENGTH_8B;
  BspCOMInit.StopBits   = COM_STOPBITS_1;
  BspCOMInit.Parity     = COM_PARITY_NONE;
  BspCOMInit.HwFlowCtl  = COM_HWCONTROL_NONE;
  if (BSP_COM_Init(COM1, &BspCOMInit) != BSP_ERROR_NONE)
  {
    Error_Handler();
  }

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)  // Hoofdlus: draait continu zolang het apparaat aan is
  {
    // Verwerk alle USB-events (verbinding, data ontvangen/verzenden, etc.)
    // MOET regelmatig aangeroepen worden, anders werkt USB niet correct
    tud_task();

    // Scan de 4x4 knoppenmatrix en detecteer indrukken/loslaten
    scan_matrix();

    // Lees potentiometers en stuur MIDI CC's
    handle_potentiometers();

    // Verwerk MIDI-taken: binnenkomende data lezen + LED laten knipperen
    midi_task();

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI48|RCC_OSCILLATORTYPE_HSI
                              |RCC_OSCILLATORTYPE_CSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSIDiv = RCC_HSI_DIV2;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
  RCC_OscInitStruct.CSIState = RCC_CSI_ON;
  RCC_OscInitStruct.CSICalibrationValue = RCC_CSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLL1_SOURCE_CSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 32;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 3;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1_VCIRANGE_2;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1_VCORANGE_WIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_PCLK3;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure the programming delay
  */
  __HAL_FLASH_SET_PROGRAM_DELAY(FLASH_PROGRAMMING_DELAY_0);
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc1.Init.Resolution = ADC_RESOLUTION_8B;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.NbrOfConversion = 2;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_EXTERNALTRIG_T6_TRGO;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.SamplingMode = ADC_SAMPLING_MODE_NORMAL;
  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc1.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_24CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  sConfig.SamplingTime = ADC_SAMPLETIME_2CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief GPDMA1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPDMA1_Init(void)
{

  /* USER CODE BEGIN GPDMA1_Init 0 */

  /* USER CODE END GPDMA1_Init 0 */

  /* Peripheral clock enable */
  __HAL_RCC_GPDMA1_CLK_ENABLE();

  /* GPDMA1 interrupt Init */
    HAL_NVIC_SetPriority(GPDMA1_Channel0_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(GPDMA1_Channel0_IRQn);

  /* USER CODE BEGIN GPDMA1_Init 1 */

  /* USER CODE END GPDMA1_Init 1 */
  /* USER CODE BEGIN GPDMA1_Init 2 */

  /* USER CODE END GPDMA1_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 0x7;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  hspi1.Init.NSSPolarity = SPI_NSS_POLARITY_LOW;
  hspi1.Init.FifoThreshold = SPI_FIFO_THRESHOLD_01DATA;
  hspi1.Init.MasterSSIdleness = SPI_MASTER_SS_IDLENESS_00CYCLE;
  hspi1.Init.MasterInterDataIdleness = SPI_MASTER_INTERDATA_IDLENESS_00CYCLE;
  hspi1.Init.MasterReceiverAutoSusp = SPI_MASTER_RX_AUTOSUSP_DISABLE;
  hspi1.Init.MasterKeepIOState = SPI_MASTER_KEEP_IO_STATE_DISABLE;
  hspi1.Init.IOSwap = SPI_IO_SWAP_DISABLE;
  hspi1.Init.ReadyMasterManagement = SPI_RDY_MASTER_MANAGEMENT_INTERNALLY;
  hspi1.Init.ReadyPolarity = SPI_RDY_POLARITY_HIGH;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief TIM6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM6_Init(void)
{

  /* USER CODE BEGIN TIM6_Init 0 */

  /* USER CODE END TIM6_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM6_Init 1 */

  /* USER CODE END TIM6_Init 1 */
  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 249;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 999;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM6_Init 2 */

  /* USER CODE END TIM6_Init 2 */

}

/**
  * @brief USB Initialization Function
  * @param None
  * @retval None
  */
static void MX_USB_PCD_Init(void)
{

  /* USER CODE BEGIN USB_Init 0 */

  /* USER CODE END USB_Init 0 */

  /* USER CODE BEGIN USB_Init 1 */

  /* USER CODE END USB_Init 1 */
  hpcd_USB_DRD_FS.Instance = USB_DRD_FS;
  hpcd_USB_DRD_FS.Init.dev_endpoints = 8;
  hpcd_USB_DRD_FS.Init.speed = USBD_FS_SPEED;
  hpcd_USB_DRD_FS.Init.phy_itface = PCD_PHY_EMBEDDED;
  hpcd_USB_DRD_FS.Init.Sof_enable = DISABLE;
  hpcd_USB_DRD_FS.Init.low_power_enable = DISABLE;
  hpcd_USB_DRD_FS.Init.lpm_enable = DISABLE;
  hpcd_USB_DRD_FS.Init.battery_charging_enable = DISABLE;
  hpcd_USB_DRD_FS.Init.vbus_sensing_enable = DISABLE;
  hpcd_USB_DRD_FS.Init.bulk_doublebuffer_enable = DISABLE;
  hpcd_USB_DRD_FS.Init.iso_singlebuffer_enable = DISABLE;
  if (HAL_PCD_Init(&hpcd_USB_DRD_FS) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USB_Init 2 */

  /* USER CODE END USB_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);

  /*Configure GPIO pin : PA4 */
  GPIO_InitStruct.Pin = GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

//--------------------------------------------------------------------+
// MCP23S17 Functions
//--------------------------------------------------------------------+

// Schrijft een waarde naar een intern register van de MCP23S17 via SPI
// Werking: stuur 3 bytes over SPI: [controle-byte | registeradres | waarde]
void mcp23s17_write_reg(uint8_t reg, uint8_t value)
{
  uint8_t tx_data[3];
  tx_data[0] = MCP23S17_ADDR | MCP23S17_WRITE; // Controle-byte: adres van de chip + schrijf-bit
  tx_data[1] = reg;                             // Welk register wil je schrijven?
  tx_data[2] = value;                           // Welke waarde wil je erin zetten?

  CS_LOW();                                     // Activeer de chip (communicatie starten)
  HAL_SPI_Transmit(&hspi1, tx_data, 3, 100);    // Stuur 3 bytes via SPI (timeout 100ms)
  CS_HIGH();                                    // Deactiveer de chip (communicatie stoppen)
}

// Leest de waarde uit een intern register van de MCP23S17 via SPI
// SPI is full-duplex: zenden en ontvangen gebeuren tegelijkertijd
// Werking: stuur 3 bytes [controle-byte | registeradres | dummy], ontvang 3 bytes tegelijk
uint8_t mcp23s17_read_reg(uint8_t reg)
{
  // Verzenddata: byte 0 = controle (adres + lees-bit), byte 1 = registeradres, byte 2 = dummy (0xFF)
  uint8_t tx_data[3] = { MCP23S17_ADDR | MCP23S17_READ, reg, 0xFF };
  uint8_t rx_data[3] = { 0, 0, 0 };  // Ontvangstbuffer (3 bytes, evenredig met verzonden bytes)

  CS_LOW();                                                    // Activeer de chip
  HAL_SPI_TransmitReceive(&hspi1, tx_data, rx_data, 3, 100);  // Stuur en ontvang tegelijk (full-duplex)
  CS_HIGH();                                                   // Deactiveer de chip

  // De echte registerwaarde zit in het DERDE ontvangen byte
  // (de eerste twee bytes zijn rommel die de chip stuurt terwijl hij het adres verwerkt)
  return rx_data[2];
}

// Initialiseer de MCP23S17: stel richtingen, pull-ups en beginwaarden in
//
// Matrixopbouw:
//   Poort A bits 0-3 = KOLOMMEN (uitgangen) -> we sturen ze aan
//   Poort B bits 0-3 = RIJEN    (ingangen)  -> we lezen ze af
//
// Hoe de matrix werkt:
//   1. Zet één kolom LAAG, de rest HOOG
//   2. Lees alle rijen: als een rij LAAG is, dan is de knop op die rij+kolom ingedrukt
//   3. Rijen hebben pull-ups: standaard HOOG, bij indrukken verbinding naar de lage kolom -> rij gaat LAAG
void mcp23s17_init(void)
{
  // Wacht zodat de MCP23S17 volledig is opgestart
  HAL_Delay(50);
  
  // Poort A: bits 0-3 = uitgang (kolommen), bits 4-7 = ingang (niet gebruikt)
  // 0xF0 = 1111 0000 in binair: bit=1 = ingang, bit=0 = uitgang
  mcp23s17_write_reg(MCP23S17_IODIRA, 0xF0);
  
  // Poort B: alle bits = ingang (rijen + niet-gebruikte pinnen)
  // 0xFF = 1111 1111 in binair: alle pinnen als ingang
  mcp23s17_write_reg(MCP23S17_IODIRB, 0xFF);
  
  // Poort A heeft geen pull-ups nodig (zijn uitgangen)
  mcp23s17_write_reg(MCP23S17_GPPUA, 0x00);
  
  // Poort B rijen (bits 0-3) krijgen interne pull-up weerstanden
  // Hierdoor staat een rij standaard HOOG en gaat LAAG als een knop ingedrukt wordt
  mcp23s17_write_reg(MCP23S17_GPPUB, 0x0F);
  
  // Alle kolommen beginnen HOOG (ruststand, geen kolom geselecteerd)
  // 0x0F = 0000 1111: bits 0-3 HOOG
  mcp23s17_write_reg(MCP23S17_GPIOA, 0x0F);
  
  // Kleine pauze om de chip te laten stabiliseren
  HAL_Delay(10);
}

//--------------------------------------------------------------------+
// Matrix Scanning
//--------------------------------------------------------------------+

// Scant de 4x4 knoppenmatrix en stuurt MIDI-berichten bij toestandsverandering
//
// Algoritme per scancyclus:
//   Voor elke kolom (0-3):
//     1. Zet die kolom LAAG, alle andere HOOG
//     2. Wacht even zodat de signalen stabiliseren
//     3. Lees de 4 rijen: rij LAAG = knop ingedrukt (door pull-up)
//     4. Pas debounceteller aan per knop
//     5. Als toestand veranderd: stuur Note On of Note Off
void scan_matrix(void)
{
  static uint32_t last_scan_ms = 0;
  
  // Scan elke 50 milliseconden (te snel scannen geeft problemen met debouncen)
  if (HAL_GetTick() - last_scan_ms < 50) {
    return;  // Nog niet 50ms verstreken -> niets doen
  }
  last_scan_ms = HAL_GetTick();  // Sla het tijdstip van deze scan op

  // Controleer of de MCP23S17 nog correct werkt door IODIRB terug te lezen
  // IODIRB moet 0xFF zijn (alles ingang) - als dat niet zo is, is er een fout
  uint8_t iodirb = mcp23s17_read_reg(MCP23S17_IODIRB);
  if (iodirb != 0xFF) {
    // MCP23S17 geeft een verkeerde waarde -> opnieuw initialiseren en scan overslaan
    mcp23s17_init();
    return;
  }

  // Debounceteller per knop: telt hoeveel keer de knop aaneengesloten hetzelfde was
  // Doel: vermijden dat een knop meerdere keren triggert door contact-stuiter (bounce)
  static uint8_t debounce_count[16] = {0};
  const uint8_t DEBOUNCE_THRESHOLD = 2; // Knop moet 2 scans stabiel zijn voor we hem accepteren

  // Loop over alle 4 kolommen
  for (uint8_t col = 0; col < MATRIX_COLS; col++)
  {
    // Maak een patroon waarbij alleen de huidige kolom LAAG is, de rest HOOG
    // Voorbeeld: col=1 -> ~(0000 0010) & 0x0F = 1111 1101 & 0x0F = 0000 1101
    uint8_t col_pattern = ~(1 << col) & 0x0F;
    mcp23s17_write_reg(MCP23S17_GPIOA, col_pattern);  // Stel kolompatroon in
    
    // Korte wachttijd zodat de spanning op de pinnen kan stabiliseren
    // (HAL_Delay kan niet want die blokkeert te lang - luttele microseconden volstaan)
    for (volatile uint16_t i = 0; i < 100; i++);
    
    // Lees de toestand van de 4 rijen (bits 0-3 van poort B)
    // 0x0F masker = we willen alleen de onderste 4 bits
    uint8_t row_data = mcp23s17_read_reg(MCP23S17_GPIOB) & 0x0F;
    
    // Controleer elke rij of de knop ingedrukt is
    for (uint8_t row = 0; row < MATRIX_ROWS; row++)
    {
      // Bereken de unieke index van deze knop (0-15)
      // Rij 0, kolom 0 = knop 0; rij 0, kolom 1 = knop 1; rij 1, kolom 0 = knop 4; enz.
      uint8_t button_index = row * MATRIX_COLS + col;
      
      // Knop is ingedrukt als de bijbehorende rijbit LAAG is (dankzij pull-up logica)
      // !(row_data & (1 << row)): als bit van de rij 0 is -> ingedrukt = true
      bool pressed = !(row_data & (1 << row));
      
      // Debounce: verhoog teller als ingedrukt, verlaag als losgelaten
      if (pressed) {
        if (debounce_count[button_index] < DEBOUNCE_THRESHOLD) {
          debounce_count[button_index]++;  // Knop stabiel ingedrukt: teller omhoog
        }
      } else {
        if (debounce_count[button_index] > 0) {
          debounce_count[button_index]--;  // Knop stabiel losgelaten: teller omlaag
        }
      }
      
      // Knop is pas "officieel" ingedrukt als de teller de drempel bereikt
      bool new_state = (debounce_count[button_index] >= DEBOUNCE_THRESHOLD);
      
      // Flank-detectie: detecteer de OVERGANG van losgelaten naar ingedrukt en omgekeerd
      if (new_state && !button_prev_state[button_index])
      {
        // Stijgende flank: knop werd NET ingedrukt -> stuur MIDI Note On
        uint8_t note = BASE_NOTE + button_index;  // Nootnummer = 60 + knoopindex (0-15)
        send_midi_note(note, true);
        button_prev_state[button_index] = true;  // Sla nieuwe toestand op
      }
      else if (!new_state && button_prev_state[button_index])
      {
        // Dalende flank: knop werd NET losgelaten -> stuur MIDI Note Off
        uint8_t note = BASE_NOTE + button_index;
        send_midi_note(note, false);
        button_prev_state[button_index] = false;  // Sla nieuwe toestand op
      }
    }
  }
  
  // Zet alle kolommen terug HOOG na de scan (ruststand)
  mcp23s17_write_reg(MCP23S17_GPIOA, 0x0F);
}

//--------------------------------------------------------------------+
// MIDI Functions
//--------------------------------------------------------------------+

// Stuurt een MIDI CC (Control Change) bericht over USB naar de computer
//
// MIDI CC bericht: [0xB0 | kanaal, controller#, waarde]
// controller# = 0-119 (verschillende MIDI CC's)
// waarde = 0-127 (controlwaarde, bijv. volume)
//
// Gebruikte CC's:
// CC #10 (Pan-potentiometer 1): LR pan
// CC #11 (Expression-potentiometer 2): expressie/dynamica
void send_midi_cc(uint8_t controller, uint8_t value)
{
  // Controleer of de USB-verbinding actief is
  if (!tud_midi_mounted()) {
    return;
  }

  uint8_t cable_num = 0;   // USB MIDI kabel nummer
  uint8_t channel = 0;     // MIDI-kanaal 0 = kanaal 1
  uint8_t msg[3];          // MIDI-bericht: 3 bytes

  // CC (Control Change) bericht: statusbyte 0xB0 + kanaalnummer
  msg[0] = 0xB0 | channel;    // 0xB0 = CC commando
  msg[1] = controller;        // Welke controller (0-119)
  msg[2] = value;             // Waarde (0-127)

  // Stuur het 3-byte MIDI-bericht via USB
  tud_midi_stream_write(cable_num, msg, 3);
}

//--------------------------------------------------------------------+
// Potentiometer Handling
//--------------------------------------------------------------------+

// Lees beide potentiometers in en stuur MIDI CC's als ze veranderen
//
// De ADC leest voortdurend PA0 (kanaal 0) en PA1 (kanaal 1) via DMA.
// Deze buffer-waarden worden aan het eind van elke ADC-conversie
// geüpdate in de DMA.
//
// PA0 (ADC_CHANNEL_0) -> CC#10 (pan)
// PA1 (ADC_CHANNEL_1) -> CC#11 (expression)
//
// Voor elke potentiometer:
//  1. Controleer of de waarde meer dan HYSTERESIS veranderd is
//  2. Als je, stuur MIDI CC bericht
//  3. Pas vorige waarde aan voor volgende keer
void handle_potentiometers(void)
{
  // Potentiometer 1 (PA0 / ADC channel 0) -> CC #10 (Pan)
  if (adc_buffer[0] != adc_prev[0] && 
      (adc_buffer[0] >= adc_prev[0] + ADC_HYSTERESIS || 
       adc_buffer[0] <= adc_prev[0] - ADC_HYSTERESIS))
  {
    send_midi_cc(10, adc_buffer[0]);  // CC #10 = Pan
    adc_prev[0] = adc_buffer[0];
  }

  // Potentiometer 2 (PA1 / ADC channel 1) -> CC #11 (Expression)
  if (adc_buffer[1] != adc_prev[1] && 
      (adc_buffer[1] >= adc_prev[1] + ADC_HYSTERESIS || 
       adc_buffer[1] <= adc_prev[1] - ADC_HYSTERESIS))
  {
    send_midi_cc(11, adc_buffer[1]);  // CC #11 = Expression
    adc_prev[1] = adc_buffer[1];
  }
}

//--------------------------------------------------------------------+
// MIDI Functions
//--------------------------------------------------------------------+

// Stuurt een MIDI Note On of Note Off bericht over USB naar de computer
//
// MIDI Note On bericht: [0x90 | kanaal, nootnummer, velocity]
// MIDI Note Off bericht: [0x80 | kanaal, nootnummer, 0]
//
// Nootnummer 60 = Middle C, 61 = C#, 62 = D, ... 75 = D# (knop 15)
// Velocity = hoe hard de noot gespeeld wordt (hier vast op 100)
void send_midi_note(uint8_t note, bool on)
{
  // Controleer of de USB-verbinding actief is (computer heeft het apparaat herkend)
  // Geen verbinding = geen reden om te sturen
  if (!tud_midi_mounted()) {
    return;
  }

  uint8_t cable_num = 0;   // USB MIDI kabel nummer (altijd 0 bij 1 kabel)
  uint8_t channel = 0;     // MIDI-kanaal 0 = kanaal 1 (MIDI-kanalen zijn 0-15 intern)
  uint8_t velocity = 100;  // Aansilagsterkte: 0=stil, 127=maximaal (100 = redelijk hard)
  uint8_t msg[3];          // MIDI-bericht is altijd 3 bytes voor Note On/Off

  if (on)
  {
    // Note On: statusbyte 0x90 + kanaalnummer, dan nootnummer, dan velocity
    msg[0] = 0x90 | channel;  // 0x90 = Note On commando, | channel voegt kanaalnummer toe
    msg[1] = note;            // Welke noot (0-127, waarbij 60 = Middle C)
    msg[2] = velocity;        // Hoe hard (1-127, 0 wordt soms behandeld als Note Off)
  }
  else
  {
    // Note Off: statusbyte 0x80 + kanaalnummer, dan nootnummer, dan velocity 0
    msg[0] = 0x80 | channel;  // 0x80 = Note Off commando
    msg[1] = note;            // Zelfde noot als bij Note On
    msg[2] = 0;               // Velocity 0 bij Note Off (loslaat-snelheid)
  }

  // Stuur het 3-byte MIDI-bericht via USB naar de computer
  tud_midi_stream_write(cable_num, msg, 3);
}

//--------------------------------------------------------------------+
// MIDI Task
//--------------------------------------------------------------------+

// MIDI-taak: verwerk binnenkomende MIDI-data en stuur de LED aan als statusindicator
void midi_task(void)
{
  // Lees en gooi alle binnenkomende MIDI-pakketten weg
  // WHY: de USB MIDI-interface heeft altijd een ingang én uitgang (zelfs als we de ingang niet gebruiken)
  // Als we binnenkomende data NIET lezen, raakt de buffer vol en blokkeert de computer met sturen
  uint8_t packet[4];  // MIDI USB-pakket is altijd 4 bytes (1 header + 3 data)
  while ( tud_midi_available() ) tud_midi_packet_read(packet);  // Lees en negeer
  
  // Gebruik de ingebouwde LED (PA5 = LD2 op Nucleo) als verbindingsindicator:
  //   - Snel knipperen (elke 250ms): USB verbonden en klaar voor gebruik
  //   - Langzaam knipperen (elke 1000ms): USB niet verbonden
  static uint32_t blink_ms = 0;
  uint32_t blink_interval = tud_midi_mounted() ? 250 : 1000;  // Kies knippersnelheid op basis van verbinding
  
  if (HAL_GetTick() - blink_ms > blink_interval)  // Is het tijd om te wisselen?
  {
    blink_ms = HAL_GetTick();                     // Sla het tijdstip op
    HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);        // Wissel de LED (aan->uit of uit->aan)
  }
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
