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

// MCP23S17 Register Addresses
#define MCP23S17_IODIRA   0x00  // I/O direction A
#define MCP23S17_IODIRB   0x01  // I/O direction B
#define MCP23S17_GPPUA    0x0C  // Pull-up A
#define MCP23S17_GPPUB    0x0D  // Pull-up B
#define MCP23S17_GPIOA    0x12  // Port A
#define MCP23S17_GPIOB    0x13  // Port B

// MCP23S17 Control Byte
#define MCP23S17_ADDR     0x40  // Hardware address (A0=A1=A2=GND)
#define MCP23S17_WRITE    0x00
#define MCP23S17_READ     0x01

// CS Pin (PC9)
#define CS_PIN            GPIO_PIN_9
#define CS_PORT           GPIOC
#define CS_LOW()          HAL_GPIO_WritePin(CS_PORT, CS_PIN, GPIO_PIN_RESET)
#define CS_HIGH()         HAL_GPIO_WritePin(CS_PORT, CS_PIN, GPIO_PIN_SET)

// Matrix configuration
#define MATRIX_ROWS       4
#define MATRIX_COLS       4
#define BASE_NOTE         60  // Middle C

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

COM_InitTypeDef BspCOMInit;

SPI_HandleTypeDef hspi1;

PCD_HandleTypeDef hpcd_USB_DRD_FS;

/* USER CODE BEGIN PV */

// Button state tracking (16 buttons in 4x4 matrix)
static bool button_prev_state[16] = {false};

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
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
  MX_USB_PCD_Init();
  MX_SPI1_Init();
  /* USER CODE BEGIN 2 */

  // Initialize tinyUSB
  tusb_init();
  tusb_hal_init();

  // Wait for USB to enumerate
  HAL_Delay(1000);
  
  // Initialize MCP23S17
  HAL_Delay(10); // Short delay for MCP23S17 to power up
  mcp23s17_init();

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
  while (1)
  {
    // tinyUSB device task
    tud_task();

    // Scan button matrix
    scan_matrix();

    // MIDI application task
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
  RCC_OscInitStruct.PLL.PLLN = 129;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
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
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_SET);  // CS high initially

  /*Configure GPIO pin : PA4 */
  GPIO_InitStruct.Pin = GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PC9 (SPI CS) */
  GPIO_InitStruct.Pin = GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PC4 */
  GPIO_InitStruct.Pin = GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

//--------------------------------------------------------------------+
// MCP23S17 Functions
//--------------------------------------------------------------------+

void mcp23s17_write_reg(uint8_t reg, uint8_t value)
{
  uint8_t tx_data[3];
  tx_data[0] = MCP23S17_ADDR | MCP23S17_WRITE; // Control byte
  tx_data[1] = reg;                             // Register address
  tx_data[2] = value;                           // Data

  CS_LOW();
  HAL_SPI_Transmit(&hspi1, tx_data, 3, 100);
  CS_HIGH();
}

uint8_t mcp23s17_read_reg(uint8_t reg)
{
  // Use a single TransmitReceive call for reliable full-duplex SPI
  uint8_t tx_data[3] = { MCP23S17_ADDR | MCP23S17_READ, reg, 0xFF };
  uint8_t rx_data[3] = { 0, 0, 0 };

  CS_LOW();
  HAL_SPI_TransmitReceive(&hspi1, tx_data, rx_data, 3, 100);
  CS_HIGH();

  // Data is in the 3rd byte (first two are dummy during address phase)
  return rx_data[2];
}

void mcp23s17_init(void)
{
  // Wait a bit for MCP23S17 to be ready
  HAL_Delay(50);
  
  // Configure Port A pins 0-3 as outputs (columns)
  // Configure Port A pins 4-7 as inputs (not used)
  mcp23s17_write_reg(MCP23S17_IODIRA, 0xF0);
  
  // Configure Port B pins 0-3 as inputs (rows)
  // Configure Port B pins 4-7 as inputs (not used)
  mcp23s17_write_reg(MCP23S17_IODIRB, 0xFF);
  
  // Disable pull-ups on Port A (outputs don't need them)
  mcp23s17_write_reg(MCP23S17_GPPUA, 0x00);
  
  // Enable pull-ups on Port B rows
  mcp23s17_write_reg(MCP23S17_GPPUB, 0x0F);
  
  // Set all columns high initially (idle state)
  mcp23s17_write_reg(MCP23S17_GPIOA, 0x0F);
  
  // Small delay after init
  HAL_Delay(10);
}

//--------------------------------------------------------------------+
// Matrix Scanning
//--------------------------------------------------------------------+

void scan_matrix(void)
{
  static uint32_t last_scan_ms = 0;
  
  // Scan every 50ms for better debouncing
  if (HAL_GetTick() - last_scan_ms < 50) {
    return;
  }
  last_scan_ms = HAL_GetTick();

  // SPI sanity check: read back IODIRB (should be 0xFF if MCP23S17 is working)
  uint8_t iodirb = mcp23s17_read_reg(MCP23S17_IODIRB);
  if (iodirb != 0xFF) {
    // MCP23S17 not responding correctly - re-init and skip this scan
    mcp23s17_init();
    return;
  }

  // Debounce counter for each button
  static uint8_t debounce_count[16] = {0};
  const uint8_t DEBOUNCE_THRESHOLD = 2; // Button must be stable for 2 scans

  // Scan each column
  for (uint8_t col = 0; col < MATRIX_COLS; col++)
  {
    // Set current column low, others high
    uint8_t col_pattern = ~(1 << col) & 0x0F;
    mcp23s17_write_reg(MCP23S17_GPIOA, col_pattern);
    
    // Small delay for signals to stabilize (in microseconds if possible)
    // Using a simple loop delay since we can't block with HAL_Delay
    for (volatile uint16_t i = 0; i < 100; i++);
    
    // Read rows
    uint8_t row_data = mcp23s17_read_reg(MCP23S17_GPIOB) & 0x0F;
    
    // Check each row
    for (uint8_t row = 0; row < MATRIX_ROWS; row++)
    {
      uint8_t button_index = row * MATRIX_COLS + col;
      
      // Button is pressed when row pin is LOW (due to pull-up)
      bool pressed = !(row_data & (1 << row));
      
      // Debouncing logic
      if (pressed) {
        if (debounce_count[button_index] < DEBOUNCE_THRESHOLD) {
          debounce_count[button_index]++;
        }
      } else {
        if (debounce_count[button_index] > 0) {
          debounce_count[button_index]--;
        }
      }
      
      // Update button state only when debounced
      bool new_state = (debounce_count[button_index] >= DEBOUNCE_THRESHOLD);
      
      // Detect button state change
      if (new_state && !button_prev_state[button_index])
      {
        // Button just pressed - send Note On
        uint8_t note = BASE_NOTE + button_index;
        send_midi_note(note, true);
        button_prev_state[button_index] = true;
      }
      else if (!new_state && button_prev_state[button_index])
      {
        // Button just released - send Note Off
        uint8_t note = BASE_NOTE + button_index;
        send_midi_note(note, false);
        button_prev_state[button_index] = false;
      }
    }
  }
  
  // Set all columns high again (idle state)
  mcp23s17_write_reg(MCP23S17_GPIOA, 0x0F);
}

//--------------------------------------------------------------------+
// MIDI Functions
//--------------------------------------------------------------------+

void send_midi_note(uint8_t note, bool on)
{
  // Only send if USB MIDI is connected
  if (!tud_midi_mounted()) {
    return;
  }

  uint8_t cable_num = 0;
  uint8_t channel = 0;   // MIDI channel 1
  uint8_t velocity = 100;
  uint8_t msg[3];

  if (on)
  {
    // Note On: 0x90 | channel
    msg[0] = 0x90 | channel;
    msg[1] = note;
    msg[2] = velocity;
  }
  else
  {
    // Note Off: 0x80 | channel
    msg[0] = 0x80 | channel;
    msg[1] = note;
    msg[2] = 0;
  }

  tud_midi_stream_write(cable_num, msg, 3);
}

//--------------------------------------------------------------------+
// MIDI Task
//--------------------------------------------------------------------+

void midi_task(void)
{
  // The MIDI interface always creates input and output port/jack descriptors
  // regardless of these being used or not. Therefore incoming traffic should be read
  // (possibly just discarded) to avoid the sender blocking in IO
  uint8_t packet[4];
  while ( tud_midi_available() ) tud_midi_packet_read(packet);
  
  // LED indicator: fast blink = connected, slow blink = not connected
  static uint32_t blink_ms = 0;
  uint32_t blink_interval = tud_midi_mounted() ? 250 : 1000;
  
  if (HAL_GetTick() - blink_ms > blink_interval)
  {
    blink_ms = HAL_GetTick();
    HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
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
