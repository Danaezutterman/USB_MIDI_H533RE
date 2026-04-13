#ifndef MCP23S17_H
#define MCP23S17_H

#include "stm32h5xx_hal.h"

/* Register addresses (IOCON.BANK = 0, default) */
#define MCP_IODIRA  0x00
#define MCP_IODIRB  0x01
#define MCP_IOCON   0x0A
#define MCP_GPPUB   0x0D
#define MCP_GPIOA   0x12
#define MCP_GPIOB   0x13

/* SPI opcodes (A2=A1=A0=0) */
#define MCP_WRITE   0x40
#define MCP_READ    0x41

/* Chip Select */
#define MCP_CS_PIN  GPIO_PIN_12
#define MCP_CS_PORT GPIOB

void    MCP23S17_Init    (SPI_HandleTypeDef *hspi);
void    MCP23S17_WriteReg(SPI_HandleTypeDef *hspi, uint8_t reg, uint8_t data);
uint8_t MCP23S17_ReadReg (SPI_HandleTypeDef *hspi, uint8_t reg);

#endif /* MCP23S17_H */
