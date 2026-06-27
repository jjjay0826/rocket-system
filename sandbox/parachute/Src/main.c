#include "main.h"

I2C_HandleTypeDef hi2c1;
SPI_HandleTypeDef hspi1;
UART_HandleTypeDef huart2;

void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_SPI1_Init(void);
static void MX_USART2_UART_Init(void);

int main(void)
{
  HAL_Init();
  SystemClock_Config();
  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_SPI1_Init();
  MX_USART2_UART_Init();

  while (1)
  {
    // Place to call sensor read and LoRa send functions
  }
}

// Stub functions for peripherals
void SystemClock_Config(void) {}
static void MX_GPIO_Init(void) {}
static void MX_I2C1_Init(void) {}
static void MX_SPI1_Init(void) {}
static void MX_USART2_UART_Init(void) {}
