/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stm32f1xx_hal.h"
#include <string.h>
#include <stdbool.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define TX_MODE
//#define RX_MODE


/* ---- NRF24L01 hardware-definities ---- */
#define NRF24_CE_PORT    GPIOB
#define NRF24_CE_PIN     GPIO_PIN_0
#define NRF24_CSN_PORT   GPIOB
#define NRF24_CSN_PIN    GPIO_PIN_1

#define NRF24_SPI &hspi1
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi1;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */


/* ---- NRF24L01 register en command defines ---- */
#define W_REGISTER     0x20
#define R_REGISTER     0x00
#define W_TX_PAYLOAD   0xA0
#define R_RX_PAYLOAD   0x61
#define FLUSH_TX       0xE1
#define FLUSH_RX       0xE2

#define CONFIG      0x00
#define EN_AA       0x01
#define EN_RXADDR   0x02
#define SETUP_AW    0x03
#define RF_CH       0x05
#define RF_SETUP    0x06
#define STATUS      0x07
#define TX_ADDR     0x10
#define RX_ADDR_P0  0x0A
#define RX_PW_P0    0x11


#define PAYLOAD_SIZE 32
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
// -- GPIO functies --
void NRF24_CE_High(void)   { HAL_GPIO_WritePin(NRF24_CE_PORT, NRF24_CE_PIN, GPIO_PIN_SET); }
void NRF24_CE_Low(void)    { HAL_GPIO_WritePin(NRF24_CE_PORT, NRF24_CE_PIN, GPIO_PIN_RESET); }
void NRF24_CSN_High(void)  { HAL_GPIO_WritePin(NRF24_CSN_PORT, NRF24_CSN_PIN, GPIO_PIN_SET); }
void NRF24_CSN_Low(void)   { HAL_GPIO_WritePin(NRF24_CSN_PORT, NRF24_CSN_PIN, GPIO_PIN_RESET); }

void nrf24_WriteReg(uint8_t reg, uint8_t value) {
    uint8_t buf[2] = {W_REGISTER | (reg & 0x1F), value};
    NRF24_CSN_Low();
    HAL_SPI_Transmit(NRF24_SPI, buf, 2, 100);
    NRF24_CSN_High();
}
void nrf24_WriteRegMulti(uint8_t reg, const uint8_t* data, uint8_t len) {
    uint8_t cmd = W_REGISTER | (reg & 0x1F);
    NRF24_CSN_Low();
    HAL_SPI_Transmit(NRF24_SPI, &cmd, 1, 100);
    HAL_SPI_Transmit(NRF24_SPI, (uint8_t*)data, len, 100);
    NRF24_CSN_High();
}
uint8_t nrf24_ReadReg(uint8_t reg) {
    uint8_t tx = reg & 0x1F, rx = 0;
    NRF24_CSN_Low();
    HAL_SPI_Transmit(NRF24_SPI, &tx, 1, 100);
    HAL_SPI_Receive(NRF24_SPI, &rx, 1, 100);
    NRF24_CSN_High();
    return rx;
}
void nrf24_SendCmd(uint8_t cmd) {
    NRF24_CSN_Low();
    HAL_SPI_Transmit(NRF24_SPI, &cmd, 1, 100);
    NRF24_CSN_High();
}

/* ---- NRF24 HIGH LEVEL ---- */

void nrf24_Init(void) {
    NRF24_CE_Low();
    NRF24_CSN_High();
    HAL_Delay(5);

    nrf24_SendCmd(FLUSH_TX);
    nrf24_SendCmd(FLUSH_RX);

    nrf24_WriteReg(CONFIG,    0x08);  // Power down, CRC aan (1 byte)
    nrf24_WriteReg(EN_AA,     0x00);  // Auto-ack uit
    nrf24_WriteReg(EN_RXADDR, 0x01);  // Pipe0 aan
    nrf24_WriteReg(SETUP_AW,  0x03);  // 5-byte adres
    nrf24_WriteReg(RF_CH,     0x00);  // Kanaal 0 (pas evt. aan!)
    nrf24_WriteReg(RF_SETUP,  0x06);  // 1Mbps, 0dBm
    nrf24_WriteReg(STATUS,    0x70);  // IRQ clear

    HAL_Delay(5);
}

// Zet module in TX mode (voorbeeldstijl)
void nrf24_SetTxMode(const uint8_t* address) {
    NRF24_CE_Low();
    nrf24_WriteRegMulti(TX_ADDR, address, 5);
    nrf24_WriteRegMulti(RX_ADDR_P0, address, 5); // (voor auto-ack)
    nrf24_WriteReg(RX_PW_P0, PAYLOAD_SIZE);
    nrf24_WriteReg(CONFIG, 0x0A); // Power-up, PRIM_RX=0 (TX), CRC aan
    HAL_Delay(2);
    NRF24_CE_High();
    HAL_Delay(2);
}

// Zet module in RX mode (voorbeeldstijl)
void nrf24_SetRxMode(const uint8_t* address) {
    NRF24_CE_Low();
    nrf24_WriteRegMulti(RX_ADDR_P0, address, 5);
    nrf24_WriteReg(RX_PW_P0, PAYLOAD_SIZE);
    nrf24_WriteReg(CONFIG, 0x0B); // Power-up, PRIM_RX=1 (RX), CRC aan
    HAL_Delay(2);
    NRF24_CE_High();
    HAL_Delay(2);
}

// Zend een data-pakket (max 32 bytes)
void nrf24_SendPacket(const uint8_t* data, uint8_t len) {
    if (len > PAYLOAD_SIZE) len = PAYLOAD_SIZE;
    nrf24_SendCmd(FLUSH_TX);
    NRF24_CSN_Low();
    uint8_t cmd = W_TX_PAYLOAD;
    HAL_SPI_Transmit(NRF24_SPI, &cmd, 1, 100);
    HAL_SPI_Transmit(NRF24_SPI, (uint8_t*)data, len, 100);
    NRF24_CSN_High();
    NRF24_CE_High();
    HAL_Delay(1);
    NRF24_CE_Low();
}

// Ontvang een data-pakket (max 32 bytes)
void nrf24_ReadPayload(uint8_t* buf, uint8_t len) {
    if (len > PAYLOAD_SIZE) len = PAYLOAD_SIZE;
    uint8_t cmd = R_RX_PAYLOAD;
    NRF24_CSN_Low();
    HAL_SPI_Transmit(NRF24_SPI, &cmd, 1, 100);
    HAL_SPI_Receive(NRF24_SPI, buf, len, 100);
    NRF24_CSN_High();
    nrf24_SendCmd(FLUSH_RX);
}

// Check of er een pakket ontvangen is (returns true/false)
bool nrf24_PacketAvailable(void) {
    uint8_t status = nrf24_ReadReg(STATUS);
    if (status & (1<<6)) { // RX_DR bit
        nrf24_WriteReg(STATUS, (1<<6)); // Clear RX_DR
        return true;
    }
    return false;
}

void print_status_uart(void) {
    uint8_t st = nrf24_ReadReg(STATUS);
    char buf[30];
    snprintf(buf, sizeof(buf), "STATUS: 0x%02X\r\n", st);
    HAL_UART_Transmit(&huart2, (uint8_t*)buf, strlen(buf), 100);
}

// ---- User settings ----
uint8_t radio_address[5] = {'A', 'A', 'A', 'x', 'R'};  // Gelijk houden op beide boards!
uint8_t tx_payload[PAYLOAD_SIZE] = {0};
uint8_t rx_buffer[PAYLOAD_SIZE+1] = {0}; // extra byte voor nulterminatie indien gewenst

// ------ Payload packing helpers -----
void uint32_to_bytes(uint32_t val, uint8_t *buf) {
    buf[0] = (val & 0xFF);
    buf[1] = ((val >> 8) & 0xFF);
    buf[2] = ((val >> 16) & 0xFF);
    buf[3] = ((val >> 24) & 0xFF);
}
uint32_t bytes_to_uint32(const uint8_t *buf) {
    return ((uint32_t)buf[0]) |
           ((uint32_t)buf[1] << 8) |
           ((uint32_t)buf[2] << 16) |
           ((uint32_t)buf[3] << 24);
}


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
  MX_SPI1_Init();
  MX_USB_DEVICE_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
  nrf24_Init();                // Init module
#ifdef TX_MODE
    nrf24_SetTxMode(radio_address);

    uint32_t tx_counter = 0;
    char buf[80];

    while (1)
    {
        memset(tx_payload, 0, sizeof(tx_payload));
        uint32_to_bytes(tx_counter, tx_payload); // Zet teller in de eerste 4 bytes
        strcpy((char*)&tx_payload[4], "NRF STM32 test"); // De rest is vrije tekst

        nrf24_SendPacket(tx_payload, PAYLOAD_SIZE);

        snprintf(buf, sizeof(buf), "Verzonden TX-nummer: %lu\r\n", tx_counter);
        HAL_UART_Transmit(&huart2, (uint8_t*)buf, strlen(buf), 100);

        tx_counter++;
        HAL_Delay(100); // Of 50ms, afhankelijk van wat je wilt
    }
#endif

#ifdef RX_MODE
    nrf24_SetRxMode(radio_address);

    uint32_t rx_counter = 0;
    char uartbuf[100];

    while (1)
    {
        if (nrf24_PacketAvailable())
        {
            memset(rx_buffer, 0, sizeof(rx_buffer));
            nrf24_ReadPayload(rx_buffer, PAYLOAD_SIZE);
            uint32_t txnum = bytes_to_uint32(rx_buffer);

            rx_counter++;

            snprintf(uartbuf, sizeof(uartbuf), "Ontvangen RX: %lu | TX-nummer: %lu | Tekst: %s\r\n",
                rx_counter, txnum, (char*)&rx_buffer[4]);
            HAL_UART_Transmit(&huart2, (uint8_t*)uartbuf, strlen(uartbuf), 100);
        }
        HAL_Delay(50);
    }
#endif
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL6;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLL;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
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
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

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
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, CE_Pin|CSN_Pin|LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : CE_Pin CSN_Pin */
  GPIO_InitStruct.Pin = CE_Pin|CSN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : LED_Pin */
  GPIO_InitStruct.Pin = LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

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
	  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_11, 0);
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
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
