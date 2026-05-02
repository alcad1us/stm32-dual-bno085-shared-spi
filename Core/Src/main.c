/* USER CODE BEGIN Header */
/**
  * @file           : main.c
  * @brief          : RACLAB ROVER - Advanced Dual IMU Navigation System
  * @author         : Muhammet Yusuf Ozkan
  * @note           : This code implements a robust Sequential Polling Boot
  *                   mechanism for dual BNO085 sensors over a shared SPI bus.
  */
/* USER CODE END Header */

#include "main.h"
#include <stdio.h>
#include <string.h>
#include <stdarg.h>
#include <math.h>

/* --- SYSTEM SETTINGS --- */
#define SHTP_CH_CONTROL   2
#define SHTP_CH_REPORTS   3
#define SET_FEATURE_CMD   0xFD
#define REPORT_ROT_VEC    0x05
#define REPORT_INTERVAL   10000UL /* 100Hz */
#define DASHBOARD_REFRESH 100
#define M_PI              3.14159265358979323846f

/* --- PIN MAPPING (PE PORT) --- */
#define CS1_PIN   GPIO_PIN_0
#define CS2_PIN   GPIO_PIN_1
#define INT1_PIN  GPIO_PIN_4
#define INT2_PIN  GPIO_PIN_5
#define RST_PORT  GPIOC
#define RST_PIN   GPIO_PIN_0

/* --- DATA STRUCTURES --- */
typedef struct {
    int16_t w, x, y, z;
    int16_t roll, pitch, yaw;
    float yaw_offset;
    uint32_t sample_count;
    uint8_t is_calibrated;
} BNO_Data_t;

BNO_Data_t imu1 = {0}, imu2 = {0};

/* --- HARDWARE --- */
SPI_HandleTypeDef hspi1;
UART_HandleTypeDef huart2;
static uint8_t shtp_payload[128];

/* --- PROTOTYPES --- */
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
static void MX_USART2_UART_Init(void);
static void uprintf(const char *fmt, ...);
static void Calculate_Euler(BNO_Data_t *imu);
static int bno_transfer_select(uint16_t cs_pin, uint16_t int_pin, uint8_t *tx, uint16_t tx_len, uint16_t *rx_len, uint8_t *rx_ch, uint32_t timeout);

/**
  * @brief  Main Application Entry Point
  */
int main(void)
{
  HAL_Init();
  SystemClock_Config();
  MX_GPIO_Init();
  MX_SPI1_Init();
  MX_USART2_UART_Init();

  /* Terminal Hazırlığı */
  uprintf("\033[2J\033[H");

  uint8_t cmd[21] = {21, 0, SHTP_CH_CONTROL, 0, SET_FEATURE_CMD, REPORT_ROT_VEC};
  cmd[9] = (uint8_t)(REPORT_INTERVAL & 0xFF);
  cmd[10] = (uint8_t)((REPORT_INTERVAL >> 8) & 0xFF);
  cmd[11] = (uint8_t)((REPORT_INTERVAL >> 16) & 0xFF);
  cmd[12] = (uint8_t)((REPORT_INTERVAL >> 24) & 0xFF);

  uint16_t rlen; uint8_t rch;

  /* SENSOR RESET */
  HAL_GPIO_WritePin(GPIOE, CS1_PIN | CS2_PIN, GPIO_PIN_SET);
  HAL_GPIO_WritePin(RST_PORT, RST_PIN, GPIO_PIN_RESET);
  HAL_Delay(300);
  HAL_GPIO_WritePin(RST_PORT, RST_PIN, GPIO_PIN_SET);
  HAL_Delay(1500);

  /* SEQUENTIAL BOOT (DİSİPLİNLİ BAŞLATMA) */
  bno_transfer_select(CS1_PIN, INT1_PIN, cmd, 21, &rlen, &rch, 500);
  HAL_Delay(200);
  bno_transfer_select(CS2_PIN, INT2_PIN, cmd, 21, &rlen, &rch, 500);

  while (1)
  {
    /* IMU 1 OKUMA */
    if (HAL_GPIO_ReadPin(GPIOE, INT1_PIN) == GPIO_PIN_RESET) {
        if (bno_transfer_select(CS1_PIN, INT1_PIN, NULL, 0, &rlen, &rch, 5) == 0 && rch == SHTP_CH_REPORTS) {
            uint8_t *rpt = (shtp_payload[0] == 0xFB) ? &shtp_payload[5] : &shtp_payload[0];
            if (rpt[0] == REPORT_ROT_VEC) {
                imu1.x = (int16_t)((rpt[5] << 8) | rpt[4]);
                imu1.y = (int16_t)((rpt[7] << 8) | rpt[6]);
                imu1.z = (int16_t)((rpt[9] << 8) | rpt[8]);
                imu1.w = (int16_t)((rpt[11] << 8) | rpt[10]);
                Calculate_Euler(&imu1);
                imu1.sample_count++;
            }
        }
    }

    /* IMU 2 OKUMA */
    if (HAL_GPIO_ReadPin(GPIOE, INT2_PIN) == GPIO_PIN_RESET) {
        if (bno_transfer_select(CS2_PIN, INT2_PIN, NULL, 0, &rlen, &rch, 5) == 0 && rch == SHTP_CH_REPORTS) {
            uint8_t *rpt = (shtp_payload[0] == 0xFB) ? &shtp_payload[5] : &shtp_payload[0];
            if (rpt[0] == REPORT_ROT_VEC) {
                imu2.x = (int16_t)((rpt[5] << 8) | rpt[4]);
                imu2.y = (int16_t)((rpt[7] << 8) | rpt[6]);
                imu2.z = (int16_t)((rpt[9] << 8) | rpt[8]);
                imu2.w = (int16_t)((rpt[11] << 8) | rpt[10]);
                Calculate_Euler(&imu2);
                imu2.sample_count++;
            }
        }
    }

    /* DASHBOARD ÇIKTISI */
    static uint32_t last_ui = 0;
    if (HAL_GetTick() - last_ui >= DASHBOARD_REFRESH) {
        last_ui = HAL_GetTick();
        uprintf("\033[H");
        uprintf("==============================================================\r\n");
        uprintf("          RACLAB ROVER: DUAL IMU STABILIZED DASHBOARD         \r\n");
        uprintf("==============================================================\r\n");
        uprintf("SENSOR | ROLL  | PITCH |  YAW  | SAMPLES | QUAT(W,X,Y,Z)\r\n");
        uprintf("-------|-------|-------|-------|---------|-------------------------\r\n");

        uprintf("IMU_01 | %3d.%d | %3d.%d | %3d.%d | %7u | (%5d,%5d,%5d,%5d)\r\n",
                imu1.roll/10,  abs(imu1.roll%10), imu1.pitch/10, abs(imu1.pitch%10),
                imu1.yaw/10,   abs(imu1.yaw%10), imu1.sample_count, imu1.w, imu1.x, imu1.y, imu1.z);

        uprintf("IMU_02 | %3d.%d | %3d.%d | %3d.%d | %7u | (%5d,%5d,%5d,%5d)\r\n",
                imu2.roll/10,  abs(imu2.roll%10), imu2.pitch/10, abs(imu2.pitch%10),
                imu2.yaw/10,   abs(imu2.yaw%10), imu2.sample_count, imu2.w, imu2.x, imu2.y, imu2.z);

        uprintf("--------------------------------------------------------------\r\n");
        uprintf("[STATUS] SPI: SHARED | MODE: POLLING | RATE: 100Hz            \r\n");

        HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_12);
    }
  }
}

/**
  * @brief  Euler Calculation with Dynamic Tare
  */
static void Calculate_Euler(BNO_Data_t *imu) {
    float w = imu->w / 16384.0f;
    float x = imu->x / 16384.0f;
    float y = imu->y / 16384.0f;
    float z = imu->z / 16384.0f;

    // Roll & Pitch Formulas
    float sinr_cosp = 2.0f * (w * x + y * z);
    float cosr_cosp = 1.0f - 2.0f * (x * x + y * y);
    imu->roll = (int16_t)(atan2f(sinr_cosp, cosr_cosp) * (180.0f / M_PI) * 10.0f);

    float sinp = 2.0f * (w * y - z * x);
    if (fabs(sinp) >= 1) imu->pitch = (int16_t)(copysignf(M_PI / 2.0f, sinp) * (180.0f / M_PI) * 10.0f);
    else imu->pitch = (int16_t)(asinf(sinp) * (180.0f / M_PI) * 10.0f);

    // Yaw with Auto-Tare Logic
    float siny_cosp = 2.0f * (w * z + x * y);
    float cosy_cosp = 1.0f - 2.0f * (y * y + z * z);
    float current_yaw = atan2f(siny_cosp, cosy_cosp) * (180.0f / M_PI);

    if (!imu->is_calibrated && imu->sample_count > 50) {
        imu->yaw_offset = current_yaw;
        imu->is_calibrated = 1;
    }

    float tared_yaw = current_yaw - imu->yaw_offset;
    if (tared_yaw > 180.0f) tared_yaw -= 360.0f;
    if (tared_yaw < -180.0f) tared_yaw += 360.0f;

    imu->yaw = (int16_t)(tared_yaw * 10.0f);
}

/**
  * @brief  BNO085 Selective Transfer with Error Recovery
  */
static int bno_transfer_select(uint16_t cs_pin, uint16_t int_pin, uint8_t *tx, uint16_t tx_len, uint16_t *rx_len, uint8_t *rx_ch, uint32_t timeout) {

    /* SPI Hata Kurtarma Bloğu */
    if (__HAL_SPI_GET_FLAG(&hspi1, SPI_FLAG_OVR)) {
        __HAL_SPI_CLEAR_OVRFLAG(&hspi1);
        hspi1.State = HAL_SPI_STATE_READY;
    }

    uint32_t t = HAL_GetTick();
    while (HAL_GPIO_ReadPin(GPIOE, int_pin) != GPIO_PIN_RESET) {
        if (HAL_GetTick() - t > timeout) return -1;
    }

    HAL_GPIO_WritePin(GPIOE, (cs_pin == CS1_PIN ? CS2_PIN : CS1_PIN), GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOE, cs_pin, GPIO_PIN_RESET);

    for(volatile int i=0; i<20; i++); /* Yerleşme gecikmesi */

    uint8_t tx_h[4] = {0}, rx_h[4] = {0};
    if (tx && tx_len >= 4) memcpy(tx_h, tx, 4);

    if (HAL_SPI_TransmitReceive(&hspi1, tx_h, rx_h, 4, 10) != HAL_OK) {
        HAL_GPIO_WritePin(GPIOE, cs_pin, GPIO_PIN_SET);
        return -1;
    }

    uint16_t pkt_len = ((uint16_t)(rx_h[1] & 0x7F) << 8) | rx_h[0];
    *rx_ch = rx_h[2];

    if (pkt_len < 4 || pkt_len > 1024) {
        HAL_GPIO_WritePin(GPIOE, cs_pin, GPIO_PIN_SET);
        return -1;
    }

    uint16_t total = (((tx_len > pkt_len ? tx_len : pkt_len) + 3) & ~3) - 4;
    uint16_t act_rx = (pkt_len > 4) ? pkt_len - 4 : 0;
    uint16_t act_tx = (tx_len > 4) ? tx_len - 4 : 0;

    for (uint16_t i = 0; i < total; i++) {
        uint8_t b_tx = (i < act_tx) ? tx[i + 4] : 0;
        uint8_t b_rx;
        if(HAL_SPI_TransmitReceive(&hspi1, &b_tx, &b_rx, 1, 10) != HAL_OK) break;
        if (i < act_rx && i < 128) shtp_payload[i] = b_rx;
    }

    HAL_GPIO_WritePin(GPIOE, cs_pin, GPIO_PIN_SET);
    *rx_len = (act_rx > 128) ? 128 : act_rx;
    return 0;
}

static void uprintf(const char *fmt, ...) {
    char buf[256]; va_list ap; va_start(ap, fmt);
    vsnprintf(buf, sizeof(buf), fmt, ap); va_end(ap);
    HAL_UART_Transmit(&huart2, (uint8_t *)buf, strlen(buf), 100);
}

void SystemClock_Config(void) {
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  HAL_RCC_OscConfig(&RCC_OscInitStruct);
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK|RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;
  HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5);
}

static void MX_SPI1_Init(void) {
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_HIGH;
  hspi1.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_64;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  HAL_SPI_Init(&hspi1);
}

static void MX_USART2_UART_Init(void) {
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  HAL_UART_Init(&huart2);
}

static void MX_GPIO_Init(void) {
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  HAL_GPIO_WritePin(GPIOE, CS1_PIN | CS2_PIN, GPIO_PIN_SET);
  HAL_GPIO_WritePin(RST_PORT, RST_PIN, GPIO_PIN_SET);

  GPIO_InitStruct.Pin = GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF5_SPI1;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = CS1_PIN | CS2_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = INT1_PIN | INT2_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = RST_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  HAL_GPIO_Init(RST_PORT, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = GPIO_PIN_12;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);
}

void Error_Handler(void) { __disable_irq(); while (1) {} }
