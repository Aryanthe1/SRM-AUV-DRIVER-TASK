/**
 * @file    main.c
 * @brief   MPU9250 driver usage example — STM32F446RE Nucleo + PlatformIO
 *
 * Wiring (Nucleo CN5 / Arduino header):
 *   MPU9250 VCC  → 3.3 V
 *   MPU9250 GND  → GND
 *   MPU9250 SDA  → PB7  (D14 / I2C1_SDA)
 *   MPU9250 SCL  → PB6  (D15 / I2C1_SCL)
 *   MPU9250 AD0  → GND  (I2C addr = 0x68)
 *   MPU9250 INT  → not connected (polled driver)
 *
 * platformio.ini:
 *   [env:nucleo_f446re]
 *   platform  = ststm32
 *   board     = nucleo_f446re
 *   framework = stm32cube
 *   build_flags = -DUSE_HAL_DRIVER -DSTM32F446xx
 */

#include "main.h"
#include "mpu9250.h"
#include <stdio.h>
#include <string.h>

/* ── Private variables ──────────────────────────────────────────────────── */
I2C_HandleTypeDef hi2c1;
UART_HandleTypeDef huart2;   // ST-Link virtual COM — for debug printf

static MPU9250_Handle imu;

/* ── Private function prototypes ─────────────────────────────────────────── */
static void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART2_UART_Init(void);

/* Route printf → UART2 */
int __io_putchar(int ch)
{
    HAL_UART_Transmit(&huart2, (uint8_t *)&ch, 1, 10);
    return ch;
}

/* ── Main ────────────────────────────────────────────────────────────────── */
int main(void)
{
    HAL_Init();
    SystemClock_Config();
    MX_GPIO_Init();
    MX_I2C1_Init();
    MX_USART2_UART_Init();

    printf("\r\n== MPU9250 driver demo ==\r\n");

    /* Initialise driver: I2C1, AD0=LOW (0x68), ±4g, ±500 °/s */
    MPU9250_Status status = MPU9250_Init(&imu, &hi2c1, 0,
                                         MPU9250_GYRO_FS_500DPS,
                                         MPU9250_ACCEL_FS_4G);
    if (status != MPU9250_OK) {
        printf("MPU9250 init FAILED (err %d)\r\n", status);
        Error_Handler();
    }
    printf("MPU9250 initialised OK\r\n");
    printf("Mag sensitivity: X=%.4f  Y=%.4f  Z=%.4f\r\n",
           imu.mag_sens.x, imu.mag_sens.y, imu.mag_sens.z);

    MPU9250_Vec3 accel, gyro, mag;
    float temp;

    while (1)
    {
        /* ── Option A: read everything in one call ───────────────────────── */
        status = MPU9250_ReadAll(&imu, &accel, &gyro, &temp, &mag);

        if (status == MPU9250_OK || status == MPU9250_MAG_NOT_READY) {
            printf("Accel (g)   : X=%7.3f  Y=%7.3f  Z=%7.3f\r\n",
                   accel.x, accel.y, accel.z);
            printf("Gyro  (°/s) : X=%7.2f  Y=%7.2f  Z=%7.2f\r\n",
                   gyro.x,  gyro.y,  gyro.z);
            printf("Temp  (°C)  : %.2f\r\n", temp);
            if (status == MPU9250_OK)
                printf("Mag   (µT)  : X=%7.2f  Y=%7.2f  Z=%7.2f\r\n",
                       mag.x,   mag.y,   mag.z);
            else
                printf("Mag   (µT)  : not ready\r\n");
            printf("---\r\n");
        } else if (status == MPU9250_ERR_MAG_OVF) {
            printf("Magnetometer overflow — reduce nearby magnetic fields\r\n");
        } else {
            printf("Read error: %d\r\n", status);
        }

        HAL_Delay(100);   // 10 Hz printout
    }
}

/* ══════════════════════════════════════════════════════════════════════════
 *  STM32CubeMX generated peripheral init (I2C1 @ 400 kHz, UART2 @ 115200)
 * ══════════════════════════════════════════════════════════════════════════ */

static void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    __HAL_RCC_PWR_CLK_ENABLE();
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
    RCC_OscInitStruct.HSIState       = RCC_HSI_ON;
    RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
    RCC_OscInitStruct.PLL.PLLState   = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource  = RCC_PLLSOURCE_HSI;
    RCC_OscInitStruct.PLL.PLLM       = 8;
    RCC_OscInitStruct.PLL.PLLN       = 180;
    RCC_OscInitStruct.PLL.PLLP       = RCC_PLLP_DIV2;   // 180 MHz
    RCC_OscInitStruct.PLL.PLLQ       = 2;
    HAL_RCC_OscConfig(&RCC_OscInitStruct);

    HAL_PWREx_EnableOverDrive();

    RCC_ClkInitStruct.ClockType      = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
                                     | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource   = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider  = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;   // 45 MHz
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;   // 90 MHz
    HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5);
}

static void MX_I2C1_Init(void)
{
    /* PB6 = SCL, PB7 = SDA (Arduino header D14/D15 on Nucleo-F446RE) */
    hi2c1.Instance             = I2C1;
    hi2c1.Init.ClockSpeed      = 400000;   // Fast mode 400 kHz
    hi2c1.Init.DutyCycle       = I2C_DUTYCYCLE_2;
    hi2c1.Init.OwnAddress1     = 0;
    hi2c1.Init.AddressingMode  = I2C_ADDRESSINGMODE_7BIT;
    hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
    hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
    hi2c1.Init.NoStretchMode   = I2C_NOSTRETCH_DISABLE;
    HAL_I2C_Init(&hi2c1);
}

static void MX_USART2_UART_Init(void)
{
    /* PA2=TX, PA3=RX — wired to ST-Link on Nucleo */
    huart2.Instance          = USART2;
    huart2.Init.BaudRate     = 115200;
    huart2.Init.WordLength   = UART_WORDLENGTH_8B;
    huart2.Init.StopBits     = UART_STOPBITS_1;
    huart2.Init.Parity       = UART_PARITY_NONE;
    huart2.Init.Mode         = UART_MODE_TX_RX;
    huart2.Init.HwFlowCtl    = UART_HWCONTROL_NONE;
    huart2.Init.OverSampling = UART_OVERSAMPLING_16;
    HAL_UART_Init(&huart2);
}

static void MX_GPIO_Init(void)
{
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();

    /* LD2 (PA5) — blink on error */
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin   = GPIO_PIN_5;
    GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull  = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
}

/**
 * @brief  I2C1 MSP init — called internally by HAL_I2C_Init().
 *         Configures PB6 (SCL) and PB7 (SDA) as AF4, open-drain.
 *         THIS IS REQUIRED — without it the I2C pins are GPIO and the
 *         peripheral is silent on the bus.
 */
void HAL_I2C_MspInit(I2C_HandleTypeDef *hi2c)
{
    if (hi2c->Instance != I2C1) return;

    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_I2C1_CLK_ENABLE();

    /*  PB6 = I2C1_SCL (AF4),  PB7 = I2C1_SDA (AF4)
     *  Open-drain + pull-up required by I2C spec.
     *  The Nucleo board has 4.7 kΩ pull-ups on these lines.        */
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin       = GPIO_PIN_6 | GPIO_PIN_7;
    GPIO_InitStruct.Mode      = GPIO_MODE_AF_OD;          // Open-drain AF
    GPIO_InitStruct.Pull      = GPIO_PULLUP;
    GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF4_I2C1;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
}

void Error_Handler(void)
{
    __disable_irq();
    while (1) {
        HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
        HAL_Delay(200);
    }
}