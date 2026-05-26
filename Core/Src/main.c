/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : BMP390 sensor interfacing with correct Bosch compensation
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include <math.h>

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct {
    volatile uint16_t par_t1;
    volatile uint16_t par_t2;
    volatile int8_t   par_t3;
    volatile int16_t  par_p1;
    volatile int16_t  par_p2;
    volatile int8_t   par_p3;
    volatile int8_t   par_p4;
    volatile uint16_t par_p5;
    volatile uint16_t par_p6;
    volatile int8_t   par_p7;
    volatile int8_t   par_p8;
    volatile int16_t  par_p9;
    volatile int8_t   par_p10;
    volatile int8_t   par_p11;
} BMP390_Calib_Data;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define BMP390_ADDR (0x76 << 1)
/* USER CODE END PD */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

/* USER CODE BEGIN PV */
volatile uint8_t chip_id = 0;
volatile uint8_t raw_data[6];

volatile float actual_temp  = 0.0f;   /* Temperature in Celsius */
volatile float actual_press = 0.0f;   /* Pressure in Pascal     */

BMP390_Calib_Data calib;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
void BMP390_Read_Calibration(void);
void BMP390_Compensate_Data(uint32_t uncomp_press, uint32_t uncomp_temp);

/* USER CODE BEGIN 0 */

void BMP390_Read_Calibration(void)
{
    uint8_t calib_buf[21];
    for (int i = 0; i < 21; i++) calib_buf[i] = 0;

    if (HAL_I2C_Mem_Read(&hi2c1, BMP390_ADDR, 0x31, 1, calib_buf, 21, 500) == HAL_OK)
    {
        calib.par_t1  = (uint16_t)((calib_buf[1]  << 8) | calib_buf[0]);
        calib.par_t2  = (uint16_t)((calib_buf[3]  << 8) | calib_buf[2]);
        calib.par_t3  = (int8_t)   calib_buf[4];
        calib.par_p1  = (int16_t) ((calib_buf[6]  << 8) | calib_buf[5]);
        calib.par_p2  = (int16_t) ((calib_buf[8]  << 8) | calib_buf[7]);
        calib.par_p3  = (int8_t)   calib_buf[9];
        calib.par_p4  = (int8_t)   calib_buf[10];
        calib.par_p5  = (uint16_t)((calib_buf[12] << 8) | calib_buf[11]);
        calib.par_p6  = (uint16_t)((calib_buf[14] << 8) | calib_buf[13]);
        calib.par_p7  = (int8_t)   calib_buf[15];
        calib.par_p8  = (int8_t)   calib_buf[16];
        calib.par_p9  = (int16_t) ((calib_buf[18] << 8) | calib_buf[17]);
        calib.par_p10 = (int8_t)   calib_buf[19];
        calib.par_p11 = (int8_t)   calib_buf[20];
    }
}

/*
 * BMP390 compensation using Bosch datasheet Table 12 scaling factors.
 *
 * Temperature scaling:
 *   par_t1  : / 2^(-8)  =  * 256.0
 *   par_t2  : / 2^(30)
 *   par_t3  : / 2^(48)
 *
 * Pressure scaling:
 *   par_p1  : (val - 2^14) / 2^20
 *   par_p2  : (val - 2^14) / 2^29
 *   par_p3  : / 2^32
 *   par_p4  : / 2^37
 *   par_p5  : / 2^(-3)   =  * 8.0
 *   par_p6  : / 2^6
 *   par_p7  : / 2^8
 *   par_p8  : / 2^15
 *   par_p9  : / 2^48
 *   par_p10 : / 2^48
 *   par_p11 : / 2^65
 */
void BMP390_Compensate_Data(uint32_t uncomp_press, uint32_t uncomp_temp)
{
    if (calib.par_t1 == 0) return;

    /* Scaled temperature calibration coefficients (Bosch Table 12) */
    double par_t1 = (double)calib.par_t1 * 256.0;                  /* / 2^-8  */
    double par_t2 = (double)calib.par_t2 / 1073741824.0;           /* / 2^30  */
    double par_t3 = (double)calib.par_t3 / 281474976710656.0;      /* / 2^48  */

    /* Temperature compensation  */
    double partial_data1 = (double)uncomp_temp - par_t1;
    double partial_data2 = partial_data1 * par_t2;
    double t_lin         = partial_data2 + (partial_data1 * partial_data1) * par_t3;

    actual_temp = (float)t_lin;   /* Result is directly in degrees Celsius */

    /* Scaled pressure calibration coefficients (Bosch Table 12)  */
    double par_p1  = ((double)calib.par_p1  - 16384.0) / 1048576.0;     /* (val-2^14)/2^20 */
    double par_p2  = ((double)calib.par_p2  - 16384.0) / 536870912.0;   /* (val-2^14)/2^29 */
    double par_p3  = (double)calib.par_p3   / 4294967296.0;             /* / 2^32          */
    double par_p4  = (double)calib.par_p4   / 137438953472.0;           /* / 2^37          */
    double par_p5  = (double)calib.par_p5   * 8.0;                      /* / 2^-3          */
    double par_p6  = (double)calib.par_p6   / 64.0;                     /* / 2^6           */
    double par_p7  = (double)calib.par_p7   / 256.0;                    /* / 2^8           */
    double par_p8  = (double)calib.par_p8   / 32768.0;                  /* / 2^15          */
    double par_p9  = (double)calib.par_p9   / 281474976710656.0;        /* / 2^48          */
    double par_p10 = (double)calib.par_p10  / 281474976710656.0;        /* / 2^48          */
    double par_p11 = (double)calib.par_p11  / 36893488147419103232.0;   /* / 2^65          */

    /* Pressure compensation (Bosch datasheet Section 8.5) */
    double partial_out1 = par_p6 * t_lin;
    double partial_out2 = par_p7 * (t_lin * t_lin);
    double partial_out3 = par_p8 * (t_lin * t_lin * t_lin);
    double partial_out4 = par_p5 + partial_out1 + partial_out2 + partial_out3;

    partial_out1 = par_p2 * t_lin;
    partial_out2 = par_p3 * (t_lin * t_lin);
    partial_out3 = par_p4 * (t_lin * t_lin * t_lin);
    double partial_out5 = (double)uncomp_press * (par_p1 + partial_out1 + partial_out2 + partial_out3);

    partial_out1 = (double)uncomp_press * (double)uncomp_press;
    partial_out2 = par_p9 + par_p10 * t_lin;
    partial_out3 = partial_out1 * partial_out2;
    double partial_out6 = partial_out3 + ((double)uncomp_press *
                          (double)uncomp_press *
                          (double)uncomp_press) * par_p11;

    actual_press = (float)(partial_out4 + partial_out5 + partial_out6);
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  */
int main(void)
{
    HAL_Init();
    SystemClock_Config();
    MX_GPIO_Init();
    MX_I2C1_Init();

    /* USER CODE BEGIN 2 */
    HAL_I2C_Mem_Read(&hi2c1, BMP390_ADDR, 0x00, 1, (uint8_t*)&chip_id, 1, 100);

    if (chip_id == 0x60)
    {
        uint8_t reset_cmd = 0xB6;
        HAL_I2C_Mem_Write(&hi2c1, BMP390_ADDR, 0x7E, 1, &reset_cmd, 1, 100);
        HAL_Delay(100);

        uint8_t pwr_ctrl = 0x33;
        HAL_I2C_Mem_Write(&hi2c1, BMP390_ADDR, 0x1B, 1, &pwr_ctrl, 1, 100);
        HAL_Delay(50);

        BMP390_Read_Calibration();
    }
    /* USER CODE END 2 */

    /* Infinite loop */
    /* USER CODE BEGIN WHILE */
    while (1)
    {
        HAL_I2C_Mem_Read(&hi2c1, BMP390_ADDR, 0x00, 1, (uint8_t*)&chip_id, 1, 100);

        if (chip_id == 0x60)
        {
            if (calib.par_t1 == 0)
            {
                uint8_t pwr_ctrl = 0x33;
                HAL_I2C_Mem_Write(&hi2c1, BMP390_ADDR, 0x1B, 1, &pwr_ctrl, 1, 100);
                HAL_Delay(50);
                BMP390_Read_Calibration();
            }
            else
            {
                HAL_I2C_Mem_Read(&hi2c1, BMP390_ADDR, 0x04, 1, (uint8_t*)raw_data, 6, 100);

                uint32_t uncomp_press = ((uint32_t)raw_data[2] << 16) |
                                        ((uint32_t)raw_data[1] <<  8) |
                                         (uint32_t)raw_data[0];

                uint32_t uncomp_temp  = ((uint32_t)raw_data[5] << 16) |
                                        ((uint32_t)raw_data[4] <<  8) |
                                         (uint32_t)raw_data[3];

                BMP390_Compensate_Data(uncomp_press, uncomp_temp);
            }
        }
        else
        {
            actual_temp  = 0.0f;
            actual_press = 0.0f;
        }

        HAL_Delay(200); /* 5 Hz */
        /* USER CODE END WHILE */
        /* USER CODE BEGIN 3 */
    }
    /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  */
void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    __HAL_RCC_PWR_CLK_ENABLE();
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    RCC_OscInitStruct.OscillatorType      = RCC_OSCILLATORTYPE_HSE;
    RCC_OscInitStruct.HSEState            = RCC_HSE_ON;
    RCC_OscInitStruct.PLL.PLLState        = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource       = RCC_PLLSOURCE_HSE;
    RCC_OscInitStruct.PLL.PLLM            = 12;
    RCC_OscInitStruct.PLL.PLLN            = 96;
    RCC_OscInitStruct.PLL.PLLP            = RCC_PLLP_DIV2;
    RCC_OscInitStruct.PLL.PLLQ            = 4;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) Error_Handler();

    RCC_ClkInitStruct.ClockType      = RCC_CLOCKTYPE_HCLK  | RCC_CLOCKTYPE_SYSCLK |
                                       RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource   = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider  = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK) Error_Handler();
}

/**
  * @brief I2C1 Initialization
  */
static void MX_I2C1_Init(void)
{
    hi2c1.Instance             = I2C1;
    hi2c1.Init.ClockSpeed      = 100000;
    hi2c1.Init.DutyCycle       = I2C_DUTYCYCLE_2;
    hi2c1.Init.OwnAddress1     = 0;
    hi2c1.Init.AddressingMode  = I2C_ADDRESSINGMODE_7BIT;
    hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
    hi2c1.Init.OwnAddress2     = 0;
    hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
    hi2c1.Init.NoStretchMode   = I2C_NOSTRETCH_DISABLE;
    if (HAL_I2C_Init(&hi2c1) != HAL_OK) Error_Handler();
}

/**
  * @brief GPIO Initialization
  */
static void MX_GPIO_Init(void)
{
    __HAL_RCC_GPIOH_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
}

void Error_Handler(void)
{
    __disable_irq();
    while (1) {}
}
