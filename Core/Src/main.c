#include "main.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"
#include "string.h"

/* USER CODE BEGIN Includes */
#include <stdint.h>
/* USER CODE END Includes */

/* USER CODE BEGIN PV */
/* Private variables */
#define ADC_MAX_VALUE 1023
#define PWM_MAX_DUTY_CYCLE 1000
#define ADC_REF_VOLTAGE 3.3
#define POTENTIOMETER_MAX_VOLTAGE 3.3

uint16_t adc_value = 0;
uint16_t pwm_duty_cycle = 0;
float pot_voltage = 0.0;
/* USER CODE END PV */

/* USER CODE BEGIN PFP */
/* Private function prototypes */
uint16_t Read_ADC_Value(void);
/* USER CODE END PFP */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* MCU Configuration--------------------------------------------------------*/
  HAL_Init();
  void SystemClock_Config(void);
  MX_GPIO_Init();
  MX_USART2_UART_Init();
  MX_SPI1_Init();
  MX_TIM1_Init();

  /* USER CODE BEGIN 2 */
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);

  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_SET);
  /* USER CODE END 2 */

  /* Infinite loop */
  while (1)
  {
    /* USER CODE BEGIN WHILE */
    adc_value = Read_ADC_Value();

    pot_voltage = (adc_value * ADC_REF_VOLTAGE) / ADC_MAX_VALUE;

    pwm_duty_cycle = (pot_voltage / POTENTIOMETER_MAX_VOLTAGE) * PWM_MAX_DUTY_CYCLE;

    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, pwm_duty_cycle);

    HAL_Delay(10);
    /* USER CODE END WHILE */
  }
}

/**
  * @brief  Reads the ADC value via SPI from the MCP3004.
  * @retval uint16_t The ADC value (0–1023 for a 10-bit ADC).
  */
uint16_t Read_ADC_Value(void)
{
  uint8_t spi_tx[3] = {0x01, 0x80, 0x00};
  uint8_t spi_rx[3];

  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_RESET);

  HAL_SPI_TransmitReceive(&hspi1, spi_tx, spi_rx, 3, HAL_MAX_DELAY);

  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_SET);

  uint16_t adc_value = ((spi_rx[1] & 0x03) << 8) | spi_rx[2];
  return adc_value;
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

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI48;
  RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI48;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART2;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  __disable_irq();
  while (1)
  {
  }
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
}
#endif /* USE_FULL_ASSERT */
