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
#include <icu_globals.h>
#include <bsp.h>
#include <parts.h>
#include <comm.h>
#include <display.h>
#include <input.h>
#include <protocol.h>
#include <lights.h>
#include "SSD1322_OLED_lib/SSD1322.h"

int main(void)
{
  /* MCU Configuration--------------------------------------------------------*/
  BSP_Init();

  /* USER CODE BEGIN 2 */
  HAL_TIM_Base_Start(&htim3);
  HAL_TIM_Base_Start_IT(&htim6);

  if (HAL_ADC_Start_DMA(&hadc1, (uint32_t*)g_AdcDmaBuffer_1, 6) != HAL_OK) {
    Error_Handler();
  }
  if (HAL_ADC_Start_DMA(&hadc2, (uint32_t*)g_AdcDmaBuffer_2, 2) != HAL_OK) {
    Error_Handler();
  }
  if (HAL_ADC_Start_DMA(&hadc3, (uint32_t*)g_AdcDmaBuffer_3, 1) != HAL_OK) {
    Error_Handler();
  }
  display_mode_update_flag = false;

  update_ROT();
  SSD1322_API_init();
  HAL_Delay(1000);
  disp_mode = DRIVING;
  /* USER CODE END 2 */

   while (1)
  {
    /* USER CODE END WHILE */
    /* USER CODE BEGIN 3 */
  }
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){
  if( htim-> Instance == TIM7){
    // Sampling every 1ms
    ENC_L.init = false;
    ENC_R.init = false;
    HAL_TIM_Base_Stop_IT(&htim7);
  }
  else if( htim-> Instance == TIM6){
    
    update_GPIO_Input();
    if (oled_refresh_counter > oled_refresh_interval) {// || display_mode_update_flag 50ms
      oled_refresh_counter = 0;
      if(!display_update_flag)
        updateDisplay();
    }
    else {
      oled_refresh_counter++;
    }

    if (Regen_throttle_counter > Regen_throttle_interval){
      Regen_throttle_counter = 0;
      Send_VCU_CAN_ICU_Command_Regen_Throttle(REGEN_THROTTLE_ADDR, HALL_ADC_input[0]);
    }
    else{
      Regen_throttle_counter++;
    }
  }
}

/* USER CODE END 4 */

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
