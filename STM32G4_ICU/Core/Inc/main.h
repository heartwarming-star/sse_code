/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32g4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */
extern SPI_HandleTypeDef hspi3;
/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define RCC_OSC_IN_Pin GPIO_PIN_0
#define RCC_OSC_IN_GPIO_Port GPIOF
#define RCC_OSC_OUT_Pin GPIO_PIN_1
#define RCC_OSC_OUT_GPIO_Port GPIOF
#define T_SWCLK_Pin GPIO_PIN_14
#define T_SWCLK_GPIO_Port GPIOA
#define T_SWO_Pin GPIO_PIN_3
#define T_SWO_GPIO_Port GPIOB
#define OLED_CS_Pin GPIO_PIN_5
#define OLED_CS_GPIO_Port GPIOB
#define OLED_DC_Pin GPIO_PIN_6
#define OLED_DC_GPIO_Port GPIOB
#define OLED_Reset_Pin GPIO_PIN_7
#define OLED_Reset_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */
// SW7/SW6/SW5 ///   SW9/SW2/SW1
//     SW4      /       SW8
//    RESET     /       SW3
// IN/IN/EX   /// WSReset / EX / IN
//     DISP     /       DISP
//      



//=====GPIOA=====//
#define ROT1_Pin GPIO_PIN_0 //ADC1_IN1
#define ROT1_Port GPIOA
#define ROT2_Pin GPIO_PIN_1 //ADC1_IN2
#define ROT2_Port GPIOA
#define ROT3_Pin GPIO_PIN_2 //ADC1_IN3
#define ROT3_Port GPIOA
#define ROT4_Pin GPIO_PIN_3 //ADC1_IN4
#define ROT4_Port GPIOA
#define LED_L_Pin GPIO_PIN_4 //GPIO_OUT / EVENT_OUT
#define LED_L_Port GPIOA
#define LED_R_Pin GPIO_PIN_5 //GPIO_OUT
#define LED_R_Port GPIOA
#define LED_C1_Pin GPIO_PIN_6 // GPIO_OUT
#define LED_C1_Port GPIOA
#define ENC1_1_Pin GPIO_PIN_7 // GPIO_EXTI
#define ENC1_1_Port GPIOA
#define SW7_Pin GPIO_PIN_8  // GPIO_Input
#define SW7_Port GPIOA
#define SW8_Pin GPIO_PIN_9 // GPIO_EXTI
#define SW8_Port GPIOA
#define SW9_Pin GPIO_PIN_10 // GPIO_EXTI
#define SW9_Port GPIOA
#define CAN_RX_Pin GPIO_PIN_11 //FDCAN1_RX
#define CAN_RX_Port GPIOA
#define CAN_TX_Pin GPIO_PIN_12 //FDCAN1_TX
#define CAN_TX_Port GPIOA
// #define _Pin GPIO_PIN_13
// #define _Port GPIOA
// #define _Pin GPIO_PIN_14
// #define _Port GPIOA
// #define _Pin GPIO_PIN_15
// #define _Port GPIOA

//=====GPIOB=====//
#define ENC2_2_Pin GPIO_PIN_0 // GPIO_Input
#define ENC2_2_Port GPIOB
#define HALL2_Pin GPIO_PIN_1 //ADC3_IN1
#define HALL2_Port GPIOB
#define ENC2_1_Pin GPIO_PIN_2 // GPIO_EXTI?
#define ENC2_1_Port GPIOB
// #define _Pin GPIO_PIN_3
// #define _Port GPIOB
// #define _Pin GPIO_PIN_4
// #define _Port GPIOB
#define OLED_CS_Pin GPIO_PIN_5 // GPIO_Output
#define OLED_CS_Port GPIOB
#define OLED_DC_Pin GPIO_PIN_6 // GPIO_Output
#define OLED_DC_Port GPIOB
#define OLED_RESET_Pin GPIO_PIN_7 // GPIO_Output
#define OLED_RESET_Port GPIOB
// #define _Pin GPIO_PIN_8
// #define _Port GPIOB
#define ENC2_BT_Pin GPIO_PIN_9 //GPIO_Input
#define ENC2_BT_Port GPIOB
#define LED_C2_Pin GPIO_PIN_10 //GPIO_Output
#define LED_C2_Port GPIOB
#define ENC1_2_Pin GPIO_PIN_11 //GPIO_Input
#define ENC1_2_Port GPIOB
#define HALL1_Pin GPIO_PIN_12 //ADC1_IN11
#define HALL1_Port GPIOB
#define SW1_Pin GPIO_PIN_13 //GPIO_Input
#define SW1_Port GPIOB
#define ROT5_Pin GPIO_PIN_14 //ADC1_IN5
#define ROT5_Port GPIOB
#define SW2_Pin GPIO_PIN_15 //GPIO EXTI
#define SW2_Port GPIOB

//=====GPIOC=====//
// #define _Pin GPIO_PIN_0
// #define _Port GPIOC
// #define _Pin GPIO_PIN_1
// #define _Port GPIOC
#define L_BLINK_Pin GPIO_PIN_2 //ADC2_IN8
#define L_BLINK_Port GPIOC
#define R_BLINK_Pin GPIO_PIN_3 //ADC2_IN9
#define R_BLINK_Port GPIOC
// #define _Pin GPIO_PIN_4
// #define _Port GPIOC
// #define _Pin GPIO_PIN_5
// #define _Port GPIOC
#define SW3_Pin GPIO_PIN_6 //GPIO_EXTI
#define SW3_Port GPIOC
#define SW4_Pin GPIO_PIN_7 //GPIO_Input
#define SW4_Port GPIOC
#define SW5_Pin GPIO_PIN_8 //GPIO_EXTI
#define SW5_Port GPIOC
#define SW6_Pin GPIO_PIN_9 //GPIO_INPUT
#define SW6_Port GPIOC
#define SCK_Pin GPIO_PIN_10 //SPI3_SCK
#define SCK_Port GPIOC
// #define _Pin GPIO_PIN_11
// #define _Port GPIOC
#define MOSI_Pin GPIO_PIN_12 //SPI3_MOSI
#define MOSI_Port GPIOC
#define ENC1_BT_Pin GPIO_PIN_13 // GPIO_EXTI
#define ENC1_BT_Port GPIOC
// #define _Pin GPIO_PIN_14
// #define _Port GPIOC
// #define _Pin GPIO_PIN_15
// #define _Port GPIOC



/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
