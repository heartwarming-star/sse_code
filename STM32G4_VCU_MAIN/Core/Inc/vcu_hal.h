#ifndef VCU_HAL_H
#define VCU_HAL_H

#include "main.h"


#ifdef __cplusplus
extern "C" {
#endif

/* Extern HAL handles defined in main.c */
extern ADC_HandleTypeDef hadc3;
extern DMA_HandleTypeDef hdma_adc3;
extern FDCAN_HandleTypeDef hfdcan1;
extern FDCAN_HandleTypeDef hfdcan2;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim6;
extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart3;
extern DMA_HandleTypeDef hdma_usart1_tx;
extern DMA_HandleTypeDef hdma_usart3_tx;

#ifdef __cplusplus
}
#endif


#endif /* VCU_HAL_H */
