/* Refactored main.c that calls module functions and preserves HAL callbacks and MX_* initializers.
   Assumes CubeMX generated functions are present (MX_GPIO_Init, MX_DMA_Init, etc.). */

#include "main.h"
#include <string.h>
#include <stdbool.h>
#include <stdio.h>

/* Modules */
#include "protocol.h"
#include "vcu_hal.h"
#include "vcu_state.h"
#include "ws22.h"
#include "cc.h"
#include "telemetry.h"
#include "mppt.h"
#include "bms.h"
#include "signals.h"
#include "sensors.h"
#include "bsp.h"

ADC_HandleTypeDef hadc3; 
DMA_HandleTypeDef hdma_adc3;
FDCAN_HandleTypeDef hfdcan1;
FDCAN_HandleTypeDef hfdcan2;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim6;
UART_HandleTypeDef huart1;
UART_HandleTypeDef huart3;
DMA_HandleTypeDef hdma_usart1_tx;
DMA_HandleTypeDef hdma_usart3_tx;

/* ---- Application entry ---- */
int main(void)
{
 BSP_Init();

  /* Start base peripherals */
  HAL_TIM_Base_Start(&htim6);

  if (HAL_ADC_Start_DMA(&hadc3, (uint32_t*)ADC3_DMA_Buffer, 1) != HAL_OK) {
    Error_Handler();
  }

  strcpy((char*)USART3_TxBuffer, "\033[2J\033[H");

  /* Gear init */
  UpdateGearState();

  /* Initial CC check */
  CC_Check();

  while (1) {
    /* All periodic logic handled by interrupts */
  }
}


void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
  if (htim->Instance == TIM2) {
    WS22_SendMotorCommand();
    UpdateGearState();
    CC_Check();

    if (turn_signal_interval_counter >= turn_signal_interval) {
      Signals_UpdateOutputs();
      turn_signal_interval_counter = 0;
    } else {
      turn_signal_interval_counter++;
    }

    if (cruise_control_signal_interval_counter >= cruise_control_signal_interval){
      CC_SendToICU(ICU_RX_BASE_ADDR+CRUISE_CONTROL_ADDR, cruise_control_on, cruise_control_error, cruise_control_speed_target);
      cruise_control_signal_interval_counter = 0;
    } else {
      cruise_control_signal_interval_counter++;
    }

    if (update_ICU_interval_counter > update_ICU_interval){
      ICU_UpdateDriveMode(ICU_RX_BASE_ADDR+DRIVE_MODE_ADDR);
      ICU_UpdateCruiseLevel(ICU_RX_BASE_ADDR+CRUISE_CONTROL_LEVEL_ADDR);
      update_ICU_interval_counter = 0;
    } else {
      update_ICU_interval_counter++;
    }

    if (USART3_Telemetry_Interval_Counter >= USART3_Telemetry_Interval) {
      USART3_Telemetry_Interval_Counter = 0;
      /* Optional: recompute throttle_portion */
      (void)throttle_input; /* already updated in ADC callback */
      Telemetry_SendUSART3();
    } else {
      USART3_Telemetry_Interval_Counter++;
    }
  }
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
  if (GPIO_Pin == GPIO_PIN_1) {
    if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_1) == GPIO_PIN_SET) {
      CC_Check();
      CC_SendToICU(ICU_RX_BASE_ADDR+CRUISE_CONTROL_ADDR, cruise_control_on, 1, cruise_control_speed_target);
    }
  }
}

void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs) {
  if ((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) != RESET) {
    if (hfdcan->Instance == FDCAN1) {
      if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &FDCAN1_RxHeader, FDCAN1_RxBuffer) == HAL_OK) {
        WS22_HandlePacket(FDCAN1_RxHeader.Identifier);
      }
    } else if (hfdcan->Instance == FDCAN2) {
      if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &FDCAN2_RxHeader, FDCAN2_RxBuffer) == HAL_OK) {
        uint32_t id = FDCAN2_RxHeader.Identifier;
        uint32_t offset;
        if (id >= 0x600 && id < 0x6A0) { /* ICU */
          offset = id-ICU_TX_BASE_ADDR;
          ICU_HandlePacket(offset);
        } else if (id >= 0x6A0 && id <= 0x6FF) { /* MPPT */
          offset = id-MPPT_ARRAY_BASE_ADDR;
          MPPT_ParsePacket(offset);
        } else if (id >= 0x100 && id <= 0x102) { /* BMS */
          offset = id-BMS_RX_BASE_ADDR;
          BMS_ParsePacket(offset);
        }
      }
    }
  }
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc) {
  if(hadc->Instance == ADC3) {
    throttle_input = ADC3_DMA_Buffer[0];
  }
}

/* Keep assert_failed if used */
#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
}
#endif
