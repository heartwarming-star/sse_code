#include <string.h>
#include "signals.h"
#include "protocol.h"
#include "vcu_state.h"
#include "vcu_hal.h"
#include "cc.h"


void ICU_SendSignalLights(uint32_t id) {
  FDCAN2_TxHeader.Identifier = id;
  if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan2, &FDCAN2_TxHeader, FDCAN2_Driver_Indicator_TxBuffer) != HAL_OK) {
    VCU_error_flags.ICU_com_error = true;
  }
}

void Signals_UpdateOutputs(void) {
  if (sig_change) {
    sig_change = false;
    switch (VS_CAN_RX_ID) {
      case FROM_ICU_LEFT_TS_ON_ADDR:
        if (left_ts_on) {
          FDCAN2_Driver_Indicator_TxBuffer[0] = 255;
          FDCAN2_Driver_Indicator_TxBuffer[1] = 0;
          FDCAN2_Driver_Indicator_TxBuffer[2] = 0;
          ICU_SendSignalLights(ICU_RX_BASE_ADDR+DRIVER_INDICATOR_ADDR);
          HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_SET);
          HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET);
        } else {
          if (hazard_sig_on) {
            FDCAN2_Driver_Indicator_TxBuffer[0] = 255;
            FDCAN2_Driver_Indicator_TxBuffer[1] = 255;
            FDCAN2_Driver_Indicator_TxBuffer[2] = 255;
            ICU_SendSignalLights(ICU_RX_BASE_ADDR+DRIVER_INDICATOR_ADDR);
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_SET);
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET);
          } else {
            FDCAN2_Driver_Indicator_TxBuffer[0] = 0;
            FDCAN2_Driver_Indicator_TxBuffer[1] = 0;
            FDCAN2_Driver_Indicator_TxBuffer[2] = 0;
            ICU_SendSignalLights(ICU_RX_BASE_ADDR+DRIVER_INDICATOR_ADDR);
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET);
          }
        }
        break;

      case FROM_ICU_RIGHT_TS_ON_ADDR:
        if (right_ts_on) {
          FDCAN2_Driver_Indicator_TxBuffer[0] = 0;
          FDCAN2_Driver_Indicator_TxBuffer[1] = 0;
          FDCAN2_Driver_Indicator_TxBuffer[2] = 255;
          ICU_SendSignalLights(ICU_RX_BASE_ADDR+DRIVER_INDICATOR_ADDR);
          HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_RESET);
          HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET);
        } else {
          if (hazard_sig_on) {
            FDCAN2_Driver_Indicator_TxBuffer[0] = 255;
            FDCAN2_Driver_Indicator_TxBuffer[1] = 255;
            FDCAN2_Driver_Indicator_TxBuffer[2] = 255;
            ICU_SendSignalLights(ICU_RX_BASE_ADDR+DRIVER_INDICATOR_ADDR);
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_SET);
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET);
          } else {
            FDCAN2_Driver_Indicator_TxBuffer[0] = 0;
            FDCAN2_Driver_Indicator_TxBuffer[1] = 0;
            FDCAN2_Driver_Indicator_TxBuffer[2] = 0;
            ICU_SendSignalLights(ICU_RX_BASE_ADDR+DRIVER_INDICATOR_ADDR);
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET);
          }
        }
        break;

      case FROM_ICU_HAZARD_SIG_ON_ADDR:
        if (hazard_sig_on) {
          FDCAN2_Driver_Indicator_TxBuffer[0] = 255;
          FDCAN2_Driver_Indicator_TxBuffer[1] = 255;
          FDCAN2_Driver_Indicator_TxBuffer[2] = 255;
          ICU_SendSignalLights(ICU_RX_BASE_ADDR+DRIVER_INDICATOR_ADDR);
          HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_SET);
          HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET);
        } else {
          if (left_ts_on && !right_ts_on) {
            FDCAN2_Driver_Indicator_TxBuffer[0] = 255;
            FDCAN2_Driver_Indicator_TxBuffer[1] = 0;
            FDCAN2_Driver_Indicator_TxBuffer[2] = 0;
            ICU_SendSignalLights(ICU_RX_BASE_ADDR+DRIVER_INDICATOR_ADDR);
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_SET);
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET);
          } else if (!left_ts_on && right_ts_on) {
            FDCAN2_Driver_Indicator_TxBuffer[0] = 0;
            FDCAN2_Driver_Indicator_TxBuffer[1] = 0;
            FDCAN2_Driver_Indicator_TxBuffer[2] = 255;
            ICU_SendSignalLights(ICU_RX_BASE_ADDR+DRIVER_INDICATOR_ADDR);
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET);
          } else {
            FDCAN2_Driver_Indicator_TxBuffer[0] = 0;
            FDCAN2_Driver_Indicator_TxBuffer[1] = 0;
            FDCAN2_Driver_Indicator_TxBuffer[2] = 0;
            ICU_SendSignalLights(ICU_RX_BASE_ADDR+DRIVER_INDICATOR_ADDR);
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET);
          }
        }
        break;

      default:
        break;
    }
    turn_signal_interval_counter = 0;
  } else {
    if (hazard_sig_on) {
      HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_5);
      HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_6);
      FDCAN2_Driver_Indicator_TxBuffer[0] = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_5) ? 255 : 0;
      FDCAN2_Driver_Indicator_TxBuffer[1] = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_5) ? 255 : 0;
      FDCAN2_Driver_Indicator_TxBuffer[2] = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_5) ? 255 : 0;
      ICU_SendSignalLights(ICU_RX_BASE_ADDR+DRIVER_INDICATOR_ADDR);
    } else {
      if (left_ts_on) {
        HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_5);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET);
        FDCAN2_Driver_Indicator_TxBuffer[0] = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_5) ? 255 : 0;
        FDCAN2_Driver_Indicator_TxBuffer[1] = 0;
        FDCAN2_Driver_Indicator_TxBuffer[2] = 0;
        ICU_SendSignalLights(ICU_RX_BASE_ADDR+DRIVER_INDICATOR_ADDR);
        return;
      }
      if (right_ts_on) {
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_RESET);
        HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_6);
        FDCAN2_Driver_Indicator_TxBuffer[0] = 0;
        FDCAN2_Driver_Indicator_TxBuffer[1] = 0;
        FDCAN2_Driver_Indicator_TxBuffer[2] = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_6) ? 255 : 0;
        ICU_SendSignalLights(ICU_RX_BASE_ADDR+DRIVER_INDICATOR_ADDR);
        return;
      }
      HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_RESET);
      HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET);
      FDCAN2_Driver_Indicator_TxBuffer[0] = 0;
      FDCAN2_Driver_Indicator_TxBuffer[1] = 0;
      FDCAN2_Driver_Indicator_TxBuffer[2] = 0;
      ICU_SendSignalLights(ICU_RX_BASE_ADDR+DRIVER_INDICATOR_ADDR);
    }
  }
}

void ICU_HandlePacket(uint32_t offset) {
  switch (offset) {
    case FROM_ICU_LEFT_TS_ON_ADDR:
      VS_CAN_RX_ID = FROM_ICU_LEFT_TS_ON_ADDR;
      if (hazard_sig_on) { left_ts_on = true; hazard_sig_on = false; }
      else                { left_ts_on = !left_ts_on; }
      right_ts_on = false;
      sig_change = true;
      Signals_UpdateOutputs();
      break;

    case FROM_ICU_RIGHT_TS_ON_ADDR:
      VS_CAN_RX_ID = FROM_ICU_RIGHT_TS_ON_ADDR;
      if (hazard_sig_on) { right_ts_on = true; hazard_sig_on = false; }
      else               { right_ts_on = !right_ts_on; }
      left_ts_on = false;
      sig_change = true;
      Signals_UpdateOutputs();
      break;

    case FROM_ICU_HAZARD_SIG_ON_ADDR:
      if (VS_CAN_RX_ID != FROM_ICU_HAZARD_SIG_ON_ADDR && (left_ts_on || right_ts_on)) {
        hazard_sig_on = true;
      } else {
        hazard_sig_on = !hazard_sig_on;
      }
      VS_CAN_RX_ID = FROM_ICU_HAZARD_SIG_ON_ADDR;
      sig_change = true;
      Signals_UpdateOutputs();
      break;

    case FROM_ICU_RESET_WS22_ADDR:
      FDCAN1_TxHeader.Identifier = WS22_TX_BASE_ADDR + WS22_RESET_COMMAND_ADDR;
      memset(FDCAN1_TxBuffer, 0, sizeof(FDCAN1_TxBuffer));
      if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &FDCAN1_TxHeader, FDCAN1_TxBuffer) != HAL_OK) {
        VCU_error_flags.WS22_CAN_error = true;
      }
      break;

    case FROM_ICU_CRUISE_CONTROL_ADDR:
      for (int i = 0; i < 8; i++) { CC_temp_data[i] = FDCAN2_RxBuffer[i]; }
      CC_Check();
      CC_UpdateFromICU();
      break;

    case FROM_ICU_CRUISE_CONTROL_LEVEL_ADDR:
      Cruise_Control_Level = FDCAN2_RxBuffer[0];
      ICU_UpdateCruiseLevel(ICU_RX_BASE_ADDR+CRUISE_CONTROL_LEVEL_ADDR);
      break;

    case FROM_ICU_REGEN_THROTTLE_ADDR:
      Regen_throttle = FDCAN2_RxBuffer[0];
      break;

    default:
      break;
  }
}