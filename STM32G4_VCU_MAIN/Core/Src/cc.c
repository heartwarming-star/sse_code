#include <string.h>
#include "cc.h"
#include "vcu_state.h"
#include "vcu_hal.h"
#include "protocol.h"
#include "main.h"
#include <math.h>

/* Local helpers */
static inline void CC_SetMode(CC_Mode m){
  cc_mode = m;
  /* Keep legacy boolean in sync for backward-compat */
  cruise_control_on = (cc_mode == CC_ACTIVE);
}

static inline void memcpy_from_volatile(void *dst, const volatile void *src, size_t n){
    uint8_t *d = (uint8_t*)dst;
    const volatile uint8_t *s = (const volatile uint8_t*)src;
    while (n--) *d++ = *s++;
}

void CC_SendToICU(uint32_t id, uint16_t data1, uint16_t data2, float data3) {
  uint8_t txData[8];
  FDCAN2_TxHeader.Identifier = id;
  memcpy(&txData[0], &data1, sizeof(uint16_t));
  memcpy(&txData[2], &data2, sizeof(uint16_t));
  memcpy(&txData[4], &data3, sizeof(float));
  if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan2, &FDCAN2_TxHeader, txData) != HAL_OK) {
    VCU_error_flags.ICU_com_error = true;
  }
}

void ICU_UpdateDriveMode(uint32_t id) {
  uint8_t txData[8] = {0};
  FDCAN2_TxHeader.Identifier = id;
  txData[0] = drive_direction;
  txData[1] = drive_neutral;
  if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan2, &FDCAN2_TxHeader, txData) != HAL_OK) {
    VCU_error_flags.ICU_com_error = true;
  }
}

void ICU_UpdateCruiseLevel(uint32_t id) {
  uint8_t txData[8] = {0};
  FDCAN2_TxHeader.Identifier = id;
  txData[0] = Cruise_Control_Level;
  if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan2, &FDCAN2_TxHeader, txData) != HAL_OK) {
    VCU_error_flags.ICU_com_error = true;
  }
}

void UpdateGearState(void) {
  bool d = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_14);
  bool r = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_15);
  if (d && r) {
    drive_neutral = true;
    CC_SetMode(CC_DISABLED);
    return;
  }
  if (!d && !r) {
    drive_neutral = true;
    CC_SetMode(CC_DISABLED);
    return;
  }
  drive_neutral  = false;
  drive_direction = d;
}

/* Evaluate conditions and set mode to ARMED or DISABLED; if ACTIVE and error arises -> drop to DISABLED */
uint16_t CC_Check(void) {
  bool break_pedal = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_1);
  uint16_t err = 0;
  if (break_pedal)                 err |=  CC_ERR_BRAKE;
  if (drive_neutral)               err |=  CC_ERR_NEUTRAL;
  if (!drive_direction)            err |=  CC_ERR_REVERSE;
  if (ws22.vehicle_velocity < 20)  err |=  CC_ERR_LOW_VELOCITY;
  if (ws22.vehicle_velocity > 140) err |=  CC_ERR_OVER_VELOCITY;;

  cruise_control_error = (cruise_control_error & ~0x7)    | err;
  cruise_control_error = (cruise_control_error & ~0x0030) | (err & 0x0030);

  if (err) {
    /* Any blocking error -> cannot cruise */
    if (cc_mode == CC_ACTIVE) {
      CC_SetMode(CC_DISABLED);
      CC_SendToICU(ICU_RX_BASE_ADDR+CRUISE_CONTROL_ADDR, cruise_control_on, cruise_control_error, cruise_control_speed_target);
    } else {
      CC_SetMode(CC_DISABLED);
    }
  } else {
    /* Eligible: if not active, become ARMED */
    if (cc_mode != CC_ACTIVE) {
      CC_SetMode(CC_ARMED);
    }
  }
  return err;
}

void CC_UpdateFromICU(void) {
  float cruise_control_speed_target_n;
  uint16_t err = 0;
  memcpy_from_volatile(&cruise_control_speed_target_n, &CC_temp_data[4], sizeof(float));

  /* OFF command */
  if (CC_temp_data[0] == OFF) {
    if (cc_mode == CC_ACTIVE) {
      CC_SetMode(CC_ARMED); /* Still eligible but not engaged */
      CC_SendToICU(ICU_RX_BASE_ADDR+CRUISE_CONTROL_ADDR, cruise_control_on, cruise_control_error, cruise_control_speed_target);
    } else {
      CC_SetMode((CC_Check()==0)?CC_ARMED:CC_DISABLED);
    }
    return;
  }

  /* ON/SET command */
  if (cc_mode == CC_DISABLED) {
    /* Ignore engage request; echo error */
    CC_SendToICU(ICU_RX_BASE_ADDR+CRUISE_CONTROL_ADDR, cruise_control_on, cruise_control_error, cruise_control_speed_target);
    return;
  }

  if (cc_mode != CC_ACTIVE) {
    /* Fresh engage at current speed */
    CC_SetMode(CC_ACTIVE);
    cruise_control_speed_target = ws22.vehicle_velocity;
    cruise_control_rpm_target   = cruise_control_speed_target * rpm_to_kph;
    cruise_control_change = true;
    cruise_control_change_counter = 0;
    CC_SendToICU(ICU_RX_BASE_ADDR+CRUISE_CONTROL_ADDR, cruise_control_on, cruise_control_error, cruise_control_speed_target);
  } else {
    /* Already active: update target with sanity checks */
    err |= CC_ERR_FROM_COMMAND;
    if (cruise_control_speed_target_n < 20)                         err |= CC_ERR_LOW_VELOCITY;
    if (cruise_control_speed_target_n > 130)                        err |= CC_ERR_OVER_VELOCITY;
    if (cruise_control_speed_target_n < ws22.vehicle_velocity-CC_threshold) err |= CC_ERR_DIFF_LOW;
    if (cruise_control_speed_target_n > ws22.vehicle_velocity+CC_threshold) err |= CC_ERR_DIFF_HIGH;

    if (err == CC_ERR_FROM_COMMAND) {
      cruise_control_speed_target = cruise_control_speed_target_n;
      cruise_control_rpm_target   = cruise_control_speed_target * rpm_to_kph;
    } else {
      cruise_control_error = (cruise_control_error & ~0x00F0) | (CC_Check() & 0x00F0);
    }
    CC_SendToICU(ICU_RX_BASE_ADDR+CRUISE_CONTROL_ADDR, cruise_control_on, cruise_control_error, cruise_control_speed_target);
  }
}