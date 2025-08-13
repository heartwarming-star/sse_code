#include <string.h>
#include "mppt.h"
#include "vcu_state.h"
#include "protocol.h"

void MPPT_ParsePacket(uint32_t offset) {
  uint8_t id;
  if (offset < 0x10) {
    id = 0;
  } else if (offset < 0x20) {
    id = 1; offset -= 0x10;
  } else if (offset < 0x30) {
    id = 2; offset -= 0x20;
  } else return;

  MPPT* mppt = &MPPT_array[id];

  switch (offset) {
    case MPPT_INPUT_MEASUERE_ADDR:
      memcpy(&mppt->input_current, &FDCAN2_RxBuffer[4], sizeof(float));
      memcpy(&mppt->input_voltage, &FDCAN2_RxBuffer[0], sizeof(float)); 
      break;
    case MPPT_OUTPUT_MEASURE_ADDR:
      memcpy(&mppt->output_current, &FDCAN2_RxBuffer[4], sizeof(float));
      memcpy(&mppt->output_voltage, &FDCAN2_RxBuffer[0], sizeof(float));
      break;
    case MPPT_TEMPERATURE_ADDR:
      memcpy(&mppt->controller_temp, &FDCAN2_RxBuffer[4], sizeof(float));
      memcpy(&mppt->mosfet_temp, &FDCAN2_RxBuffer[0], sizeof(float));
      break;
    case MPPT_AUXILIARY_POWER_SUPPLY_ADDR:
      memcpy(&mppt->volt_3v, &FDCAN2_RxBuffer[4], sizeof(float));
      memcpy(&mppt->volt_12v, &FDCAN2_RxBuffer[0], sizeof(float));
      break;
    case MPPT_LIMITS_ADDR:
      memcpy(&mppt->max_input_current, &FDCAN2_RxBuffer[4], sizeof(float));
      memcpy(&mppt->max_output_voltage, &FDCAN2_RxBuffer[0], sizeof(float));
      break;
    case MPPT_STATUS_ADDR:
      mppt->Status.can_rx_error   = FDCAN2_RxBuffer[0];
      mppt->Status.can_tx_error   = FDCAN2_RxBuffer[1];
      mppt->Status.can_tx_overflow= FDCAN2_RxBuffer[2];
      mppt->Status.error_flags.raw= FDCAN2_RxBuffer[3];
      mppt->Status.limit_flags.raw= FDCAN2_RxBuffer[4];
      mppt->Status.mode           = FDCAN2_RxBuffer[5];
      mppt->Status.reserved       = FDCAN2_RxBuffer[6];
      mppt->Status.test_counter   = FDCAN2_RxBuffer[7];
      break;
    case MPPT_POWER_CONNECTOR_ADDR:
      memcpy(&mppt->power_connector_temp, &FDCAN2_RxBuffer[4], sizeof(float));
      memcpy(&mppt->output_voltage_power_connector, &FDCAN2_RxBuffer[0], sizeof(float));
      break;
    default:
      break;
  }
}