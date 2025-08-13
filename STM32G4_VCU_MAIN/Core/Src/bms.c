#include "bms.h"
#include "vcu_state.h"
#include "protocol.h"

void BMS_ParsePacket(uint32_t offset){
  switch (offset) {
    case BMS_DATA_0_ADDR:
      BMS.battery_voltage = (FDCAN2_RxBuffer[0] << 8) | FDCAN2_RxBuffer[1];
      BMS.battery_current = ((FDCAN2_RxBuffer[2] << 8) | FDCAN2_RxBuffer[3]);
      BMS.battery_current -= 32767;
      BMS.SOC = FDCAN2_RxBuffer[4];
      BMS.bms_status.raw = FDCAN2_RxBuffer[5];
      BMS.NTC0 = FDCAN2_RxBuffer[6] - 55;
      BMS.NTC1 = FDCAN2_RxBuffer[7] - 55;
      break;
    case BMS_DATA_1_ADDR:
      BMS.NTC2 = FDCAN2_RxBuffer[0] - 55;
      BMS.NTC3 = FDCAN2_RxBuffer[1] - 55;
      BMS.NTC4 = FDCAN2_RxBuffer[2] - 55;
      BMS.NTC5 = FDCAN2_RxBuffer[3] - 55;
      BMS.module_segment_1_max_cell_voltage = (FDCAN2_RxBuffer[4] << 8) | FDCAN2_RxBuffer[5];
      BMS.module_segment_2_max_cell_voltage = (FDCAN2_RxBuffer[6] << 8) | FDCAN2_RxBuffer[7];
      break;
    case BMS_DATA_2_ADDR:
      BMS.module_segment_3_max_cell_voltage = (FDCAN2_RxBuffer[0] << 8) | FDCAN2_RxBuffer[1];
      BMS.module_segment_1_min_cell_voltage = (FDCAN2_RxBuffer[2] << 8) | FDCAN2_RxBuffer[3];
      BMS.module_segment_2_min_cell_voltage = (FDCAN2_RxBuffer[4] << 8) | FDCAN2_RxBuffer[5];
      BMS.module_segment_3_min_cell_voltage = (FDCAN2_RxBuffer[6] << 8) | FDCAN2_RxBuffer[7];
      break;
    default:
      break;
  }
}