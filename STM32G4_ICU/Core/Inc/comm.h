#ifndef COMMS
#define COMMS

#include <icu_globals.h>

#ifdef __cplusplus
extern "C" {
#endif

void Send_VCU_CAN_ICU_Command(uint32_t id, uint32_t data1, uint32_t data2);
void Send_VCU_CAN_ICU_Command_CC(uint32_t id, uint32_t data1, float data2);
void Send_VCU_CAN_ICU_Command_Regen_Throttle(uint32_t id, uint16_t data1);

void Parse_BMS_CAN_Packet();
void Parse_MPPT_CAN_Packet(uint32_t offset);
void Handle_VCU_CAN_Packet(uint32_t offset);

void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs);


#ifdef __cplusplus
}
#endif

#endif
