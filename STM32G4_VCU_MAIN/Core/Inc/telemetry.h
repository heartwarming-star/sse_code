#ifndef TELEMETRY_H
#define TELEMETRY_H

#include <stdint.h>
#include "parts.h"

#ifdef __cplusplus
extern "C" {
#endif

void Telemetry_UpdateBytePacket(void);
void Telemetry_SendFull(BYTE_Telemetry_Packet* packet, uint16_t size);
void Telemetry_SendUSART3(void);
void Telemetry_InsertFloat(uint32_t size_1, float data_1, uint32_t size_2, char * data_2);
void Telemetry_InsertInt(uint32_t size_1, uint16_t data_1, uint32_t size_2, char * data_2);

#ifdef __cplusplus
}
#endif
#endif /* TELEMETRY_H */