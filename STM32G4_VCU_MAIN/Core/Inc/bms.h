#ifndef BMS_H
#define BMS_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

void BMS_ParsePacket(uint32_t offset);

#ifdef __cplusplus
}
#endif
#endif /* BMS_H */