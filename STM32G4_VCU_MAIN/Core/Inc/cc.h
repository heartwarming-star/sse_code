#ifndef CC_H
#define CC_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

uint16_t CC_Check(void);
void     CC_UpdateFromICU(void);
void     CC_SendToICU(uint32_t id, uint16_t data1, uint16_t data2, float data3);
void     ICU_UpdateDriveMode(uint32_t id);
void     ICU_UpdateCruiseLevel(uint32_t id);

/* Helper to update gear state */
void     UpdateGearState(void);

#ifdef __cplusplus
}
#endif

#endif /* CC_H */
