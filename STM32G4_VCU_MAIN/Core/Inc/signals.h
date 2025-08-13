#ifndef SIGNALS_H
#define SIGNALS_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

void Signals_UpdateOutputs(void);
void ICU_HandlePacket(uint32_t offset);
void ICU_SendSignalLights(uint32_t id);

#ifdef __cplusplus
}
#endif
#endif /* SIGNALS_H */