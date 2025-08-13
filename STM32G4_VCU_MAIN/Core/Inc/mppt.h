#ifndef MPPT_H
#define MPPT_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

void MPPT_ParsePacket(uint32_t offset);

#ifdef __cplusplus
}
#endif
#endif /* MPPT_H */