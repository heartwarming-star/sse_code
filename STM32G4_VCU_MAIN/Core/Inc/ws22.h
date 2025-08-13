#ifndef WS22_H
#define WS22_H

#include <stdint.h>
#include "protocol.h"

#ifdef __cplusplus
extern "C" {
#endif

void WS22_HandlePacket(uint32_t id);
void WS22_SendMotorCommand(void);
void WS22_ForwardToICU(uint32_t id);
                   
#ifdef __cplusplus
}
#endif
#endif /* WS22_H */
