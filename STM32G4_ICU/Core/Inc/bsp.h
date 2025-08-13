// bsp.h — Board‑Support Package public interface

#ifndef BSP_H
#define BSP_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

void BSP_Init(void);
void Error_Handler(void);

#ifdef __cplusplus
}
#endif

#endif