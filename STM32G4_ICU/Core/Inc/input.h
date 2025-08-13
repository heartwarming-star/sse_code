#ifndef INPUT_H
#define INPUT_H

#include <icu_globals.h>

#ifdef __cplusplus
extern "C" {
#endif

void update_GPIO_Input(void);
void GPIO_call(int i);
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin);

void update_ROT();
void Convert_ROT_input_into_command();
                                                     

#ifdef __cplusplus
}
#endif


#endif
