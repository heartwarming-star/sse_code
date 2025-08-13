#ifndef SENSORS
#define SENSORS
#include <icu_globals.h>

#ifdef __cplusplus
extern "C" {
#endif

void updateTSFlags();
void updateHazardSigFlag();
void updateVSLEDOutputs();
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc);


#ifdef __cplusplus
}
#endif

#endif
