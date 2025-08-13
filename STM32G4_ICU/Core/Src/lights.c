#include <icu_globals.h>
#include "protocol.h"
#include "comm.h"
#include "input.h"

void updateTSFlags() {
  if (left_turn_sig_adc_input>TS_ADC_THRESHOLD && right_turn_sig_adc_input>TS_ADC_THRESHOLD) {
    return;
  }
  if (left_turn_sig_adc_input>TS_ADC_THRESHOLD) {
    if (!left_ts_input_was_on) {
      Send_VCU_CAN_ICU_Command(LEFT_TS_ADDR, light_sig_output, 0);
    }
    left_ts_input_was_on = true;
  }
  else {
    left_ts_input_was_on = false;
  }
  
  if (right_turn_sig_adc_input>TS_ADC_THRESHOLD) {
    if (!right_ts_input_was_on) {
      Send_VCU_CAN_ICU_Command(RIGHT_TS_ADDR, light_sig_output, 0);
    }
    right_ts_input_was_on = true;
  }
  else {
    right_ts_input_was_on = false;
  }
}


void updateVSLEDOutputs() {//VCU에서 온 제어. sincronize 필요.
  // buf_Layout_2[OLED_L_TS] = FDCAN1_RxBuffer[0];
  // buf_Layout_2[OLED_R_TS] = FDCAN1_RxBuffer[2];
  // buf_Layout_2[OLED_H_TS] = FDCAN1_RxBuffer[1];
  left_ts_light_on = FDCAN1_RxBuffer[0];
  right_ts_light_on = FDCAN1_RxBuffer[2];
  hazard_sig_light_on = FDCAN1_RxBuffer[1];
}


void updateHazardSigFlag() {
  Send_VCU_CAN_ICU_Command(HAZARD_SIG_ADDR, light_sig_output,0);
}




