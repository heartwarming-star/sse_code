#include "input.h"
#include "protocol.h"
#include "main.h"
#include "icu_globals.h"
#include "comm.h"
#include "lights.h"
#include "parts.h"


void update_GPIO_Input(){
  volatile bool buf_GPIO_Input_old[5];
  for(int i = 0; i < 5; i++){
    buf_GPIO_Input_old[i] = buf_GPIO_Input[i];
  }
  buf_GPIO_Input[0] = HAL_GPIO_ReadPin(ENC2_BT_Port, ENC2_BT_Pin);
  buf_GPIO_Input[1] = HAL_GPIO_ReadPin(SW7_Port, SW7_Pin);
  buf_GPIO_Input[2] = HAL_GPIO_ReadPin(SW1_Port, SW1_Pin);
  buf_GPIO_Input[3] = HAL_GPIO_ReadPin(SW4_Port, SW4_Pin);
  buf_GPIO_Input[4] = HAL_GPIO_ReadPin(SW6_Port, SW6_Pin);
  for(int i = 0; i < 5; i++){
    if(buf_GPIO_Input[i]!=buf_GPIO_Input_old[i]){
      GPIO_call(i);
    }
  }
}

void GPIO_call(int i){
  if(i ==4){
    if(disp_mode != ERR){
      disp_mode = (disp_mode+3)%4;
      display_mode_update_flag = true;
    }
  }
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
  // This function is called when the GPIO interrupt is triggered
  ENC_L.quadratic_phase = HAL_GPIO_ReadPin(ENC1_2_Port, ENC1_2_Pin);
  ENC_R.quadratic_phase = HAL_GPIO_ReadPin(ENC2_2_Port, ENC2_2_Pin);

  if (GPIO_Pin == SW3_Pin) {
    // Hazard signal input
    updateHazardSigFlag();
    }
  else if (GPIO_Pin == ENC1_1_Pin){
    if(ICU.Target_speed <= 0 ){
      if(ENC_L.quadratic_phase){
        Send_VCU_CAN_ICU_Command_CC(CRUISE_CONTROL_ADDR, ICU.CC, ICU.Target_speed+0.1);
      }
    }
    else{
      if(ENC_L.quadratic_phase){
        Send_VCU_CAN_ICU_Command_CC(CRUISE_CONTROL_ADDR, ICU.CC, ICU.Target_speed+0.1);
      }
      else{
        Send_VCU_CAN_ICU_Command_CC(CRUISE_CONTROL_ADDR, ICU.CC, ICU.Target_speed-0.1);
      }
    }
    
  }
  else if (GPIO_Pin == ENC2_1_Pin){
    if(ICU.Target_speed <= 0){
      if(!ENC_R.quadratic_phase)
        Send_VCU_CAN_ICU_Command_CC(CRUISE_CONTROL_ADDR, ICU.CC, ICU.Target_speed+1);
    }
    else{
      if(!ENC_R.quadratic_phase)
        Send_VCU_CAN_ICU_Command_CC(CRUISE_CONTROL_ADDR, ICU.CC, ICU.Target_speed+1);
      else
        Send_VCU_CAN_ICU_Command_CC(CRUISE_CONTROL_ADDR, ICU.CC, ICU.Target_speed-1);
    }
    
  }
  else if (GPIO_Pin == ENC1_BT_Pin){
    ENC_L.button_state = !ENC_L.button_state;//cruise on or off
    if(ICU.CC == ON){
      Send_VCU_CAN_ICU_Command_CC(CRUISE_CONTROL_ADDR, OFF, ICU.Target_speed);
    }
    else{
      ICU.Target_speed = ws22.vehicle_velocity;
      Send_VCU_CAN_ICU_Command_CC(CRUISE_CONTROL_ADDR, ON, ICU.Target_speed);
    }
      

  }
  else if(GPIO_Pin == SW8_Pin){
    //RIGHT DISPLAY
    if(disp_mode != ERR){
      disp_mode = (disp_mode +1)%4;
      display_mode_update_flag = true;
    }
  }
  else if(GPIO_Pin == SW2_Pin){ // 
    disp_mode = 0;
    display_mode_update_flag = false;
    //oled_refresh_counter = 0;
    
  }
  else if(GPIO_Pin == SW5_Pin){ // 

  }
  else if(GPIO_Pin == SW9_Pin){//WS22 초기화
    Send_VCU_CAN_ICU_Command(WS22_RESET_ADDR, 0, 0);
  }
}

void update_ROT(){
  uint16_t rot_0_temp = ICU.ROT[0];
  Convert_ROT_input_into_command(0);
  Convert_ROT_input_into_command(1);
  Convert_ROT_input_into_command(2);
  Convert_ROT_input_into_command(3);
  Convert_ROT_input_into_command(4);
  if(rot_0_temp != ICU.ROT[0]){
    if(ICU.ROT[0] > 9){
      Send_VCU_CAN_ICU_Command(REGEN_LEVEL_ADDR, 9, 0); 
    }
    else
      Send_VCU_CAN_ICU_Command(REGEN_LEVEL_ADDR, ICU.ROT[0], 0);
  }
  return;
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc) {
  // This function is called when the ADC conversion is complete
  // 100Hz polling rate tied to TIM6
  if(hadc->Instance == ADC1) {
    for(int i = 0; i < 5; i++){
      ROT_ADC_input[i] = g_AdcDmaBuffer_1[i];
    }
    HALL_ADC_input[0] = g_AdcDmaBuffer_1[5];
    update_ROT();
    
  }
  else if(hadc->Instance == ADC2) {
    left_turn_sig_adc_input  = g_AdcDmaBuffer_2[0];
    right_turn_sig_adc_input = g_AdcDmaBuffer_2[1];
    updateTSFlags();
  }
  else if(hadc->Instance == ADC3) {
    HALL_ADC_input[1] = g_AdcDmaBuffer_3[0];
    //update_HALL();
  }
}

void Convert_ROT_input_into_command(int index){
  if(ROT_ADC_input[index] < 103 || ROT_ADC_input[index] > 4200){
    return;
  }

  if(index == 0||index == 1) {
    
    int ROT_diff = ROT_ADC_input[index] - ROT_threshold[(ICU.ROT[index]+6)%12];
    if((0 < ROT_diff && ROT_diff < ROT_diff_threshold)||(ROT_diff < 0 && ROT_diff > -ROT_diff_threshold)){
      return;
    }
    else if(ROT_diff < -ROT_diff_threshold){
      ICU.ROT[index] = (ICU.ROT[index]+11)%12;
      Convert_ROT_input_into_command(index);
      return;
    }
    else if(ROT_diff > ROT_diff_threshold){
      ICU.ROT[index] = (ICU.ROT[index]+1)%12;
      Convert_ROT_input_into_command(index);
      return;
    }
  }
  else if(index == 2||index == 3||index == 4) {
    int ROT_diff = ROT_ADC_input[index] - ROT_threshold[ICU.ROT[index]];
    if((0 <= ROT_diff && ROT_diff < ROT_diff_threshold)||(ROT_diff <= 0 && ROT_diff > -ROT_diff_threshold)){
      return;
    }
    else if(ROT_diff < -ROT_diff_threshold){
      ICU.ROT[index] = (ICU.ROT[index]+11)%12;
      Convert_ROT_input_into_command(index);
      return;
    }
    else if(ROT_diff > ROT_diff_threshold){
      ICU.ROT[index] = (ICU.ROT[index]+1)%12;
      Convert_ROT_input_into_command(index);
      return;
    }
  }
  
  return;
}