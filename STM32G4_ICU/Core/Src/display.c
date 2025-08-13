#include "display.h"
#include "protocol.h"
#include "display.h"
#include "SSD1322_OLED_lib/SSD1322.h"
#include "icu_globals.h"
#include "parts.h"


void updateDisplay(){
  display_update_flag = true;
  switch (disp_mode)
  {
  case BOOT:
    SSD1322_SSE(OLED_buf);
    
    break;
  case DRIVING:
    update_data();
    SSD1322_Layout(OLED_buf_2, buf_Layout_2);
    break;

  case MOTOR: // ERR
    // uint8_t *ws22_status[8];

    // uint8_t* MPPT_status[3][8];
    // memcpy(&MPPT_status[0][0], &MPPT_array[0].Status, sizeof(uint64_t));
    // memcpy(&MPPT_status[1][0], &MPPT_array[1].Status, sizeof(uint64_t));
    // memcpy(&MPPT_status[2][0], &MPPT_array[2].Status, sizeof(uint64_t));
    
    
    // BMS_Status BMS_status = BMS.bms_status;
    // uint16_t *CC_status = ICU.CC_error_code;

    //SSD1322_ERROR(OLED_buf_1, ws22.error_flags);
    break;  
  case ENERGY:
    //SSD1322_buf_data(OLED_buf, Target_speed);
    float *mppt_energy_disp[3][4];
    for(int i = 0; i < 3; i ++){
      mppt_energy_disp[i][0] = &MPPT_array[i].input_voltage;
      mppt_energy_disp[i][1] = &MPPT_array[i].input_current;
      mppt_energy_disp[i][2] = &MPPT_array[i].output_voltage;
      mppt_energy_disp[i][3] = &MPPT_array[i].output_current;
    }
    SSD1322_MPPT_energy(OLED_buf_1, mppt_energy_disp);
    
    break;
  case ERR:
    //SSD1322_Layout(OLED_buf, );2
    break;
  default:
    break;
  }
  display_update_flag = false;
  return;
}

void update_data(){
  buf_Layout_2[OLED_Error] = 0;
  buf_Layout_2[OLED_L_TS] = left_ts_light_on;
  buf_Layout_2[OLED_R_TS] = right_ts_light_on;
  buf_Layout_2[OLED_H_TS] = hazard_sig_light_on;
  buf_Layout_2[OLED_Cruise_Control] = ICU.CC; // VCU 오는 것 받아야함.
  buf_Layout_2[OLED_Drive_Mode] = ICU.drive_mode;
  buf_Layout_2[OLED_Regen_Lv] = ICU.Regen_Lv;
  buf_Layout_2[OLED_Motor_T] = ws22.motor_temp;
  buf_Layout_2[OLED_D_T] = ws22.DSP_board_temp;
  buf_Layout_2[OLED_Voltage] = ws22.bus_voltage;
  buf_Layout_2[OLED_Current] = ws22.bus_current;
  buf_Layout_2[OLED_SOC] = ICU.SOC; //BMS data 
  buf_Layout_2[OLED_Velocity] = ws22.motor_rpm/9.5074f;
  buf_Layout_2[OLED_Target_Speed] = ICU.Target_speed;
  
  buf_Layout_2[OLED_Brightness] = 5;
  buf_Layout_2[OLED_CC_Error] = ICU.CC_error_code;

}

