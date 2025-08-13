#include "../SSD1322_OLED_lib/SSD1322_HW_Driver.h"
#include "../SSD1322_OLED_lib/SSD1322_API.h"
#include "../SSD1322_OLED_lib/SSD1322_GFX.h"

#include "stdbool.h"
#include "stm32g4xx_hal.h"

#define OLED_Error 0
#define OLED_L_TS 1
#define OLED_R_TS 2
#define OLED_H_TS 3
#define OLED_Cruise_Control 4
#define OLED_Drive_Mode 5
#define OLED_Regen_Lv 6
#define OLED_Motor_T 7
#define OLED_D_T 8
#define OLED_Voltage 9
#define OLED_Current 10
#define OLED_SOC 11
#define OLED_Velocity 12
#define OLED_Target_Speed 13
#define OLED_Brightness 14
#define OLED_CC_Error 15
#define OLED_VCU_Error 16



void SSD1322_test(uint8_t *tx_buf);
void SSD1322_buf_data(uint8_t *tx_buf, float data);
void SSD1322_layout_test(uint8_t *tx_buf, uint32_t ENC_1, uint32_t ENC_2, bool bt1, bool bt2, bool sl_l, bool sl_r);
void SSD1322(uint8_t*tx_buf, bool cc);
void SSD1322_SSE(uint8_t *tx_buf);
void SSD1322_Layout(uint8_t *tx_buf, float *data_buf);
void SSD1322_Layout_init(uint8_t *tx_buf);
void SSD1322_MPPT_energy(uint8_t *tx_buf,
                         float *mppt_display[3][4]); 