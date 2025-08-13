#ifndef ICU_GLOBALS_H
#define ICU_GLOBALS_H

#include <stdint.h>
#include <stdbool.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
#include <main.h>


#ifdef __cplusplus
extern "C" {
#endif

/*---------------- HAL / LL 핸들 ----------------*/
extern ADC_HandleTypeDef   hadc1, hadc2, hadc3;
extern DMA_HandleTypeDef   hdma_adc1, hdma_adc2, hdma_adc3;
extern FDCAN_HandleTypeDef hfdcan1;
extern SPI_HandleTypeDef   hspi3;
extern TIM_HandleTypeDef   htim3, htim6, htim7;

/*------------------ 애플리케이션 전역 ----------------*/
extern const uint16_t TS_ADC_THRESHOLD;

extern uint16_t g_AdcDmaBuffer_1[6];   /* ROT1-5, HALL1 */
extern uint16_t g_AdcDmaBuffer_2[2];   /* BLINK L, R   */
extern uint16_t g_AdcDmaBuffer_3[1];   /* HALL2        */

/* Turn-signal 입력/상태 */
extern volatile uint16_t left_turn_sig_adc_input;
extern volatile uint16_t right_turn_sig_adc_input;
extern volatile bool     hazard_sig_digital_input;
extern const  uint16_t   light_sig_output;

extern volatile bool left_ts_input_was_on;
extern volatile bool right_ts_input_was_on;
extern volatile bool hazard_sig_input_was_on;
extern volatile bool display_sig_input_was_on;
extern volatile bool left_ts_light_on;
extern volatile bool right_ts_light_on;
extern volatile bool hazard_sig_light_on;
extern volatile int  vsle_counter;
extern volatile uint32_t can_fill_level;

/* ROTARY / HALL 센서 */
extern const uint16_t ROT_threshold[12];
extern const int      ROT_diff_threshold;
extern volatile uint16_t ROT_ADC_input[5];
extern volatile uint16_t ROT[5];
extern volatile uint16_t HALL_ADC_input[2];
extern volatile bool     HALL[2];
extern volatile bool     signal_log[1024];

/* Rotary-encoder 구조체 & 상태 */
enum Cruise_Control {OFF, ON};

typedef struct {
    volatile bool button_state;
    volatile bool init;
    volatile bool quadratic_phase;
} Rotary_ENC_t;

extern Rotary_ENC_t ENC_R;
extern Rotary_ENC_t ENC_L;

/* 운전 / 표시 모드 enum & 변수 */
typedef enum { DRIVE, NEUTRAL, REVERSE } driveMode;
typedef enum { BOOT, DRIVING, MOTOR, ENERGY, ERR } dispMode;

extern volatile uint16_t drive_mode;
extern volatile bool     CC;
extern volatile uint8_t  disp_mode;
extern volatile bool     display_mode_update_flag;
extern volatile bool     display_update_flag;

/* OLED 버퍼 */
extern uint8_t OLED_buf[256*64/2];
extern uint8_t OLED_buf_1[256*64/2];
extern uint8_t OLED_buf_2[256*64/2];
extern volatile float    buf_Layout_2[20];
extern volatile bool     buf_GPIO_Input[5];

extern volatile uint16_t oled_refresh_counter;
extern const  uint16_t   oled_refresh_interval;
extern volatile uint16_t Regen_throttle_counter;
extern const  uint16_t   Regen_throttle_interval;

/* CAN Tx/Rx 헤더·버퍼 */
extern FDCAN_TxHeaderTypeDef FDCAN1_TxHeader;
extern FDCAN_RxHeaderTypeDef FDCAN1_RxHeader;
extern uint8_t FDCAN1_TxBuffer[8];
extern uint8_t FDCAN1_RxBuffer[8];
extern volatile uint32_t FDCAN1_error_sig_count;


#ifdef __cplusplus
}
#endif


#endif
