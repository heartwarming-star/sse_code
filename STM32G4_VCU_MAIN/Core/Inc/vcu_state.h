#ifndef VCU_STATE_H
#define VCU_STATE_H

#include "parts.h"
#include <stdint.h>
#include <stdbool.h>
#include "main.h" /* for HAL types */

#ifdef __cplusplus
extern "C" {
#endif

/* Global state (extern) */
extern WS22_State ws22;
extern MPPT MPPT_array[3];
extern BMS_Data BMS;
extern const int NUM_MPPT;

/* Error flags */
typedef struct {
  bool throttle_ADC_error;
  bool WS22_CAN_error;
  bool WS22_motor_error;
  bool main_CAN_error;
  bool ICU_com_error;
  bool USART3_OVERFLOW;
  bool USART3_error;
} VCU_ErrorFlags;

extern VCU_ErrorFlags VCU_error_flags;

/* Drive & inputs */
extern volatile uint32_t throttle_input;
extern volatile float throttle_portion;
extern const float slew_rate;
extern const float slew_rate_minus;
extern const uint32_t throttle_dead_zone_threshold;
extern const uint32_t throttle_dynamic_range;
extern const float max_forward_motor_current;
extern const float max_reverse_motor_current;
extern volatile bool drive_direction; /* true: forward */
extern volatile bool drive_neutral;

extern volatile uint16_t Regen_throttle;
extern volatile float Regen_portion;
extern volatile uint8_t Cruise_Control_Level;
extern const uint32_t Regen_throttle_dead_zone_threshold;
extern const uint32_t Regen_throttle_dynamic_range;
extern const float max_regen_motor_current;

/* CC */
/* Explicit CC mode */
typedef enum {
  CC_DISABLED = 0,  /* Not allowed due to conditions */
  CC_ARMED,         /* Allowed, waiting for engage */
  CC_ACTIVE,        /* Actively controlling */
  CC_FAULT          /* Latched fault (until cleared) */
} CC_Mode;

extern volatile CC_Mode cc_mode;

enum Cruise_Control {OFF, ON};
extern volatile uint8_t  CC_temp_data[8];
extern volatile float    CC_threshold;
extern volatile bool     cruise_control_on;
extern volatile bool     cruise_control_able;
extern const float       rpm_to_kph;
extern volatile float    cruise_control_rpm_target;
extern volatile float    cruise_control_speed_target;
extern const float       cruise_control_current_limit_forward;
extern const float       cruise_control_current_limit_backward;
extern volatile bool     cruise_control_change;
extern const uint8_t     cruise_control_change_counter_Interval;
extern uint8_t           cruise_control_change_counter;
extern volatile float    cruise_control_before_target;
extern volatile uint16_t cruise_control_error;
extern volatile bool     brake_sw_input;



/* FDCAN buffers shared */

extern FDCAN_RxHeaderTypeDef FDCAN1_RxHeader;
extern FDCAN_TxHeaderTypeDef FDCAN1_TxHeader;
extern uint8_t FDCAN1_RxBuffer[8];
extern uint8_t FDCAN1_TxBuffer[8];
extern volatile uint8_t test_counter;

extern FDCAN_RxHeaderTypeDef FDCAN2_RxHeader;
extern FDCAN_TxHeaderTypeDef FDCAN2_TxHeader;
extern uint8_t FDCAN2_RxBuffer[8];
extern uint8_t FDCAN2_TxBuffer[8];
extern uint8_t FDCAN2_Driver_Indicator_TxBuffer[8];

/* USART3 telemetry buffer & mode */
#define USART3_TX_BUFFER_SIZE 1024
extern uint8_t  USART3_TxBuffer[USART3_TX_BUFFER_SIZE];
extern uint32_t USART3_TxBuffer_Load;
enum USART3_mode {TEXT, BYTE};
extern volatile int USART3_mode;

/* Timing counters/intervals */
extern const uint8_t USART3_Telemetry_Interval;         /* 500 ms (at 50ms tick) = 10; user had 9 => ~450ms */
extern const uint8_t USART3_Telemetry_Interval_lv_1;
extern const uint8_t USART3_Telemetry_Interval_lv_2;

extern const uint8_t turn_signal_interval;              /* 400ms */
extern const uint8_t cruise_control_signal_interval;    /* 500ms */
extern const uint8_t update_ICU_interval;               /* 500ms */

extern uint8_t USART3_Telemetry_Interval_Counter;
extern uint8_t USART3_Telemetry_Interval_Counter_lv_1;
extern uint8_t USART3_Telemetry_Interval_Counter_lv_2;

extern volatile uint8_t turn_signal_interval_counter;
extern volatile uint8_t cruise_control_signal_interval_counter;
extern volatile uint8_t update_ICU_interval_counter;
extern volatile uint8_t update_BMS_interval_counter;

/* Signals */
extern volatile bool left_ts_on;
extern volatile bool right_ts_on;
extern volatile bool hazard_sig_on;
extern volatile bool sig_change;
extern volatile uint32_t VS_CAN_RX_ID;

/* BYTE telemetry packet */
extern BYTE_Telemetry_Packet byte_packet;

#ifdef __cplusplus
}
#endif
#endif /* VCU_STATE_H */
