#include "vcu_state.h"

/* --- Instances --- */
WS22_State ws22 = {0};
MPPT MPPT_array[3] = {0};
BMS_Data BMS = {0};
VCU_ErrorFlags VCU_error_flags = {0};

/* --- Drive & inputs --- */
volatile uint32_t throttle_input = 0;
volatile float throttle_portion = 0.f;
volatile float last_current_command = 0.f;
const float slew_rate = 0.1f;
const float slew_rate_minus = 0.2f;
const uint32_t throttle_dead_zone_threshold = 1030;
const uint32_t throttle_dynamic_range = 1997;
const float max_forward_motor_current = 100.f;
const float max_reverse_motor_current = 40.f;
volatile bool drive_direction = true;
volatile bool drive_neutral = true;

volatile uint16_t Regen_throttle = 0;
volatile float Regen_portion = 0.f;
volatile uint8_t Cruise_Control_Level = 0;
const uint32_t Regen_throttle_dead_zone_threshold = 2742;
const uint32_t Regen_throttle_dynamic_range = (3920-2742);
const float max_regen_motor_current = 100.f;

/* --- CC --- */
volatile uint8_t  CC_temp_data[8] = {0};
volatile float    CC_threshold = 20;
volatile bool     cruise_control_on = false;
volatile bool     cruise_control_able = false;
const float       rpm_to_kph = 9.5074f;
volatile float    cruise_control_rpm_target = 0;
volatile float    cruise_control_speed_target = 0;
const float       cruise_control_current_limit_forward = 0.2f;
const float       cruise_control_current_limit_backward = 0.f;
volatile bool     cruise_control_change = false;
const uint8_t     cruise_control_change_counter_Interval = 99;
uint8_t           cruise_control_change_counter = 0;
volatile float    cruise_control_before_target = 0;
volatile uint16_t cruise_control_error = 0;
volatile bool     brake_sw_input = false;

/* CC explicit mode */
volatile CC_Mode cc_mode = CC_DISABLED;

/* --- FDCAN shared --- */
FDCAN_RxHeaderTypeDef FDCAN1_RxHeader = {0};
FDCAN_TxHeaderTypeDef FDCAN1_TxHeader = {0};
uint8_t FDCAN1_RxBuffer[8] = {0};
uint8_t FDCAN1_TxBuffer[8] = {0};
volatile uint8_t test_counter = 0;

FDCAN_RxHeaderTypeDef FDCAN2_RxHeader = {0};
FDCAN_TxHeaderTypeDef FDCAN2_TxHeader = {0};
uint8_t FDCAN2_RxBuffer[8] = {0};
uint8_t FDCAN2_TxBuffer[8] = {0};
uint8_t FDCAN2_Driver_Indicator_TxBuffer[8] = {0};

/* --- USART3 telemetry --- */
uint8_t  USART3_TxBuffer[USART3_TX_BUFFER_SIZE] = {0};
uint32_t USART3_TxBuffer_Load = 0;
volatile int USART3_mode = TEXT;

const uint8_t USART3_Telemetry_Interval = 9;      /* ~450-500ms at 50ms tick */
const uint8_t USART3_Telemetry_Interval_lv_1 = 10;
const uint8_t USART3_Telemetry_Interval_lv_2 = 12;

const uint8_t turn_signal_interval = 10;          /* ~400ms */
const uint8_t cruise_control_signal_interval = 9; /* ~450ms */
const uint8_t update_ICU_interval = 9;            /* ~450ms */

uint8_t USART3_Telemetry_Interval_Counter = 0;
uint8_t USART3_Telemetry_Interval_Counter_lv_1 = 0;
uint8_t USART3_Telemetry_Interval_Counter_lv_2 = 0;

volatile uint8_t turn_signal_interval_counter = 0;
volatile uint8_t cruise_control_signal_interval_counter = 0;
volatile uint8_t update_ICU_interval_counter = 0;
volatile uint8_t update_BMS_interval_counter = 0;

/* --- Signals --- */
volatile bool left_ts_on = false;
volatile bool right_ts_on = false;
volatile bool hazard_sig_on = false;
volatile bool sig_change = false;
volatile uint32_t VS_CAN_RX_ID = 0;

/* --- BYTE telemetry packet --- */
BYTE_Telemetry_Packet byte_packet = {0};