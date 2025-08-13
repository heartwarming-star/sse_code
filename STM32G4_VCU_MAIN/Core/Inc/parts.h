#ifndef VCU_TYPES_H
#define VCU_TYPES_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

struct ws22_error_flags {
  bool motor_over_speed;
  bool desaturation_fault;
  bool rail_15V_UVLO;
  bool config_read_error;
  bool watchdog_caused_last_reset;
  bool bad_motor_posion_hall_seq;
  bool DC_bus_over_voltage;
  bool SW_over_current;
  bool HW_over_current;
};

struct ws22_limit_flags {
  bool motor_temp;
  bool bus_voltage_lower_limit;
  bool bus_voltage_upper_limit;
  bool bus_current;
  bool velocity;
  bool motor_current;
  bool output_voltage_PWM;
};

typedef union {
  uint8_t raw;
  struct {
    bool low_array_power    : 1;
    bool mosfet_overheat    : 1;
    bool battery_low        : 1;
    bool battery_full       : 1;
    bool undervoltage_12v   : 1;
    bool reserved           : 1;
    bool HW_overcurrent     : 1;
    bool HW_overvoltage     : 1;
  };
} MPPT_error_flags;

typedef union {
  uint8_t raw;
  struct {
    bool input_current_min     : 1;
    bool input_current_max     : 1;
    bool output_voltage_max    : 1;
    bool mosfet_temp_limit     : 1;
    bool duty_cycle_min        : 1;
    bool duty_cycle_max        : 1;
    bool local_mppt            : 1;
    bool global_mppt           : 1;
  };
} MPPT_limit_flags;

struct MPPT_Status{
  uint8_t can_rx_error;     
  uint8_t can_tx_error;     
  uint8_t can_tx_overflow;  
  MPPT_error_flags error_flags;      
  MPPT_limit_flags limit_flags;      
  uint8_t mode;             
  uint8_t reserved;         
  uint8_t test_counter;     
};

typedef union {
  uint8_t raw;
  struct {
    bool Cell_Imbalance_Protection  : 1;
    bool SCD                        : 1;
    bool UTC                        : 1;
    bool OTD                        : 1;
    bool OCD                        : 1;
    bool OCC                        : 1;
    bool UC                         : 1;
    bool OV                         : 1;
  };
} BMS_Status;

typedef struct {
  float input_voltage;
  float input_current;

  float output_voltage;
  float output_current;

  float mosfet_temp;
  float controller_temp;

  float volt_12v;
  float volt_3v;
  
  float max_output_voltage;
  float max_input_current;

  struct MPPT_Status Status;

  float output_voltage_power_connector;
  float power_connector_temp;

  uint32_t last_updated_ms;
} MPPT;

typedef struct {
  uint32_t serial_number;
  uint32_t PRH_id;

  uint8_t rx_err_count;
  uint8_t tx_err_count;
  uint16_t active_motor;
  uint16_t error_flags_vector;
  uint16_t limit_flags_vector;

  struct ws22_error_flags error_flags;
  struct ws22_limit_flags limit_flags;

  float bus_current;
  float bus_voltage;

  float vehicle_velocity;
  float motor_rpm;

  float phase_C_current;
  float phase_B_current;

  float Vd;
  float Vq;

  float Id;
  float Iq;

  float BEMFd;
  float BEMFq;

  float rail_15V;

  float rail_3_3V;
  float rail_1_9V;

  float heatsink_temp;
  float motor_temp;

  float DSP_board_temp;
  
  float DC_BUS_AH;
  float odometer;

  float slip_speed;
} WS22_State;

typedef struct {
  uint16_t battery_voltage;
  int battery_current;
  uint8_t SOC;
  BMS_Status bms_status;
  uint8_t NTC0;
  uint8_t NTC1;
  uint8_t NTC2;
  uint8_t NTC3;
  uint8_t NTC4;
  uint8_t NTC5;
  uint16_t module_segment_1_max_cell_voltage;
  uint16_t module_segment_2_max_cell_voltage;
  uint16_t module_segment_3_max_cell_voltage;
  uint16_t module_segment_1_min_cell_voltage;
  uint16_t module_segment_2_min_cell_voltage;
  uint16_t module_segment_3_min_cell_voltage;
} BMS_Data;

#pragma pack(push,1)
typedef struct {
    uint8_t   stx;
    uint8_t   id;
    uint32_t  t_ms;
    float     veh_speed;
    float     bus_voltage;
    float     bus_current;
    uint16_t  soc_d10;
    uint8_t   gear;
    uint8_t   regen_lvl;
    uint16_t  cc_error;
    uint16_t  vcu_err_flags;
    uint16_t  crc;
} VCU_Pkt_t;
#pragma pack(pop)

#pragma pack(push,1)
typedef struct {
  /* --- VCU --- */
  bool     drive_direction;
  bool     drive_neutral;
  uint16_t Regen_throttle;
  float    Regen_portion;
  uint8_t  Cruise_Control_Level;

  struct {
    bool throttle_ADC_error;
    bool WS22_CAN_error;
    bool WS22_motor_error;
    bool main_CAN_error;
    bool ICU_com_error;
    bool USART3_OVERFLOW;
    bool USART3_error;
  } vcu_error;

  /* --- ICU --- */
  bool     left_ts_on;
  bool     right_ts_on;
  bool     hazard_sig_on;
  bool     cruise_control_on;
  uint16_t cruise_control_error;
  float    cruise_control_target_speed;

  /* --- BMS --- */
  uint16_t battery_voltage;
  int16_t  battery_current;
  uint8_t  SOC;
  uint8_t  bms_status;
  int8_t   NTC[6];
  uint16_t max_cell_voltage[3];
  uint16_t min_cell_voltage[3];

  /* --- MPPT (3) --- */
  struct {
    float input_voltage;
    float input_current;
    float output_voltage;
    float output_current;
    float mosfet_temp;
    float controller_temp;
    float volt_12v;
    float volt_3v;
    float max_output_voltage;
    float max_input_current;
    uint8_t error_flags;
    uint8_t limit_flags;
  } MPPT_array[3];
} BYTE_Telemetry_Packet;
#pragma pack(pop)

#ifdef __cplusplus
}
#endif

#endif /* VCU_TYPES_H */
