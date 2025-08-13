#ifndef PARTS_H
#define PARTS_H

#include <icu_globals.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
  bool motor_over_speed;
  bool desaturation_fault;
  bool rail_15V_UVLO;
  bool config_read_error;
  bool watchdog_caused_last_reset;
  bool bad_motor_posion_hall_seq;
  bool DC_bus_over_voltage;
  bool SW_over_current;
  bool HW_over_current;
}ws22_error_flags_t;

typedef struct {
  bool motor_temp;
  bool bus_voltage_lower_limit;
  bool bus_voltage_upper_limit;
  bool bus_current;
  bool velocity;
  bool motor_current;
  bool output_voltage_PWM;
}ws22_limit_flags_t;

typedef union {
  uint8_t raw;
  struct {
    bool low_array_power    : 1; // bit0 (MSB)
    bool mosfet_overheat    : 1; // bit1
    bool battery_low        : 1; // bit2
    bool battery_full       : 1; // bit3
    bool undervoltage_12v   : 1; // bit4
    bool reserved           : 1; // bit5
    bool HW_overcurrent     : 1; // bit6
    bool HW_overvoltage     : 1; // bit7 (LSB)
  };
} MPPT_error_flags;

typedef union {
  uint8_t raw;
  struct {
    bool input_current_min     : 1; // bit0 (MSB)
    bool input_current_max     : 1; // bit1
    bool output_voltage_max    : 1; // bit2
    bool mosfet_temp_limit     : 1; // bit3
    bool duty_cycle_min        : 1; // bit4
    bool duty_cycle_max        : 1; // bit5
    bool local_mppt            : 1; // bit6
    bool global_mppt           : 1; // bit7 (LSB)
  };
} MPPT_limit_flags;

typedef struct {
  uint8_t can_rx_error;     // Byte 0
  uint8_t can_tx_error;     // Byte 1
  uint8_t can_tx_overflow;  // Byte 2
  MPPT_error_flags error_flags;      // Byte 3 (bitwise interpretation)
  MPPT_limit_flags limit_flags;      // Byte 4 (bitwise interpretation)
  uint8_t mode;             // Byte 5 (0 = standby, 1 = on)
  uint8_t reserved;         // Byte 6 Empty.
  uint8_t test_counter;     // Byte 7 (+1 every second)
}MPPT_Status_t;

typedef union {
  uint8_t raw;
  struct {
    bool Cell_Imbalance_Protection  : 1; // bit0 (MSB)
    bool SCD                        : 1; // bit1
    bool UTC                        : 1; // bit2
    bool OTD                        : 1; // bit3
    bool OCD                        : 1; // bit4
    bool OCC                        : 1; // bit5
    bool UC                         : 1; // bit6
    bool OV                         : 1; // bit7 (LSB)
  };
} BMS_Status;

typedef struct {
  uint32_t serial_number;
  uint32_t PRH_id;

  uint8_t rx_err_count;
  uint8_t tx_err_count;
  uint16_t active_motor;
  uint16_t error_flags_vector;
  uint16_t limit_flags_vector;

  ws22_error_flags_t error_flags;
  ws22_limit_flags_t limit_flags;

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

} ws22_t;

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

  MPPT_Status_t Status;

  float output_voltage_power_connector;
  float power_connector_temp;

  uint32_t last_updated_ms;
} MPPT_t;

typedef struct {
  uint16_t battery_voltage;
  uint16_t battery_current;
  uint8_t SOC;
  BMS_Status bms_status;
  uint8_t NTC0;//Battery Pack temperature.
  uint8_t NTC1;//Battery Pack temperature.
} BMS_t;

typedef struct {
  volatile uint16_t ROT[5];
  volatile bool HALL[2];
  volatile bool buf_GPIO_Input[5];
  Rotary_ENC_t ENC_R;
  Rotary_ENC_t ENC_L;

  volatile uint8_t disp_mode;
  volatile uint16_t drive_mode;
  volatile uint16_t CC;
  volatile uint16_t  CC_error_code;
  volatile float Target_speed;
  volatile uint8_t Regen_Lv;
  volatile uint8_t SOC;

} ICU_t;

extern ws22_error_flags_t ws22_error_flags;
extern ws22_limit_flags_t ws22_limit_flags;
extern ws22_t ws22;
extern ICU_t ICU;
extern BMS_t BMS;
extern MPPT_t MPPT_array[3];
extern MPPT_Status_t MPPT_Status;
#ifdef __cplusplus
}
#endif

#endif

