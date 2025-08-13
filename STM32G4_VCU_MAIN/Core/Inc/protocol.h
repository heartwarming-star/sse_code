#ifndef PROTOCOL_H
#define PROTOCOL_H

#ifdef __cplusplus
extern "C" {
#endif

/* ICU Base Address */
#define ICU_TX_BASE_ADDR 0x600
#define ICU_RX_BASE_ADDR 0x000

/* From ICU Commands offset */
#define FROM_ICU_LEFT_TS_ON_ADDR       0x01
#define FROM_ICU_RIGHT_TS_ON_ADDR      0x02
#define FROM_ICU_HAZARD_SIG_ON_ADDR    0x03
#define FROM_ICU_RESET_WS22_ADDR       0x05
#define FROM_ICU_CRUISE_CONTROL_ADDR   0x07
#define FROM_ICU_CRUISE_CONTROL_LEVEL_ADDR 0x08
#define FROM_ICU_REGEN_THROTTLE_ADDR   0x09

/* To ICU Commands offset */
#define DRIVER_INDICATOR_ADDR          0x04
#define DRIVE_MODE_ADDR                0x06
#define CRUISE_CONTROL_ADDR            0x07
#define CRUISE_CONTROL_LEVEL_ADDR      0x08
/* #define PV_CHARGE_ADDR 0x09 */

/* From WS22 to ICU Forward Address */
#define ICU_FW_WS22_BASE_ADDR          0x300

/* Forward to ICU offset */
#define ICU_FW_WS22_STATUS_ADDR             0x01
#define ICU_FW_WS22_BUS_MEAS_ADDR           0x02
#define ICU_FW_WS22_VELOCITY_MEAS_ADDR      0x03
#define ICU_FW_WS22_HS_MOTOR_TEMP_MEAS_ADDR 0x0B
#define ICU_FW_WS22_DSP_TEMP_MEAS_ADDR      0x0C
#define ICU_FW_WS22_ODOMETER_AMPHOUR_MEAS_ADDR 0x0E

/* WaveSculptor22 Base Address */
#define WS22_RX_BASE_ADDR 0x400
#define WS22_TX_BASE_ADDR 0x500

/* WS22 Telemetry offset */
#define WS22_ID_ADDR                    0x00
#define WS22_STATUS_ADDR                0x01
#define WS22_BUS_MEAS_ADDR              0x02
#define WS22_VELOCITY_MEAS_ADDR         0x03
#define WS22_PHASE_CURRENT_MEAS_ADDR    0x04
#define WS22_VOLTAGE_VECTOR_MEAS_ADDR   0x05
#define WS22_CURRENT_VECTOR_MEAS_ADDR   0x06
#define WS22_BACK_EMF_MEAS_ADDR         0x07
#define WS22_15V_RAIL_MEAS_ADDR         0x08
#define WS22_3_3V_1_9V_RAIL_MEAS_ADDR   0x09
#define WS22_HS_MOTOR_TEMP_MEAS_ADDR    0x0B
#define WS22_DSP_TEMP_MEAS_ADDR         0x0C
#define WS22_ODOMETER_AMPHOUR_MEAS_ADDR 0x0E
#define WS22_SLIP_SPEED_MEAS_ADDR       0x17

/* Drive Commands Address */
#define WS22_MOTOR_DRIVE_COMMAND_ADDR 0x01
#define WS22_MOTOR_POWER_COMMAND_ADDR 0x02
#define WS22_RESET_COMMAND_ADDR       0x03

/* MPPT Base Address */
#define MPPT_ARRAY_BASE_ADDR 0x6A0
#define MPPT_1_BASE_ADDR     0x6A0
#define MPPT_2_BASE_ADDR     0x6B0
#define MPPT_3_BASE_ADDR     0x6C0

/* MPPT Telemetry offset */
#define MPPT_INPUT_MEASUERE_ADDR          0x00
#define MPPT_OUTPUT_MEASURE_ADDR          0x01
#define MPPT_TEMPERATURE_ADDR             0x02
#define MPPT_AUXILIARY_POWER_SUPPLY_ADDR  0x03
#define MPPT_LIMITS_ADDR                  0x04
#define MPPT_STATUS_ADDR                  0x05
#define MPPT_POWER_CONNECTOR_ADDR         0x06

/* MPPT COMMAND offset */
#define MPPT_MODE_COMMAND_ADDR            0x08
#define MPPT_MAXIMUM_OUTPUT_VOLTAGE_ADDR  0x0A
#define MPPT_MAXIMUM_INPUT_CURRENT_ADDR   0x0B

/* BMS Base Address */
#define BMS_RX_BASE_ADDR 0x100

/* BMS Telemetry offset */
#define BMS_DATA_0_ADDR 0x00
#define BMS_DATA_1_ADDR 0x01
#define BMS_DATA_2_ADDR 0x02

/* CC Error code */
#define CC_ERR_OK            0x0000
#define CC_ERR_BRAKE         0x0001
#define CC_ERR_NEUTRAL       0x0002
#define CC_ERR_REVERSE       0x0004
#define CC_ERR_THROTTLE      0x0008
#define CC_ERR_LOW_VELOCITY  0x0010
#define CC_ERR_OVER_VELOCITY 0x0020
#define CC_ERR_DIFF_HIGH     0x0040
#define CC_ERR_DIFF_LOW      0x0080
#define CC_ERR_FROM_COMMAND  0x0100

#define CC_ERR_THROTTLE_OKAY 0x8000
#define CC_ERR_OTHER         0x8000

/* TELEMETRY PACKET HEADER */
#define PKT_STX          0xAA
#define PKT_ID_VCU       0x01
#define PKT_ID_WS22      0x02
#define PKT_ID_MPPT      0x03
#define PKT_ID_BMS       0x04
#ifdef __cplusplus
}
#endif

#endif /* PROTOCOL_H */