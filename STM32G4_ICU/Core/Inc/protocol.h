// protocol.h — CAN message ID map & command/telemetry offsets for ICU/VCU
//
// This header is shared by ICU firmware, VCU firmware, and any host‑side tools
// so that all components stay byte‑level compatible.
// Modify with extreme caution:  changing a value here breaks backward CAN
// compatibility unless *all* nodes are reflashed together.
// -----------------------------------------------------------------------------
#ifndef PROTOCOL_H
#define PROTOCOL_H

#ifdef __cplusplus
extern "C" {
#endif

// ─────────────────────────────────────────────────────────────────────────────
// 1. Base CAN identifiers (11‑bit standard frame)
// ─────────────────────────────────────────────────────────────────────────────
#define BMS_ID_ADDR          0x100  // One‑shot ID, no offset

#define VCU_RX_BASE_ADDR     0x600  // ICU → VCU command stream
#define VCU_TX_BASE_ADDR     0x000  // VCU → ICU telemetry/status

#define WS22_FW_BASE_ADDR    0x300  // Inverter WS22 telemetry
#define ICU_FW_MPPT_BASE_ADDR 0x6A0 // MPPT 0 (6A0) 6B0 6C0
#define ICU_FW_BMS_BASE_ADDR 0x200  // (reserved for future BMS‑specific frames)

// ─────────────────────────────────────────────────────────────────────────────
// 2. VCU command offsets (ICU‑>VCU, added to 0x600)
// ─────────────────────────────────────────────────────────────────────────────
#define LEFT_TS_ADDR             0x01
#define RIGHT_TS_ADDR            0x02
#define HAZARD_SIG_ADDR          0x03
#define DRIVER_INDICATOR_ADDR    0x04
#define WS22_RESET_ADDR          0x05
#define DRIVE_MODE_ADDR          0x06
#define CRUISE_CONTROL_ADDR      0x07
#define REGEN_LEVEL_ADDR         0x08
#define REGEN_THROTTLE_ADDR      0x09

// ─────────────────────────────────────────────────────────────────────────────
// 3. WS22 inverter telemetry offsets (0x300 + offset)
// ─────────────────────────────────────────────────────────────────────────────
#define WS22_ID_ADDR                 0x00
#define WS22_STATUS_ADDR             0x01
#define WS22_BUS_MEAS_ADDR           0x02
#define WS22_VELOCITY_MEAS_ADDR      0x03
#define WS22_PHASE_CURRENT_MEAS_ADDR 0x04
#define WS22_VOLTAGE_VECTOR_MEAS_ADDR 0x05
#define WS22_CURRENT_VECTOR_MEAS_ADDR 0x06
#define WS22_BACK_EMF_MEAS_ADDR      0x07
#define WS22_15V_RAIL_MEAS_ADDR      0x08
#define WS22_3_3V_1_9V_RAIL_MEAS_ADDR 0x09
#define WS22_HS_MOTOR_TEMP_MEAS_ADDR 0x0B
#define WS22_DSP_TEMP_MEAS_ADDR      0x0C
#define WS22_ODOMETER_AMPHOUR_MEAS_ADDR 0x0E
#define WS22_SLIP_SPEED_MEAS_ADDR    0x17

// ─────────────────────────────────────────────────────────────────────────────
// 4. MPPT telemetry offsets (added to 0x6A0 / 0x6B0 / 0x6C0)
// ─────────────────────────────────────────────────────────────────────────────
#define MPPT_INPUT_MEASUERE_ADDR        0x000
#define MPPT_OUTPUT_MEASURE_ADDR        0x001
#define MPPT_TEMPERATURE_ADDR           0x002
#define MPPT_AUXILIARY_POWER_SUPPLY_ADDR 0x003
#define MPPT_LIMITS_ADDR                0x004
#define MPPT_STATUS_ADDR                0x005
#define MPPT_POWER_CONNECTOR_ADDR       0x006

// ─────────────────────────────────────────────────────────────────────────────
// 5. Cruise‑Control error codes (shared between nodes)
// ─────────────────────────────────────────────────────────────────────────────
#define CC_ERR_OK             0x0000  /* 정상 */
#define CC_ERR_BRAKE          0x0001  /* 브레이크 페달 입력 */
#define CC_ERR_NEUTRAL        0x0002  /* 기어 Neutral */
#define CC_ERR_REVERSE        0x0004  /* 후진(Drive 방향 아님) */
#define CC_ERR_THROTTLE       0x0008
#define CC_ERR_LOW_TARGET     0x0010
#define CC_ERR_OVER_TARGET    0x0020
#define CC_ERR_DIFF_HIGH      0x0040
#define CC_ERR_DIFF_LOW       0x0080
#define CC_ERR_FROM_COMMAND   0x0100

#define CC_ERR_THROTTLE_OKAY  0x8000
#define CC_ERR_OTHER          0x8000


#ifdef __cplusplus
}
#endif

// -----------------------------------------------------------------------------
#endif /* PROTOCOL_H */
