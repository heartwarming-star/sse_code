#include <string.h>
#include "ws22.h"
#include "ws22_ctrl_sm.h"
#include "vcu_state.h"
#include "vcu_hal.h"
#include "protocol.h"
#include <math.h>

/* --- Cruise/Coast behaviour & safety caps --- */

#ifndef WS22_FWD_RPM_CAP
#define WS22_FWD_RPM_CAP 5000.0f
#endif

// ===== Percent-based current command (0..100%), NO regen on release =====
#define VCU_DT_S                 0.05f   // 50 ms
// ↑ 올라갈 때만 제한(필수)
#define SLEW_UP_PCT_PER_S        20.0f   
#define JERK_UP_PCT_PER_S2       400.0f  // 틱당 ±20 %/s (업 저크
// ↓ 내릴 때 제한은 기본 비활성(즉시 0). 필요시 1로 바꾸면 아주 빠른 완충 적용
#define ENABLE_FAST_DOWN_LIMIT   0

#if ENABLE_FAST_DOWN_LIMIT
  #define SLEW_DOWN_RELEASE_PCT_PER_S 900.0f  // 틱당 -45 %
  #define RELEASE_SNAP_THRESHOLD_PCT   15.0f  // 여기 이하면 0으로 스냅
#endif

static float last_current_command = 0.f;   // [%]
static float last_rate_pct_per_s  = 0.f;   // [%/s]
const float max_reverse_factor = 0.4f;

static void Parse_WS22_CAN_Packet(uint32_t offset);
static uint32_t Read_throttle_ADC(void);
static float Convert_throttle_input_into_current_command(uint32_t thr_in,
                                                         uint32_t thr_dead_zone_threshold,
                                                         uint32_t thr_dynamic_range);
                         

static inline float shape_throttle(float x){      // smoothstep
    if (x<=0.f) return 0.f; 
    
    if (x>=1.f) return 1.f;
    return x*x*(3.f - 2.f*x);
}

static float Convert_throttle_input_into_current_command(uint32_t thr_in,
                                                         uint32_t thr_dead_zone_threshold,
                                                         uint32_t thr_dynamic_range){

    // 1) 0..1 정규화
    float portion = 0.f;
    if (thr_in > thr_dead_zone_threshold) {
        uint32_t num = thr_in - thr_dead_zone_threshold;
        if (num >= thr_dynamic_range) portion = 1.f;
        else                          portion = (float)num / (float)thr_dynamic_range;
    }

    // 2) 곡선 + 크루즈 스케일
    float shaped = shape_throttle(portion);

    float level_scale = 1.f;

    // 3) 목표[%]
    float desired_pct = 100.f * shaped * level_scale;

    // 4) 가속 페달 오프 → 즉시 0% or 빠른 다운
    if (thr_in <= thr_dead_zone_threshold) {
    #if ENABLE_FAST_DOWN_LIMIT
        float max_drop = SLEW_DOWN_RELEASE_PCT_PER_S * VCU_DT_S;
        float out = last_current_command - max_drop;
        if (out <= RELEASE_SNAP_THRESHOLD_PCT) out = 0.f;
        last_rate_pct_per_s  = (out - last_current_command) / VCU_DT_S;
        last_current_command = (out < 0.f) ? 0.f : out;
        return last_current_command;
    #else
        last_rate_pct_per_s  = 0.f;
        last_current_command = 0.f;
        return 0.f;
    #endif
    }

    // 5) 업 슬루/저크 (다운은 자유)
    float delta   = desired_pct - last_current_command;
    float max_up  = SLEW_UP_PCT_PER_S* VCU_DT_S;
    float step    = (delta > max_up) ? max_up : delta;  // 불필요 삼항 제거

    // 저크 제한(현재는 ±방향 모두 제한됨)
    float rate_cmd = step / VCU_DT_S;                // [%/s]
    float dr       = rate_cmd - last_rate_pct_per_s; // 가속도의 변화
    float max_dr   = JERK_UP_PCT_PER_S2 * VCU_DT_S;  // 틱당 허용 저크
    if (dr >  max_dr) rate_cmd = last_rate_pct_per_s + max_dr;
    if (dr < -max_dr) rate_cmd = last_rate_pct_per_s - max_dr;

    float out_pct = last_current_command + rate_cmd * VCU_DT_S;
    if (out_pct < 0.f)   out_pct = 0.f;
    if (out_pct > 100.f) out_pct = 100.f;
    return out_pct;
}

static uint32_t Read_throttle_ADC(void) {
  // Read throttle input from ADC3
  uint32_t thr_input = throttle_input;
  GPIO_PinState pinState = !HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0);
  if (pinState == GPIO_PIN_RESET) {
    // set throttle input to 0 when throttle digital input is low
    thr_input = 0;
  }
  
  return thr_input;
}

static void Parse_WS22_CAN_Packet(uint32_t offset) {
  // Parse the received WS22 CAN packet
  switch (offset) {
    case WS22_ID_ADDR:
      // Serial Number (RxBuffer[4-7]) and Prohelion ID (RxBuffer[0-3])
      memcpy(&ws22.serial_number, &FDCAN1_RxBuffer[4], sizeof(uint32_t));    // Serial Number (uint32_t, 63-32 bits)
      memcpy(&ws22.PRH_id, &FDCAN1_RxBuffer[0], sizeof(uint32_t));           // Prohelion ID (uint32_t, 31-0 bits)
      break;

    case WS22_STATUS_ADDR:
      // Receive Error Count (RxBuffer[7]), Transmit Error Count (RxBuffer[6]),
      // Active Motor (RxBuffer[4-5]), Error Flags (RxBuffer[2-3]), Limit Flags (RxBuffer[0-1])
      ws22.rx_err_count = FDCAN1_RxBuffer[7];  // Receive Error Count (uint8_t)
      ws22.tx_err_count = FDCAN1_RxBuffer[6];  // Transmit Error Count (uint8_t)
      ws22.active_motor = (uint16_t)((FDCAN1_RxBuffer[5] << 8) | FDCAN1_RxBuffer[4]); // Active Motor (uint16_t)
      memcpy(&ws22.error_flags_vector, &FDCAN1_RxBuffer[2], sizeof(uint16_t));        // Error Flags (uint16_t)
      ws22.error_flags.HW_over_current = (ws22.error_flags_vector & (1 << 0)) != 0;
      ws22.error_flags.SW_over_current = (ws22.error_flags_vector & (1 << 1)) != 0;
      ws22.error_flags.DC_bus_over_voltage = (ws22.error_flags_vector & (1 << 2)) != 0;
      ws22.error_flags.bad_motor_posion_hall_seq = (ws22.error_flags_vector & (1 << 3)) != 0;
      ws22.error_flags.watchdog_caused_last_reset = (ws22.error_flags_vector & (1 << 4)) != 0;
      ws22.error_flags.config_read_error = (ws22.error_flags_vector & (1 << 5)) != 0;
      ws22.error_flags.rail_15V_UVLO = (ws22.error_flags_vector & (1 << 6)) != 0;
      ws22.error_flags.desaturation_fault = (ws22.error_flags_vector & (1 << 7)) != 0;
      ws22.error_flags.motor_over_speed = (ws22.error_flags_vector & (1 << 8)) != 0;
      memcpy(&ws22.limit_flags_vector, &FDCAN1_RxBuffer[0], sizeof(uint16_t));        // Limit Flags (uint16_t)
      ws22.limit_flags.output_voltage_PWM = (ws22.limit_flags_vector & (1 << 0)) != 0;
      ws22.limit_flags.motor_current = (ws22.limit_flags_vector & (1 << 1)) != 0;
      ws22.limit_flags.velocity = (ws22.limit_flags_vector & (1 << 2)) != 0;
      ws22.limit_flags.bus_current = (ws22.limit_flags_vector & (1 << 3)) != 0;
      ws22.limit_flags.bus_voltage_upper_limit = (ws22.limit_flags_vector & (1 << 4)) != 0;
      ws22.limit_flags.bus_voltage_lower_limit = (ws22.limit_flags_vector & (1 << 5)) != 0;
      ws22.limit_flags.motor_temp = (ws22.limit_flags_vector & (1 << 6)) != 0;
      break;

    case WS22_BUS_MEAS_ADDR:
      // Bus Current (RxBuffer[4-7]) and Bus Voltage (RxBuffer[0-3])
      memcpy(&ws22.bus_current, &FDCAN1_RxBuffer[4], sizeof(float)); // Bus Current (float)
      ws22.bus_current *= 0.85;
      memcpy(&ws22.bus_voltage, &FDCAN1_RxBuffer[0], sizeof(float)); // Bus Voltage (float)
      break;

    case WS22_VELOCITY_MEAS_ADDR:
      // Vehicle Velocity (RxBuffer[4-7]) and Motor Velocity (RxBuffer[0-3])
      memcpy(&ws22.vehicle_velocity, &FDCAN1_RxBuffer[4], sizeof(float)); // Vehicle Velocity (float)
      ws22.vehicle_velocity *= 3.6;
      memcpy(&ws22.motor_rpm, &FDCAN1_RxBuffer[0], sizeof(float));       // Motor Velocity (float)
      break;

    case WS22_PHASE_CURRENT_MEAS_ADDR:
      // Phase C Current (RxBuffer[4-7]) and Phase B Current (RxBuffer[0-3])
      memcpy(&ws22.phase_C_current, &FDCAN1_RxBuffer[4], sizeof(float)); // Phase C Current (float)
      memcpy(&ws22.phase_B_current, &FDCAN1_RxBuffer[0], sizeof(float)); // Phase B Current (float)
      break;

    case WS22_VOLTAGE_VECTOR_MEAS_ADDR:
      // Vd (RxBuffer[4-7]) and Vq (RxBuffer[0-3])
      memcpy(&ws22.Vd, &FDCAN1_RxBuffer[4], sizeof(float)); // Vd (float)
      memcpy(&ws22.Vq, &FDCAN1_RxBuffer[0], sizeof(float)); // Vq (float)
      break;

    case WS22_CURRENT_VECTOR_MEAS_ADDR:
      // Id (RxBuffer[4-7]) and Iq (RxBuffer[0-3])
      memcpy(&ws22.Id, &FDCAN1_RxBuffer[4], sizeof(float)); // Id (float)
      memcpy(&ws22.Iq, &FDCAN1_RxBuffer[0], sizeof(float)); // Iq (float)
      break;

    case WS22_BACK_EMF_MEAS_ADDR:
      // BEMFd (RxBuffer[4-7]) and BEMFq (RxBuffer[0-3])
      memcpy(&ws22.BEMFd, &FDCAN1_RxBuffer[4], sizeof(float)); // BEMFd (float)
      memcpy(&ws22.BEMFq, &FDCAN1_RxBuffer[0], sizeof(float)); // BEMFq (float)
      break;

    case WS22_15V_RAIL_MEAS_ADDR:
      // 15V Rail (RxBuffer[4-7])
      memcpy(&ws22.rail_15V, &FDCAN1_RxBuffer[4], sizeof(float)); // 15V Rail (float)
      break;

    case WS22_3_3V_1_9V_RAIL_MEAS_ADDR:
      // 3.3V Rail (RxBuffer[4-7]) and 1.9V Rail (RxBuffer[0-3])
      memcpy(&ws22.rail_3_3V, &FDCAN1_RxBuffer[4], sizeof(float)); // 3.3V Rail (float)
      memcpy(&ws22.rail_1_9V, &FDCAN1_RxBuffer[0], sizeof(float)); // 1.9V Rail (float)
      break;

    case WS22_HS_MOTOR_TEMP_MEAS_ADDR:
      // Heat-sink Temp (RxBuffer[4-7]) and Motor Temp (RxBuffer[0-3])
      memcpy(&ws22.heatsink_temp, &FDCAN1_RxBuffer[4], sizeof(float)); // Heat-sink Temp (float)
      memcpy(&ws22.motor_temp, &FDCAN1_RxBuffer[0], sizeof(float));    // Motor Temp (float)
      break;

    case WS22_DSP_TEMP_MEAS_ADDR:
      // DSP Board Temp (RxBuffer[0-3])
      memcpy(&ws22.DSP_board_temp, &FDCAN1_RxBuffer[0], sizeof(float)); // DSP Board Temp (float)
      break;

    case WS22_ODOMETER_AMPHOUR_MEAS_ADDR:
      // DC Bus Amp Hours (RxBuffer[4-7]) and Odometer (RxBuffer[0-3])
      memcpy(&ws22.DC_BUS_AH, &FDCAN1_RxBuffer[4], sizeof(float)); // DC Bus Amp Hours (float)
      memcpy(&ws22.odometer, &FDCAN1_RxBuffer[0], sizeof(float));  // Odometer (float)
      break;

    case WS22_SLIP_SPEED_MEAS_ADDR:
      // Slip Speed (RxBuffer[4-7])
      memcpy(&ws22.slip_speed, &FDCAN1_RxBuffer[4], sizeof(float)); // Slip Speed (float)
      break;

    default:
      // Unknown or unhandled telemetry address
      break;
  }
}

void WS22_SendMotorCommand(void)
{
    float motor_velocity_command = 0.f;
    float motor_current_command  = 0.f;

    /* Refresh throttle reading */
    throttle_input = Read_throttle_ADC();

    /* Update mode selector */
    WS22_FSM_Update();

    /* -------------- Gear & Mode Logic ------------------- */
    if (drive_neutral) {
        motor_velocity_command = 0.f;
        motor_current_command  = 0.f;
    }
    else if (drive_direction) {           /* forward */

        const int accel_pressed = (throttle_input > throttle_dead_zone_threshold);
        
        if (ws22_ctrl_mode == WS22_CTRL_CRUISE) {
            /* --- Coasting Cruise (no regen) --- */

            float base_current = 0.f;
            if (ws22.motor_rpm < cruise_control_rpm_target) {
                /* below target: push forward with forward limit */
                base_current = cruise_control_current_limit_forward;
            } else {
                /* at/above target: coast (0A), absolutely no backward current */
                base_current = 0.f;
            }

            float accel_current = 0.f;
            if (accel_pressed) {
                accel_current = Convert_throttle_input_into_current_command(
                                    throttle_input,
                                    throttle_dead_zone_threshold,
                                    throttle_dynamic_range);
                motor_velocity_command = WS22_FWD_RPM_CAP;   /* allow overshoot when accelerating */
            } else {
                motor_velocity_command = cruise_control_rpm_target;
            }

            /* choose the stronger forward torque; forbid negative current */
            motor_current_command = (accel_current > base_current) ? accel_current : base_current;
        }

        else { /* --- TORQUE mode (safe) --- */
            if (accel_pressed) {
                motor_current_command = Convert_throttle_input_into_current_command(
                                            throttle_input,
                                            throttle_dead_zone_threshold,
                                            throttle_dynamic_range);
                motor_velocity_command = WS22_FWD_RPM_CAP;    /* forward rpm cap */
            } else {
                /* No throttle: send explicit 0A and neutral velocity target */
                motor_current_command  = 0.f;
                motor_velocity_command = 0.f;
                /* also clear ramp so a subsequent tip-in starts from 0 */
                last_current_command = 0.f;
                last_rate_pct_per_s  = 0.f;
            }
        }
    }

    else {                                 /* reverse */
        /* keep conservative behaviour; zero if not pressed */
        if (throttle_input > throttle_dead_zone_threshold) {
            motor_current_command = Convert_throttle_input_into_current_command(
                                        throttle_input,
                                        throttle_dead_zone_threshold,
                                        throttle_dynamic_range)*max_reverse_factor;
        } else {
            motor_current_command = 0.f;
            /* clear ramp at zero input to avoid jump on next press */
            last_current_command = 0.f;
            last_rate_pct_per_s  = 0.f;
        }
        motor_velocity_command = -5000.f;
    }
    
    const float prev = last_current_command;
    last_current_command = motor_current_command;                 // [%] 실제 보낸 값
    last_rate_pct_per_s  = (last_current_command - prev) / VCU_DT_S;

    /* -------------- CAN transmit ----------------------- */
    memcpy(&FDCAN1_TxBuffer[0], &motor_velocity_command, sizeof(motor_velocity_command));
    memcpy(&FDCAN1_TxBuffer[4], &motor_current_command,  sizeof(motor_current_command));

    FDCAN1_TxHeader.Identifier = WS22_TX_BASE_ADDR + WS22_MOTOR_DRIVE_COMMAND_ADDR;
    if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1,
                                      &FDCAN1_TxHeader,
                                      FDCAN1_TxBuffer) != HAL_OK)
    {
        VCU_error_flags.WS22_CAN_error = true;
    }
}

void WS22_ForwardToICU(uint32_t id) {
  memcpy(&FDCAN2_TxBuffer[0], &FDCAN1_RxBuffer[0], 8);
  FDCAN2_TxHeader.Identifier = id;
  if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan2, &FDCAN2_TxHeader, FDCAN2_TxBuffer) != HAL_OK) {
    VCU_error_flags.main_CAN_error = true;
  }
}

void WS22_HandlePacket(uint32_t id) {
  uint32_t offset = id - WS22_RX_BASE_ADDR;
  Parse_WS22_CAN_Packet(offset);
  switch (offset) {
    case WS22_STATUS_ADDR:
      WS22_ForwardToICU(ICU_FW_WS22_BASE_ADDR + ICU_FW_WS22_STATUS_ADDR);
      break;
    case WS22_BUS_MEAS_ADDR:
      WS22_ForwardToICU(ICU_FW_WS22_BASE_ADDR + ICU_FW_WS22_BUS_MEAS_ADDR);
      break;
    case WS22_VELOCITY_MEAS_ADDR:
      WS22_ForwardToICU(ICU_FW_WS22_BASE_ADDR + ICU_FW_WS22_VELOCITY_MEAS_ADDR);
      break;
    case WS22_HS_MOTOR_TEMP_MEAS_ADDR:
      WS22_ForwardToICU(ICU_FW_WS22_BASE_ADDR + ICU_FW_WS22_HS_MOTOR_TEMP_MEAS_ADDR);
      break;
    case WS22_DSP_TEMP_MEAS_ADDR:
      WS22_ForwardToICU(ICU_FW_WS22_BASE_ADDR + ICU_FW_WS22_DSP_TEMP_MEAS_ADDR);
      break;
    case WS22_ODOMETER_AMPHOUR_MEAS_ADDR:
      WS22_ForwardToICU(ICU_FW_WS22_BASE_ADDR + ICU_FW_WS22_ODOMETER_AMPHOUR_MEAS_ADDR);
      break;
    default:
      break;
  }
}
