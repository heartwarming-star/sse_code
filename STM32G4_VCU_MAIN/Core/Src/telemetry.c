#include <string.h>
#include <stdio.h>
#include "telemetry.h"
#include "vcu_state.h"
#include "vcu_hal.h"

void Telemetry_UpdateBytePacket(void)
{
  byte_packet.drive_direction      = drive_direction;
  byte_packet.drive_neutral        = drive_neutral;
  byte_packet.Regen_throttle       = Regen_throttle;
  byte_packet.Regen_portion        = Regen_portion;
  byte_packet.Cruise_Control_Level = Cruise_Control_Level;

  byte_packet.vcu_error.throttle_ADC_error  = VCU_error_flags.throttle_ADC_error;
  byte_packet.vcu_error.WS22_CAN_error      = VCU_error_flags.WS22_CAN_error;
  byte_packet.vcu_error.WS22_motor_error    = VCU_error_flags.WS22_motor_error;
  byte_packet.vcu_error.main_CAN_error      = VCU_error_flags.main_CAN_error;
  byte_packet.vcu_error.ICU_com_error       = VCU_error_flags.ICU_com_error;
  byte_packet.vcu_error.USART3_OVERFLOW     = VCU_error_flags.USART3_OVERFLOW;
  byte_packet.vcu_error.USART3_error        = VCU_error_flags.USART3_error;

  byte_packet.left_ts_on             = left_ts_on;
  byte_packet.right_ts_on            = right_ts_on;
  byte_packet.hazard_sig_on          = hazard_sig_on;
  byte_packet.cruise_control_on      = cruise_control_on;
  byte_packet.cruise_control_error   = cruise_control_error;
  byte_packet.cruise_control_target_speed = cruise_control_speed_target;

  byte_packet.battery_voltage  = BMS.battery_voltage;
  byte_packet.battery_current  = BMS.battery_current;
  byte_packet.SOC              = BMS.SOC;
  byte_packet.bms_status       = BMS.bms_status.raw;

  byte_packet.NTC[0] = (int8_t)BMS.NTC0;
  byte_packet.NTC[1] = (int8_t)BMS.NTC1;
  byte_packet.NTC[2] = (int8_t)BMS.NTC2;
  byte_packet.NTC[3] = (int8_t)BMS.NTC3;
  byte_packet.NTC[4] = (int8_t)BMS.NTC4;
  byte_packet.NTC[5] = (int8_t)BMS.NTC5;

  byte_packet.max_cell_voltage[0] = BMS.module_segment_1_max_cell_voltage;
  byte_packet.max_cell_voltage[1] = BMS.module_segment_2_max_cell_voltage;
  byte_packet.max_cell_voltage[2] = BMS.module_segment_3_max_cell_voltage;

  byte_packet.min_cell_voltage[0] = BMS.module_segment_1_min_cell_voltage;
  byte_packet.min_cell_voltage[1] = BMS.module_segment_2_min_cell_voltage;
  byte_packet.min_cell_voltage[2] = BMS.module_segment_3_min_cell_voltage;

  for (uint8_t i = 0; i < 3; i++)
  {
    byte_packet.MPPT_array[i].input_voltage   = MPPT_array[i].input_voltage;
    byte_packet.MPPT_array[i].input_current   = MPPT_array[i].input_current;
    byte_packet.MPPT_array[i].output_voltage  = MPPT_array[i].output_voltage;
    byte_packet.MPPT_array[i].output_current  = MPPT_array[i].output_current;
    byte_packet.MPPT_array[i].mosfet_temp     = MPPT_array[i].mosfet_temp;
    byte_packet.MPPT_array[i].controller_temp = MPPT_array[i].controller_temp;
    byte_packet.MPPT_array[i].volt_12v        = MPPT_array[i].volt_12v;
    byte_packet.MPPT_array[i].volt_3v         = MPPT_array[i].volt_3v;
    byte_packet.MPPT_array[i].max_output_voltage = MPPT_array[i].max_output_voltage;
    byte_packet.MPPT_array[i].max_input_current  = MPPT_array[i].max_input_current;
    byte_packet.MPPT_array[i].error_flags     = MPPT_array[i].Status.error_flags.raw;
    byte_packet.MPPT_array[i].limit_flags     = MPPT_array[i].Status.limit_flags.raw;
  }
}

void Telemetry_SendFull(BYTE_Telemetry_Packet* packet, uint16_t size) {
  HAL_UART_Transmit_DMA(&huart3, (uint8_t*)packet, size);
}

/* Safe buffer insertions (size_1/size_2 kept for API compatibility) */
void Telemetry_InsertFloat(uint32_t size_1, float data_1, uint32_t size_2, char * data_2) {
  (void)size_1; (void)size_2;
  int n = snprintf((char*)(USART3_TxBuffer + USART3_TxBuffer_Load),
                   USART3_TX_BUFFER_SIZE - USART3_TxBuffer_Load,
                   "%.2f%s", data_1, data_2 ? data_2 : "");
  if (n < 0 || USART3_TxBuffer_Load + (uint32_t)n >= USART3_TX_BUFFER_SIZE) {
    VCU_error_flags.USART3_OVERFLOW = true;
    return;
  }
  USART3_TxBuffer_Load += (uint32_t)n;
}

void Telemetry_InsertInt(uint32_t size_1, uint16_t data_1, uint32_t size_2, char * data_2) {
  (void)size_1; (void)size_2;
  int n = snprintf((char*)(USART3_TxBuffer + USART3_TxBuffer_Load),
                   USART3_TX_BUFFER_SIZE - USART3_TxBuffer_Load,
                   "%u%s", (unsigned)data_1, data_2 ? data_2 : "");
  if (n < 0 || USART3_TxBuffer_Load + (uint32_t)n >= USART3_TX_BUFFER_SIZE) {
    VCU_error_flags.USART3_OVERFLOW = true;
    return;
  }
  USART3_TxBuffer_Load += (uint32_t)n;
}

void Telemetry_SendUSART3(void) {
  USART3_TxBuffer_Load = 0;
  memset(USART3_TxBuffer, 32, sizeof(USART3_TxBuffer));

  switch (USART3_mode) {
    case TEXT:
      Telemetry_InsertFloat(0, 0, 9, "\033[2J\033[H\r\n");

      Telemetry_InsertFloat(6, ws22.vehicle_velocity, 7, "\tkm/h\r\n");
      Telemetry_InsertInt(3, BMS.SOC, 8, "\t%\tSOC\r\n");
      Telemetry_InsertFloat(6, (float)(BMS.battery_voltage * BMS.battery_current) / 10000.f, 4, "\tW\r\n");

      if (drive_neutral) {
        Telemetry_InsertFloat(0, 0, 14, "GEAR\tNEUTRAL\r\n");
      } else if (drive_direction) {
        Telemetry_InsertFloat(0, 0, 14, "GEAR\tFORWARD\r\n");
      } else {
        Telemetry_InsertFloat(0, 0, 14, "GEAR\tREVERSE\r\n");
      }

      Telemetry_InsertFloat(5, ws22.motor_rpm, 6, "\tRPM\r\n");
      Telemetry_InsertFloat(6, throttle_portion*100, 8, "\t%\tTHR\r\n");
      Telemetry_InsertFloat(6, Regen_portion*100, 18, "\t%\tRegen_portion\r\n");
      Telemetry_InsertFloat(6, Regen_throttle, 13, "\t\tRegen_THR\r\n");
      Telemetry_InsertInt(3 , Cruise_Control_Level, 12, "\t\tRegen_lv\r\n");
      Telemetry_InsertFloat(0, 0, 2, "\r\n");

      Telemetry_InsertFloat(5, ws22.bus_voltage, 9, "\tV\tWS22\r\n");
      Telemetry_InsertFloat(5, ws22.bus_current, 9, "\tA\tWS22\r\n");
      Telemetry_InsertFloat(6, ws22.bus_current*ws22.bus_voltage, 10, "\tW\tWS22\r\n");
      Telemetry_InsertFloat(4, ws22.DSP_board_temp, 14, "\t'C\tWS22\tDSP\r\n");
      Telemetry_InsertFloat(4, ws22.heatsink_temp, 13, "\t'C\tWS22\tHS\r\n");
      Telemetry_InsertFloat(4, ws22.motor_temp, 13, "\t'C\tWS22\tMT\r\n");
      Telemetry_InsertInt(3, ws22.limit_flags_vector, 11, "\t\tWS22\tLC\r\n");
      Telemetry_InsertInt(3, ws22.error_flags_vector, 11, "\t\tWS22\tEC\r\n");
      Telemetry_InsertFloat(0, 0, 2, "\r\n");

      Telemetry_InsertFloat(5, MPPT_array[0].output_voltage * MPPT_array[0].output_current
                               + MPPT_array[1].output_voltage * MPPT_array[1].output_current
                               + MPPT_array[2].output_voltage * MPPT_array[2].output_current, 15, "\tW\tMPPT\tTOTAL\r\n");
      Telemetry_InsertFloat(0, 0, 2, "\r\n");

      for (int i=0;i<3;i++) {
        char tag[8]; snprintf(tag, sizeof(tag), "MPPT%d", i+1);
        Telemetry_InsertFloat(5, MPPT_array[i].output_voltage, 8, "\tV\t");
        Telemetry_InsertFloat(0, 0, 6, tag); Telemetry_InsertFloat(0,0,6,"\tOUT\r\n");
        Telemetry_InsertFloat(5, MPPT_array[i].output_current, 8, "\tA\t");
        Telemetry_InsertFloat(0, 0, 6, tag); Telemetry_InsertFloat(0,0,6,"\tOUT\r\n");
        Telemetry_InsertFloat(6, MPPT_array[i].output_voltage * MPPT_array[i].output_current, 8, "\tW\t");
        Telemetry_InsertFloat(0, 0, 6, tag); Telemetry_InsertFloat(0,0,6,"\tOUT\r\n");

        Telemetry_InsertFloat(5, MPPT_array[i].input_voltage, 8, "\tV\t");
        Telemetry_InsertFloat(0, 0, 6, tag); Telemetry_InsertFloat(0,0,6,"\tIN\r\n");
        Telemetry_InsertFloat(5, MPPT_array[i].input_current, 8, "\tA\t");
        Telemetry_InsertFloat(0, 0, 6, tag); Telemetry_InsertFloat(0,0,6,"\tIN\r\n");
        Telemetry_InsertFloat(6, MPPT_array[i].input_voltage * MPPT_array[i].input_current, 8, "\tW\t");
        Telemetry_InsertFloat(0, 0, 6, tag); Telemetry_InsertFloat(0,0,6,"\tIN\r\n");

        Telemetry_InsertFloat(5, MPPT_array[i].mosfet_temp, 8, "\t'C\t");
        Telemetry_InsertFloat(0, 0, 6, tag); Telemetry_InsertFloat(0,0,9,"\tMOSFET\r\n");
        Telemetry_InsertInt(3, MPPT_array[i].Status.limit_flags.raw, 8, "\t\t");
        Telemetry_InsertFloat(0, 0, 6, tag); Telemetry_InsertFloat(0,0,5,"\tLC\r\n");
        Telemetry_InsertInt(3, MPPT_array[i].Status.error_flags.raw, 8, "\t\t");
        Telemetry_InsertFloat(0, 0, 6, tag); Telemetry_InsertFloat(0,0,5,"\tEC\r\n");
        Telemetry_InsertFloat(0, 0, 2, "\r\n");
      }

      Telemetry_InsertFloat(5, ((float)BMS.battery_voltage) / 100.f, 8, "\tV\tBMS\r\n");
      Telemetry_InsertFloat(5, ((float)BMS.battery_current) / 100.f, 8, "\tA\tBMS\r\n");
      Telemetry_InsertInt(3, BMS.NTC0, 9, "\t'C\tBMS\r\n");
      Telemetry_InsertInt(3, BMS.bms_status.raw, 10, "\t\tBMS\tEC\r\n");
      Telemetry_InsertInt(0, 0, 2, "\r\n");

      Telemetry_InsertInt(5, cruise_control_error, 7, "\t\tCC\tEC\r\n");

      memset(USART3_TxBuffer+USART3_TxBuffer_Load, 0, sizeof(USART3_TxBuffer)-USART3_TxBuffer_Load);
      if (HAL_UART_Transmit_DMA(&huart3, USART3_TxBuffer, sizeof(USART3_TxBuffer)) != HAL_OK) {
        VCU_error_flags.USART3_error = true;
      }
      break;

    case BYTE:
      Telemetry_UpdateBytePacket();
      Telemetry_SendFull(&byte_packet, sizeof(byte_packet));
      break;

    default:
      break;
  }
}