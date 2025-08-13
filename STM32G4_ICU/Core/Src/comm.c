#include "comm.h"
#include "protocol.h"
#include "bsp.h"
#include "main.h"
#include "lights.h"
#include "display.h"
#include "parts.h"

void Send_VCU_CAN_ICU_Command(uint32_t id, uint32_t data1, uint32_t data2){

  memcpy(&FDCAN1_TxBuffer[0], &data1, sizeof(uint32_t));
  memcpy(&FDCAN1_TxBuffer[4], &data2, sizeof(uint32_t));
  FDCAN1_TxHeader.Identifier = VCU_RX_BASE_ADDR + id;
  if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &FDCAN1_TxHeader, FDCAN1_TxBuffer) != HAL_OK) {
    // Error_Handler();
  }
}

void Send_VCU_CAN_ICU_Command_CC(uint32_t id, uint32_t data1, float data2){

  memcpy(&FDCAN1_TxBuffer[0], &data1, sizeof(uint32_t));
  memcpy(&FDCAN1_TxBuffer[4], &data2, sizeof(float));
  FDCAN1_TxHeader.Identifier = VCU_RX_BASE_ADDR + id;
  if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &FDCAN1_TxHeader, FDCAN1_TxBuffer) != HAL_OK) {
    // Error_Handler();
  }
}

void Send_VCU_CAN_ICU_Command_Regen_Throttle(uint32_t id, uint16_t data1){

  memcpy(&FDCAN1_TxBuffer[0], &data1, sizeof(uint16_t));
  FDCAN1_TxHeader.Identifier = VCU_RX_BASE_ADDR + id;
  if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &FDCAN1_TxHeader, FDCAN1_TxBuffer) != HAL_OK) {
    // Error_Handler();
  }
}

void Handle_VCU_CAN_Packet(uint32_t offset) {

  uint16_t tmp;
  uint16_t cc, err;
  float    spd;
  switch (offset) {
    case DRIVER_INDICATOR_ADDR:
      updateVSLEDOutputs();
      if(!display_update_flag){
        updateDisplay();
        oled_refresh_counter = 0;
      }
        
      if(vsle_counter < 1024)
        signal_log[vsle_counter] = hazard_sig_light_on;

      if(vsle_counter > 2 && signal_log[vsle_counter] == signal_log[vsle_counter-1]){
        vsle_counter++; 
        break;
      }
      
        vsle_counter++;

      
      break;
    case DRIVE_MODE_ADDR:
      memcpy(&tmp, &FDCAN1_RxBuffer[0], 2);
      ICU.drive_mode = tmp;   // volatile write
      break;
    case CRUISE_CONTROL_ADDR:
      memcpy(&cc,  &FDCAN1_RxBuffer[0], 2);
      memcpy(&err, &FDCAN1_RxBuffer[2], 2);
      memcpy(&spd, &FDCAN1_RxBuffer[4], 4);
      ICU.CC            = cc;
      ICU.CC_error_code = err;
      ICU.Target_speed  = spd;
      break;

    case REGEN_LEVEL_ADDR:
      ICU.Regen_Lv = FDCAN1_RxBuffer[0];
      break;
    
    default:
      break;
  }
}

void Parse_MPPT_CAN_Packet(uint32_t offset) {

  uint8_t id;
  if(offset < 0x10){
    id = 0;
  }
  else if(offset < 0x20){
    id = 1;
    offset -= 0x10;
  }
  else if(offset < 0x30){
    id = 2;
    offset -= 0x20;
  }
  else
    return;

  MPPT_t* mppt = &MPPT_array[id];

  switch (offset) {
    case MPPT_INPUT_MEASUERE_ADDR:
      memcpy(&mppt->input_current, &FDCAN1_RxBuffer[4], sizeof(float));
      memcpy(&mppt->input_voltage, &FDCAN1_RxBuffer[0], sizeof(float)); 
      break;
    
    case MPPT_OUTPUT_MEASURE_ADDR:
      memcpy(&mppt->output_current, &FDCAN1_RxBuffer[4], sizeof(float));
      memcpy(&mppt->output_voltage, &FDCAN1_RxBuffer[0], sizeof(float));
      break;

    case MPPT_TEMPERATURE_ADDR:
      memcpy(&mppt->controller_temp, &FDCAN1_RxBuffer[4], sizeof(float));
      memcpy(&mppt->mosfet_temp, &FDCAN1_RxBuffer[0], sizeof(float));
      break;
    
    case MPPT_AUXILIARY_POWER_SUPPLY_ADDR:
      memcpy(&mppt->volt_3v, &FDCAN1_RxBuffer[4], sizeof(float));
      memcpy(&mppt->volt_12v, &FDCAN1_RxBuffer[0], sizeof(float));
      break;

    case MPPT_LIMITS_ADDR:
      memcpy(&mppt->max_input_current, &FDCAN1_RxBuffer[4], sizeof(float));
      memcpy(&mppt->max_output_voltage, &FDCAN1_RxBuffer[0], sizeof(float));
      break;

    case MPPT_STATUS_ADDR:
      mppt->Status.can_rx_error = FDCAN1_RxBuffer[0];
      mppt->Status.can_tx_error = FDCAN1_RxBuffer[1];
      mppt->Status.can_tx_overflow = FDCAN1_RxBuffer[2];
      mppt->Status.error_flags.raw = FDCAN1_RxBuffer[3];
      mppt->Status.limit_flags.raw = FDCAN1_RxBuffer[4];
      mppt->Status.mode = FDCAN1_RxBuffer[5];
      mppt->Status.reserved = FDCAN1_RxBuffer[6];
      mppt->Status.test_counter = FDCAN1_RxBuffer[7];
      break;
    
    case MPPT_POWER_CONNECTOR_ADDR:
      memcpy(&mppt->power_connector_temp, &FDCAN1_RxBuffer[4], sizeof(float));
      memcpy(&mppt->output_voltage_power_connector, &FDCAN1_RxBuffer[0], sizeof(float));
      break;
    
    default:
      // Unknown or unhandled telemetry address
      break;
  }
}

void Parse_BMS_CAN_Packet(){
  BMS.battery_voltage = (FDCAN1_RxBuffer[0] << 8) | FDCAN1_RxBuffer[1];
  BMS.battery_current = ((FDCAN1_RxBuffer[2] << 8) | FDCAN1_RxBuffer[3]);
  BMS.SOC = FDCAN1_RxBuffer[4];
  ICU.SOC = BMS.SOC;
  BMS.bms_status.raw = FDCAN1_RxBuffer[5];
  BMS.NTC0 = FDCAN1_RxBuffer[6];
  BMS.NTC1 = FDCAN1_RxBuffer[7];
  
  return;
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
      memcpy(&ws22.limit_flags_vector, &FDCAN1_RxBuffer[0], sizeof(uint16_t));        // Limit Flags (uint16_t)
      break;

    case WS22_BUS_MEAS_ADDR:
      // Bus Current (RxBuffer[4-7]) and Bus Voltage (RxBuffer[0-3])
      memcpy(&ws22.bus_current, &FDCAN1_RxBuffer[4], sizeof(float)); // Bus Current (float)
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

void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs) {
  // This function is called when a new message is received in FDCAN RX FIFO0
  if (__HAL_FDCAN_GET_FLAG(&hfdcan1, FDCAN_FLAG_RX_FIFO0_MESSAGE_LOST)) {
    FDCAN1_error_sig_count++;
    __HAL_FDCAN_CLEAR_FLAG(&hfdcan1, FDCAN_FLAG_RX_FIFO0_MESSAGE_LOST);
  }
  
  
  if ((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) != RESET) {
    if (hfdcan->Instance == FDCAN1) {
      // WS22 CAN packet received
      can_fill_level = HAL_FDCAN_GetRxFifoFillLevel(hfdcan, FDCAN_RX_FIFO0);
      while (HAL_FDCAN_GetRxFifoFillLevel(hfdcan, FDCAN_RX_FIFO0) > 0){
        if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &FDCAN1_RxHeader, FDCAN1_RxBuffer) == HAL_OK) {
          uint32_t id = FDCAN1_RxHeader.Identifier;
          uint32_t offset;
          if (id >= VCU_TX_BASE_ADDR && id <= (VCU_TX_BASE_ADDR + 0xFF)) {
            offset = id - VCU_TX_BASE_ADDR;
            Handle_VCU_CAN_Packet(offset);
          }
          else if (id == BMS_ID_ADDR){
            Parse_BMS_CAN_Packet();
          }
          else if (id >= WS22_FW_BASE_ADDR && id <= WS22_FW_BASE_ADDR + 0xFF) {
            offset = id - WS22_FW_BASE_ADDR;
            Parse_WS22_CAN_Packet(offset);
          }
          else if ( id >= ICU_FW_MPPT_BASE_ADDR && id < 0x700){
            offset = id-ICU_FW_MPPT_BASE_ADDR;
            Parse_MPPT_CAN_Packet(offset);
          }
          else {
            offset = id;
            FDCAN1_error_sig_count++;
          }
        }
      }
    }
  }
}

