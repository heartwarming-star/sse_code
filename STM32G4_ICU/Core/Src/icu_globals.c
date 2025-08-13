#include <icu_globals.h>
#include <parts.h>

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc2;
ADC_HandleTypeDef hadc3;
DMA_HandleTypeDef hdma_adc1;
DMA_HandleTypeDef hdma_adc2;
DMA_HandleTypeDef hdma_adc3;

FDCAN_HandleTypeDef hfdcan1;

SPI_HandleTypeDef hspi3;

TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim6;
TIM_HandleTypeDef htim7;

/* USER CODE BEGIN PV */
const uint16_t TS_ADC_THRESHOLD = 3200;

uint16_t g_AdcDmaBuffer_1[6];// ROT1~5. HALL1
uint16_t g_AdcDmaBuffer_2[2];// BLINK L, R
uint16_t g_AdcDmaBuffer_3[1];// HALL2

//turn signal
volatile uint16_t left_turn_sig_adc_input = 0;
volatile uint16_t right_turn_sig_adc_input = 0;
volatile bool hazard_sig_digital_input = false;
const uint16_t light_sig_output = 0xff;

volatile bool left_ts_input_was_on = false;
volatile bool right_ts_input_was_on = false;
volatile bool hazard_sig_input_was_on = false;
volatile bool display_sig_input_was_on = false;
volatile bool left_ts_light_on = false;
volatile bool right_ts_light_on = false;
volatile bool hazard_sig_light_on = false;
volatile int vsle_counter = 0;
volatile uint32_t can_fill_level = 0;

//ROT signal
const uint16_t ROT_threshold[12] = {274, 618, 959, 1300, 1641, 1982, 2323, 2664, 3005, 3346, 3687, 4028};
const int ROT_diff_threshold = 171;
volatile uint16_t ROT_ADC_input[5];
volatile uint16_t ROT[5];
volatile uint16_t HALL_ADC_input[2];
volatile bool HALL[2];
volatile bool signal_log[1024];

Rotary_ENC_t ENC_R;
Rotary_ENC_t ENC_L;

volatile uint16_t drive_mode;
volatile bool CC;
volatile uint8_t disp_mode;
volatile bool display_mode_update_flag = false;
volatile bool display_update_flag = false;
uint8_t OLED_buf[256 * 64 / 2];
uint8_t OLED_buf_1[256 * 64 / 2];
uint8_t OLED_buf_2[256 * 64 / 2];
volatile float buf_Layout_2[20];
volatile bool buf_GPIO_Input[5];

volatile uint16_t oled_refresh_counter = 0;
const uint16_t oled_refresh_interval = 9;
volatile uint16_t Regen_throttle_counter = 0;
const uint16_t Regen_throttle_interval = 9;

FDCAN_TxHeaderTypeDef FDCAN1_TxHeader;
FDCAN_RxHeaderTypeDef FDCAN1_RxHeader;
uint8_t FDCAN1_TxBuffer[8] = {0};
uint8_t FDCAN1_RxBuffer[8] = {0};
volatile uint32_t FDCAN1_error_sig_count =0;