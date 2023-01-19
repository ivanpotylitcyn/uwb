#pragma once

#include "bq.h"
#include "bme280.h"
#include "ps.h"
#include "adc.h"
//#include "modbus.h"

#define UWB_RARE_DELAY              10000
#define UWB_DENSE_DELAY             1000

#define SETTINGS_ADDRESS            0x08009000

#define UWB_SUBMERGED_THRESHOLD     20000 // Под водой
#define UWB_ENMERGED_THRESHOLD      25000 // Всплыли
#define BITRATE_RS485               256000
#define LEDMASK                     0xAAAAAAAAAAAAAAAA
#define UWB_BLINK_DELAY             500

#define REF_CLK    ((uint32_t)16000000U)

typedef enum {
    UWB_MODE_SLEEP = 0,
    UWB_MODE_COMMAND,
    UWB_MODE_EMERGENCY,
} uwb_mode_t;

typedef enum {
    UWB_ONBOARD = 0,
    UWB_SUBMERGED,
    UWB_ENMERGED,
} uwb_state_t;

#define UWB_MODBUS_ID 0x42

typedef enum {
    UWB_PING            = 0x00,
    UWB_TEMPERATURE     = 0x01,
    UWB_HUMIDITY        = 0x02,
    UWB_PRESSURE        = 0x03,
    UWB_PRESSURE_ADS_L  = 0x04,
    UWB_PRESSURE_ADS_H  = 0x05,
    UWB_PRESS_TRIG_1_H  = 0x06,
    UWB_PRESS_TRIG_1_L  = 0x07,
    UWB_PRESS_TRIG_2_H  = 0x08,
    UWB_PRESS_TRIG_2_L  = 0x09,
    UWB_BITRATE_RS485_H = 0x0A,
    UWB_BITRATE_RS485_L = 0x0B,
    UWB_MASK_LED_H1     = 0x0C,
    UWB_MASK_LED_L1     = 0x0D,
    UWB_MASK_LED_H0     = 0x0E,
    UWB_MASK_LED_L0     = 0x0F,
    UWB_LEDRATE         = 0x10,
    UWB_LED_TOGGLE      = 0x11,
    UWB_LED_BLINK       = 0x12,
    UWB_WATER_SINK      = 0x13,
    UWB_CHARGE_CURRENT  = 0x14,
    UWB_CHARGE_VOLTAGE  = 0x15,
    UWB_INPUT_CURRENT   = 0x16,
    UWB_CHARGE_OPTION   = 0x17,
	UWB_POWER_PERCENT   = 0x18,
	UWB_IOUT_MV   		= 0x19,
    UWB_RESET           = 0x1A,
    UWB_RESTART         = 0x1B,
    UWB_SAVE_FLSH       = 0x1C,
	UWB_SET_DEFAULTS    = 0x1D,
} uwb_modbus_register_t;

typedef struct {

    uint64_t            led_mask;      // 8
    uint32_t            press_rtig1;   // 4
    uint32_t            press_rtig2;   // 4
    uint32_t            bitrate_rs485; // 4
    uint16_t            ledrate;       // 2
    uint16_t            chekCRC16;     // 2

    uint16_t            ping;

    uint16_t            led_toggle;
    uint16_t            led_blink;

    uwb_mode_t          mode;
    uwb_state_t         state;

    bq24735_context_t   bq;
    bme280_context_t    bme280;
    ps_context_t        ps;

    uint16_t            water_sink;

    uint16_t 			charge_current;
    uint16_t 			charge_voltage;
    uint16_t 			input_current;
    uint16_t 			charge_option;

    uint16_t            power_percent;
    uint16_t            iout_mv;
} uwb_context_t;

extern uwb_context_t uwb;

void uwb_init();
void uwb_handle();
void write_flash();
void switch_RS485();
void read_flash();
void TM_CRC_INIT();

void modbus_init();
void rs485_transmit(uint8_t* buff_uart, uint16_t cnt);

bool uwb_enable_led(uint16_t enable);
bool uwb_enable_led_blink(uint16_t enable);

void charge_handle();
