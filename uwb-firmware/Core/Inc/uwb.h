#pragma once

#include "bq.h"
#include "bme280.h"
#include "ps.h"
//#include "modbus.h"

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
    UWB_PRESSURE_ADS    = 0x04,
    UWB_PRESS_TRIG_1    = 0x05,
    UWB_PRESS_TRIG_2    = 0x06,
    UWB_BITRATE_RS485_H = 0x07,
    UWB_BITRATE_RS485_L = 0x08,
    UWB_MASK_LED_H1     = 0x09,
    UWB_MASK_LED_L1     = 0x0A,
    UWB_MASK_LED_H0     = 0x0B,
    UWB_MASK_LED_L0     = 0x0C,
    UWB_LEDRATE         = 0x0D,
    UWB_LED_TOGGLE      = 0x0E,
	UWB_LED_BLINK	    = 0x0F,
	UWB_WATER_SINK      = 0x10,
	UWB_RESET           = 0x11,
	UWB_RESTART         = 0x12,
	UWB_SAVE_FLSH       = 0x13,
} uwb_modbus_register_t;

typedef struct {

    uint64_t            led_mask;
    uint16_t            press_rtig1;
    uint16_t            press_rtig2;
    uint32_t            bitrate_rs485;
    uint16_t            ledrate;

    uint16_t            led_toggle;
    uint16_t            led_blink;

    uint16_t            ping;

    uwb_mode_t          mode;
    uwb_state_t         state;

    bq_context_t        bq;
    bme280_context_t    bme280;
    ps_context_t        ps;
    
    uint16_t            water_sink;
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

bool uwb_enable_led(bool enable);
bool uwb_enable_led_blink(bool enable);

