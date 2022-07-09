#pragma once

#include "bq.h"
#include "bme280.h"
#include "ps.h"
//#include "modbus.h"

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
    UWB_PING        = 0x00,
    UWB_TEMPERATURE = 0x01,
    UWB_HUMIDITY    = 0x02,
    UWB_PRESSURE    = 0x03,
    UWB_LED_TOGGLE    = 0x04,
} uwb_modbus_register_t;

typedef struct {
    uwb_mode_t          mode;
    uwb_state_t         state;
    
    bq_context_t        bq;
    bme280_context_t    bme280;
    ps_context_t        ps;
    
    bool                water_sink;
} uwb_context_t;

extern uwb_context_t uwb;

void uwb_init();
void uwb_handle();

void modbus_init();
