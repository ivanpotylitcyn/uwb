#pragma once

#include "bq.h"
#include "bme280.h"
#include "ps.h"

typedef enum {
    UWB_MODE_STARTUP = 0,
    UWB_MODE_SLEEP,
    UWB_MODE_COMMAND,
    UWB_MODE_EMERGENCY,
} uwb_mode_t;

typedef enum {
    UWB_ONBOARD = 0,
    UWB_SUBMERGED,
    UWB_ENMERGED,
} uwb_state_t;

typedef struct {
    uwb_mode_t          mode;
    uwb_state_t         state;
    
    bq_context_t        bq;
    bme280_context_t    bme280;
    ps_context_t        ps;
    
    bool                water_sink;
} uwb_context_t;

void uwb_init();
void uwb_handle();

void modbus_init();
void modbus_handle();