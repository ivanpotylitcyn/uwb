#pragma once

#include "bq.h"

#include <stdbool.h>

typedef enum {
    UWB_MODE_SLEEP = 0,
    UWB_MODE_COMMAND,
    UWB_MODE_EMERGENCY,
} uwb_mode_t;

typedef struct {
    bool power_6v0;
    bool power_3v3;
    bool power_5v0;
    bool power_12v0;
} uwb_power_stste_t;

typedef struct {
    bool        enabled;
    bool        acok;
    uint32_t    charge_current;
    uint32_t    charge_voltage;
    uint32_t    input_current;
} bq_context_t;

typedef struct {
    uwb_mode_t          mode;
    bq_context_t        bq;
    uwb_power_stste_t   power;
    bool                water_sink;
    
} uwb_context_t;

void uwb_init();
void uwb_handle();