#pragma once

#include <stdint.h>
#include <stdbool.h>

typedef struct {
    bool        enabled;
    bool        acok;
    uint32_t    charge_current;
    uint32_t    charge_voltage;
    uint32_t    input_current;
} bq_context_t;

void bq_init(bq_context_t* bq);
void bq_handle(bq_context_t* bq);