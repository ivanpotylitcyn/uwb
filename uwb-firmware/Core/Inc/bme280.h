#pragma once

#include <stdint.h>

typedef struct {
    uint32_t humidity;
    uint32_t temperature;
} bme280_context_t;

void bme280_read(bme280_context_t* bme);