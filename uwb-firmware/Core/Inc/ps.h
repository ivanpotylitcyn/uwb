#pragma once

#include <stdint.h>

typedef struct {
    uint32_t pressure;
} ps_context_t;

void ps_read(ps_context_t* ps);
