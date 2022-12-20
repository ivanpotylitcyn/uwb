#pragma once

#include <stdint.h>
#include <stdbool.h>
#include <main.h>

#define BQ24735_SMBUS_ADDR 0x12

typedef struct {
    uint16_t charge_current;
    uint16_t charge_voltage;
    uint16_t input_current;

    bool     i2c_connected;
    bool     charger_is_present;
    bool     charger_is_charging;

    bool	 acok;
} bq24735_context_t;

typedef enum {
	BQ24735_OK = 0,
	BQ24735_INVALID_I2C,
	BQ24735_READ_ERR,
	BQ24735_WRITE_ERR,
	BQ24735_MANUFACTURER_ID_MISMATCH,
	BQ24735_DEVICE_ID_MISMATCH,
} bq24735_error_t;

/* BQ24735 available commands and their respective masks */
#define BQ24735_CHARGE_OPT          0x12
#define BQ24735_CHARGE_CURRENT      0x14
#define BQ24735_CHARGE_CURRENT_MASK 0x1fc0
#define BQ24735_CHARGE_VOLTAGE      0x15
#define BQ24735_CHARGE_VOLTAGE_MASK 0x7ff0
#define BQ24735_INPUT_CURRENT       0x3f
#define BQ24735_INPUT_CURRENT_MASK  0x1f80
#define BQ24735_MANUFACTURER_ID     0xfe
#define BQ24735_DEVICE_ID           0xff

/* ChargeOptions bits of interest */
#define BQ24735_CHARGE_OPT_CHG_DISABLE  (1 << 0)
#define BQ24735_CHARGE_OPT_AC_PRESENT   (1 << 4)

/* BQ24735 ManufacturerID and DeviceID values */
#define BQ24735_MANUFACTURER_ID_VALUE     0x0040
#define BQ24735_DEVICE_ID_VALUE           0x000B

void bq24735_init(bq24735_context_t* bq);
void bq24735_handle(bq24735_context_t* bq);
