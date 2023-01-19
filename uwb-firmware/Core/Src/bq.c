#include "bq.h"

extern I2C_HandleTypeDef hi2c1;


/* Private functions */

static int bq24735_write_word(uint8_t reg, uint16_t value) {
    HAL_StatusTypeDef status = HAL_I2C_Mem_Write(&hi2c1, BQ24735_SMBUS_ADDR, reg, I2C_MEMADD_SIZE_8BIT, (uint8_t*)&value, 2, 0x10000);

    return status == HAL_OK ? HAL_OK : -BQ24735_WRITE_ERR;
}

static int bq24735_read_word(uint8_t reg) {
    uint16_t value;
    HAL_StatusTypeDef status = HAL_I2C_Mem_Read(&hi2c1, BQ24735_SMBUS_ADDR, reg, I2C_MEMADD_SIZE_8BIT, (uint8_t*)&value, 2, 0x10000);

    return status == HAL_OK ? value : -BQ24735_READ_ERR;
}

static int bq24735_update_word(uint8_t reg, uint16_t mask, uint16_t value) {
    unsigned int tmp;
    int ret;

    ret = bq24735_read_word(reg);

    if (ret < 0) return ret;

    tmp = ret & ~mask;
    tmp |= value & mask;

    return bq24735_write_word(reg, tmp);
}


/* Public functions */

bool bq24735_is_connected() {
    int ret;

    ret = bq24735_read_word(BQ24735_MANUFACTURER_ID);
    if (ret < 0 || ret != BQ24735_MANUFACTURER_ID_VALUE)
        return false;

    ret = bq24735_read_word(BQ24735_DEVICE_ID);
    if (ret < 0 || ret != BQ24735_DEVICE_ID_VALUE)
        return false;

    return true;
}

bool bq24735_ac_is_present() {
    int opt = bq24735_read_word(BQ24735_CHARGE_OPT);
    return (opt >= 0) && (opt & BQ24735_CHARGE_OPT_AC_PRESENT) ? true : false;
}

int bq24735_enable_charging(bq24735_context_t* bq) {
    int ret;
    uint16_t value;

    bq->charge_current = bq->charge_current == 0 ? 1024 : bq->charge_current;
    bq->charge_voltage = bq->charge_voltage == 0 ? 4096 : bq->charge_voltage;
    bq->input_current  = bq->input_current  == 0 ? 1024 : bq->input_current;

    // Setup config

    ret = bq24735_write_word(BQ24735_CHARGE_CURRENT, bq->charge_current & BQ24735_CHARGE_CURRENT_MASK);
    if (ret < 0)
        return ret;

    ret = bq24735_write_word(BQ24735_CHARGE_VOLTAGE, bq->charge_voltage & BQ24735_CHARGE_VOLTAGE_MASK);
    if (ret < 0)
        return ret;

    ret = bq24735_write_word(BQ24735_INPUT_CURRENT, bq->input_current & BQ24735_INPUT_CURRENT_MASK);
    if (ret < 0)
        return ret;

    ret = bq24735_update_word(BQ24735_CHARGE_OPT, BQ24735_CHARGE_OPT_CHG_DISABLE, 0);
    if (ret < 0)
    	return ret;

    return BQ24735_OK;
}

uint16_t bq24735_read_charge_current() {
    return bq24735_read_word(BQ24735_CHARGE_CURRENT);
}

uint16_t bq24735_read_charge_voltage() {
    return bq24735_read_word(BQ24735_CHARGE_VOLTAGE);
}

uint16_t bq24735_read_input_current() {
    return bq24735_read_word(BQ24735_INPUT_CURRENT);
}

uint16_t bq24735_read_charge_option() {
    return bq24735_read_word(BQ24735_CHARGE_OPT);
}

int bq24735_write_charge_option(uint16_t charge_option) {
	return bq24735_write_word(BQ24735_CHARGE_OPT, charge_option);
}
