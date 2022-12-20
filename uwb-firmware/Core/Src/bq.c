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

static int bq24735_config_charger(bq24735_context_t* bq) {
    int ret;
    uint16_t value;

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

    // Read back config

    ret = bq24735_read_word(BQ24735_CHARGE_CURRENT);
    if (ret < 0)
        return ret;
    bq->charge_current = ret;

    ret = bq24735_read_word(BQ24735_CHARGE_VOLTAGE);
    if (ret < 0)
        return ret;
    bq->charge_voltage = ret;

    ret = bq24735_read_word(BQ24735_INPUT_CURRENT);
    if (ret < 0)
        return ret;
    bq->input_current = ret;

    return BQ24735_OK;
}

static bool bq24735_charger_is_present() {
    int opt = bq24735_read_word(BQ24735_CHARGE_OPT);
    if (opt < 0)
        return false;

    return (opt & BQ24735_CHARGE_OPT_AC_PRESENT) ? true : false;
}

static bool bq24735_charger_is_charging() {
    int opt = bq24735_read_word(BQ24735_CHARGE_OPT);
    if (opt < 0)
        return false;

    if (!(opt & BQ24735_CHARGE_OPT_AC_PRESENT))
        false;

    return (opt & BQ24735_CHARGE_OPT_CHG_DISABLE) ? false : true;
}

static inline int bq24735_enable_charging(bq24735_context_t* bq) {
    int ret;

    ret = bq24735_config_charger(bq);
    if (ret < 0)
        return ret;

    return bq24735_update_word(BQ24735_CHARGE_OPT, BQ24735_CHARGE_OPT_CHG_DISABLE, 0);
}

static inline int bq24735_disable_charging(bq24735_context_t* bq) {
    return bq24735_update_word(BQ24735_CHARGE_OPT, BQ24735_CHARGE_OPT_CHG_DISABLE, BQ24735_CHARGE_OPT_CHG_DISABLE);
}


/* Public functions */

void bq24735_init(bq24735_context_t* bq) {
    bq->i2c_connected = false;

    // Check IDs
    int ret;

    ret = bq24735_read_word(BQ24735_MANUFACTURER_ID);
    if (ret < 0)
        return ret;
    if (ret != BQ24735_MANUFACTURER_ID_VALUE)
        return -BQ24735_MANUFACTURER_ID_MISMATCH;

    ret = bq24735_read_word(BQ24735_DEVICE_ID);
    if (ret < 0)
        return ret;
    if (ret != BQ24735_DEVICE_ID_VALUE)
        return -BQ24735_DEVICE_ID_MISMATCH;

    bq->i2c_connected = true;
}

void bq24735_handle(bq24735_context_t* bq)
{
	bq->acok = HAL_GPIO_ReadPin(ACOK_bat_GPIO_Port, ACOK_bat_Pin);

    if (bq24735_charger_is_present(bq)) {
        bq->charger_is_present = true;
        bq24735_enable_charging(bq);
    }
    else {
        bq->charger_is_present = false;
        bq24735_disable_charging(bq);
    }

    bq->charger_is_charging = bq24735_charger_is_charging();
}
