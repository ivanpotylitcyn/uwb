#include "ads1220.h"

extern I2C_HandleTypeDef hi2c2;

#define  VREF1 5

static uint8_t iic_recvbuf[3];

void Error1(void) {
    // ошибка
}

//------------------------------------------------
static void I2Cx_WriteData(uint16_t Addr, uint8_t Reg, uint8_t Value, uint8_t size) {
    HAL_StatusTypeDef status = HAL_OK;
    status = HAL_I2C_Mem_Write(&hi2c2, Addr, (uint16_t)Reg, I2C_MEMADD_SIZE_8BIT, &Value, size, ADS122C04_CONVERSION_TIMEOUT);
    if(status != HAL_OK) Error1();
}
//------------------------------------------------
uint8_t I2Cx_ReadData(uint16_t Addr, uint8_t Reg, uint8_t size) {
    HAL_StatusTypeDef status = HAL_OK;
    uint8_t value = 0;
    status = HAL_I2C_Mem_Read(&hi2c2, Addr, Reg, I2C_MEMADD_SIZE_8BIT, &value, size, ADS122C04_CONVERSION_TIMEOUT);
    if(status != HAL_OK) Error1();
    return value;
}
//------------------------------------------------
void ADS_WriteReg(uint8_t Reg, uint8_t Value) {
    I2Cx_WriteData(ADS122C04_ADRESS, Reg, Value, 1);
}
//------------------------------------------------
uint8_t ADS_ReadReg(uint8_t Reg) {
    uint8_t res = I2Cx_ReadData(ADS122C04_ADRESS, Reg, 1);
    return res;
}
//------------------------------------------------
void ADS_SetInpMux(uint8_t mux_config) {
    uint8_t reg;
    reg = ADS_ReadReg(ADS122C04_READ_CMD(0)) & ~ADS_INP_MUX_MSK;
    reg |= (mux_config<<4) & ADS_INP_MUX_MSK;
    ADS_WriteReg(ADS122C04_WRITE_CMD(0), reg);
}
//------------------------------------------------
void ADS_SetGain(uint8_t gain_config) {
    uint8_t reg;
    reg = ADS_ReadReg(ADS122C04_READ_CMD(0)) & ~ADS_GAIM_MSK;
    reg |= (gain_config<<1) & ADS_GAIM_MSK;
    ADS_WriteReg(ADS122C04_WRITE_CMD(0), reg);
}
//------------------------------------------------
void ADS_SetPgaBypass(uint8_t PgaBypass_config) {
    uint8_t reg;
    reg = ADS_ReadReg(ADS122C04_READ_CMD(0)) & ~ADS_PGABYPASS_MSK;
    reg |= (PgaBypass_config) & ADS_PGABYPASS_MSK;
    ADS_WriteReg(ADS122C04_WRITE_CMD(0), reg);
}
//------------------------------------------------

//------------------------------------------------
// Второй регистр
void ADS_SetDR(uint8_t mux_config) {
    uint8_t reg;
    reg = ADS_ReadReg(ADS122C04_READ_CMD(1)) & ~ADS_DR_MSK;
    reg |= mux_config & ADS_DR_MSK;
    ADS_WriteReg(ADS122C04_WRITE_CMD(1), reg);
}
//------------------------------------------------
void ADS_SetMode(uint8_t gain_config) {
    uint8_t reg;
    reg = ADS_ReadReg(ADS122C04_READ_CMD(1)) & ~ADS_MODE_MSK;
    reg |= gain_config & ADS_MODE_MSK;
    ADS_WriteReg(ADS122C04_WRITE_CMD(1), reg);
}
//------------------------------------------------
void ADS_SetCM(uint8_t CM_config) {
    uint8_t reg;
    reg = ADS_ReadReg(ADS122C04_READ_CMD(1)) & ~ADS_CM_MSK;
    reg |= CM_config & ADS_CM_MSK;
    ADS_WriteReg(ADS122C04_WRITE_CMD(1), reg);
}
//------------------------------------------------
void ADS_SetVREF(uint8_t VREFF_config) {
    uint8_t reg;
    reg = ADS_ReadReg(ADS122C04_READ_CMD(1)) & ~ADS_VREF_MSK;
    reg |= VREFF_config & ADS_VREF_MSK;
    ADS_WriteReg(ADS122C04_WRITE_CMD(1), reg);
}
//------------------------------------------------
void ADS_SetTS(uint8_t TS_config) {
    uint8_t reg;
    reg = ADS_ReadReg(ADS122C04_READ_CMD(1)) & ~ADS_TS_MSK;
    reg |= TS_config & ADS_TS_MSK;
    ADS_WriteReg(ADS122C04_WRITE_CMD(1), reg);
}
//------------------------------------------------

//------------------------------------------------
// Третий регистр
void ADS_SetDRDY(uint8_t DRDY_config) {
    uint8_t reg;
    reg = ADS_ReadReg(ADS122C04_READ_CMD(2)) & ~(ADS_DRDY_MSK);
    reg |= DRDY_config & ADS_DRDY_MSK;
    ADS_WriteReg(ADS122C04_WRITE_CMD(2), reg);
}
//------------------------------------------------
void ADS_SetDCNT(uint8_t DCNT_config) {
    uint8_t reg;
    reg = ADS_ReadReg(ADS122C04_READ_CMD(2)) & ~(ADS_DCNT_MSK);
    reg |= DCNT_config & ADS_DCNT_MSK;
    ADS_WriteReg(ADS122C04_WRITE_CMD(2), reg);
}
//------------------------------------------------
void ADS_SetCRC(uint8_t CRC_config) {
    uint8_t reg;
    reg = ADS_ReadReg(ADS122C04_READ_CMD(2)) & ~(ADS_CRC_MSK);
    reg |= CRC_config & ADS_CRC_MSK;
    ADS_WriteReg(ADS122C04_WRITE_CMD(2), reg);
}
//------------------------------------------------
void ADS_SetBCS(uint8_t BCS_config) {
    uint8_t reg;
    reg = ADS_ReadReg(ADS122C04_READ_CMD(2)) & ~(ADS_BCS_MSK);
    reg |= BCS_config & ADS_BCS_MSK;
    ADS_WriteReg(ADS122C04_WRITE_CMD(2), reg);
}
//------------------------------------------------
void ADS_SetIDAC(uint8_t IDAC_config) {
    uint8_t reg;
    reg = ADS_ReadReg(ADS122C04_READ_CMD(2)) & ~ADS_IDAC_MSK;
    reg |= IDAC_config & ADS_IDAC_MSK;
    ADS_WriteReg(ADS122C04_WRITE_CMD(2), reg);
}
//------------------------------------------------

//------------------------------------------------
// Четвертый регистр
void ADS_SetI2MUX(uint8_t I2MUX_config) {
    uint8_t reg;
    reg = ADS_ReadReg(ADS122C04_READ_CMD(3)) & ~(ADS_I2MUX_MSK);
    reg |= I2MUX_config & ADS_I2MUX_MSK;
    ADS_WriteReg(ADS122C04_WRITE_CMD(3), reg);
}
//------------------------------------------------
void ADS_SetI1MUX(uint8_t I1MUX_config) {
    uint8_t reg;
    reg = ADS_ReadReg(ADS122C04_READ_CMD(3)) & ~(ADS_I1MUX_MSK);
    reg |= I1MUX_config & ADS_I1MUX_MSK;
    ADS_WriteReg(ADS122C04_WRITE_CMD(3), reg);
}
//------------------------------------------------
void ads_reset(void) {
  uint8_t commad = ADS122C04_RESET_CMD;
  HAL_I2C_Master_Transmit(&hi2c2, 0x80, &commad, 1, 0x10000);
}
//------------------------------------------------
void ads_start(void) {
  uint8_t commad = ADS122C04_START_CMD;
  HAL_I2C_Master_Transmit(&hi2c2, 0x80, &commad, 1, 0x10000);
}
//------------------------------------------------
void ads_powerdown(void) {
  uint8_t commad = ADS122C04_POWERDOWN_CMD;
  HAL_I2C_Master_Transmit(&hi2c2, 0x80, &commad, 1, 0x10000);
}
//------------------------------------------------

void ADS122_init(void) {

  HAL_GPIO_WritePin(nRESET_MD_GPIO_Port, nRESET_MD_Pin, GPIO_PIN_SET);

  HAL_Delay(1000);

  ads_reset();

  HAL_Delay(1000);

  ADS_SetInpMux(ADS122C04_MUX_AIN1_AIN2);
  ADS_SetGain(ADS122C04_GAIN_1);
  ADS_SetPgaBypass(ADS122C04_PGA_ENABLED);
  ADS_SetDR(ADS122C04_DATA_RATE_20SPS);
  ADS_SetMode(ADS122C04_OP_MODE_NORMAL);
  ADS_SetCM(ADS122C04_CONVERSION_MODE_SINGLE_SHOT);
  ADS_SetVREF(ADS122C04_VREF_EXT_REF_PINS);
  ADS_SetTS(ADS122C04_TEMP_SENSOR_OFF);
  ADS_SetDCNT(ADS122C04_DCNT_DISABLE);
  ADS_SetCRC(ADS122C04_CRC_DISABLED);
  ADS_SetBCS(ADS122C04_BURN_OUT_CURRENT_OFF);
  ADS_SetIDAC(ADS122C04_IDAC_CURRENT_OFF);
  ADS_SetI2MUX(ADS122C04_IDAC1_DISABLED);
  ADS_SetI1MUX(ADS122C04_IDAC2_DISABLED);
}

uint32_t ADS122C04_getConversionData(void) {

    uint32_t conversionData = 0;

    HAL_I2C_Mem_Read(&hi2c2, ADS122C04_ADRESS, ADS122C04_RDATA_CMD, I2C_MEMADD_SIZE_8BIT, iic_recvbuf, 3, ADS122C04_CONVERSION_TIMEOUT);

    conversionData = ((uint32_t)iic_recvbuf[2]) | ((uint32_t)iic_recvbuf[1]<<8) | ((uint32_t)iic_recvbuf[0]<<16);

    return (conversionData << 0);
}
