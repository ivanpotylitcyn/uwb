
#ifndef ADS122C04_H
#define ADS122C04_H

#include "main.h"

// Single Conversion Timeout (millis)
// The maximum time we will wait for DRDY to go valid for a single conversion
#define ADS122C04_CONVERSION_TIMEOUT 75

#define ADS122C04_ADRESS 0x80

// Define 2/3/4-Wire, Temperature and Raw modes
#define ADS122C04_4WIRE_CH1_MODE         0x0
#define ADS122C04_4WIRE_CH2_MODE         0x1
#define ADS122C04_TEMPERATURE_MODE       0x2
#define ADS122C04_RAW_MODE               0x3
#define ADS122C04_4WIRE_CH1_HGAIN        0x4
#define ADS122C04_4WIRE_CH2_HGAIN        0x5

// ADS122C04 Table 16 in Datasheet
#define ADS122C04_RESET_CMD          0x06     //0000 011x      Reset
#define ADS122C04_START_CMD          0x08     //0000 100x      Start/Sync
#define ADS122C04_POWERDOWN_CMD      0x02     //0000 001x      PowerDown
#define ADS122C04_RDATA_CMD          0x10     //0001 xxxx      RDATA
#define ADS122C04_RREG_CMD           0x20     //0010 rrxx      Read REG rr= register address 00 to 11
#define ADS122C04_WREG_CMD           0x40     //0100 rrxx      Write REG rr= register address 00 to 11

#define ADS122C04_WRITE_CMD(reg)     (ADS122C04_WREG_CMD | (reg << 2))    //Shift is 2-bit in ADS122C04
#define ADS122C04_READ_CMD(reg)      (ADS122C04_RREG_CMD | (reg << 2))    //Shift is 2-bit in ADS122C04

// ADS122C04 Table 16 in Datasheet
#define ADS122C04_CONFIG_0_REG      0 // Configuration Register 0
#define ADS122C04_CONFIG_1_REG      1 // Configuration Register 1
#define ADS122C04_CONFIG_2_REG      2 // Configuration Register 2
#define ADS122C04_CONFIG_3_REG      3 // Configuration Register 3

// Unshifted register definitions
// The bit field register definitions will do the bit shifting

// Configuration Register 0
// ADS122C04 Table 19 in Datasheet


//МАСКИ
//1 рег
#define ADS_INP_MUX_MSK 0xF0
#define ADS_GAIM_MSK 0x0E
#define ADS_PGABYPASS_MSK 0x01

//2 рег
#define ADS_DR_MSK 0xE0
#define ADS_MODE_MSK 0x10
#define ADS_CM_MSK 0x08
#define ADS_VREF_MSK 0x06
#define ADS_TS_MSK 0x01

//3 рег
#define ADS_DRDY_MSK 0x01<<7
#define ADS_DCNT_MSK 0x01<<6
#define ADS_CRC_MSK 0x03<<4
#define ADS_BCS_MSK 0x01<<3
#define ADS_IDAC_MSK 0x07

//4 рег
#define ADS_I2MUX_MSK 0x07<<5
#define ADS_I1MUX_MSK 0x07<<2

// Input Multiplexer Configuration
#define ADS122C04_MUX_AIN0_AIN1     0x0
#define ADS122C04_MUX_AIN0_AIN2     0x1
#define ADS122C04_MUX_AIN0_AIN3     0x2
#define ADS122C04_MUX_AIN1_AIN0     0x3
#define ADS122C04_MUX_AIN1_AIN2     0x4
#define ADS122C04_MUX_AIN1_AIN3     0x5
#define ADS122C04_MUX_AIN2_AIN3     0x6
#define ADS122C04_MUX_AIN3_AIN2     0x7
#define ADS122C04_MUX_AIN0_AVSS     0x8
#define ADS122C04_MUX_AIN1_AVSS     0x9
#define ADS122C04_MUX_AIN2_AVSS     0xa
#define ADS122C04_MUX_AIN3_AVSS     0xb
#define ADS122C04_MUX_REFPmREFN     0xc
#define ADS122C04_MUX_AVDDmAVSS     0xd
#define ADS122C04_MUX_SHORTED       0xe

// Gain Configuration
#define ADS122C04_GAIN_1            0x0
#define ADS122C04_GAIN_2            0x1
#define ADS122C04_GAIN_4            0x2
#define ADS122C04_GAIN_8            0x3
#define ADS122C04_GAIN_16           0x4
#define ADS122C04_GAIN_32           0x5
#define ADS122C04_GAIN_64           0x6
#define ADS122C04_GAIN_128          0x7

// PGA Bypass
#define ADS122C04_PGA_DISABLED      0x0
#define ADS122C04_PGA_ENABLED       0x1

// Configuration Register 1
// ADS122C04 Table 19 in Datasheet

// Data Rate
// Turbo mode = Normal mode * 2 (Samples per Second)
// Normal mode
#define ADS122C04_DATA_RATE_20SPS   0x0
#define ADS122C04_DATA_RATE_45SPS   0x1
#define ADS122C04_DATA_RATE_90SPS   0x2
#define ADS122C04_DATA_RATE_175SPS  0x3
#define ADS122C04_DATA_RATE_330SPS  0x4
#define ADS122C04_DATA_RATE_600SPS  0x5
#define ADS122C04_DATA_RATE_1000SPS 0x6

// Operating Mode
#define ADS122C04_OP_MODE_NORMAL    0x0
#define ADS122C04_OP_MODE_TURBO     0x1

// Conversion Mode
#define ADS122C04_CONVERSION_MODE_SINGLE_SHOT   0x0
#define ADS122C04_CONVERSION_MODE_CONTINUOUS    0x1

// Voltage Reference Selection
#define ADS122C04_VREF_INTERNAL            0x0 //2.048V internal
#define ADS122C04_VREF_EXT_REF_PINS        0x1 //REFp and REFn external
#define ADS122C04_VREF_AVDD                0x2 //Analog Supply AVDD and AVSS

// Temperature Sensor Mode
#define ADS122C04_TEMP_SENSOR_OFF          0x0
#define ADS122C04_TEMP_SENSOR_ON           0x1

// Configuration Register 2
// ADS122C04 Table 22 in Datasheet

// Conversion Result Ready Flag (READ ONLY)

// Data Counter Enable
#define ADS122C04_DCNT_DISABLE             0x0
#define ADS122C04_DCNT_ENABLE              0x1

// Data Integrity Check Enable
#define ADS122C04_CRC_DISABLED             0x0
#define ADS122C04_CRC_INVERTED             0x1
#define ADS122C04_CRC_CRC16_ENABLED        0x2

// Burn-Out Current Source
#define ADS122C04_BURN_OUT_CURRENT_OFF     0x0
#define ADS122C04_BURN_OUT_CURRENT_ON      0x1

// IDAC Current Setting
#define ADS122C04_IDAC_CURRENT_OFF         0x0
#define ADS122C04_IDAC_CURRENT_10_UA       0x1
#define ADS122C04_IDAC_CURRENT_50_UA       0x2
#define ADS122C04_IDAC_CURRENT_100_UA      0x3
#define ADS122C04_IDAC_CURRENT_250_UA      0x4
#define ADS122C04_IDAC_CURRENT_500_UA      0x5
#define ADS122C04_IDAC_CURRENT_1000_UA     0x6
#define ADS122C04_IDAC_CURRENT_1500_UA     0x7

// Configuration Register 3
// ADS122C04 Table 23 in Datasheet

// IDAC1 Routing Configuration
#define ADS122C04_IDAC1_DISABLED           0x0
#define ADS122C04_IDAC1_AIN0               0x1
#define ADS122C04_IDAC1_AIN1               0x2
#define ADS122C04_IDAC1_AIN2               0x3
#define ADS122C04_IDAC1_AIN3               0x4
#define ADS122C04_IDAC1_REFP               0x5
#define ADS122C04_IDAC1_REFN               0x6

// IDAC2 Routing Configuration
#define ADS122C04_IDAC2_DISABLED           0x0
#define ADS122C04_IDAC2_AIN0               0x1
#define ADS122C04_IDAC2_AIN1               0x2
#define ADS122C04_IDAC2_AIN2               0x3
#define ADS122C04_IDAC2_AIN3               0x4
#define ADS122C04_IDAC2_REFP               0x5
#define ADS122C04_IDAC2_REFN               0x6

void ADS_SetInpMux(uint8_t mux_config);
void ads_start(void);
void ads122_init(void);

//Временные
//uint8_t ADS_ReadReg(uint8_t Reg);
//void ADS_SetGain(uint8_t gain_config);
//void ADS_SetPgaBypass(uint8_t PgaBypass_config);
//void ADS_SetDR(uint8_t mux_config);
//void ADS_SetMode(uint8_t gain_config);
//void ADS_SetCM(uint8_t CM_config);
//void ADS_SetVREF(uint8_t VREFF_config);
//void ADS_SetTS(uint8_t TS_config);
//void ADS_SetDRDY(uint8_t DRDY_config);
//void ADS_SetDCNT(uint8_t DCNT_config);
//void ADS_SetCRC(uint8_t CRC_config);
//void ADS_SetBCS(uint8_t BCS_config);
//void ADS_SetIDAC(uint8_t IDAC_config);
//void ADS_SetI2MUX(uint8_t I2MUX_config);
//void ADS_SetI1MUX(uint8_t I1MUX_config);

uint32_t ADS122C04_getConversionData(void);

#endif
