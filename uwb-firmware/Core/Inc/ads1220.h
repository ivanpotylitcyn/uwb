/*
  This is a library written for the TI ADS122C04
  24-Bit 4-Channel 2-kSPS Delta-Sigma ADC With I2C Interface
  It allows you to measure temperature very accurately using a
  Platinum Resistance Thermometer
  SparkFun sells these at its website: www.sparkfun.com
  Do you like this library? Help support SparkFun. Buy a board!
  Written by: Paul Clark (PaulZC)
  Date: May 4th 2020
  Based on the TI datasheet:
  https://www.ti.com/product/ADS122C04
  https://www.ti.com/lit/ds/symlink/ads122c04.pdf
  Using the example code from the "High Precision Temperature Measurement
  for Heat and Cold Meters Reference Design" (TIDA-01526) for reference:
  http://www.ti.com/tool/TIDA-01526
  http://www.ti.com/lit/zip/tidcee5
  The MIT License (MIT)
  Copyright (c) 2020 SparkFun Electronics
  Permission is hereby granted, free of charge, to any person obtaining a copy of this software and
  associated documentation files (the "Software"), to deal in the Software without restriction,
  including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense,
  and/or sell copies of the Software, and to permit persons to whom the Software is furnished to
  do so, subject to the following conditions:
  The above copyright notice and this permission notice shall be included in all copies or substantial
  portions of the Software.
  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT
  NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
  IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
  WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE
  SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

#ifndef ADS122C04_H
#define ADS122C04_H

#include <stdint.h>
#include <stdbool.h>
#include <main.h>

// Single Conversion Timeout (millis)
// The maximum time we will wait for DRDY to go valid for a single conversion
#define ADS122C04_CONVERSION_TIMEOUT 75

#define ADS122C04_ADRESS 0x80

// Define 2/3/4-Wire, Temperature and Raw modes
#define ADS122C04_4WIRE_CH1_MODE         0x0
#define ADS122C04_4WIRE_CH2_MODE         0x1
#define ADS122C04_TEMPERATURE_MODE       0x2
#define ADS122C04_RAW_MODE                           0x3
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

bool ADS122C04_sendCommand(uint8_t command);
void ADS_SetInpMux(uint8_t mux_config);

void ads122_init();

//Временные
uint8_t ADS_ReadReg(uint8_t Reg);
void ADS_SetGain(uint8_t gain_config);
void ADS_SetPgaBypass(uint8_t PgaBypass_config);
void ADS_SetDR(uint8_t mux_config);
void ADS_SetMode(uint8_t gain_config);
void ADS_SetCM(uint8_t CM_config);
void ADS_SetVREF(uint8_t VREFF_config);
void ADS_SetTS(uint8_t TS_config);
void ADS_SetDRDY(uint8_t DRDY_config);
void ADS_SetDCNT(uint8_t DCNT_config);
void ADS_SetCRC(uint8_t CRC_config);
void ADS_SetBCS(uint8_t BCS_config);
void ADS_SetIDAC(uint8_t IDAC_config);
void ADS_SetI2MUX(uint8_t I2MUX_config);
void ADS_SetI1MUX(uint8_t I1MUX_config);

float ADS122C04_getConversionData(void);

#endif


///* --COPYRIGHT--,BSD
// * Copyright (c) 2018, Texas Instruments Incorporated
// * All rights reserved.
// *
// * Redistribution and use in source and binary forms, with or without
// * modification, are permitted provided that the following conditions
// * are met:
// *
// * *  Redistributions of source code must retain the above copyright
// *    notice, this list of conditions and the following disclaimer.
// *
// * *  Redistributions in binary form must reproduce the above copyright
// *    notice, this list of conditions and the following disclaimer in the
// *    documentation and/or other materials provided with the distribution.
// *
// * *  Neither the name of Texas Instruments Incorporated nor the names of
// *    its contributors may be used to endorse or promote products derived
// *    from this software without specific prior written permission.
// *
// * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
// * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
// * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
// * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
// * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
// * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
// * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
// * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
// * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
// * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
// * --/COPYRIGHT--*/
///*
// * ADS1220.h
// *
// */
///*******************************************************************************
//// ADS1220 Header File for Demo Functions
////
////
//// Description: Use of the MSP430F5528 USCI A0 peripheral for setting up and
////    communicating to the ADS1220 24-bit ADC.
////
////
////
////                 MSP430x552x
////             ------------------
////         /|\|                  |
////          | |                  |
////          --|RST           P3.4|<-- MISO (DOUT)
////            |                  |
////            |              P3.3|--> MOSI (DIN)
////            |                  |
////            |              P2.7|--> SCLK
////            |                  |
////            |              P2.6|<-- INT (DRDY)
////            |                  |
////            |              P1.2|--> CS
////
////******************************************************************************/
//#ifndef ADS1220_H_
//#define ADS1220_H_
///* Definition of GPIO Port Bits Used for Communication */
///* P1.2 */
//#define ADS1220_CS      	0x04
///* P3.3  */
//#define ADS1220_DIN     	0x08
///* P3.4 */
//#define ADS1220_DOUT    	0x10
///* P2.6 */
//#define ADS1220_DRDY    	0x40
///* P2.7 */
//#define ADS1220_SCLK    	0x80
///* Error Return Values */
//#define ADS1220_NO_ERROR           0
//#define ADS1220_ERROR				-1
///* Command Definitions */
//#define ADS1220_CMD_RDATA    	0x10
//#define ADS1220_CMD_RREG     	0x20
//#define ADS1220_CMD_WREG     	0x40
//#define ADS1220_CMD_SYNC    	0x08
//#define ADS1220_CMD_SHUTDOWN    0x02
//#define ADS1220_CMD_RESET    	0x06
///* ADS1220 Register Definitions */
//#define ADS1220_0_REGISTER   	0x00
//#define ADS1220_1_REGISTER     	0x01
//#define ADS1220_2_REGISTER     	0x02
//#define ADS1220_3_REGISTER    	0x03
///* ADS1220 Register 0 Definition */
///*   Bit 7   |   Bit 6   |   Bit 5   |   Bit 4   |   Bit 3   |   Bit 2   |   Bit 1   |   Bit 0
////--------------------------------------------------------------------------------------------
////                     MUX [3:0]                 |             GAIN[2:0]             | PGA_BYPASS
//*/
///* Define MUX */
//#define ADS1220_MUX_0_1   	0x00
//#define ADS1220_MUX_0_2   	0x10
//#define ADS1220_MUX_0_3   	0x20
//#define ADS1220_MUX_1_2   	0x30
//#define ADS1220_MUX_1_3   	0x40
//#define ADS1220_MUX_2_3   	0x50
//#define ADS1220_MUX_1_0   	0x60
//#define ADS1220_MUX_3_2   	0x70
//#define ADS1220_MUX_0_G		0x80
//#define ADS1220_MUX_1_G   	0x90
//#define ADS1220_MUX_2_G   	0xa0
//#define ADS1220_MUX_3_G   	0xb0
//#define ADS1220_MUX_EX_VREF 0xc0
//#define ADS1220_MUX_AVDD   	0xd0
//#define ADS1220_MUX_DIV2   	0xe0
///* Define GAIN */
//#define ADS1220_GAIN_1      0x00
//#define ADS1220_GAIN_2      0x02
//#define ADS1220_GAIN_4      0x04
//#define ADS1220_GAIN_8      0x06
//#define ADS1220_GAIN_16     0x08
//#define ADS1220_GAIN_32     0x0a
//#define ADS1220_GAIN_64     0x0c
//#define ADS1220_GAIN_128    0x0e
///* Define PGA_BYPASS */
//#define ADS1220_PGA_BYPASS 	0x01
///* ADS1220 Register 1 Definition */
///*   Bit 7   |   Bit 6   |   Bit 5   |   Bit 4   |   Bit 3   |   Bit 2   |   Bit 1   |   Bit 0
////--------------------------------------------------------------------------------------------
////                DR[2:0]            |      MODE[1:0]        |     CM    |     TS    |    BCS
//*/
///* Define DR (data rate) */
//#define ADS1220_DR_20		0x00
//#define ADS1220_DR_45		0x20
//#define ADS1220_DR_90		0x40
//#define ADS1220_DR_175		0x60
//#define ADS1220_DR_330		0x80
//#define ADS1220_DR_600		0xa0
//#define ADS1220_DR_1000		0xc0
///* Define MODE of Operation */
//#define ADS1220_MODE_NORMAL 0x00
//#define ADS1220_MODE_DUTY	0x08
//#define ADS1220_MODE_TURBO 	0x10
//#define ADS1220_MODE_DCT	0x18
///* Define CM (conversion mode) */
//#define ADS1220_CC			0x04
///* Define TS (temperature sensor) */
//#define ADS1220_TEMP_SENSOR	0x02
///* Define BCS (burnout current source) */
//#define ADS1220_BCS			0x01
///* ADS1220 Register 2 Definition */
///*   Bit 7   |   Bit 6   |   Bit 5   |   Bit 4   |   Bit 3   |   Bit 2   |   Bit 1   |   Bit 0
////--------------------------------------------------------------------------------------------
////         VREF[1:0]     |        50/60[1:0]     |    PSW    |             IDAC[2:0]
//*/
///* Define VREF */
//#define ADS1220_VREF_INT	0x00
//#define ADS1220_VREF_EX_DED	0x40
//#define ADS1220_VREF_EX_AIN	0x80
//#define ADS1220_VREF_SUPPLY	0xc0
///* Define 50/60 (filter response) */
//#define ADS1220_REJECT_OFF	0x00
//#define ADS1220_REJECT_BOTH	0x10
//#define ADS1220_REJECT_50	0x20
//#define ADS1220_REJECT_60	0x30
///* Define PSW (low side power switch) */
//#define ADS1220_PSW_SW		0x08
///* Define IDAC (IDAC current) */
//#define ADS1220_IDAC_OFF	0x00
//#define ADS1220_IDAC_10		0x01
//#define ADS1220_IDAC_50		0x02
//#define ADS1220_IDAC_100	0x03
//#define ADS1220_IDAC_250	0x04
//#define ADS1220_IDAC_500	0x05
//#define ADS1220_IDAC_1000	0x06
//#define ADS1220_IDAC_2000	0x07
///* ADS1220 Register 3 Definition */
///*   Bit 7   |   Bit 6   |   Bit 5   |   Bit 4   |   Bit 3   |   Bit 2   |   Bit 1   |   Bit 0
////--------------------------------------------------------------------------------------------
////               I1MUX[2:0]          |               I2MUX[2:0]          |   DRDYM   | RESERVED
//*/
///* Define I1MUX (current routing) */
//#define ADS1220_IDAC1_OFF	0x00
//#define ADS1220_IDAC1_AIN0	0x20
//#define ADS1220_IDAC1_AIN1	0x40
//#define ADS1220_IDAC1_AIN2	0x60
//#define ADS1220_IDAC1_AIN3	0x80
//#define ADS1220_IDAC1_REFP0	0xa0
//#define ADS1220_IDAC1_REFN0	0xc0
///* Define I2MUX (current routing) */
//#define ADS1220_IDAC2_OFF	0x00
//#define ADS1220_IDAC2_AIN0	0x04
//#define ADS1220_IDAC2_AIN1	0x08
//#define ADS1220_IDAC2_AIN2	0x0c
//#define ADS1220_IDAC2_AIN3	0x10
//#define ADS1220_IDAC2_REFP0	0x14
//#define ADS1220_IDAC2_REFN0	0x18
///* define DRDYM (DOUT/DRDY behaviour) */
//#define ADS1220_DRDY_MODE	0x02
///* Low Level ADS1220 Device Functions */
//void ADS1220Init(void);							/* Device initialization */
//int ADS1220WaitForDataReady(int Timeout);		/* DRDY polling */
//void ADS1220AssertCS(int fAssert);				/* Assert/deassert CS */
//void ADS1220SendByte(unsigned char cData );		/* Send byte to the ADS1220 */
//unsigned char ADS1220ReceiveByte(void);			/* Receive byte from the ADS1220 */
///* ADS1220 Higher Level Functions */
//long ADS1220ReadData(void);						/* Read the data results */
//void ADS1220ReadRegister(int StartAddress, int NumRegs, unsigned * pData);	/* Read the register(s) */
//void ADS1220WriteRegister(int StartAddress, int NumRegs, unsigned * pData); /* Write the register(s) */
//void ADS1220SendResetCommand(void);				/* Send a device Reset Command */
//void ADS1220SendStartCommand(void);				/* Send a Start/SYNC command */
//void ADS1220SendShutdownCommand(void);			/* Place the device in powerdown mode */
///* Register Set Value Commands */
//void ADS1220Config(void);
//int ADS1220SetChannel(int Mux);
//int ADS1220SetGain(int Gain);
//int ADS1220SetPGABypass(int Bypass);
//int ADS1220SetDataRate(int DataRate);
//int ADS1220SetClockMode(int ClockMode);
//int ADS1220SetPowerDown(int PowerDown);
//int ADS1220SetTemperatureMode(int TempMode);
//int ADS1220SetBurnOutSource(int BurnOut);
//int ADS1220SetVoltageReference(int VoltageRef);
//int ADS1220Set50_60Rejection(int Rejection);
//int ADS1220SetLowSidePowerSwitch(int PowerSwitch);
//int ADS1220SetCurrentDACOutput(int CurrentOutput);
//int ADS1220SetIDACRouting(int IDACRoute);
//int ADS1220SetDRDYMode(int DRDYMode);
///* Register Get Value Commands */
//int ADS1220GetChannel(void);
//int ADS1220GetGain(void);
//int ADS1220GetPGABypass(void);
//int ADS1220GetDataRate(void);
//int ADS1220GetClockMode(void);
//int ADS1220GetPowerDown(void);
//int ADS1220GetTemperatureMode(void);
//int ADS1220GetBurnOutSource(void);
//int ADS1220GetVoltageReference(void);
//int ADS1220Get50_60Rejection(void);
//int ADS1220GetLowSidePowerSwitch(void);
//int ADS1220GetCurrentDACOutput(void);
//int ADS1220GetIDACRouting(int WhichOne);
//int ADS1220GetDRDYMode(void);
///* Useful Functions within Main Program for Setting Register Contents
//*
//*  	These functions show the programming flow based on the header definitions.
//*  	The calls are not made within the demo example, but could easily be used by calling the function
//*  		defined within the program to complete a fully useful program.
//*	Similar function calls were made in the firmware design for the ADS1220EVM.
//*
//*  The following function calls use ASCII data sent from a COM port to control settings
//*	on the ADS1220.  The data is reconstructed from ASCII and then combined with the
//*	register contents to save as new configuration settings.
//*
//* 	Function names correspond to datasheet register definitions
//*/
//void set_MUX(char c);
//void set_GAIN(char c);
//void set_PGA_BYPASS(char c);
//void set_DR(char c);
//void set_MODE(char c);
//void set_CM(char c);
//void set_TS(char c);
//void set_BCS(char c);
//void set_VREF(char c);
//void set_50_60(char c);
//void set_PSW(char c);
//void set_IDAC(char c);
//void set_IMUX(char c, int i);
//void set_DRDYM(char c);
//void set_ERROR(void);
//#endif /*ADS1220_H_*/
