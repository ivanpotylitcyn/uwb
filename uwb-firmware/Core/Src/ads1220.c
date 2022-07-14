#include "ads1220.h"
#include "math.h"

extern I2C_HandleTypeDef hi2c2;

#define  VREF1 5

static uint8_t iic_recvbuf[20];

void Error1(void) {
    // ошибка
}

//------------------------------------------------
static void I2Cx_WriteData(uint16_t Addr, uint8_t Reg, uint8_t Value, uint8_t size) {
    HAL_StatusTypeDef status = HAL_OK;
    status = HAL_I2C_Mem_Write(&hi2c2, Addr, (uint16_t)Reg, I2C_MEMADD_SIZE_8BIT, &Value, size, 0x10000);
    if(status != HAL_OK) Error1();
}
//------------------------------------------------
uint8_t I2Cx_ReadData(uint16_t Addr, uint8_t Reg, uint8_t size) {
    HAL_StatusTypeDef status = HAL_OK;
    uint8_t value = 0;
    status = HAL_I2C_Mem_Read(&hi2c2, Addr, Reg, I2C_MEMADD_SIZE_8BIT, &value, size, 0x10000);
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

void ads122_init() {

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
  ADS_SetVREF(ADS122C04_VREF_INTERNAL);
  ADS_SetTS(ADS122C04_TEMP_SENSOR_OFF);
  ADS_SetDCNT(ADS122C04_DCNT_DISABLE);
  ADS_SetCRC(ADS122C04_CRC_DISABLED);
  ADS_SetBCS(ADS122C04_BURN_OUT_CURRENT_OFF);
  ADS_SetIDAC(ADS122C04_IDAC_CURRENT_OFF);
  ADS_SetI2MUX(ADS122C04_IDAC1_DISABLED);
  ADS_SetI1MUX(ADS122C04_IDAC2_DISABLED);
}

float ADS122C04_getConversionData(void) {

    float voltageResult = 0;

    uint32_t conversionData = 0;

    HAL_I2C_Mem_Read(&hi2c2, ADS122C04_ADRESS, ADS122C04_RDATA_CMD, I2C_MEMADD_SIZE_8BIT, iic_recvbuf, 3, 0x10000);

    conversionData = ((uint32_t)iic_recvbuf[2]) | ((uint32_t)iic_recvbuf[1]<<8) | ((uint32_t)iic_recvbuf[0]<<16);

    if ((conversionData & 0x00800000) == 0x00800000)
        conversionData |= 0xFF000000;

    voltageResult = ((float)conversionData);

    return voltageResult;
}

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
// * ADS1220.c
// *
// */
///******************************************************************************
//// ADS1220 Demo C Function Calls
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
////
// ******************************************************************************/
//#include "ads1220.h"
///* This is MSP430 Code */
//void ADS1220Init(void)
//{
////	P3SEL |= ADS1220_DIN + ADS1220_DOUT ;
////	P2SEL |= ADS1220_SCLK;
////	P2SEL &= ~(ADS1220_DRDY | ADS1220_CS);
////	/* define initial states */
////	P1OUT |=  (ADS1220_CS); 					/* CS is really 'not' CS, so it should be disabled high */
////	/* define inputs */
////	P2DIR &= ~(ADS1220_DRDY);					/* DRDY is an input to the micro */
////	P2IES |= ADS1220_DRDY;						/* and should be used as an interrupt to retrieve data */
////	/* define outputs */
////	P1DIR |= ADS1220_CS;
//   return;
//}
//
///* ADS1220 Initial Configuration */
//void ADS1220Config(void)
//{
//	unsigned Temp;
//	ADS1220ReadRegister(ADS1220_0_REGISTER, 0x01, &Temp);
//	/* clear prev value; */
//   	Temp &= 0x0f;
//   	Temp |= ADS1220_MUX_0_G;
//   	/* write the register value containing the new value back to the ADS */
//   	ADS1220WriteRegister(ADS1220_0_REGISTER, 0x01, &Temp);
//	ADS1220ReadRegister(ADS1220_1_REGISTER, 0x01, &Temp);
//	/* clear prev DataRate code; */
//	Temp &= 0x1f;
//	Temp |= (ADS1220_DR_600 + ADS1220_CC);		/* Set default start mode to 600sps and continuous conversions */
//	/* write the register value containing the new value back to the ADS */
//	ADS1220WriteRegister(ADS1220_1_REGISTER, 0x01, &Temp);
//}
///*  Polling Function */
//int ADS1220WaitForDataReady(int Timeout)
//{
////   if (Timeout > 0)
////   {
////      /* wait for /DRDY = 1 */
////      while (!(P2IN & ADS1220_DRDY) && (Timeout-- >= 0));
////      /* wait for /DRDY = 0 */
////      while ( (P2IN & ADS1220_DRDY) && (Timeout-- >= 0))         ;
////      if (Timeout < 0)
////         return 0; /* ADS1220_TIMEOUT_WARNING; */
////   }
////   else
////   {
////      /* wait for /DRDY = 1 */
////      while (!(P2IN & ADS1220_DRDY));
////      /* wait for /DRDY = 0 */
////      while ( (P2IN & ADS1220_DRDY));
////   }
//   return ADS1220_NO_Error1;
//}
//void ADS1220AssertCS( int fAssert)
//{
////   if (fAssert)
////		P1OUT &=  ~(ADS1220_CS);
////   else
////		P1OUT |=  (ADS1220_CS);
////}
////void ADS1220SendByte(unsigned char Byte)
////{	char dummy;
////	while(!(UCA0IFG&UCTXIFG));
////	UCA0TXBUF = Byte;
////	while(!(UCA0IFG&UCRXIFG));
////	dummy = UCA0RXBUF;
//
//}
//unsigned char ADS1220ReceiveByte(void)
//{
//   unsigned char Result = 0;
////	while(!(UCA0IFG&UCTXIFG));	/* Make sure nothing is currently transmitting */
////	UCA0TXBUF = 0xff;		 	/* Send out NOP to initiate SCLK */
////	while(!(UCA0IFG&UCRXIFG));	/* Wait until all data is transmitted (received) */
////	Result = UCA0RXBUF;			/* Capture the receive buffer */
//	return Result;
//}
///*
//******************************************************************************
// higher level functions
//*/
//long ADS1220ReadData(void)
//{
//   long Data;
//   /* assert CS to start transfer */
//   ADS1220AssertCS(1);
//   /* send the command byte */
//// TODO: ADS1220SendByte(ADS1220_CMD_RDATA);
//   /* get the conversion result */
//#ifdef ADS1120
//   Data = ADS1220ReceiveByte();
//   Data = (Data << 8) | ADS1220ReceiveByte();
//   /* sign extend data */
//   if (Data & 0x8000)
//      Data |= 0xffff0000;
//#else
//   Data = ADS1220ReceiveByte();
//   Data = (Data << 8) | ADS1220ReceiveByte();
//   Data = (Data << 8) | ADS1220ReceiveByte();
//   /* sign extend data */
//   if (Data & 0x800000)
//      Data |= 0xff000000;
//#endif
//   /* de-assert CS */
//   ADS1220AssertCS(0);
//   return Data;
//}
//void ADS1220ReadRegister(int StartAddress, int NumRegs, unsigned * pData)
//{
//   int i;
//	/* assert CS to start transfer */
//	ADS1220AssertCS(1);
// 	/* send the command byte */
//	ADS1220SendByte(ADS1220_CMD_RREG | (((StartAddress<<2) & 0x0c) |((NumRegs-1)&0x03)));
//   	/* get the register content */
//	for (i=0; i< NumRegs; i++)
//	{
//		*pData++ = ADS1220ReceiveByte();
//	}
//   	/* de-assert CS */
//	ADS1220AssertCS(0);
//	return;
//}
//void ADS1220WriteRegister(int StartAddress, int NumRegs, unsigned * pData)
//{
//	int i;
//   	/* assert CS to start transfer */
//	ADS1220AssertCS(1);
//   	/* send the command byte */
//	ADS1220SendByte(ADS1220_CMD_WREG | (((StartAddress<<2) & 0x0c) |((NumRegs-1)&0x03)));
//    /* send the data bytes */
//	for (i=0; i< NumRegs; i++)
//	{
//		ADS1220SendByte(*pData++);
//	}
//   	/* de-assert CS */
//	ADS1220AssertCS(0);
//   	return;
//}
//void ADS1220SendResetCommand(void)
//{
//	/* assert CS to start transfer */
//	ADS1220AssertCS(1);
//   	/* send the command byte */
//	ADS1220SendByte(ADS1220_CMD_RESET);
//   	/* de-assert CS */
//	ADS1220AssertCS(0);
//   	return;
//}
//void ADS1220SendStartCommand(void)
//{
//	/* assert CS to start transfer */
//	ADS1220AssertCS(1);
//    /* send the command byte */
//	ADS1220SendByte(ADS1220_CMD_SYNC);
//   	/* de-assert CS */
//	ADS1220AssertCS(0);
//    return;
//}
//void ADS1220SendShutdownCommand(void)
//{
//	/* assert CS to start transfer */
//	ADS1220AssertCS(1);
//   	/* send the command byte */
//	ADS1220SendByte(ADS1220_CMD_SHUTDOWN);
//   	/* de-assert CS */
//	ADS1220AssertCS(0);
//    return;
//}
///*
//******************************************************************************
//register set value commands
//*/
//int ADS1220SetChannel(int Mux)
//{
//	unsigned int cMux = Mux;
//   /* write the register value containing the new value back to the ADS */
//   ADS1220WriteRegister(ADS1220_0_REGISTER, 0x01, &cMux);
//   return ADS1220_NO_Error1;
//}
//int ADS1220SetGain(int Gain)
//{
//	unsigned int cGain = Gain;
//	/* write the register value containing the new code back to the ADS */
//	ADS1220WriteRegister(ADS1220_0_REGISTER, 0x01, &cGain);
//	return ADS1220_NO_Error1;
//}
//int ADS1220SetPGABypass(int Bypass)
//{
//	unsigned int cBypass = Bypass;
//	/* write the register value containing the new code back to the ADS */
//	ADS1220WriteRegister(ADS1220_0_REGISTER, 0x01, &cBypass);
//	return ADS1220_NO_Error1;
//}
//int ADS1220SetDataRate(int DataRate)
//{
//	unsigned int cDataRate = DataRate;
//	/* write the register value containing the new value back to the ADS */
//	ADS1220WriteRegister(ADS1220_1_REGISTER, 0x01, &cDataRate);
//	return ADS1220_NO_Error1;
//}
//int ADS1220SetClockMode(int ClockMode)
//{
//	unsigned int cClockMode = ClockMode;
//   	/* write the register value containing the value back to the ADS */
//	ADS1220WriteRegister(ADS1220_1_REGISTER, 0x01, &cClockMode);
//	return ADS1220_NO_Error1;
//}
//int ADS1220SetPowerDown(int PowerDown)
//{
//	unsigned int cPowerDown = PowerDown;
//   	/* write the register value containing the new value back to the ADS */
//	ADS1220WriteRegister(ADS1220_1_REGISTER, 0x01, &cPowerDown);
//	return ADS1220_NO_Error1;
//}
//int ADS1220SetTemperatureMode(int TempMode)
//{
//	unsigned int cTempMode = TempMode;
//   	/* write the register value containing the new value back to the ADS */
//	ADS1220WriteRegister(ADS1220_1_REGISTER, 0x01, &cTempMode);
//	return ADS1220_NO_Error1;
//}
//int ADS1220SetBurnOutSource(int BurnOut)
//{
//	unsigned int cBurnOut = BurnOut;
//   	/* write the register value containing the new value back to the ADS */
//	ADS1220WriteRegister(ADS1220_1_REGISTER, 0x01, &cBurnOut);
//	return ADS1220_NO_Error1;
//}
//int ADS1220SetVoltageReference(int VoltageRef)
//{
//	unsigned int cVoltageRef = VoltageRef;
//   	/* write the register value containing the new value back to the ADS */
//	ADS1220WriteRegister(ADS1220_2_REGISTER, 0x01, &cVoltageRef);
//	return ADS1220_NO_Error1;
//}
//int ADS1220Set50_60Rejection(int Rejection)
//{
//	unsigned int cRejection = Rejection;
//   	/* write the register value containing the new value back to the ADS */
//	ADS1220WriteRegister(ADS1220_2_REGISTER, 0x01, &cRejection);
//	return ADS1220_NO_Error1;
//}
//int ADS1220SetLowSidePowerSwitch(int PowerSwitch)
//{
//	unsigned int cPowerSwitch = PowerSwitch;
//   	/* write the register value containing the new value back to the ADS */
//	ADS1220WriteRegister(ADS1220_2_REGISTER, 0x01, &cPowerSwitch);
//	return ADS1220_NO_Error1;
//}
//int ADS1220SetCurrentDACOutput(int CurrentOutput)
//{
//	unsigned int cCurrentOutput = CurrentOutput;
//   	/* write the register value containing the new value back to the ADS */
//	ADS1220WriteRegister(ADS1220_2_REGISTER, 0x01, &cCurrentOutput);
//	return ADS1220_NO_Error1;
//}
//int ADS1220SetIDACRouting(int IDACRoute)
//{
//	unsigned int cIDACRoute = IDACRoute;
//	/* write the register value containing the new value back to the ADS */
//	ADS1220WriteRegister(ADS1220_3_REGISTER, 0x01, &cIDACRoute);
//	return ADS1220_NO_Error1;
//}
//int ADS1220SetDRDYMode(int DRDYMode)
//{
//	unsigned int cDRDYMode = DRDYMode;
//   	/* write the register value containing the new gain code back to the ADS */
//	ADS1220WriteRegister(ADS1220_3_REGISTER, 0x01, &cDRDYMode);
//	return ADS1220_NO_Error1;
//}
///*
//******************************************************************************
//register get value commands
//*/
//int ADS1220GetChannel(void)
//{
//	unsigned Temp;
//	/* Parse Mux data from register */
//	ADS1220ReadRegister(ADS1220_0_REGISTER, 0x01, &Temp);
//	/* return the parsed data */
//	return (Temp >>4);
//}
//int ADS1220GetGain(void)
//{
//	unsigned Temp;
//	/* Parse Gain data from register */
//	ADS1220ReadRegister(ADS1220_0_REGISTER, 0x01, &Temp);
//	/* return the parsed data */
//	return ( (Temp & 0x0e) >>1);
//}
//int ADS1220GetPGABypass(void)
//{
//	unsigned Temp;
//	/* Parse Bypass data from register */
//	ADS1220ReadRegister(ADS1220_0_REGISTER, 0x01, &Temp);
//	/* return the parsed data */
//	return (Temp & 0x01);
//}
//int ADS1220GetDataRate(void)
//{
//	unsigned Temp;
//	/* Parse DataRate data from register */
//	ADS1220ReadRegister(ADS1220_1_REGISTER, 0x01, &Temp);
//	/* return the parsed data */
//	return ( Temp >>5 );
//}
//int ADS1220GetClockMode(void)
//{
//	unsigned Temp;
//	/* Parse ClockMode data from register */
//	ADS1220ReadRegister(ADS1220_1_REGISTER, 0x01, &Temp);
//	/* return the parsed data */
//	return ( (Temp & 0x18) >>3 );
//}
//int ADS1220GetPowerDown(void)
//{
//	unsigned Temp;
//	/* Parse PowerDown data from register */
//	ADS1220ReadRegister(ADS1220_1_REGISTER, 0x01, &Temp);
//	/* return the parsed data */
//	return ( (Temp & 0x04) >>2 );
//}
//int ADS1220GetTemperatureMode(void)
//{
//	unsigned Temp;
//	/* Parse TempMode data from register */
//	ADS1220ReadRegister(ADS1220_1_REGISTER, 0x01, &Temp);
//	/* return the parsed data */
//	return ( (Temp & 0x02) >>1 );
//}
//int ADS1220GetBurnOutSource(void)
//{
//	unsigned Temp;
//	/* Parse BurnOut data from register */
//	ADS1220ReadRegister(ADS1220_1_REGISTER, 0x01, &Temp);
//	/* return the parsed data */
//	return ( Temp & 0x01 );
//}
//int ADS1220GetVoltageReference(void)
//{
//	unsigned Temp;
//	/* Parse VoltageRef data from register */
//	ADS1220ReadRegister(ADS1220_2_REGISTER, 0x01, &Temp);
//	/* return the parsed data */
//	return ( Temp >>6 );
//}
//int ADS1220Get50_60Rejection(void)
//{
//	unsigned Temp;
//	/* Parse Rejection data from register */
//	ADS1220ReadRegister(ADS1220_2_REGISTER, 0x01, &Temp);
//	/* return the parsed data */
//	return ( (Temp & 0x30) >>4 );
//}
//int ADS1220GetLowSidePowerSwitch(void)
//{
//	unsigned Temp;
//	/* Parse PowerSwitch data from register */
//	ADS1220ReadRegister(ADS1220_2_REGISTER, 0x01, &Temp);
//	/* return the parsed data */
//	return ( (Temp & 0x08) >>3);
//}
//int ADS1220GetCurrentDACOutput(void)
//{
//	unsigned Temp;
//	/* Parse IDACOutput data from register */
//	ADS1220ReadRegister(ADS1220_2_REGISTER, 0x01, &Temp);
//	/* return the parsed data */
//	return ( Temp & 0x07 );
//}
//int ADS1220GetIDACRouting(int WhichOne)
//{
//	/* Check WhichOne sizing */
//	if (WhichOne >1) return ADS1220_Error1;
//	unsigned Temp;
//	/* Parse Mux data from register */
//	ADS1220ReadRegister(ADS1220_3_REGISTER, 0x01, &Temp);
//	/* return the parsed data */
//	if (WhichOne) return ( (Temp & 0x1c) >>2);
//	else return ( Temp >>5 );
//}
//int ADS1220GetDRDYMode(void)
//{
//	unsigned Temp;
//	/* Parse DRDYMode data from register */
//	ADS1220ReadRegister(ADS1220_3_REGISTER, 0x01, &Temp);
//	/* return the parsed data */
//	return ( (Temp & 0x02) >>1 );
//}
///* Useful Functions within Main Program for Setting Register Contents
//*
//*  	These functions show the programming flow based on the header definitions.
//*  	The calls are not made within the demo example, but could easily be used by calling the function
//*  		defined within the program to complete a fully useful program.
//*	Similar function calls were made in the firwmare design for the ADS1220EVM.
//*
//*  The following function calls use ASCII data sent from a COM port to control settings
//*	on the ADS1220.  The data is recontructed from ASCII and then combined with the
//*	register contents to save as new configuration settings.
//*
//* 	Function names correspond to datasheet register definitions
//*/
//void set_MUX(char c)
//{
//	int mux = (int) c - 48;
//	int dError1;
//	unsigned Temp;
//	if (mux>=49 && mux<=54) mux -= 39;
//	/* The MUX value is only part of the register, so we have to read it back
//	   and massage the new value into it */
//	ADS1220ReadRegister(ADS1220_0_REGISTER, 0x01, &Temp);
//	Temp &= 0x0f;									/* strip out old settings */
//	/* Change Data rate */
//	switch(mux) {
//		case 0:
//			dError1 = ADS1220SetChannel(Temp + ADS1220_MUX_0_1);
//			break;
//		case 1:
//			dError1 = ADS1220SetChannel(Temp + ADS1220_MUX_0_2);
//			break;
//		case 2:
//			dError1 = ADS1220SetChannel(Temp + ADS1220_MUX_0_3);
//			break;
//		case 3:
//			dError1 = ADS1220SetChannel(Temp + ADS1220_MUX_1_2);
//			break;
//		case 4:
//			dError1 = ADS1220SetChannel(Temp + ADS1220_MUX_1_3);
//			break;
//		case 5:
//			dError1 = ADS1220SetChannel(Temp + ADS1220_MUX_2_3);
//			break;
//		case 6:
//			dError1 = ADS1220SetChannel(Temp + ADS1220_MUX_1_0);
//			break;
//		case 7:
//			dError1 = ADS1220SetChannel(Temp + ADS1220_MUX_3_2);
//			break;
//		case 8:
//			dError1 = ADS1220SetChannel(Temp + ADS1220_MUX_0_G);
//			break;
//		case 9:
//			dError1 = ADS1220SetChannel(Temp + ADS1220_MUX_1_G);
//			break;
//		case 10:
//			dError1 = ADS1220SetChannel(Temp + ADS1220_MUX_2_G);
//			break;
//		case 11:
//			dError1 = ADS1220SetChannel(Temp + ADS1220_MUX_3_G);
//			break;
//		case 12:
//			dError1 = ADS1220SetChannel(Temp + ADS1220_MUX_EX_VREF);
//			break;
//		case 13:
//			dError1 = ADS1220SetChannel(Temp + ADS1220_MUX_AVDD);
//			break;
//		case 14:
//			dError1 = ADS1220SetChannel(Temp + ADS1220_MUX_DIV2);
//			break;
//		case 15:
//			dError1 = ADS1220SetChannel(Temp + ADS1220_MUX_DIV2);
//			break;
//		default:
//			dError1 = ADS1220_Error1;
//			break;
//	}
//	if (dError1==ADS1220_Error1)
//		set_Error1();
//}
//void set_GAIN(char c)
//{
//	int pga = (int) c - 48;
//	int dError1;
//	unsigned Temp;
//	/* The GAIN value is only part of the register, so we have to read it back
//	   and massage the new value into it*/
//	ADS1220ReadRegister(ADS1220_0_REGISTER, 0x01, &Temp);
//		Temp &= 0xf1;									/* strip out old settings */
//	/* Change gain rate */
//	switch(pga) {
//		case 0:
//			dError1 = ADS1220SetGain(Temp + ADS1220_GAIN_1);
//			break;
//		case 1:
//			dError1 = ADS1220SetGain(Temp + ADS1220_GAIN_2);
//			break;
//		case 2:
//			dError1 = ADS1220SetGain(Temp + ADS1220_GAIN_4);
//			break;
//		case 3:
//			dError1 = ADS1220SetGain(Temp + ADS1220_GAIN_8);
//			break;
//		case 4:
//			dError1 = ADS1220SetGain(Temp + ADS1220_GAIN_16);
//			break;
//		case 5:
//			dError1 = ADS1220SetGain(Temp + ADS1220_GAIN_32);
//			break;
//		case 6:
//			dError1 = ADS1220SetGain(Temp + ADS1220_GAIN_64);
//			break;
//		case 7:
//			dError1 = ADS1220SetGain(Temp + ADS1220_GAIN_128);
//			break;
//		default:
//			dError1 = ADS1220_Error1;
//			break;
//		}
//	if (dError1==ADS1220_Error1)
//		set_Error1();
//}
//void set_PGA_BYPASS(char c)
//{
//	int buff = (int) c - 48;
//	int dError1;
//	unsigned Temp;
//	/* the PGA Bypass value is only part of the register, so we have to read it back
//	   and massage the new value into it */
//	ADS1220ReadRegister(ADS1220_0_REGISTER, 0x01, &Temp);
//	Temp &= 0xfe;									/* strip out old settings */
//	/* Change PGA Bypass */
//	switch(buff) {
//		case 0:
//			dError1 = ADS1220SetPGABypass(Temp);
//			break;
//		case 1:
//			dError1 = ADS1220SetPGABypass(Temp + ADS1220_PGA_BYPASS);
//			break;
//		default:
//			dError1 = ADS1220_Error1;
//			break;
//	}
//	if (dError1==ADS1220_Error1)
//		set_Error1();
//}
//void set_DR(char c)
//{
//	int spd = (int) c - 48;
//	int dError1;
//	unsigned Temp;
//	/* the DataRate value is only part of the register, so we have to read it back
//	   and massage the new value into it */
//	ADS1220ReadRegister(ADS1220_1_REGISTER, 0x01, &Temp);
//	Temp &= 0x1f;									/* strip out old settings */
//	/* Change Data rate */
//	switch(spd) {
//		case 0:
//			dError1 = ADS1220SetDataRate(Temp + ADS1220_DR_20);
//			break;
//		case 1:
//			dError1 = ADS1220SetDataRate(Temp + ADS1220_DR_45);
//			break;
//		case 2:
//			dError1 = ADS1220SetDataRate(Temp + ADS1220_DR_90);
//			break;
//		case 3:
//			dError1 = ADS1220SetDataRate(Temp + ADS1220_DR_175);
//			break;
//		case 4:
//			dError1 = ADS1220SetDataRate(Temp + ADS1220_DR_330);
//			break;
//		case 5:
//			dError1 = ADS1220SetDataRate(Temp + ADS1220_DR_600);
//			break;
//		case 6:
//			dError1 = ADS1220SetDataRate(Temp + ADS1220_DR_1000);
//			break;
//		case 7:
//			dError1 = ADS1220SetDataRate(Temp + ADS1220_DR_1000);
//			break;
//		default:
//			dError1 = ADS1220_Error1;
//			break;
//	}
//	if (dError1==ADS1220_Error1)
//		set_Error1();
//}
//void set_MODE(char c)
//{
//	int spd = (int) c - 48;
//	int dError1;
//	unsigned Temp;
//	/* the MODE value is only part of the register, so we have to read it back
//	   and massage the new value into it */
//	ADS1220ReadRegister(ADS1220_1_REGISTER, 0x01, &Temp);
//	Temp &= 0xe7;									/* strip out old settings */
//	/* Change the operating Mode */
//	switch(spd) {
//		case 0:
//			dError1 = ADS1220SetClockMode(Temp + ADS1220_MODE_NORMAL);
//			break;
//		case 1:
//			dError1 = ADS1220SetClockMode(Temp + ADS1220_MODE_DUTY);
//			break;
//		case 2:
//			dError1 = ADS1220SetClockMode(Temp + ADS1220_MODE_TURBO);
//			break;
//		case 3:
//			dError1 = ADS1220SetClockMode(Temp + ADS1220_MODE_DCT);
//			break;
//		default:
//			dError1 = ADS1220_Error1;
//			break;
//	}
//	if (dError1==ADS1220_Error1)
//		set_Error1();
//}
//void set_CM(char c)
//{
//	int pwrdn = (int) c - 48;
//	int dError1;
//	unsigned Temp;
//	/* the Conversion Mode value is only part of the register, so we have to read it back
//	   and massage the new value into it */
//	ADS1220ReadRegister(ADS1220_1_REGISTER, 0x01, &Temp);
//	Temp &= 0xfb;									/* strip out old settings */
//	/* Change power down mode */
//	switch(pwrdn) {
//		case 0:
//			dError1 = ADS1220SetPowerDown(Temp);
//			break;
//		case 1:
//			dError1 = ADS1220SetPowerDown(Temp + ADS1220_CC);
//			break;
//		default:
//			dError1 = ADS1220_Error1;
//			break;
//	}
//	if (dError1==ADS1220_Error1)
//		set_Error1();
//}
//void set_TS(char c)
//{
//	int tmp = (int) c - 48;
//	int dError1;
//	unsigned Temp;
//	/* the Temperature Sensor mode value is only part of the register, so we have to read it back
//	   and massage the new value into it */
//	ADS1220ReadRegister(ADS1220_1_REGISTER, 0x01, &Temp);
//	Temp &= 0xfd;									/* strip out old settings */
//	/* Change Temp Diode Setting */
//	switch(tmp) {
//		case 0:
//			dError1 = ADS1220SetTemperatureMode(Temp);
//			break;
//		case 1:
//			dError1 = ADS1220SetTemperatureMode(Temp + ADS1220_TEMP_SENSOR);
//			break;
//		default:
//			dError1 = ADS1220_Error1;
//			break;
//	}
//	if (dError1==ADS1220_Error1)
//		set_Error1();
//}
//void set_BCS(char c)
//{
//	int bo = (int) c - 48;
//	int dError1;
//	unsigned Temp;
//	/* the Burnout Current Source value is only part of the register, so we have to read it back
//	   and massage the new value into it */
//	ADS1220ReadRegister(ADS1220_1_REGISTER, 0x01, &Temp);
//	Temp &= 0xfe;									/* strip out old settings */
//	/* Change Burnout Current */
//	switch(bo) {
//		case 0:
//			dError1 = ADS1220SetBurnOutSource(Temp);
//			break;
//		case 1:
//			dError1 = ADS1220SetBurnOutSource(Temp + ADS1220_BCS);
//			break;
//		default:
//			dError1 = ADS1220_Error1;
//			break;
//	}
//	if (dError1==ADS1220_Error1)
//		set_Error1();
//}
//void set_VREF(char c)
//{
//	int ref = (int) c - 48;
//	int dError1;
//	unsigned Temp;
//	/* the Voltage Reference value is only part of the register, so we have to read it back
//	   and massage the new value into it */
//	ADS1220ReadRegister(ADS1220_2_REGISTER, 0x01, &Temp);
//	Temp &= 0x3f;									/* strip out old settings */
//	/* Change Reference */
//	switch(ref) {
//		case 0:
//			dError1 = ADS1220SetVoltageReference(Temp + ADS1220_VREF_INT);
//			break;
//		case 1:
//			dError1 = ADS1220SetVoltageReference(Temp + ADS1220_VREF_EX_DED);
//			break;
//		case 2:
//			dError1 = ADS1220SetVoltageReference(Temp + ADS1220_VREF_EX_AIN);
//			break;
//		case 3:
//			dError1 = ADS1220SetVoltageReference(Temp + ADS1220_VREF_SUPPLY);
//			break;
//		default:
//			dError1 = ADS1220_Error1;
//			break;
//	}
//	if (dError1==ADS1220_Error1)
//		set_Error1();
//}
//void set_50_60(char c)
//{
//	int flt = (int) c - 48;
//	int dError1;
//	unsigned Temp;
//	/* the Digital Filter value is only part of the register, so we have to read it back
//	   and massage the new value into it */
//	ADS1220ReadRegister(ADS1220_2_REGISTER, 0x01, &Temp);
//	Temp &= 0xcf;									/* strip out old settings */
//	/* Change Filter Setting */
//	switch(flt) {
//		case 0:
//			dError1 = ADS1220Set50_60Rejection(Temp + ADS1220_REJECT_OFF);
//			break;
//		case 1:
//			dError1 = ADS1220Set50_60Rejection(Temp + ADS1220_REJECT_BOTH);
//			break;
//		case 2:
//			dError1 = ADS1220Set50_60Rejection(Temp + ADS1220_REJECT_50);
//			break;
//		case 3:
//			dError1 = ADS1220Set50_60Rejection(Temp + ADS1220_REJECT_60);
//			break;
//		default:
//			dError1 = ADS1220_Error1;
//			break;
//	}
//	if (dError1==ADS1220_Error1)
//		set_Error1();
//}
//void set_PSW(char c)
//{
//	int sw = (int) c - 48;
//	int dError1;
//	unsigned Temp;
//	/* the Low-side Switch value is only part of the register, so we have to read it back
//	   and massage the new value into it */
//	ADS1220ReadRegister(ADS1220_2_REGISTER, 0x01, &Temp);
//	Temp &= 0xf7;									/* strip out old settings */
//		/* Change low-side switch mode */
//	switch(sw) {
//		case 0:
//			dError1 = ADS1220SetLowSidePowerSwitch(Temp);
//			break;
//		case 1:
//			dError1 = ADS1220SetLowSidePowerSwitch(Temp + ADS1220_PSW_SW);
//			break;
//		default:
//			dError1 = ADS1220_Error1;
//			break;
//	}
//	if (dError1==ADS1220_Error1)
//		set_Error1();
//}
//void set_IDAC(char c)
//{
//	int current = (int) c - 48;
//	int dError1;
//	unsigned Temp;
//	/* the IDAC Current value is only part of the register, so we have to read it back
//	   and massage the new value into it */
//	ADS1220ReadRegister(ADS1220_2_REGISTER, 0x01, &Temp);
//	Temp &= 0xf8;									/* strip out old settings */
//	/* Change IDAC Current Output */
//	switch(current) {
//		case 0:
//			dError1 = ADS1220SetCurrentDACOutput(Temp + ADS1220_IDAC_OFF);
//			break;
//		case 1:
//#ifdef ADS1120
//			dError1 = ADS1220SetCurrentDACOutput(Temp + ADS1220_IDAC_OFF);
//#else
//			dError1 = ADS1220SetCurrentDACOutput(Temp + ADS1220_IDAC_10);
//#endif
//			break;
//		case 2:
//			dError1 = ADS1220SetCurrentDACOutput(Temp + ADS1220_IDAC_50);
//			break;
//		case 3:
//			dError1 = ADS1220SetCurrentDACOutput(Temp + ADS1220_IDAC_100);
//			break;
//		case 4:
//			dError1 = ADS1220SetCurrentDACOutput(Temp + ADS1220_IDAC_250);
//			break;
//		case 5:
//			dError1 = ADS1220SetCurrentDACOutput(Temp + ADS1220_IDAC_500);
//			break;
//		case 6:
//			dError1 = ADS1220SetCurrentDACOutput(Temp + ADS1220_IDAC_1000);
//			break;
//		case 7:
//			dError1 = ADS1220SetCurrentDACOutput(Temp + ADS1220_IDAC_2000);
//			break;
//		default:
//			dError1 = ADS1220_Error1;
//			break;
//		}
//	if (dError1==ADS1220_Error1)
//		set_Error1();
//}
//void set_IMUX(char c, int i)
//{
//	int mux = (int) c - 48;
//	int dError1;
//	unsigned Temp;
//	/* the IDAC Mux value is only part of the register, so we have to read it back
//	   and massage the new value into it */
//	ADS1220ReadRegister(ADS1220_3_REGISTER, 0x01, &Temp);
//	if (i==1) {
//		Temp &= 0xe3;									/* strip out old settings */
//		/* Change IDAC2 MUX Output */
//		switch(mux) {
//			case 0:
//				Temp |= ADS1220_IDAC2_OFF;
//				break;
//			case 1:
//				Temp |= ADS1220_IDAC2_AIN0;
//				break;
//			case 2:
//				Temp |= ADS1220_IDAC2_AIN1;
//				break;
//			case 3:
//				Temp |= ADS1220_IDAC2_AIN2;
//				break;
//			case 4:
//				Temp |= ADS1220_IDAC2_AIN3;
//				break;
//			case 5:
//				Temp |= ADS1220_IDAC2_REFP0;
//				break;
//			case 6:
//				Temp |= ADS1220_IDAC2_REFN0;
//				break;
//			case 7:
//				Temp |= ADS1220_IDAC2_REFN0;
//				break;
//			default:
//				dError1 = ADS1220_Error1;
//				break;
//		}
//	}
//	else {
//		Temp &= 0x1f;
//		/* Change IDAC1 MUX Output */
//		switch(mux) {
//			case 0:
//				Temp |= ADS1220_IDAC1_OFF;
//				break;
//			case 1:
//				Temp |= ADS1220_IDAC1_AIN0;
//				break;
//			case 2:
//				Temp |= ADS1220_IDAC1_AIN1;
//				break;
//			case 3:
//				Temp |= ADS1220_IDAC1_AIN2;
//				break;
//			case 4:
//				Temp |= ADS1220_IDAC1_AIN3;
//				break;
//			case 5:
//				Temp |= ADS1220_IDAC1_REFP0;
//				break;
//			case 6:
//				Temp |= ADS1220_IDAC1_REFN0;
//				break;
//			case 7:
//				Temp |= ADS1220_IDAC1_REFN0;
//				break;
//			default:
//				dError1 = ADS1220_Error1;
//				break;
//		}
//	}
//	if (dError1==ADS1220_NO_Error1)
//		dError1 = ADS1220SetIDACRouting(Temp);
//	if (dError1==ADS1220_Error1)
//		set_Error1();
//}
//void set_DRDYM(char c)
//{
//	int drdy = (int) c - 48;
//	int dError1;
//	unsigned Temp;
//	/* the DRDY output mode value is only part of the register, so we have to read it back
//	   and massage the new value into it */
//	ADS1220ReadRegister(ADS1220_3_REGISTER, 0x01, &Temp);
//	Temp &= 0xfd;									/* strip out old settings */
//	/* Change DRDY Mode Setting */
//	switch(drdy) {
//		case 0:
//			dError1 = ADS1220SetDRDYMode(Temp);
//			break;
//		case 1:
//			dError1 = ADS1220SetDRDYMode(Temp + ADS1220_DRDY_MODE);
//			break;
//		default:
//			dError1 = ADS1220_Error1;
//			break;
//	}
//	if (dError1==ADS1220_Error1)
//		set_Error1();
//}
//void set_Error1(void)
//{
//	/* Add some Error1 routine here is desired */
//}
//
