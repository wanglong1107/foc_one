#ifndef __AS5047P_H_
#define __AS5047P_H_

//#include <stdint.h>


// AS5047P Register Addresses

/** volatile **/
#define AS5047P_NOP 0x0000
#define AS5047P_ERRFL 0x0001
#define AS5047P_PROG 0x0003
#define AS5047P_DIAAGC 0x3FFC
#define AS5047P_CORDICMAG 0x3FFD
#define AS5047P_ANGLEUNC 0x3FFE
#define AS5047P_ANGLECOM 0x3FFF

/** non-volatile **/
#define AS5047P_ZPOSM 0x0016
#define AS5047P_ZPOSL 0x0017
#define AS5047P_SETTINGS1 0x0018
#define AS5047P_SETTINGS2 0x0019

#define AS5047P_RD 0x4000    // bit 14 = "1" is Read + parity even
#define AS5047P_WR 0x3FFF    // bit 14 = "0" is Write


#define AS5047P_Check_MAG_TooLow(DIAAGC)      ((DIAAGC >> 11) & 0x0001)
#define AS5047P_Check_MAG_TooHigh(DIAAGC)     ((DIAAGC >> 10) & 0x0001)
#define AS5047P_Check_CORDIC_Overflow(DIAAGC) ((DIAAGC >> 9)  & 0x0001)
#define AS5047P_Check_LF_finished(DIAAGC)     ((DIAAGC >> 8)  & 0x0001)

void AS5047P_Write(unsigned int address, unsigned int data);
unsigned int AS5047P_Read(unsigned int address);
unsigned char AS5047P_Check_Transmission_Error(void);
unsigned int AS5047D_GetZero(void);
unsigned int AS5047P_Get_AGC_Value(void);
unsigned int AS5047D_Get_CORDICMAG_Value(void);
unsigned int AS5047P_Get_ANGLECOM_Value(void);
unsigned int AS5047P_Get_ANGLEUNC_Value(void);
float AS5047P_Get_True_Angle_Value(void);
void AS5047P_SetZero(void);
unsigned int SPI_READ(void);
void SPI_SEND(unsigned int data);
void AS5047P_Init(void);
////////////////////////
unsigned int AS5047P_ReadData(void);

#endif /* AS5047P_H_ */



