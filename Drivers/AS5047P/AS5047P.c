#include "AS5047P.h"
//#include "stm32f4xx_hal_gpio.h"
//#include "stm32f4xx_hal_spi.h"
//#include "delay.h"
//#include "usart.h"
#include "spi.h"
#include "main.h"
#define AS5047P_CS_1  		HAL_GPIO_WritePin(GPIOB, AS_nCS_Pin, GPIO_PIN_SET)//;delay_ms(1);  //PA4
#define AS5047P_CS_0  		HAL_GPIO_WritePin(GPIOB, AS_nCS_Pin, GPIO_PIN_RESET)//;delay_ms(1);   //PA4
//extern SPI_HandleTypeDef hspi2;
unsigned int parity(unsigned int x)
{
	unsigned int parity = 0;

	while(x != 0)
	{
		parity ^= x;
		x >>= 1;
	}
	return (parity & 0x1);
}

void AS5047P_Init(void)
{
	/* Initiaize AS4047D */
	AS5047P_Write(AS5047P_SETTINGS1,5);//0b00000101 默认值5
	//AS5047D_Check_Transmission_Error();
	AS5047P_Write(AS5047P_SETTINGS2,0);//0b00000000
	//AS5047D_Check_Transmission_Error();
	
}

void AS5047P_Write(unsigned int address, unsigned int data)
{
	if (parity(address & 0x3FFF) == 1) address = address | 0x8000; // set parity bit
	//address = address & (WR | 0x8000);  // its  a write command and don't change the parity bit (0x8000)
	AS5047P_CS_0;
	HAL_SPI_Transmit(&hspi2, (uint8_t *)(&address), 1, 1000);
	AS5047P_CS_1;
	if (parity(data & 0x3FFF) == 1) data = data | 0x8000; // set parity bit
	//data = data & (WR | 0x8000); // its a write command and don't change the parity bit (0x8000)
	AS5047P_CS_0;
	HAL_SPI_Transmit(&hspi2, (uint8_t *)(&data), 1, 1000);
	AS5047P_CS_1;
}

unsigned int AS5047P_Read(unsigned int address)//读取带地址的数据
{
	unsigned int data = 0;
	if (parity(address | AS5047P_RD) == 1) address = address | 0x8000; // set parity bit
	address = address | AS5047P_RD; // it's a read command
	AS5047P_CS_0;
	HAL_SPI_Transmit(&hspi2, (uint8_t *)(&address), 1, 1000);
	AS5047P_CS_1;

	AS5047P_CS_0;
	HAL_SPI_Receive(&hspi2, (uint8_t *)(&data), 1, 1000);
	AS5047P_CS_1;
	data = data & 0x3FFF;  // filter bits outside data, strip bit 14..15
	return data;
}
////////////////////////////////////////////////////////
unsigned int AS5047P_ReadData(void)//读取SPI数据
{
	unsigned int data = 0;
	static unsigned int address = 0xffff;
	AS5047P_CS_0;
	HAL_SPI_TransmitReceive(&hspi2, (uint8_t *)(&address), (uint8_t *)(&data), 1, 100);
	AS5047P_CS_1;
	data = data & 0x3FFF;  // filter bits outside data, strip bit 14..15
	return data;
}
//////////////////////////////////////////////////////
void AS5047P_SetZero(void)
{
	/** Check diagnostics reg **/
	unsigned int DIAAGC = AS5047P_Read(AS5047P_DIAAGC);
	/** Get uncompensated angle reg value **/
	unsigned int ANGLEUNC = AS5047P_Read(AS5047P_ANGLEUNC);
	ANGLEUNC = 0x3f00;
	/** Write to zero pos regs **/
	AS5047P_Write(AS5047P_ZPOSM, (ANGLEUNC >> 6) & 0x00FF);
	//AS5047P_Check_Transmission_Error();
	AS5047P_Write(AS5047P_ZPOSL, ANGLEUNC & 0x003F);
	//AS5047P_Check_Transmission_Error();
}

unsigned int AS5047P_Get_ANGLECOM_Value(void)
{
	//unsigned int ANGLECOM = AS5047P_Read(AS5047P_ANGLECOM);
	unsigned int ANGLECOM =AS5047P_ReadData();
	//AS5047D_Check_Transmission_Error();
	return ANGLECOM;
}

unsigned int AS5047D_Get_CORDICMAG_Value(void)
{
	unsigned int CORDICMAG = AS5047P_Read(AS5047P_CORDICMAG);
	//AS5047D_Check_Transmission_Error();
	return CORDICMAG;
}

unsigned int AS5047P_Get_AGC_Value(void)
{
	/** Read diagnostics reg **/
	unsigned int DIAAGC = AS5047P_Read(AS5047P_DIAAGC);
	//AS5047D_Check_Transmission_Error();
	return (unsigned char)((DIAAGC >> 8) & 0x00FF);
}

unsigned int AS5047D_GetZero(void)
{
	unsigned int ZPOSM = 0;
	unsigned int ZPOSL = 0;

	ZPOSM = AS5047P_Read(AS5047P_ZPOSM);
	//AS5047D_Check_Transmission_Error();
	ZPOSL = AS5047P_Read(AS5047P_ZPOSL);
	//AS5047D_Check_Transmission_Error();

	return (((ZPOSM << 6) & 0x3FC0) | (ZPOSL & 0x003F));
}

unsigned char AS5047P_Check_Transmission_Error(void)
{
	/** Check if transmission error **/
	return AS5047P_Read(AS5047P_ERRFL);
}




