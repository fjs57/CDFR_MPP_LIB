/*
 * TMC2209_UART.c
 *
 *  Created on: Nov 2, 2024
 *      Author: franc
 */

#include "TMC2209_UART.h"

#define TMC2209_UART_SYNC_BYTE 0x05

#define TMC2209_UART_DESCRIPTOR instance->descriptor
#define TMC2209_UART_CONFIG instance->config
#define TMC2209_UART_DATA instance->data

typedef enum
{
	TMC2209_UART_REG_GCONF = 0x00,
	TMC2209_UART_REG_GSTAT,
	TMC2209_UART_REG_IFCNT,
	TMC2209_UART_REG_NODECONF,
	TMC2209_UART_REG_OTP_PROG,
	TMC2209_UART_REG_OTP_READ,
	TMC2209_UART_REG_IOIN,
	TMC2209_UART_REG_FACTORY_CONF,

	TMC2209_UART_REG_IHOLD_IRUN = 0x10,
	TMC2209_UART_REG_TPOWERDOWN,
	TMC2209_UART_REG_TSTEP,
	TMC2209_UART_REG_TPWMTHRS,
	TMC2209_UART_REG_TCOOLTHRS,

	TMC2209_UART_REG_VACTUAL = 0x22,

	TMC2209_UART_REG_SGTHRS = 0x40,
	TMC2209_UART_REG_SG_RESULT,
	TMC2209_UART_REG_COOLCONF,

	TMC2209_UART_REG_MSCNT = 0x6A,
	TMC2209_UART_REG_MSCURACT,
	TMC2209_UART_REG_CHOPCONF,
	TMC2209_UART_REG_DRV_STATUS,
	TMC2209_UART_REG_PWMCONF,
	TMC2209_UART_REG_PWM_SCALE,
	TMC2209_UART_REG_PWM_AUTO,
}
TMC2209_UART_Registers_t;

static uint8_t TMC2209_UART_CRC(uint8_t *data, uint8_t len);
static HAL_StatusTypeDef TMC2209_UART_Write(TMC2209_UART_Instance_t *instance, TMC2209_UART_Registers_t reg, uint32_t data);
static HAL_StatusTypeDef TMC2209_UART_Read(TMC2209_UART_Instance_t *instance, TMC2209_UART_Registers_t reg, uint32_t *data);
static HAL_StatusTypeDef TMC2209_UART_WriteWithCheck(TMC2209_UART_Instance_t *instance, TMC2209_UART_Registers_t reg, uint32_t data);


static uint8_t TMC2209_UART_CRC(uint8_t *data, uint8_t len)
{
	int i,j;
	uint8_t crc; // CRC located in last byte of message
	uint8_t currentByte;
	crc = 0;
	for (i=0; i<(len-1); i++)
	{ // Execute for all bytes of a message
		currentByte = data[i]; // Retrieve a byte to be sent from Array
		for (j=0; j<8; j++)
		{
			if ((crc >> 7) ^ (currentByte&0x01)) // update CRC based result of XOR operation
			{
				crc = (crc << 1) ^ 0x07;
			}
			else
			{
				crc = (crc << 1);
			}
			currentByte = currentByte >> 1;
		} // for CRC bit
	} // for message byte
	return crc;
}

static HAL_StatusTypeDef TMC2209_UART_Write(TMC2209_UART_Instance_t *instance, TMC2209_UART_Registers_t reg, uint32_t data)
{
	uint8_t txd[8];
	txd[0] = TMC2209_UART_SYNC_BYTE;
	txd[1] = TMC2209_UART_CONFIG.node_addr;
	txd[2] = reg | 0x80;
	txd[3] = (uint8_t)((data >> 24)&0x000000FF);
	txd[4] = (uint8_t)((data >> 16)&0x000000FF);
	txd[5] = (uint8_t)((data >> 8 )&0x000000FF);
	txd[6] = (uint8_t)((data      )&0x000000FF);
	txd[7] = TMC2209_UART_CRC(txd, 8);

	return HAL_UART_Transmit(
		TMC2209_UART_DESCRIPTOR->huart,
		txd,
		8,
		10
	);
}

static HAL_StatusTypeDef TMC2209_UART_Read(TMC2209_UART_Instance_t *instance, TMC2209_UART_Registers_t reg, uint32_t *data)
{
	uint8_t txd[4], rxd[12], i;
	HAL_StatusTypeDef status;
	uint32_t tmp;

	txd[0] = TMC2209_UART_SYNC_BYTE;
	txd[1] = TMC2209_UART_CONFIG.node_addr;
//	txd[2] = (reg << 1) & 0xFE;
	txd[2] = reg & 0x7F;
	txd[3] = TMC2209_UART_CRC(txd, 4);

	status = HAL_UART_Transmit(
		TMC2209_UART_DESCRIPTOR->huart,
		txd,
		4,
		10
	);

	if ( status != HAL_OK ) return status;

	status = HAL_UART_Receive(
		TMC2209_UART_DESCRIPTOR->huart,
		rxd,
		9,
		10
	);

	if ( status != HAL_OK ) return status;

	if ( rxd[0+1] != TMC2209_UART_SYNC_BYTE ) return HAL_ERROR;
	if ( rxd[1+1] != 0xFF ) return HAL_ERROR;
//	if ( rxd[2] != ((reg << 1) & 0xFE) ) return HAL_ERROR;
	if ( rxd[2+1] != (reg & 0x7F) ) return HAL_ERROR;
	if ( rxd[7+1] != TMC2209_UART_CRC(&rxd[1], 8) ) return HAL_ERROR;

	tmp = 0;
	for ( i=0; i<4; i++ )
	{
		tmp |= (uint32_t)(rxd[3+i+1]) << ( 8 * (3-i) );
	}

	*data = tmp;

	return HAL_OK;
}


static HAL_StatusTypeDef TMC2209_UART_WriteWithCheck(TMC2209_UART_Instance_t *instance, TMC2209_UART_Registers_t reg, uint32_t data)
{
	HAL_StatusTypeDef status;
	uint32_t sequence_before = 0, sequence_after = 0;

	status = TMC2209_UART_Read(instance, TMC2209_UART_REG_IFCNT, &sequence_before);
	if ( status != HAL_OK ) return status;

	status = TMC2209_UART_Write(instance, reg, data);

	status = TMC2209_UART_Read(instance, TMC2209_UART_REG_IFCNT, &sequence_after);
	if ( status != HAL_OK ) return status;

	if ( sequence_before+1 != sequence_after ) return HAL_ERROR;
	return HAL_OK;
}


void TMC2209_UART_Init(TMC2209_UART_Instance_t *instance)
{

	TMC2209_UART_DATA.error_code = TMC2209_UART_WriteWithCheck(instance, 0x03, 0x00000700);
	TMC2209_UART_DATA.error_code = TMC2209_UART_Read(instance, 0x06, &(TMC2209_UART_DATA.var));
}

void TMC2209_UART_Read_CurrentSpeed(TMC2209_UART_Instance_t *instance)
{
	TMC2209_UART_DATA.error_code = TMC2209_UART_Read(instance, 0x06, &(TMC2209_UART_DATA.var));
}

#include <stdio.h>
void TMC2209_UART_Process(TMC2209_UART_Instance_t *instance)
{
	TMC2209_UART_DATA.error_code = TMC2209_UART_Read(instance, TMC2209_UART_REG_SG_RESULT, &(TMC2209_UART_DATA.var));
	printf("%d\t0x%08lx\n", TMC2209_UART_DATA.error_code, TMC2209_UART_DATA.var);
}
