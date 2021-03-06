#ifndef __APL_H__
#define __APL_H__

#include <stdio.h>
#include <stdint.h>
#include <inttypes.h>
#include "dll/includes/dll.h"

#define USER														0xff
#define SENSOR													0xaa
#define REGULATE												0xbb
#define STATUS													0xcc

#define CC_MONITOR											0x04
#define PC_VALUE												0x01
#define PC_REGULATION										0x02

#define ON										0x01
#define OFF										0x00

//*****************************************************************************
//
// Defines values for temperature's API
//
//*****************************************************************************
#define DEFAULT_TEMPERATURE							2500
#define AMBI_TEMP_MIN										1500 
#define AMBI_TEMP_MAX									 	3500
#define TEMP_OFFSET_25										25
#define TEMP_OFFSET_50										50	
#define TEMP_OFFSET_100									 100
#define PC_VAL_OFFSET											0
#define CC_OFFSET													2
#define PC_REG_OFFSET											1

//*****************************************************************************
//
// Global variables
//
//*****************************************************************************
#pragma pack(push)
#pragma pack(1)
typedef struct sAPLData_t
{
	uint16_t devID;
	uint8_t packNum;
	uint32_t data;
	uint32_t userData;
	uint8_t regulation;
	uint8_t status;
} APLData_t;
#pragma pack(pop)


//*****************************************************************************
//
// Defining devices address
//
//*****************************************************************************

void aplInit(void);
void aplSendData(uint32_t data, uint16_t devID, uint8_t port);
void aplProcessCommand(void);
void updateData(Data_t *pData, uint8_t port);
void resetStatus(uint16_t devID, 
									uint8_t value);
uint32_t getData(uint16_t devID,
								 uint8_t flag
								 );

void regulateTemperature(uint8_t state,
												 int16_t diference);


#endif //__APL_H__
