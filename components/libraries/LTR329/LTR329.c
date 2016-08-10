/*
 *  LTR329 c File for function definition for read/write of all registers
 *  Author: John Litzenberger
 *  Date: 06132016
 */
 
 
 
#include "LTR329.h"

uint8_t const ltr329_light_data_addr   = LIGHT_DATA_REG;
uint8_t const ltr329_partid_reg_addr   = PARTID_REG;
uint8_t const ltr329_manufid_reg_addr  = MANID_REG;
uint8_t const ltr329_alscontr_reg_addr = ALSCONTR_REG;

static uint8_t const default_alscontr[] = {ALSCONTR_REG, 0x01};

app_twi_transfer_t const ltr329_init_transfers[1] = 
{
	APP_TWI_WRITE(LTR329_ADDR, default_alscontr, sizeof(default_alscontr), 0)	
};

