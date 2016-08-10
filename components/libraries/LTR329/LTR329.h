/*
 *  LTR329 header file 
 *  Author: John Litzenberger
 *  Date: 06132016
 */
 
 
#ifndef LTR329_H__
#define LTR329_H__

#include "app_twi.h"
 

#define LTR329_ADDR         0x29
#define LIGHT_DATA_REG	    0x88
#define PARTID_REG	        0x86
#define MANID_REG           0x87
#define ALSCONTR_REG        0x80

 
extern uint8_t const ltr329_light_data_addr;
extern uint8_t const ltr329_partid_reg_addr;
extern uint8_t const ltr329_manufid_reg_addr;
extern uint8_t const ltr329_alscontr_reg_addr;
 
#define LTR329_READ(p_reg_addr, p_buffer, byte_cnt) \
    APP_TWI_WRITE(LTR329_ADDR, p_reg_addr, 1,        APP_TWI_NO_STOP), \
    APP_TWI_READ (LTR329_ADDR, p_buffer,   byte_cnt, 0)
			
#define LTR329_READ_LIGHT_DATA(p_buffer) \
	LTR329_READ(&ltr329_light_data_addr, p_buffer, 4)
	
#define LTR329_READ_PARTID(p_buffer) \
	LTR329_READ(&ltr329_partid_reg_addr, p_buffer, 1)
	
#define LTR329_READ_MANID(p_buffer) \
  LTR329_READ(&ltr329_manufid_reg_addr, p_buffer, 1)
	
extern app_twi_transfer_t const ltr329_init_transfers[1];
 
#endif
