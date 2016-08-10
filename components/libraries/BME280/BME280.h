/*
 *  BME280 header file 
 *  Author: John Litzenberger
 *  Date: 06132016
 */
 
 
#ifndef BME280_H__
#define BME280_H__

#include "app_twi.h"
 

#define BME280_ADDR   0x76
#define DATA_REG	 	  0xF7
#define ID_REG	      0xD0
#define CTRLMEAS_REG  0xF4
#define CTRLHUM_REG   0xF2
#define CAL1_REG	 	  0x88
#define CAL2_REG	 	  0xE1

 
extern uint8_t const bme280_data_reg_addr;
extern uint8_t const bme280_id_reg_addr;
extern uint8_t const bme280_ctrlmeas_reg_addr;
extern uint8_t const bme280_ctrlhum_reg_addr;
extern uint8_t const bme280_cal1_reg_addr;
extern uint8_t const bme280_cal2_reg_addr;
 
#define BME280_READ(p_reg_addr, p_buffer, byte_cnt) \
    APP_TWI_WRITE(BME280_ADDR, p_reg_addr, 1,        APP_TWI_NO_STOP), \
    APP_TWI_READ (BME280_ADDR, p_buffer,   byte_cnt, 0)
			
#define BME280_READ_DATA(p_buffer) \
	BME280_READ(&bme280_data_reg_addr, p_buffer, 8)
	
#define BME280_READ_ID(p_buffer) \
	BME280_READ(&bme280_id_reg_addr, p_buffer, 1)
	
#define BME280_READ_CAL1(p_buffer) \
  BME280_READ(&bme280_cal1_reg_addr, p_buffer, 26)
	
#define BME280_READ_CAL2(p_buffer) \
  BME280_READ(&bme280_cal2_reg_addr, p_buffer, 7)
	
extern app_twi_transfer_t const bme280_init_transfers[2];
 
#endif
