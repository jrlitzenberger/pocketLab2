/*
 *  bme280c File for function definition for read/write of all registers
 *  Author: John Litzenberger
 *  Date: 06132016
 */
 
 
 
#include "BME280.h"

uint8_t const bme280_data_reg_addr     = DATA_REG;
uint8_t const bme280_id_reg_addr       = ID_REG;
uint8_t const bme280_ctrlmeas_reg_addr = CTRLMEAS_REG;
uint8_t const bme280_ctrlhum_reg_addr  = CTRLHUM_REG;
uint8_t const bme280_cal1_reg_addr     = CAL1_REG;
uint8_t const bme280_cal2_reg_addr 	   = CAL2_REG;

static uint8_t const default_ctrlmeas[] = {CTRLMEAS_REG, 0x27};
static uint8_t const defualt_ctrlhum[]  = {CTRLHUM_REG, 0x01};
app_twi_transfer_t const bme280_init_transfers[2] = 
{
	APP_TWI_WRITE(BME280_ADDR, defualt_ctrlhum, sizeof(defualt_ctrlhum), 0),
	APP_TWI_WRITE(BME280_ADDR, default_ctrlmeas, sizeof(default_ctrlmeas), 0)
};


