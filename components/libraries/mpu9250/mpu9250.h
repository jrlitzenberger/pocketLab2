/*
 *  mpu9250 header file 
 *  Author: John Litzenberger
 *  Date: 06132016
 */
 
 
#ifndef MPU9250_H__
#define MPU9250_H__

#include "app_twi.h"
 

#define MPU9250_ADDR       0x68 
#define AK8963_ADDR	 	 	 	 0x0C
#define WHOAMI_REG         0x75
#define ACCEL_REG	 	 	 	 	 0x3B
#define GYRO_REG	 	 	 	 	 0x43
#define COMPASS_REG        0x03
#define INTPINCFG_REG      0x37
#define INTEN_REG          0x38
#define USRCTRL_REG	 	 	   0x6A
#define PWRMGMT1_REG       0x6B
#define I2CMSTRCTRL_REG    0x24
#define CTRL_REG 	 	 	 	   0x0A
#define WIA_REG	 	 	  	 	 0x00
 
extern uint8_t const mpu9250_whoami_reg_addr;
extern uint8_t const mpu9250_accel_reg_addr;
extern uint8_t const mpu9250_gyro_reg_addr;
extern uint8_t const mpu9250_compass_reg_addr;
extern uint8_t const mpu9250_intpincfg_reg_addr;
extern uint8_t const mpu9250_inten_reg_addr;
extern uint8_t const mpu9250_usrctrl_reg_addr;
extern uint8_t const mpu9250_pwrmgmt1_reg_addr;
extern uint8_t const mpu9250_i2cmstrctrl_reg_addr;
extern uint8_t const ak8963_control_reg_addr; 
extern uint8_t const ak8963_wia_reg_addr;

 
#define MPU9250_READ(p_reg_addr, p_buffer, byte_cnt) \
    APP_TWI_WRITE(MPU9250_ADDR, p_reg_addr, 1,        APP_TWI_NO_STOP), \
    APP_TWI_READ (MPU9250_ADDR, p_buffer,   byte_cnt, 0)
		
#define MPU9250_WRITE(p_reg_addr, p_buffer, byte_cnt) \
		APP_TWI_WRITE(MPU9250_ADDR, p_reg_addr, byte_cnt, 0)

#define AK8963_READ(p_reg_addr, p_buffer, byte_cnt) \
		APP_TWI_WRITE(AK8963_ADDR, p_reg_addr, 1,         APP_TWI_NO_STOP), \
		APP_TWI_READ (AK8963_ADDR, p_reg_addr,  byte_cnt, 0)
		
#define AK8963_WRITE(p_reg_addr, p_buffer, byte_cnt) \
	  APP_TWI_WRITE(AK8963_ADDR, p_reg_addr, byte_cnt, 0)
	
#define MPU9250_READ_WHOAMI(p_buffer) \
	MPU9250_READ(&mpu9250_whoami_reg_addr, p_buffer, 1)
	
#define MPU9250_READ_ACCEL(p_buffer) \
	MPU9250_READ(&mpu9250_accel_reg_addr, p_buffer, 6)
	
#define MPU9250_READ_GYRO(p_buffer) \
	MPU9250_READ(&mpu9250_gyro_reg_addr, p_buffer, 6)

#define MPU9250_READ_COMPASS(p_buffer) \
	AK8963_READ(&mpu9250_compass_reg_addr, p_buffer, 6)
	
#define MPU9250_READ_INT_PIN_CFG(p_buffer) \
	MPU9250_READ(&mpu9250_intpincfg_reg_addr, p_buffer, 1)

#define AK8963_READ_WIA(p_buffer) \
  AK8963_READ(&ak8963_wia_reg_addr, p_buffer, 1)
	
#define MPU9250_WRITE_INT_PIN_CFG(p_buffer) \
	MPU9250_WRITE(&mpu9250_intpincfg_reg_addr, p_buffer, 1)
	
extern app_twi_transfer_t const mpu9250_init_transfers[1];
extern app_twi_transfer_t const mpu9250_passthrough_transfers[2];
//extern app_twi_transfer_t const ak8963_enable_transfer[1];
//extern app_twi_transfer_t const mpu9250_
	
#endif
