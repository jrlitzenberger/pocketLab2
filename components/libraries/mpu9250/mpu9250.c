/*
 *  mpu9250.c File for function definition for read/write of all registers
 *  Author: John Litzenberger
 *  Date: 06132016
 */
 
 
 
#include "mpu9250.h"

uint8_t const mpu9250_whoami_reg_addr      = WHOAMI_REG;
uint8_t const mpu9250_accel_reg_addr       = ACCEL_REG;
uint8_t const mpu9250_gyro_reg_addr        = GYRO_REG;
uint8_t const mpu9250_compass_reg_addr     = COMPASS_REG;
uint8_t const mpu9250_intpincfg_reg_addr   = INTPINCFG_REG;
uint8_t const mpu9250_inten_reg_addr       = INTEN_REG;
uint8_t const mpu9250_usrctrl_reg_addr 	 	 = USRCTRL_REG;
uint8_t const mpu9250_pwrmgmt1_reg_addr    = PWRMGMT1_REG;
uint8_t const mpu9250_i2cmstrctrl_reg_addr = I2CMSTRCTRL_REG;
uint8_t const ak8963_control_reg_addr 	 	 = CTRL_REG;
uint8_t const ak8963_wia_reg_addr 	 	 	   = WIA_REG;

static uint8_t default_usrctrl[]    = {USRCTRL_REG, 0x01};
static uint8_t default_pwrmgmt1[]   = {PWRMGMT1_REG, 0x01};
static uint8_t default_i2cmstctrl[] = {I2CMSTRCTRL_REG, 0x00};
static uint8_t default_intpincfg[]  = {INTPINCFG_REG, 0x02};
static uint8_t default_inten[]      = {INTEN_REG, 0x01};
uint8_t const default_magcfg[]      = {CTRL_REG, 0x01};

app_twi_transfer_t const mpu9250_init_transfers[1] = 
{
	APP_TWI_WRITE(MPU9250_ADDR, default_pwrmgmt1, sizeof(default_pwrmgmt1), 0)
};

app_twi_transfer_t const mpu9250_passthrough_transfers[2] = 
{
	APP_TWI_WRITE(MPU9250_ADDR, default_intpincfg, sizeof(default_intpincfg), 0),
	APP_TWI_WRITE(MPU9250_ADDR, default_usrctrl, sizeof(default_usrctrl), 0)
	//APP_TWI_WRITE(MPU9250_ADDR, default_i2cmstctrl, sizeof(default_i2cmstctrl), 0),
	//APP_TWI_WRITE(MPU9250_ADDR, default_inten, sizeof(default_inten), 0)
};
/*
app_twi_transfer_t const ak8963_enable_transfer[1] = 
{
	APP_TWI_WRITE(AK8963_ADDR, default_magcfg, sizeof(default_magcfg), 0)
};
*/
