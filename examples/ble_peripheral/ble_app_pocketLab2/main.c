/* Copyright (c) 2014 Nordic Semiconductor. All Rights Reserved.
 *
 * The information contained herein is property of Nordic Semiconductor ASA.
 * Terms and conditions of usage are described in detail in NORDIC
 * SEMICONDUCTOR STANDARD SOFTWARE LICENSE AGREEMENT.
 *
 * Licensees are granted free, non-transferable use of the information. NO
 * WARRANTY of ANY KIND is provided. This heading must NOT be removed from
 * the file.
 *
 */

/** @file
 *
 * @defgroup ble_sdk_uart_over_ble_main main.c
 * @{
 * @ingroup  ble_sdk_app_nus_eval
 * @brief    UART over BLE application main file.
 *
 * This file contains the source code for a sample application that uses the Nordic UART service.
 * This application uses the @ref srvlib_conn_params module.
 */

#include <stdint.h>
#include <string.h>
#include "nordic_common.h"
#include "nrf.h"
#include "nrf_gpio.h"
#include "nrf_delay.h"
#include "ble_hci.h"
#include "ble_advdata.h"
#include "ble_advertising.h"
#include "ble_conn_params.h"
#include "state_machine.h"
#include "softdevice_handler.h"
#include "app_timer.h"
#include "app_button.h"
#include "mpu9250.h"
#include "BME280.h"
#include "LTR329.h"
#include "ble_pcs.h"
#include "ble_bas.h"
#include "ble_gss.h"
#include "app_uart.h"
#include "app_pwm.h"
#include "app_twi.h"
#include "app_util_platform.h"
#include "bsp.h"
#include "bsp_btn_ble.h"


#define IS_SRVC_CHANGED_CHARACT_PRESENT 0                                           /**< Include the service_changed characteristic. If not enabled, the server's database cannot be changed for the lifetime of the device. */

#define CENTRAL_LINK_COUNT              0                                           /**< Number of central links used by the application. When changing this number remember to adjust the RAM settings*/
#define PERIPHERAL_LINK_COUNT           1                                           /**< Number of peripheral links used by the application. When changing this number remember to adjust the RAM settings*/

#define DEVICE_NAME                     "Smart Module"                               /**< Name of device. Will be included in the advertising data. */
#define PCS_SERVICE_UUID_TYPE           BLE_UUID_TYPE_VENDOR_BEGIN                  /**< UUID type for the Nordic UART Service (vendor specific). */

#define APP_ADV_INTERVAL                64                                          /**< The advertising interval (in units of 0.625 ms. This value corresponds to 40 ms). */
#define APP_ADV_TIMEOUT_IN_SECONDS      180                                         /**< The advertising timeout (in units of seconds). */

#define APP_TIMER_PRESCALER             0                                           /**< Value of the RTC1 PRESCALER register. */
#define APP_TIMER_OP_QUEUE_SIZE         7                                           /**< Size of timer operation queues. */

#define MAX_PENDING_TRANSACTIONS        6

#define PWM1_INTERVAL	 	 	 	 	 	 	 	 	  APP_TIMER_TICKS(20, APP_TIMER_PRESCALER)
#define SENSOR_INTERVAL	 	 	 	  	 	 	  APP_TIMER_TICKS(125, APP_TIMER_PRESCALER)
#define ADC_MIC_INTERVAL	              APP_TIMER_TICKS(10, APP_TIMER_PRESCALER)
#define STATE_MACHINE_INTERVAL          APP_TIMER_TICKS(100, APP_TIMER_PRESCALER)
#define BUTTON_CHECK_INTERVAL           APP_TIMER_TICKS(50, APP_TIMER_PRESCALER)

#define MIN_CONN_INTERVAL               MSEC_TO_UNITS(20, UNIT_1_25_MS)             /**< Minimum acceptable connection interval (20 ms), Connection interval uses 1.25 ms units. */
#define MAX_CONN_INTERVAL               MSEC_TO_UNITS(75, UNIT_1_25_MS)             /**< Maximum acceptable connection interval (75 ms), Connection interval uses 1.25 ms units. */
#define SLAVE_LATENCY                   0                                           /**< Slave latency. */
#define CONN_SUP_TIMEOUT                MSEC_TO_UNITS(4000, UNIT_10_MS)             /**< Connection supervisory timeout (4 seconds), Supervision Timeout uses 10 ms units. */
#define FIRST_CONN_PARAMS_UPDATE_DELAY  APP_TIMER_TICKS(5000, APP_TIMER_PRESCALER)  /**< Time from initiating event (connect or start of notification) to first time sd_ble_gap_conn_param_update is called (5 seconds). */
#define NEXT_CONN_PARAMS_UPDATE_DELAY   APP_TIMER_TICKS(30000, APP_TIMER_PRESCALER) /**< Time between each call to sd_ble_gap_conn_param_update after the first call (30 seconds). */
#define MAX_CONN_PARAMS_UPDATE_COUNT    3                                           /**< Number of attempts before giving up the connection parameter negotiation. */

#define MAX_BUFFER_SIZE	 	 	 	 	 		 	 	42

#define DEAD_BEEF                       0xDEADBEEF                                  /**< Value used as error code on stack dump, can be used to identify stack location on stack unwind. */

#define UART_TX_BUF_SIZE                256                                         /**< UART TX buffer size. */
#define UART_RX_BUF_SIZE                256                                         /**< UART RX buffer size. */

//#define VREF	 	 	 	 	 	 								3.1  //was 3.22
#define ADC_RES													255
#define MAX_ADC	 	 	 	 	 	 	 	 	 	 	 	  3.6

//#define MIN_MIC_THRESHOLD               (VREF/2 - 0.025)
//#define MAX_MIC_THRESHOLD 	 	 	        (VREF/2 + 0.025)

static ble_pcs_t                        m_pcs;                                      /**< Structure to identify the Nordic UART Service. */
static ble_bas_t                        m_bas;                                     /**< Structure used to identify the battery service. */
static ble_gss_t												m_gss;

static uint16_t                         m_conn_handle = BLE_CONN_HANDLE_INVALID;    /**< Handle of the current connection. */

static ble_uuid_t                       m_adv_uuids[] = {{BLE_UUID_PCS_SERVICE, PCS_SERVICE_UUID_TYPE}};  /**< Universally unique service identifier. */

static uint8_t pwmMode = 0;
static uint16_t pwmCount = 0;
static bool turn = true;
static bool enablexyz = true;
static bool enablebar = true;
static bool enablelight = true;

static uint8_t m_buffer[MAX_BUFFER_SIZE];
static uint8_t accel[6];
static uint8_t gyro[6];
static uint8_t compass[6];
static uint8_t bar[8];
static uint8_t light[4];
static uint8_t bme280calT[20];
static uint8_t bme280calP[20];
static uint8_t bme280calH[20];

static uint16_t micInputRead;
static double micOutputVoltage = 0; 
static double m1, m2, b1, b2, max_thres, min_thres, baseline;
static app_twi_t m_app_twi = APP_TWI_INSTANCE(0); 
static volatile bool ready_flag;            // A flag indicating PWM status.


APP_TIMER_DEF(pwm1_timer_id);
APP_TIMER_DEF(sensor_timer_id);
APP_TIMER_DEF(adc_mic_timer_id);
APP_TIMER_DEF(state_machine_timer_id);
APP_TIMER_DEF(button_check_timer_id);
APP_PWM_INSTANCE(PWM1, 1);              //create instance PWM1 with TIMER1
//APP_PWM_INSTANCE(PWM2, 1);
//APP_PWM_INSTANCE(PWM3, 1);
 

void (*state_machine_table[])(void);

typedef enum{
	Idle = 0, Init, Seeking, Connecting, Connected, Poweroff, Lowbattery, Syncing, Checkcharge,
	Charging, Chargecomplete
} State_Machine_Type;

void(*state_machine_table[])(void) = {
	idle,
	init,
	seeking,
	connecting,
	connected,
	poweroff,
	lowbattery,
	syncing,
	checkcharge,
	charging,
	chargecomplete
};

State_Machine_Type current_state = Idle;

void state_machine_run(void)
{
	state_machine_table[current_state]();
}

void set_state(State_Machine_Type state)
{
	current_state = state;
}

void idle(void)
{
	nrf_gpio_pin_clear(RED_LED);
	nrf_gpio_pin_clear(GREEN_LED);
}

void init(void)
{
	
}

void seeking(void)
{
  static uint8_t cnt;
	static bool toggle;
	
	if(!cnt || (cnt == 0)) { cnt = 1; }
  else
	{
		cnt++;
		if((cnt % 2) == 0)
		{
			if(!toggle){ nrf_gpio_pin_clear(RED_LED); nrf_gpio_pin_set(GREEN_LED); toggle = true; }
			else{ nrf_gpio_pin_clear(GREEN_LED); nrf_gpio_pin_set(RED_LED); toggle = false; }
			cnt = 0;
		}
	}	
}

void connecting(void)
{
  static uint8_t cnt;
	
	if(!cnt || (cnt == 0)) { cnt = 1; }
	else
	{
		cnt++;
		if(((cnt % 2) == 1) && (cnt <= 10))
		{
			nrf_gpio_pin_clear(RED_LED);
			nrf_gpio_pin_set(GREEN_LED);
		}
		else
		{
      nrf_gpio_pin_clear(GREEN_LED);
		}
		if(cnt > 10)
		{
			cnt = 0;
			set_state(Connected);
		}
	}
}

void connected(void)
{
  static uint8_t cnt;
	
	if(!cnt || (cnt == 0)) { cnt = 1; }
	else
	{
		if(cnt <= 16)
		{
				nrf_gpio_pin_clear(RED_LED);
				nrf_gpio_pin_clear(GREEN_LED);			
		}
		if((cnt < 18) && (cnt > 16))
		{
				nrf_gpio_pin_set(RED_LED);
				nrf_gpio_pin_set(GREEN_LED);		
		}
		if(cnt > 17)
		{
			cnt = 0;
		}
		cnt++;
	}	
}

void poweroff(void)
{
		static uint8_t cnt;
		
		if(!cnt || (cnt == 0)) { cnt = 1; }
		else
		{
			cnt++;
			nrf_gpio_pin_set(RED_LED);
			nrf_gpio_pin_clear(GREEN_LED);
		}	

		if(cnt >= 30)
		{
			cnt = 0;
			nrf_gpio_pin_clear(RED_LED);
			nrf_gpio_pin_clear(GREEN_LED);
			nrf_gpio_pin_clear(LM3407_EN);
			nrf_gpio_pin_clear(ISET2);
			app_pwm_uninit(&PWM1);
				// Prepare wakeup buttons.
			bsp_btn_ble_sleep_mode_prepare();
			// Go to system-off mode (this function will not return; wakeup will cause a reset).
			sd_power_system_off();
		}
}

void lowbattery(void)
{
	
}

void syncing(void)
{
	
}

void checkcharge(void)
{
	
}

void charging(void)
{
	
}

void chargecomplete(void)
{
	
}

void pwm_ready_callback(uint32_t pwm_id)    // PWM callback function
{
    ready_flag = true;
}

void read_data_cb(ret_code_t result, void * p_user_data)
{
	accel[0]   = m_buffer[0];
	accel[1]   = m_buffer[1];
	accel[2]   = m_buffer[2];
	accel[3]   = m_buffer[3];
	accel[4]   = m_buffer[4];
	accel[5]   = m_buffer[5];
	gyro[0]    = m_buffer[6];
	gyro[1]    = m_buffer[7];
	gyro[2]    = m_buffer[8];
	gyro[3]    = m_buffer[9];
	gyro[4]    = m_buffer[10];
	gyro[5]    = m_buffer[11];
	compass[0] = m_buffer[12];
	compass[1] = m_buffer[13];
	compass[2] = m_buffer[14];
	compass[3] = m_buffer[15];
	compass[4] = m_buffer[16];
	compass[5] = m_buffer[17];
	bar[0]	   = m_buffer[18];
	bar[1]     = m_buffer[19];
	bar[2]	   = m_buffer[20];
	bar[3]     = m_buffer[21];	
	bar[4]	   = m_buffer[22];
	bar[5]     = m_buffer[23];
	bar[6]	   = m_buffer[24];
	bar[7]     = m_buffer[25];	
	light[0]   = m_buffer[26];
	light[1]   = m_buffer[27];
	light[2]   = m_buffer[28];
	light[3]   = m_buffer[29];	
}

void read_bme280_cal_cb(ret_code_t result, void * p_user_data)
{
	bme280calT[0] = m_buffer[0];
	bme280calT[1] = m_buffer[1];
	bme280calT[2] = m_buffer[2];
	bme280calT[3] = m_buffer[3];
	bme280calT[4] = m_buffer[4];
	bme280calT[5] = m_buffer[5];
	
	bme280calP[0] = m_buffer[6];
	bme280calP[1] = m_buffer[7];
	bme280calP[2] = m_buffer[8];
	bme280calP[3] = m_buffer[9];
	bme280calP[4] = m_buffer[10];
	bme280calP[5] = m_buffer[11];
	bme280calP[6] = m_buffer[12];
	bme280calP[7] = m_buffer[13];
	bme280calP[8] = m_buffer[14];
	bme280calP[9] = m_buffer[15];
	bme280calP[10] = m_buffer[16];
	bme280calP[11] = m_buffer[17];
	bme280calP[12] = m_buffer[18];
	bme280calP[13] = m_buffer[19];
	bme280calP[14] = m_buffer[20];
	bme280calP[15] = m_buffer[21];
	bme280calP[16] = m_buffer[22];
	bme280calP[17] = m_buffer[23];
	
	bme280calH[0] = m_buffer[25];
	bme280calH[1] = m_buffer[26];
	bme280calH[2] = m_buffer[27];
	bme280calH[3] = m_buffer[28];
	bme280calH[4] = m_buffer[29];
	bme280calH[5] = m_buffer[30];
	bme280calH[6] = m_buffer[31];
	bme280calH[7] = m_buffer[32];

}
/**@brief Function for assert macro callback.
 *
 * @details This function will be called in case of an assert in the SoftDevice.
 *
 * @warning This handler is an example only and does not fit a final product. You need to analyse 
 *          how your product is supposed to react in case of Assert.
 * @warning On assert from the SoftDevice, the system can only recover on reset.
 *
 * @param[in] line_num    Line number of the failing ASSERT call.
 * @param[in] p_file_name File name of the failing ASSERT call.
 */
void assert_nrf_callback(uint16_t line_num, const uint8_t * p_file_name)
{
    app_error_handler(DEAD_BEEF, line_num, p_file_name);
}


/**@brief Function for the GAP initialization.
 *
 * @details This function will set up all the necessary GAP (Generic Access Profile) parameters of 
 *          the device. It also sets the permissions and appearance.
 */
static void gap_params_init(void)
{
    uint32_t                err_code;
    ble_gap_conn_params_t   gap_conn_params;
    ble_gap_conn_sec_mode_t sec_mode;

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&sec_mode);
    
    err_code = sd_ble_gap_device_name_set(&sec_mode,
                                          (const uint8_t *) DEVICE_NAME,
                                          strlen(DEVICE_NAME));
    APP_ERROR_CHECK(err_code);

    memset(&gap_conn_params, 0, sizeof(gap_conn_params));

    gap_conn_params.min_conn_interval = MIN_CONN_INTERVAL;
    gap_conn_params.max_conn_interval = MAX_CONN_INTERVAL;
    gap_conn_params.slave_latency     = SLAVE_LATENCY;
    gap_conn_params.conn_sup_timeout  = CONN_SUP_TIMEOUT;

    err_code = sd_ble_gap_ppcp_set(&gap_conn_params);
    APP_ERROR_CHECK(err_code);
}

static void gss_xyz_config_handler(ble_gss_t * p_gss, uint8_t * p_data, uint16_t length)
{
	static unsigned char command[20];
	for (uint32_t i = 0; i < length; i++)
	{
		command[i] = p_data[i];
	}
	if(command[0] == 0x00)
		enablexyz = false;
	if(command[0] == 0x01)
		enablexyz = true;
}

static void gss_mic_config_handler(ble_gss_t * p_gss, uint8_t * p_data, uint16_t length)
{
	static unsigned char command[20];
	for (uint32_t i = 0; i < length; i++)
	{
		command[i] = p_data[i];
	}
	if(command[0] == 0xFF)   //calibrate the microphone
	{
			pwmMode = 0;
			baseline =(((double)micInputRead*MAX_ADC) / ADC_RES);  //ADC hexidecimal range (255), VCC (3V);  
			max_thres = baseline + 0.025;
			min_thres = baseline - 0.025;
			m1 = (0 - 100)/(3 - max_thres);
			b1 = (m1 * -3.0);
			m2 = (0 - 100)/(0 - min_thres);
			b2 = 0;		
	}
}

static void gss_bar_config_handler(ble_gss_t * p_gss, uint8_t * p_data, uint16_t length)
{
	static unsigned char command[20];
	for (uint32_t i = 0; i < length; i++)
	{
		command[i] = p_data[i];
	}
	if(command[0] == 0x00)
		enablebar = false;
	if(command[0] == 0x01)
		enablebar = true;
	if(command[0] == 0x02)
	{
		ble_gss_bar_cal_send(&m_gss, bme280calT);
	}
	if(command[0] == 0x03)
	{
		ble_gss_bar_cal_send(&m_gss, bme280calP);
	}
	if(command[0] == 0x04)
	{
		ble_gss_bar_cal_send(&m_gss, bme280calH);
	}
}

static void gss_light_config_handler(ble_gss_t * p_gss, uint8_t * p_data, uint16_t length)
{
	
}

static void button_check_timer_handler(void * p_context)
{
	static uint8_t button_1;
	
	static uint8_t pulsecount; 
	static uint8_t buttonpresscount;
	static uint8_t timeout;
	
	button_1    = nrf_gpio_pin_read(BUTTON_1);
	
	if(!timeout){ timeout = 0; }
  if(!pulsecount){ pulsecount = 0; }
  if(!buttonpresscount){ buttonpresscount = 0; }
	
	/*
	this is the main if block for the MFB.  things can only happen if the button is pressed or not pressed...meaning I only validate 
  a double tap in hold if the button is actually still being held.  a double tap can only happen after the button is released
  Doing it this way ensures that I get the correct answer from user input every time
  ****IF BUTTON IS PRESSED*****
	1) start counting length of press
  2) initialize timeout between events
  3) what kind of event is it?
	****IF BUTTON IS NOT PRESSED*****
	1)when button is released check to see if it has been pressed before
		a) increment timeout handler because we know something has happened
		b) if timeout has expired we must deal with whatever has happened, process it and change states
			 increment button press counter, clear pulsecounter
	2)if nothing has happened or something is done happening clear button press count
   */
	if(!button_1)
  {
		pulsecount++;
		timeout = 0;
		if((pulsecount >= 60) && (buttonpresscount == 0))
		{
			set_state(Poweroff);
		}
		/* FUTURE USE
		if((pulsecount > 3) && (buttonpresscount == 1))
		{
       doubleTapHold = true;
		}
		if((pulsecount > 3) && (buttonpresscount == 2))
		{
			 tripleTapHold = true;
		}
		*/
	}
	else
	{
		/*  FUTURE USE
		if(doubleTapHold || tripleTapHold)
		{
			buttonpresscount = 0;
			pulsecount = 0;
			doubleTapHold = false;
			tripleTapHold = false;
		}
		*/
		if((pulsecount >= 1) && (pulsecount <= 5))
		{
			pulsecount = 0;
			buttonpresscount++;
		}
		else
		{
			timeout++;
			if(timeout >= 4)
			{				
				if(buttonpresscount == 1)
				{				
					turn = true;
					pwmCount = 0;
					if(pwmMode < 4)
					{
						nrf_gpio_pin_set(LM3407_EN);
						pwmMode++;
					}
					else
					{
						pwmMode = 0;
					}
					
				}
				if(buttonpresscount == 2)
				{
					//future use
				}			
				if(buttonpresscount == 3)
				{
		      //future use
				}					
				buttonpresscount = 0;
				pulsecount = 0;
				timeout = 0;
			}
		}
	}	
}

/**@brief Function for handling the data from the PWM Service.
 *
 * @details This function will process the data received from the Nordic UART BLE Service and send
 *          it to the UART module.
 *
 * @param[in] p_nus    Nordic UART Service structure.
 * @param[in] p_data   Data to be send to UART module.
 * @param[in] length   Length of the data.
 */
/**@snippet [Handling the data received over BLE] */
static void pcs_pwm1_handler(ble_pcs_t * p_pcs, uint8_t * p_data, uint16_t length)
{
	static unsigned char command[20];
	static int test = 0;
	memset(command, 0, sizeof(command));
	for (uint32_t i = 0; i < length; i++)
	{
		command[i] = p_data[i];
	}
	switch(command[0])
	{
		case 0x00:  //turn OFF
			pwmMode = 0;
		break;
		
		case 0x01:  //turn ON
			nrf_gpio_pin_set(LM3407_EN);
			pwmCount = 100 - command[1];
			turn = true;		
			pwmMode = 1;			
		break;

		case 0x02:  //music
			nrf_gpio_pin_set(LM3407_EN);
			pwmMode = 2;			
		break;
		
		case 0x03:  //dimming
			nrf_gpio_pin_set(LM3407_EN);
			pwmCount = 0;
			turn = true;
			pwmMode = 3;			
		break;
		
		case 0x04:  //strobe
			nrf_gpio_pin_set(LM3407_EN);
		  pwmCount = 0;
		  turn = true;
		  pwmMode = 4;
		break;
		
		defualt:
		break;
	}
}
/**@snippet [Handling the data received over BLE] */

static void pcs_pwm2_handler(ble_pcs_t * p_pcs, uint8_t * p_data, uint16_t length)
{
		for (uint32_t i = 0; i < length; i++)
		{
			
		}
}

static void pcs_pwm3_handler(ble_pcs_t * p_pcs, uint8_t * p_data, uint16_t length)
{
		for (uint32_t i = 0; i < length; i++)
		{
			
		}
}

static void state_machine_timer_handler(void * p_context)
{
		state_machine_run();
}

static void adc_mic_timer_handler(void * p_context)
{
		uint32_t p_is_running = 0;
			
		sd_clock_hfclk_request();
		while(! p_is_running) {  							//wait for the hfclk to be available
			sd_clock_hfclk_is_running((&p_is_running));
		}               
		NRF_ADC->TASKS_START = 1;							//Start ADC sampling	
}


static void sensor_timer_handler(void * p_context)
{
		//read all sensors at this point...
	
	static app_twi_transfer_t const transfers[] =
	{
			MPU9250_READ_ACCEL(&m_buffer[0]),
			MPU9250_READ_GYRO(&m_buffer[6]),
			//APP_TWI_WRITE(AK8963_ADDR, default_magcfg, sizeof(default_magcfg), 0),
		  //MPU9250_READ_COMPASS(&m_buffer[12]),
		  //AK8963_READ_WIA(&m_buffer[12]),
			//BME280_READ_ID(&m_buffer[12]),
		  //LTR329_READ_MANID(&m_buffer[12])
		  BME280_READ_DATA(&m_buffer[18]),
		  LTR329_READ_LIGHT_DATA(&m_buffer[26])
		  
	};
	static app_twi_transaction_t const transaction = 
	{
		.callback	 	 	 	  = read_data_cb,
		.p_user_data	 	 	= NULL,
		.p_transfers 	 	  = transfers,
		.number_of_transfers = sizeof(transfers)/sizeof(transfers[0])
	};
	APP_ERROR_CHECK(app_twi_schedule(&m_app_twi, &transaction));
	

	if(enablexyz)
	{
		ble_gss_xyz_measurement_send(&m_gss, accel, gyro, compass);
	}
	if(enablebar)
	{
		ble_gss_bar_measurement_send(&m_gss, bar);
	}
	if(enablelight)
	{
		ble_gss_light_measurement_send(&m_gss, light);
	}
}

static void pwm1_timer_handler(void * p_context)
{
	  if(pwmMode == 0)  //off
		{
			pwmCount = 100;
			nrf_gpio_pin_clear(LM3407_EN);
		}
		
	  if(pwmMode == 1) //on
		{

		}
		
		if(pwmMode == 2)  //music
		{
			//compute mic input to PWM percentage
			micOutputVoltage = (((double)micInputRead*MAX_ADC) / ADC_RES);  //ADC hexidecimal range (255), VCC (3V)
			
			if(micOutputVoltage > max_thres)
			{
				pwmCount = (uint16_t)((m1*micOutputVoltage) + b1);
			}
			else if((micOutputVoltage > min_thres) && (micOutputVoltage < max_thres))
			{
				pwmCount = 85;
			}
			else
			{
				pwmCount = (uint16_t)((m2*micOutputVoltage) + b2);
			}
			/*
			if(micOutputVoltage > MAX_MIC_THRESHOLD)
			{
				//pwmCount = (uint16_t)((74.07*micOutputVoltage) - 122.2);
				pwmCount = (uint16_t)((-70.17*micOutputVoltage) + 210.52);
			}
			else if((micOutputVoltage > MIN_MIC_THRESHOLD)  && (micOutputVoltage < MAX_MIC_THRESHOLD))
			{
				pwmCount = 75;
			}
			else
			{
				//pwmCount = (uint16_t)((-64.51*micOutputVoltage) + 100);
				pwmCount = (uint16_t)(65.56*micOutputVoltage);
			}			
			*/			
		}
		
		if(pwmMode == 3)  //dimming
		{
			if(turn)
			{
				pwmCount++;
				if(pwmCount == 100)
				{
					turn = false;
				}
			}
			if(!turn)
			{
				pwmCount--;
				if(pwmCount == 0)
				{
					turn = true;
				}
			}
		}
		
		if(pwmMode == 4)  //strobe
		{
			if(turn)
			{
				pwmCount += 50;
				if(pwmCount == 100)
				{
					turn = false;
				}
			}
			if(!turn)
			{
				pwmCount -= 50;
				if(pwmCount == 0)
				{
					turn = true;
				}
			}	
		}	
  while (app_pwm_channel_duty_set(&PWM1, 0, pwmCount) == NRF_ERROR_BUSY);		
}


/**@brief Function for initializing services that will be used by the application.
 */
static void services_init(void)
{
    uint32_t       err_code;
    ble_pcs_init_t pcs_init;
	  ble_bas_init_t bas_init;
	  ble_gss_init_t gss_init;
	
	  memset(&gss_init, 0, sizeof(gss_init));
	  gss_init.xyz_config_handler = gss_xyz_config_handler;
	  gss_init.mic_config_handler = gss_mic_config_handler;
	  gss_init.bar_config_handler = gss_bar_config_handler;
	  gss_init.light_config_handler = gss_light_config_handler;
	
	 	ble_gss_init(&m_gss, &gss_init);
    APP_ERROR_CHECK(err_code);  
	
    memset(&pcs_init, 0, sizeof(pcs_init));

    pcs_init.pwm1_handler = pcs_pwm1_handler;
	  pcs_init.pwm2_handler = pcs_pwm2_handler;
	  pcs_init.pwm3_handler = pcs_pwm3_handler;
    
    err_code = ble_pcs_init(&m_pcs, &pcs_init);
    APP_ERROR_CHECK(err_code);
	

    // Initialize Battery Service.
    memset(&bas_init, 0, sizeof(bas_init));

    // Here the sec level for the Battery Service can be changed/increased.
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&bas_init.battery_level_char_attr_md.cccd_write_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&bas_init.battery_level_char_attr_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_NO_ACCESS(&bas_init.battery_level_char_attr_md.write_perm);

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&bas_init.battery_level_report_read_perm);

    bas_init.evt_handler          = NULL;
    bas_init.support_notification = true;
    bas_init.p_report_ref         = NULL;
    bas_init.initial_batt_level   = 100;

    err_code = ble_bas_init(&m_bas, &bas_init);
    APP_ERROR_CHECK(err_code);		
		
}


/**@brief Function for handling an event from the Connection Parameters Module.
 *
 * @details This function will be called for all events in the Connection Parameters Module
 *          which are passed to the application.
 *
 * @note All this function does is to disconnect. This could have been done by simply setting
 *       the disconnect_on_fail config parameter, but instead we use the event handler
 *       mechanism to demonstrate its use.
 *
 * @param[in] p_evt  Event received from the Connection Parameters Module.
 */
static void on_conn_params_evt(ble_conn_params_evt_t * p_evt)
{
    uint32_t err_code;
    
    if(p_evt->evt_type == BLE_CONN_PARAMS_EVT_FAILED)
    {
        err_code = sd_ble_gap_disconnect(m_conn_handle, BLE_HCI_CONN_INTERVAL_UNACCEPTABLE);
        APP_ERROR_CHECK(err_code);
    }
}


/**@brief Function for handling errors from the Connection Parameters module.
 *
 * @param[in] nrf_error  Error code containing information about what went wrong.
 */
static void conn_params_error_handler(uint32_t nrf_error)
{
    APP_ERROR_HANDLER(nrf_error);
}


/**@brief Function for initializing the Connection Parameters module.
 */
static void conn_params_init(void)
{
    uint32_t               err_code;
    ble_conn_params_init_t cp_init;
    
    memset(&cp_init, 0, sizeof(cp_init));

    cp_init.p_conn_params                  = NULL;
    cp_init.first_conn_params_update_delay = FIRST_CONN_PARAMS_UPDATE_DELAY;
    cp_init.next_conn_params_update_delay  = NEXT_CONN_PARAMS_UPDATE_DELAY;
    cp_init.max_conn_params_update_count   = MAX_CONN_PARAMS_UPDATE_COUNT;
    cp_init.start_on_notify_cccd_handle    = BLE_GATT_HANDLE_INVALID;
    cp_init.disconnect_on_fail             = false;
    cp_init.evt_handler                    = on_conn_params_evt;
    cp_init.error_handler                  = conn_params_error_handler;
    
    err_code = ble_conn_params_init(&cp_init);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for putting the chip into sleep mode.
 *
 * @note This function will not return.
 */
static void sleep_mode_enter(void)
{
    //uint32_t err_code = bsp_indication_set(BSP_INDICATE_IDLE);
    //APP_ERROR_CHECK(err_code);
    //uint32_t err_code;
		set_state(Poweroff);
}


/**@brief Function for handling advertising events.
 *
 * @details This function will be called for advertising events which are passed to the application.
 *
 * @param[in] ble_adv_evt  Advertising event.
 */
static void on_adv_evt(ble_adv_evt_t ble_adv_evt)
{
    uint32_t err_code;

    switch (ble_adv_evt)
    {
        case BLE_ADV_EVT_FAST:
						set_state(Seeking);
            //err_code = bsp_indication_set(BSP_INDICATE_ADVERTISING);
            //APP_ERROR_CHECK(err_code);
            break;
        case BLE_ADV_EVT_IDLE:
            sleep_mode_enter();
            break;
        default:
            break;
    }
}


/**@brief Function for the application's SoftDevice event handler.
 *
 * @param[in] p_ble_evt SoftDevice event.
 */
static void on_ble_evt(ble_evt_t * p_ble_evt)
{
    uint32_t                         err_code;
    
    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GAP_EVT_CONNECTED:
            //err_code = bsp_indication_set(BSP_INDICATE_CONNECTED);
            //APP_ERROR_CHECK(err_code);
						set_state(Connecting);
            m_conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
            break;
            
        case BLE_GAP_EVT_DISCONNECTED:
            //err_code = bsp_indication_set(BSP_INDICATE_IDLE);
            //APP_ERROR_CHECK(err_code);
						set_state(Seeking);
						m_conn_handle = BLE_CONN_HANDLE_INVALID;
            break;

        case BLE_GAP_EVT_SEC_PARAMS_REQUEST:
            // Pairing not supported
            err_code = sd_ble_gap_sec_params_reply(m_conn_handle, BLE_GAP_SEC_STATUS_PAIRING_NOT_SUPP, NULL, NULL);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_GATTS_EVT_SYS_ATTR_MISSING:
            // No system attributes have been stored.
            err_code = sd_ble_gatts_sys_attr_set(m_conn_handle, NULL, 0, 0);
            APP_ERROR_CHECK(err_code);
            break;

        default:
            // No implementation needed.
            break;
    }
}


/**@brief Function for dispatching a SoftDevice event to all modules with a SoftDevice 
 *        event handler.
 *
 * @details This function is called from the SoftDevice event interrupt handler after a 
 *          SoftDevice event has been received.
 *
 * @param[in] p_ble_evt  SoftDevice event.
 */
static void ble_evt_dispatch(ble_evt_t * p_ble_evt)
{
    ble_conn_params_on_ble_evt(p_ble_evt);
    ble_pcs_on_ble_evt(&m_pcs, p_ble_evt);
    ble_bas_on_ble_evt(&m_bas, p_ble_evt);	
	  ble_gss_on_ble_evt(&m_gss, p_ble_evt);
    on_ble_evt(p_ble_evt);
    ble_advertising_on_ble_evt(p_ble_evt);
    bsp_btn_ble_on_ble_evt(p_ble_evt);
    
}


/**@brief Function for the SoftDevice initialization.
 *
 * @details This function initializes the SoftDevice and the BLE event interrupt.
 */
static void ble_stack_init(void)
{
    uint32_t err_code;
    
    nrf_clock_lf_cfg_t clock_lf_cfg = NRF_CLOCK_LFCLKSRC;
    
    // Initialize SoftDevice.
    SOFTDEVICE_HANDLER_INIT(&clock_lf_cfg, NULL);
    
    ble_enable_params_t ble_enable_params;
    err_code = softdevice_enable_get_default_config(CENTRAL_LINK_COUNT,
                                                    PERIPHERAL_LINK_COUNT,
                                                    &ble_enable_params);
    APP_ERROR_CHECK(err_code);
        
    //Check the ram settings against the used number of links
    CHECK_RAM_START_ADDR(CENTRAL_LINK_COUNT,PERIPHERAL_LINK_COUNT);
    // Enable BLE stack.
    err_code = softdevice_enable(&ble_enable_params);
    APP_ERROR_CHECK(err_code);
    
    // Subscribe for BLE events.
    err_code = softdevice_ble_evt_handler_set(ble_evt_dispatch);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling events from the BSP module.
 *
 * @param[in]   event   Event generated by button press.
 */
void bsp_event_handler(bsp_event_t event)
{
    uint32_t err_code;
    switch (event)
    {
        case BSP_EVENT_SLEEP:
            sleep_mode_enter();
            break;
				/*
        case BSP_EVENT_DISCONNECT:
            err_code = sd_ble_gap_disconnect(m_conn_handle, BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            if (err_code != NRF_ERROR_INVALID_STATE)
            {
                APP_ERROR_CHECK(err_code);
            }
            break;

        case BSP_EVENT_WHITELIST_OFF:
            err_code = ble_advertising_restart_without_whitelist();
            if (err_code != NRF_ERROR_INVALID_STATE)
            {
                APP_ERROR_CHECK(err_code);
            }
            break;
				*/
        default:
            break;
    }
}


/**@brief   Function for handling app_uart events.
 *
 * @details This function will receive a single character from the app_uart module and append it to 
 *          a string. The string will be be sent over BLE when the last character received was a 
 *          'new line' i.e '\n' (hex 0x0D) or if the string has reached a length of 
 *          @ref NUS_MAX_DATA_LENGTH.
 */
/**@snippet [Handling the data received over UART] */
void uart_event_handle(app_uart_evt_t * p_event)
{
    static uint8_t data_array[BLE_PCS_MAX_DATA_LEN];
    static uint8_t index = 0;
    uint32_t       err_code;

    switch (p_event->evt_type)
    {
        case APP_UART_DATA_READY:
            UNUSED_VARIABLE(app_uart_get(&data_array[index]));
            index++;

            if ((data_array[index - 1] == '\n') || (index >= (BLE_PCS_MAX_DATA_LEN)))
            {
                //err_code = ble_pcs_string_send(&m_pcs, data_array, index);
                err_code = 0;
								if (err_code != NRF_ERROR_INVALID_STATE)
                {
                    APP_ERROR_CHECK(err_code);
                }
                
                index = 0;
            }
            break;

        case APP_UART_COMMUNICATION_ERROR:
            APP_ERROR_HANDLER(p_event->data.error_communication);
            break;

        case APP_UART_FIFO_ERROR:
            APP_ERROR_HANDLER(p_event->data.error_code);
            break;

        default:
            break;
    }
}
/**@snippet [Handling the data received over UART] */


static void timers_init(void)
{
		uint32_t 				err_code;
    APP_TIMER_INIT(APP_TIMER_PRESCALER, APP_TIMER_OP_QUEUE_SIZE, false);	
	
		err_code = app_timer_create(&pwm1_timer_id,
															  APP_TIMER_MODE_REPEATED,
															  pwm1_timer_handler);
	
		APP_ERROR_CHECK(err_code);
	
		err_code = app_timer_create(&sensor_timer_id,
															  APP_TIMER_MODE_REPEATED,
														    sensor_timer_handler);
	  APP_ERROR_CHECK(err_code);
	
		err_code = app_timer_create(&adc_mic_timer_id,
														    APP_TIMER_MODE_REPEATED,
															  adc_mic_timer_handler);
	
	  APP_ERROR_CHECK(err_code);
	
		err_code = app_timer_create(&state_machine_timer_id,
																APP_TIMER_MODE_REPEATED,
																state_machine_timer_handler);
																
		APP_ERROR_CHECK(err_code);
		
		err_code = app_timer_create(&button_check_timer_id,
																APP_TIMER_MODE_REPEATED,
																button_check_timer_handler);
}

/**@brief  Function for initializing the UART module.
 */
/**@snippet [UART Initialization] */
static void uart_init(void)
{
    uint32_t                     err_code;
    const app_uart_comm_params_t comm_params =
    {
        RX_PIN_NUMBER,
        TX_PIN_NUMBER,
        RTS_PIN_NUMBER,
        CTS_PIN_NUMBER,
        APP_UART_FLOW_CONTROL_DISABLED,
        false,
        UART_BAUDRATE_BAUDRATE_Baud115200
    };

    APP_UART_FIFO_INIT( &comm_params,
                       UART_RX_BUF_SIZE,
                       UART_TX_BUF_SIZE,
                       uart_event_handle,
                       APP_IRQ_PRIORITY_LOW,
                       err_code);
    APP_ERROR_CHECK(err_code);
}
/**@snippet [UART Initialization] */


/**@brief Function for initializing the Advertising functionality.
 */
static void advertising_init(void)
{
    uint32_t      err_code;
    ble_advdata_t advdata;
    ble_advdata_t scanrsp;

    // Build advertising data struct to pass into @ref ble_advertising_init.
    memset(&advdata, 0, sizeof(advdata));
    advdata.name_type          = BLE_ADVDATA_FULL_NAME;
    advdata.include_appearance = false;
    advdata.flags              = BLE_GAP_ADV_FLAGS_LE_ONLY_LIMITED_DISC_MODE;

    memset(&scanrsp, 0, sizeof(scanrsp));
    scanrsp.uuids_complete.uuid_cnt = sizeof(m_adv_uuids) / sizeof(m_adv_uuids[0]);
    scanrsp.uuids_complete.p_uuids  = m_adv_uuids;

    ble_adv_modes_config_t options = {0};
    options.ble_adv_fast_enabled  = BLE_ADV_FAST_ENABLED;
    options.ble_adv_fast_interval = APP_ADV_INTERVAL;
    options.ble_adv_fast_timeout  = APP_ADV_TIMEOUT_IN_SECONDS;

    err_code = ble_advertising_init(&advdata, &scanrsp, &options, on_adv_evt, NULL);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for initializing buttons and leds.
 *
 * @param[out] p_erase_bonds  Will be true if the clear bonding button was pressed to wake the application up.
 */
static void button_init(bool * p_erase_bonds)
{
    bsp_event_t startup_event;

    uint32_t err_code = bsp_init(BSP_INIT_BUTTONS,
                                 APP_TIMER_TICKS(100, APP_TIMER_PRESCALER), 
                                 bsp_event_handler);
    APP_ERROR_CHECK(err_code);

    err_code = bsp_btn_ble_init(NULL, &startup_event);
    APP_ERROR_CHECK(err_code);

    *p_erase_bonds = (startup_event == BSP_EVENT_CLEAR_BONDING_DATA);
}


static void pwm_init(void)
{
	  uint32_t err_code;
	    /* 1-channel PWM, 200Hz, output on DK LED pins. */
    app_pwm_config_t pwm1_cfg = APP_PWM_DEFAULT_CONFIG_1CH(5000L, PWM1_OUT);
    
    /* Switch the polarity of the second channel. */
    //pwm1_cfg.pin_polarity[1] = APP_PWM_POLARITY_ACTIVE_HIGH;
    
    /* Initialize and enable PWM. */
    err_code = app_pwm_init(&PWM1,&pwm1_cfg,pwm_ready_callback);
	
	  //app_pwm_config_t pwm2_cfg = APP_PWM_DEFAULT_CONFIG_1CH(5000L, 11);
	
	  //err_code = app_pwm_init(&PWM2, &pwm2_cfg, NULL);
	
	  //app_pwm_config_t pwm3_cfg = APP_PWM_DEFAULT_CONFIG_1CH(5000L, 12);
	
		//err_code = app_pwm_init(&PWM3, &pwm3_cfg, NULL);
	
    APP_ERROR_CHECK(err_code);
    app_pwm_enable(&PWM1);
	  //app_pwm_enable(&PWM2);
		//app_pwm_enable(&PWM3);
}

static void gpio_init(void)
{
		nrf_gpio_cfg(LM3407_EN, 
								 NRF_GPIO_PIN_DIR_OUTPUT, 
								 NRF_GPIO_PIN_INPUT_DISCONNECT, 
							   NRF_GPIO_PIN_NOPULL, 
								 NRF_GPIO_PIN_S0S1, 
								 NRF_GPIO_PIN_NOSENSE);

		nrf_gpio_pin_set(LM3407_EN);
	
		nrf_gpio_cfg(GREEN_LED, 
								 NRF_GPIO_PIN_DIR_OUTPUT, 
								 NRF_GPIO_PIN_INPUT_DISCONNECT, 
							   NRF_GPIO_PIN_NOPULL, 
								 NRF_GPIO_PIN_S0S1, 
								 NRF_GPIO_PIN_NOSENSE);

		nrf_gpio_pin_clear(GREEN_LED);

		nrf_gpio_cfg(RED_LED, 
								 NRF_GPIO_PIN_DIR_OUTPUT, 
								 NRF_GPIO_PIN_INPUT_DISCONNECT, 
							   NRF_GPIO_PIN_NOPULL, 
								 NRF_GPIO_PIN_S0S1, 
								 NRF_GPIO_PIN_NOSENSE);

		nrf_gpio_pin_clear(RED_LED);
		
		nrf_gpio_cfg(BUTTON_1,
								 NRF_GPIO_PIN_DIR_INPUT,
								 NRF_GPIO_PIN_INPUT_CONNECT,
								 NRF_GPIO_PIN_PULLUP,
								 NRF_GPIO_PIN_S0S1,
								 NRF_GPIO_PIN_SENSE_LOW);
								 
		nrf_gpio_cfg(ISET2, 
								 NRF_GPIO_PIN_DIR_OUTPUT, 
								 NRF_GPIO_PIN_INPUT_DISCONNECT, 
							   NRF_GPIO_PIN_NOPULL, 
								 NRF_GPIO_PIN_S0S1, 
								 NRF_GPIO_PIN_NOSENSE);	
								 
								 
    
		nrf_delay_ms(250);
		nrf_gpio_pin_clear(ISET2);
		nrf_delay_ms(10);
		nrf_gpio_pin_set(ISET2);
		nrf_delay_ms(10);
		nrf_gpio_pin_clear(ISET2);
														 
}

static void bme280_cal_init(void)
{
	static app_twi_transfer_t const transfers[] =
	{
			BME280_READ_CAL1(&m_buffer[0]),
			BME280_READ_CAL2(&m_buffer[26])
	};
	static app_twi_transaction_t const transaction = 
	{
		.callback	 	 	 	  = read_bme280_cal_cb,
		.p_user_data	 	 	= NULL,
		.p_transfers 	 	  = transfers,
		.number_of_transfers = sizeof(transfers)/sizeof(transfers[0])
	};
	
	APP_ERROR_CHECK(app_twi_schedule(&m_app_twi, &transaction));
}
static void application_timers_start(void)
{
		uint32_t err_code;
	
		err_code = app_timer_start(pwm1_timer_id, PWM1_INTERVAL, NULL);
	  APP_ERROR_CHECK(err_code);
	
	  err_code = app_timer_start(sensor_timer_id, SENSOR_INTERVAL, NULL);
	  APP_ERROR_CHECK(err_code);
	
	  err_code = app_timer_start(adc_mic_timer_id, ADC_MIC_INTERVAL, NULL);
	  APP_ERROR_CHECK(err_code);
	
		err_code = app_timer_start(state_machine_timer_id, STATE_MACHINE_INTERVAL, NULL);
	  APP_ERROR_CHECK(err_code);
	
		err_code = app_timer_start(button_check_timer_id, BUTTON_CHECK_INTERVAL, NULL);
	  APP_ERROR_CHECK(err_code);
}

static void twi_config(void)
{
    uint32_t err_code;

    nrf_drv_twi_config_t const config = {
       .scl                = I2C_SCL,
       .sda                = I2C_SDA,
       .frequency          = NRF_TWI_FREQ_400K,
       .interrupt_priority = APP_IRQ_PRIORITY_LOW
    };

    APP_TWI_INIT(&m_app_twi, &config, MAX_PENDING_TRANSACTIONS, err_code);
    APP_ERROR_CHECK(err_code);
}

static void adc_init(void)
{
	/* Enable interrupt on ADC sample ready event*/		
	NRF_ADC->INTENSET = ADC_INTENSET_END_Msk;   
	sd_nvic_SetPriority(ADC_IRQn, APP_IRQ_PRIORITY_LOW);  
	sd_nvic_EnableIRQ(ADC_IRQn);
	
	NRF_ADC->CONFIG	= (ADC_CONFIG_EXTREFSEL_None << ADC_CONFIG_EXTREFSEL_Pos) /* Bits 17..16 : ADC external reference pin selection. */
									| (ADC_CONFIG_PSEL_AnalogInput3 << ADC_CONFIG_PSEL_Pos)					/*!< Use analog input 2 as analog input. */
									| (ADC_CONFIG_REFSEL_VBG << ADC_CONFIG_REFSEL_Pos)							/*!< Use internal 1.2V bandgap voltage as reference for conversion. */
									| (ADC_CONFIG_INPSEL_AnalogInputOneThirdPrescaling << ADC_CONFIG_INPSEL_Pos) /*!< Analog input specified by PSEL with no prescaling used as input for the conversion. */
									| (ADC_CONFIG_RES_8bit << ADC_CONFIG_RES_Pos);									/*!< 8bit ADC resolution. */ 
	
	/* Enable ADC*/
	NRF_ADC->ENABLE = ADC_ENABLE_ENABLE_Enabled;	
}

void ADC_IRQHandler(void)
{
	NRF_ADC->EVENTS_END = 0;
	
	micInputRead = NRF_ADC->RESULT;
	
	NRF_ADC->TASKS_STOP = 1;
	
	sd_clock_hfclk_release();
}

/**@brief Function for placing the application in low power state while waiting for events.
 */
static void power_manage(void)
{
    uint32_t err_code = sd_app_evt_wait();
    APP_ERROR_CHECK(err_code);
}


/**@brief Application main function.
 */
int main(void)
{
    uint32_t err_code;
    bool erase_bonds;
	
	  baseline = 1.6;        // nominal values
	  max_thres = baseline + 0.025;
	  min_thres = baseline - 0.025;
	  m1 = (0 - 100)/(3 - max_thres);
	  b1 = (m1 * -3.0);
	  m2 = (0 - 100)/(0 - min_thres);
	  b2 = 0;
	
		memset(bme280calT, 0, sizeof(bme280calT));
		memset(bme280calP, 0, sizeof(bme280calP));
		memset(bme280calH, 0, sizeof(bme280calH));	

    // Initialize.
    uart_init();
    twi_config();
	  adc_init();
	  //initialize sensors
	 // APP_ERROR_CHECK(app_twi_perform(&m_app_twi, bme280_init_transfers, 2, NULL));
	  app_twi_perform(&m_app_twi, bme280_init_transfers, 2, NULL);
	 // APP_ERROR_CHECK(app_twi_perform(&m_app_twi, mpu9250_init_transfers, 1, NULL));
	  app_twi_perform(&m_app_twi, mpu9250_init_transfers, 1, NULL);
	 // APP_ERROR_CHECK(app_twi_perform(&m_app_twi, mpu9250_passthrough_transfers, 2, NULL));	
	  app_twi_perform(&m_app_twi, mpu9250_passthrough_transfers, 2, NULL);
	  nrf_delay_ms(200);
	 // APP_ERROR_CHECK(app_twi_perform(&m_app_twi, ltr329_init_transfers, 1, NULL)); 
	  app_twi_perform(&m_app_twi, ltr329_init_transfers, 1, NULL);
	  bme280_cal_init();
	  timers_init();
	  //button_init(&erase_bonds);	
	  gpio_init();
	  pwm_init();
    ble_stack_init();
    gap_params_init();
    services_init();
    advertising_init();
    conn_params_init();

    //printf("\r\nUART Start!\r\n");
    err_code = ble_advertising_start(BLE_ADV_MODE_FAST);
    set_state(Seeking);
    APP_ERROR_CHECK(err_code);
	
		application_timers_start();
    
    // Enter main loop.
    for (;;)
    {
        power_manage();
    }
}


/** 
 * @}
 */
