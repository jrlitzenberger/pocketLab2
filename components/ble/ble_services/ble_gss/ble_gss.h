/* Copyright (c) Nordic Semiconductor ASA
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 * 
 *   1. Redistributions of source code must retain the above copyright notice, this
 *   list of conditions and the following disclaimer.
 * 
 *   2. Redistributions in binary form must reproduce the above copyright notice, this
 *   list of conditions and the following disclaimer in the documentation and/or
 *   other materials provided with the distribution.
 * 
 *   3. Neither the name of Nordic Semiconductor ASA nor the names of other
 *   contributors to this software may be used to endorse or promote products
 *   derived from this software without specific prior written permission.
 * 
 *   4. This software must only be used in a processor manufactured by Nordic
 *   Semiconductor ASA, or in a processor manufactured by a third party that
 *   is used in combination with a processor manufactured by Nordic Semiconductor.
 * 
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * 
 */

#ifndef BLE_GSS_H__
#define BLE_GSS_H__

#include <stdint.h>
#include "ble.h"
#include "ble_srv_common.h"

#define BLE_UUID_GSS_BASE_UUID                    {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xB0, 0x00, 0x40, 0x51, 0x04, 0x00, 0x00, 0x00, 0xF0} // 128-bit base UUID
#define BLE_UUID_GSS_SERVICE                      0xAA11 // Just a random, but recognizable value
#define BLE_UUID_GSS_XYZ_DATA_CHARACTERISTIC   0xAA15
#define BLE_UUID_GSS_XYZ_CONFIG_CHARACTERISTIC 0xAA14
#define BLE_UUID_GSS_MIC_DATA_CHARACTERISTIC   0xAA13
#define BLE_UUID_GSS_MIC_CONFIG_CHARACTERISTIC 0xAA12
#define BLE_UUID_GSS_BAR_DATA_CHARACTERISTIC   0xAA17
#define BLE_UUID_GSS_BAR_CONFIG_CHARACTERISTIC 0xAA16
#define BLE_UUID_GSS_LIGHT_DATA_CHARACTERISTIC   0xAA19
#define BLE_UUID_GSS_LIGHT_CONFIG_CHARACTERISTIC 0xAA18

#define BLE_MAX_WRITE_LENGTH	 	 	 	 	 	 	 	 	 	 	20

typedef enum{
	BLE_GSS_EVT_NOTIFICATION_ENABLED,
	BLE_GSS_EVT_NOTIFICATION_DISABLED
}ble_gss_evt_type_t;

typedef struct
{
	ble_gss_evt_type_t evt_type;
}ble_gss_evt_t;

typedef struct ble_gss_s ble_gss_t;

typedef void (*ble_gss_xyz_config_handler_t) (ble_gss_t * p_gss, uint8_t * p_data, uint16_t length);
typedef void (*ble_gss_mic_config_handler_t) (ble_gss_t * p_gss, uint8_t * p_data, uint16_t length);
typedef void (*ble_gss_bar_config_handler_t) (ble_gss_t * p_gss, uint8_t * p_data, uint16_t length);
typedef void (*ble_gss_light_config_handler_t) (ble_gss_t * p_gss, uint8_t * p_data, uint16_t length);
typedef void (*ble_gss_evt_handler_t) (ble_gss_t * p_gss, ble_gss_evt_t * p_evt);

typedef struct
{
    ble_gss_xyz_config_handler_t   xyz_config_handler; /**< Event handler to be called for handling received data. */
    ble_gss_mic_config_handler_t   mic_config_handler; /**< Event handler to be called for handling received data. */
    ble_gss_bar_config_handler_t   bar_config_handler; /**< Event handler to be called for handling received data. */
    ble_gss_light_config_handler_t light_config_handler; /**< Event handler to be called for handling received data. */
    ble_gss_evt_handler_t          evt_handler;
} ble_gss_init_t;
/**
 * @brief This structure contains various status information for our service. 
 * It only holds one entry now, but will be populated with more items as we go.
 * The name is based on the naming convention used in Nordic's SDKs. 
 * 'ble’ indicates that it is a Bluetooth Low Energy relevant structure and 
 * ‘os’ is short for Our Service). 
 */
struct ble_gss_s
{
	  ble_gss_evt_handler_t        evt_handler;
	  uint8_t                      uuid_type;
    uint16_t                     service_handle;     /**< Handle of Our Service (as provided by the BLE stack). */
    ble_gatts_char_handles_t     xyz_data_handles;	
    ble_gatts_char_handles_t     xyz_config_handles;
	  ble_gatts_char_handles_t     mic_data_handles;
	  ble_gatts_char_handles_t     mic_config_handles;
	  ble_gatts_char_handles_t     bar_data_handles;
	  ble_gatts_char_handles_t     bar_config_handles;	
	  ble_gatts_char_handles_t     light_data_handles;
	  ble_gatts_char_handles_t     light_config_handles;	
		uint16_t                     conn_handle; 
    bool                         is_notification_enabled; 
		ble_gss_xyz_config_handler_t xyz_config_handler;
		ble_gss_mic_config_handler_t mic_config_handler;	
		ble_gss_bar_config_handler_t bar_config_handler;
		ble_gss_light_config_handler_t light_config_handler;	
};

uint32_t ble_gss_xyz_measurement_send(ble_gss_t * p_gss, uint8_t * accel, uint8_t * gyro, uint8_t * mag);

uint32_t ble_gss_bar_measurement_send(ble_gss_t * p_gss, uint8_t * bar);

uint32_t ble_gss_light_measurement_send(ble_gss_t * p_gss, uint8_t * light);

uint32_t ble_gss_bar_cal_send(ble_gss_t * p_gss, uint8_t * cal);

/**@brief Function for initializing our new service.
 *
 * @param[in]   p_our_service       Pointer to Our Service structure.
 */
void ble_gss_init(ble_gss_t * p_gss, const ble_gss_init_t * p_gss_init);


void ble_gss_on_ble_evt(ble_gss_t * p_gss, ble_evt_t * p_ble_evt);
#endif  /* _ OUR_SERVICE_H__ */
