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

#include <stdint.h>
#include <string.h>
#include "ble_gss.h"
#include "ble_srv_common.h"
#include "app_error.h"
#include "SEGGER_RTT.h"

static void on_connect(ble_gss_t * p_gss, ble_evt_t * p_ble_evt)
{
    p_gss->conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
}

static void on_disconnect(ble_gss_t * p_gss, ble_evt_t * p_ble_evt)
{
    UNUSED_PARAMETER(p_ble_evt);
    p_gss->conn_handle = BLE_CONN_HANDLE_INVALID;
}

static void on_xyz_cccd_write(ble_gss_t * p_gss, ble_gatts_evt_write_t * p_evt_write)
{
    if (p_evt_write->len == 2)
    {
        // CCCD written, update notification state
        if (p_gss->evt_handler != NULL)
        {
            ble_gss_evt_t evt;

            if (ble_srv_is_notification_enabled(p_evt_write->data))
            {
                evt.evt_type = BLE_GSS_EVT_NOTIFICATION_ENABLED;
            }
            else
            {
                evt.evt_type = BLE_GSS_EVT_NOTIFICATION_DISABLED;
            }

            p_gss->evt_handler(p_gss, &evt);
        }
    }	
}

static void on_bar_cccd_write(ble_gss_t * p_gss, ble_gatts_evt_write_t * p_evt_write)
{
    if (p_evt_write->len == 2)
    {
        // CCCD written, update notification state
        if (p_gss->evt_handler != NULL)
        {
            ble_gss_evt_t evt;

            if (ble_srv_is_notification_enabled(p_evt_write->data))
            {
                evt.evt_type = BLE_GSS_EVT_NOTIFICATION_ENABLED;
            }
            else
            {
                evt.evt_type = BLE_GSS_EVT_NOTIFICATION_DISABLED;
            }

            p_gss->evt_handler(p_gss, &evt);
        }
    }	
}

static void on_write(ble_gss_t * p_gss, ble_evt_t * p_ble_evt)
{
    ble_gatts_evt_write_t * p_evt_write = &p_ble_evt->evt.gatts_evt.params.write;

    if ((p_evt_write->handle == p_gss->xyz_config_handles.value_handle) && (p_gss->xyz_config_handler != NULL))
    {
        p_gss->xyz_config_handler(p_gss, p_evt_write->data, p_evt_write->len);
    }
		else if(p_evt_write->handle == p_gss->xyz_data_handles.cccd_handle)
		{
			  on_xyz_cccd_write(p_gss, p_evt_write);
		}
		else if((p_evt_write->handle == p_gss->mic_config_handles.value_handle) && (p_gss->mic_config_handler != NULL))
		{
				p_gss->mic_config_handler(p_gss, p_evt_write->data, p_evt_write->len);
		}
		else if((p_evt_write->handle == p_gss->bar_config_handles.value_handle) && (p_gss->bar_config_handler != NULL))
		{
				p_gss->bar_config_handler(p_gss, p_evt_write->data, p_evt_write->len);
		}	
		else if(p_evt_write->handle == p_gss->bar_data_handles.cccd_handle)
		{
				on_bar_cccd_write(p_gss, p_evt_write);
		}
		else if((p_evt_write->handle == p_gss->light_config_handles.value_handle) && (p_gss->light_config_handler != NULL))
		{
				p_gss->light_config_handler(p_gss, p_evt_write->data, p_evt_write->len);
		}			
		else
		{
			//do nothing
		}
}

static uint32_t xyz_data_char_add(ble_gss_t * p_gss, const ble_gss_init_t * p_gss_init)
{
    uint32_t   err_code = 0; // Variable to hold return codes from library and softdevice functions
    
    // OUR_JOB: Step 2.A, Add a custom characteristic UUID
    ble_uuid_t          char_uuid;
    ble_uuid128_t       base_uuid = BLE_UUID_GSS_BASE_UUID;
    char_uuid.uuid      = BLE_UUID_GSS_XYZ_DATA_CHARACTERISTIC;
    sd_ble_uuid_vs_add(&base_uuid, &char_uuid.type);
    APP_ERROR_CHECK(err_code);
    
    // OUR_JOB: Step 2.F Add read/write properties to our characteristic
    ble_gatts_char_md_t char_md;
    memset(&char_md, 0, sizeof(char_md));
    char_md.char_props.read = 1;
    char_md.char_props.write = 0;

    
    // OUR_JOB: Step 3.A, Configuring Client Characteristic Configuration Descriptor metadata and add to char_md structure
    ble_gatts_attr_md_t cccd_md;
    memset(&cccd_md, 0, sizeof(cccd_md));
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.write_perm);
    cccd_md.vloc                = BLE_GATTS_VLOC_STACK;    
    char_md.p_cccd_md           = &cccd_md;
    char_md.char_props.notify   = 1;
   
    
    // OUR_JOB: Step 2.B, Configure the attribute metadata
    ble_gatts_attr_md_t attr_md;
    memset(&attr_md, 0, sizeof(attr_md)); 
    attr_md.vloc        = BLE_GATTS_VLOC_STACK;   
    
    
    // OUR_JOB: Step 2.G, Set read/write security levels to our characteristic
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.write_perm);
    
    
    // OUR_JOB: Step 2.C, Configure the characteristic value attribute
    ble_gatts_attr_t    attr_char_value;
    memset(&attr_char_value, 0, sizeof(attr_char_value));        
    attr_char_value.p_uuid      = &char_uuid;
    attr_char_value.p_attr_md   = &attr_md;
    
    // OUR_JOB: Step 2.H, Set characteristic length in number of bytes
    attr_char_value.max_len     = 20;
    attr_char_value.init_len    = 20;
    uint8_t value[4]            = {0x00, 0x01, 0x02, 0x03};
    attr_char_value.p_value     = value;

    // OUR_JOB: Step 2.E, Add our new characteristic to the service
    err_code = sd_ble_gatts_characteristic_add(p_gss->service_handle,
                                       &char_md,
                                       &attr_char_value,
                                       &p_gss->xyz_data_handles);
    APP_ERROR_CHECK(err_code);
    
    return NRF_SUCCESS;
    /**@snippet [Adding proprietary characteristic to S110 SoftDevice] */
}

static uint32_t xyz_config_char_add(ble_gss_t * p_gss, const ble_gss_init_t * p_gss_init)
{
    uint32_t   err_code = 0; // Variable to hold return codes from library and softdevice functions
    
    // OUR_JOB: Step 2.A, Add a custom characteristic UUID
    ble_uuid_t          char_uuid;
    ble_uuid128_t       base_uuid = BLE_UUID_GSS_BASE_UUID;
    char_uuid.uuid      = BLE_UUID_GSS_XYZ_CONFIG_CHARACTERISTIC;
    sd_ble_uuid_vs_add(&base_uuid, &char_uuid.type);
    APP_ERROR_CHECK(err_code);
    
    // OUR_JOB: Step 2.F Add read/write properties to our characteristic
    ble_gatts_char_md_t char_md;
    memset(&char_md, 0, sizeof(char_md));
    char_md.char_props.read = 1;
    char_md.char_props.write = 1;
	  //char_md.char_props.write_wo_resp = 1;

    
    // OUR_JOB: Step 3.A, Configuring Client Characteristic Configuration Descriptor metadata and add to char_md structure
    ble_gatts_attr_md_t cccd_md;
    memset(&cccd_md, 0, sizeof(cccd_md));
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.write_perm);
    cccd_md.vloc                = BLE_GATTS_VLOC_STACK;    
    char_md.p_cccd_md           = &cccd_md;
    char_md.char_props.notify   = 1;
   
    
    // OUR_JOB: Step 2.B, Configure the attribute metadata
    ble_gatts_attr_md_t attr_md;
    memset(&attr_md, 0, sizeof(attr_md)); 
    attr_md.vloc        = BLE_GATTS_VLOC_STACK;   
    
    
    // OUR_JOB: Step 2.G, Set read/write security levels to our characteristic
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.write_perm);
    
    
    // OUR_JOB: Step 2.C, Configure the characteristic value attribute
    ble_gatts_attr_t    attr_char_value;
    memset(&attr_char_value, 0, sizeof(attr_char_value));        
    attr_char_value.p_uuid      = &char_uuid;
    attr_char_value.p_attr_md   = &attr_md;
    
    // OUR_JOB: Step 2.H, Set characteristic length in number of bytes
    attr_char_value.max_len     = 20;
    attr_char_value.init_len    = 20;
    uint8_t value               = {0x00};
    attr_char_value.p_value     = &value;

    // OUR_JOB: Step 2.E, Add our new characteristic to the service
    err_code = sd_ble_gatts_characteristic_add(p_gss->service_handle,
                                       &char_md,
                                       &attr_char_value,
                                       &p_gss->xyz_config_handles);
    APP_ERROR_CHECK(err_code);
    
    return NRF_SUCCESS;
}

static uint32_t mic_data_char_add(ble_gss_t * p_gss, const ble_gss_init_t * p_gss_init)
{
    uint32_t   err_code = 0; // Variable to hold return codes from library and softdevice functions
    
    // OUR_JOB: Step 2.A, Add a custom characteristic UUID
    ble_uuid_t          char_uuid;
    ble_uuid128_t       base_uuid = BLE_UUID_GSS_BASE_UUID;
    char_uuid.uuid      = BLE_UUID_GSS_MIC_DATA_CHARACTERISTIC;
    sd_ble_uuid_vs_add(&base_uuid, &char_uuid.type);
    APP_ERROR_CHECK(err_code);
    
    // OUR_JOB: Step 2.F Add read/write properties to our characteristic
    ble_gatts_char_md_t char_md;
    memset(&char_md, 0, sizeof(char_md));
    char_md.char_props.read = 1;
    char_md.char_props.write = 0;

    
    // OUR_JOB: Step 3.A, Configuring Client Characteristic Configuration Descriptor metadata and add to char_md structure
    ble_gatts_attr_md_t cccd_md;
    memset(&cccd_md, 0, sizeof(cccd_md));
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.write_perm);
    cccd_md.vloc                = BLE_GATTS_VLOC_STACK;    
    char_md.p_cccd_md           = &cccd_md;
    char_md.char_props.notify   = 1;
   
    
    // OUR_JOB: Step 2.B, Configure the attribute metadata
    ble_gatts_attr_md_t attr_md;
    memset(&attr_md, 0, sizeof(attr_md)); 
    attr_md.vloc        = BLE_GATTS_VLOC_STACK;   
    
    
    // OUR_JOB: Step 2.G, Set read/write security levels to our characteristic
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.write_perm);
    
    
    // OUR_JOB: Step 2.C, Configure the characteristic value attribute
    ble_gatts_attr_t    attr_char_value;
    memset(&attr_char_value, 0, sizeof(attr_char_value));        
    attr_char_value.p_uuid      = &char_uuid;
    attr_char_value.p_attr_md   = &attr_md;
    
    // OUR_JOB: Step 2.H, Set characteristic length in number of bytes
    attr_char_value.max_len     = 20;
    attr_char_value.init_len    = 20;
    uint8_t value[4]            = {0x00};
    attr_char_value.p_value     = value;

    // OUR_JOB: Step 2.E, Add our new characteristic to the service
    err_code = sd_ble_gatts_characteristic_add(p_gss->service_handle,
                                       &char_md,
                                       &attr_char_value,
                                       &p_gss->mic_data_handles);
    APP_ERROR_CHECK(err_code);
    
    return NRF_SUCCESS;
    /**@snippet [Adding proprietary characteristic to S110 SoftDevice] */
}

static uint32_t mic_config_char_add(ble_gss_t * p_gss, const ble_gss_init_t * p_gss_init)
{
    uint32_t   err_code = 0; // Variable to hold return codes from library and softdevice functions
    
    // OUR_JOB: Step 2.A, Add a custom characteristic UUID
    ble_uuid_t          char_uuid;
    ble_uuid128_t       base_uuid = BLE_UUID_GSS_BASE_UUID;
    char_uuid.uuid      = BLE_UUID_GSS_MIC_CONFIG_CHARACTERISTIC;
    sd_ble_uuid_vs_add(&base_uuid, &char_uuid.type);
    APP_ERROR_CHECK(err_code);
    
    // OUR_JOB: Step 2.F Add read/write properties to our characteristic
    ble_gatts_char_md_t char_md;
    memset(&char_md, 0, sizeof(char_md));
    char_md.char_props.read = 1;
    char_md.char_props.write = 1;

    
    // OUR_JOB: Step 3.A, Configuring Client Characteristic Configuration Descriptor metadata and add to char_md structure
    ble_gatts_attr_md_t cccd_md;
    memset(&cccd_md, 0, sizeof(cccd_md));
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.write_perm);
    cccd_md.vloc                = BLE_GATTS_VLOC_STACK;    
    char_md.p_cccd_md           = &cccd_md;
    char_md.char_props.notify   = 1;
   
    
    // OUR_JOB: Step 2.B, Configure the attribute metadata
    ble_gatts_attr_md_t attr_md;
    memset(&attr_md, 0, sizeof(attr_md)); 
    attr_md.vloc        = BLE_GATTS_VLOC_STACK;   
    
    
    // OUR_JOB: Step 2.G, Set read/write security levels to our characteristic
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.write_perm);
    
    
    // OUR_JOB: Step 2.C, Configure the characteristic value attribute
    ble_gatts_attr_t    attr_char_value;
    memset(&attr_char_value, 0, sizeof(attr_char_value));        
    attr_char_value.p_uuid      = &char_uuid;
    attr_char_value.p_attr_md   = &attr_md;
    
    // OUR_JOB: Step 2.H, Set characteristic length in number of bytes
    attr_char_value.max_len     = 20;
    attr_char_value.init_len    = 20;
    uint8_t value[4]            = {0x00};
    attr_char_value.p_value     = value;

    // OUR_JOB: Step 2.E, Add our new characteristic to the service
    err_code = sd_ble_gatts_characteristic_add(p_gss->service_handle,
                                       &char_md,
                                       &attr_char_value,
                                       &p_gss->mic_config_handles);
    APP_ERROR_CHECK(err_code);
    
    return NRF_SUCCESS;
}

static uint32_t bar_data_char_add(ble_gss_t * p_gss, const ble_gss_init_t * p_gss_init)
{
    uint32_t   err_code = 0; // Variable to hold return codes from library and softdevice functions
    
    // OUR_JOB: Step 2.A, Add a custom characteristic UUID
    ble_uuid_t          char_uuid;
    ble_uuid128_t       base_uuid = BLE_UUID_GSS_BASE_UUID;
    char_uuid.uuid      = BLE_UUID_GSS_BAR_DATA_CHARACTERISTIC;
    sd_ble_uuid_vs_add(&base_uuid, &char_uuid.type);
    APP_ERROR_CHECK(err_code);
    
    // OUR_JOB: Step 2.F Add read/write properties to our characteristic
    ble_gatts_char_md_t char_md;
    memset(&char_md, 0, sizeof(char_md));
    char_md.char_props.read = 1;
    char_md.char_props.write = 0;

    
    // OUR_JOB: Step 3.A, Configuring Client Characteristic Configuration Descriptor metadata and add to char_md structure
    ble_gatts_attr_md_t cccd_md;
    memset(&cccd_md, 0, sizeof(cccd_md));
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.write_perm);
    cccd_md.vloc                = BLE_GATTS_VLOC_STACK;    
    char_md.p_cccd_md           = &cccd_md;
    char_md.char_props.notify   = 1;
   
    
    // OUR_JOB: Step 2.B, Configure the attribute metadata
    ble_gatts_attr_md_t attr_md;
    memset(&attr_md, 0, sizeof(attr_md)); 
    attr_md.vloc        = BLE_GATTS_VLOC_STACK;   
    
    
    // OUR_JOB: Step 2.G, Set read/write security levels to our characteristic
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.write_perm);
    
    
    // OUR_JOB: Step 2.C, Configure the characteristic value attribute
    ble_gatts_attr_t    attr_char_value;
    memset(&attr_char_value, 0, sizeof(attr_char_value));        
    attr_char_value.p_uuid      = &char_uuid;
    attr_char_value.p_attr_md   = &attr_md;
    
    // OUR_JOB: Step 2.H, Set characteristic length in number of bytes
    attr_char_value.max_len     = 20;
    attr_char_value.init_len    = 20;
    uint8_t value[4]            = {0x00};
    attr_char_value.p_value     = value;

    // OUR_JOB: Step 2.E, Add our new characteristic to the service
    err_code = sd_ble_gatts_characteristic_add(p_gss->service_handle,
                                       &char_md,
                                       &attr_char_value,
                                       &p_gss->bar_data_handles);
    APP_ERROR_CHECK(err_code);
    
    return NRF_SUCCESS;
    /**@snippet [Adding proprietary characteristic to S110 SoftDevice] */
}

static uint32_t bar_config_char_add(ble_gss_t * p_gss, const ble_gss_init_t * p_gss_init)
{
    uint32_t   err_code = 0; // Variable to hold return codes from library and softdevice functions
    
    // OUR_JOB: Step 2.A, Add a custom characteristic UUID
    ble_uuid_t          char_uuid;
    ble_uuid128_t       base_uuid = BLE_UUID_GSS_BASE_UUID;
    char_uuid.uuid      = BLE_UUID_GSS_BAR_CONFIG_CHARACTERISTIC;
    sd_ble_uuid_vs_add(&base_uuid, &char_uuid.type);
    APP_ERROR_CHECK(err_code);
    
    // OUR_JOB: Step 2.F Add read/write properties to our characteristic
    ble_gatts_char_md_t char_md;
    memset(&char_md, 0, sizeof(char_md));
    char_md.char_props.read = 1;
    char_md.char_props.write = 1;

    
    // OUR_JOB: Step 3.A, Configuring Client Characteristic Configuration Descriptor metadata and add to char_md structure
    ble_gatts_attr_md_t cccd_md;
    memset(&cccd_md, 0, sizeof(cccd_md));
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.write_perm);
    cccd_md.vloc                = BLE_GATTS_VLOC_STACK;    
    char_md.p_cccd_md           = &cccd_md;
    char_md.char_props.notify   = 1;
   
    
    // OUR_JOB: Step 2.B, Configure the attribute metadata
    ble_gatts_attr_md_t attr_md;
    memset(&attr_md, 0, sizeof(attr_md)); 
    attr_md.vloc        = BLE_GATTS_VLOC_STACK;   
    
    
    // OUR_JOB: Step 2.G, Set read/write security levels to our characteristic
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.write_perm);
    
    
    // OUR_JOB: Step 2.C, Configure the characteristic value attribute
    ble_gatts_attr_t    attr_char_value;
    memset(&attr_char_value, 0, sizeof(attr_char_value));        
    attr_char_value.p_uuid      = &char_uuid;
    attr_char_value.p_attr_md   = &attr_md;
    
    // OUR_JOB: Step 2.H, Set characteristic length in number of bytes
    attr_char_value.max_len     = 20;
    attr_char_value.init_len    = 20;
    uint8_t value[4]            = {0x00};
    attr_char_value.p_value     = value;

    // OUR_JOB: Step 2.E, Add our new characteristic to the service
    err_code = sd_ble_gatts_characteristic_add(p_gss->service_handle,
                                       &char_md,
                                       &attr_char_value,
                                       &p_gss->bar_config_handles);
    APP_ERROR_CHECK(err_code);
    
    return NRF_SUCCESS;
}

static uint32_t light_data_char_add(ble_gss_t * p_gss, const ble_gss_init_t * p_gss_init)
{
    uint32_t   err_code = 0; // Variable to hold return codes from library and softdevice functions
    
    // OUR_JOB: Step 2.A, Add a custom characteristic UUID
    ble_uuid_t          char_uuid;
    ble_uuid128_t       base_uuid = BLE_UUID_GSS_BASE_UUID;
    char_uuid.uuid      = BLE_UUID_GSS_LIGHT_DATA_CHARACTERISTIC;
    sd_ble_uuid_vs_add(&base_uuid, &char_uuid.type);
    APP_ERROR_CHECK(err_code);
    
    // OUR_JOB: Step 2.F Add read/write properties to our characteristic
    ble_gatts_char_md_t char_md;
    memset(&char_md, 0, sizeof(char_md));
    char_md.char_props.read = 1;
    char_md.char_props.write = 0;

    
    // OUR_JOB: Step 3.A, Configuring Client Characteristic Configuration Descriptor metadata and add to char_md structure
    ble_gatts_attr_md_t cccd_md;
    memset(&cccd_md, 0, sizeof(cccd_md));
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.write_perm);
    cccd_md.vloc                = BLE_GATTS_VLOC_STACK;    
    char_md.p_cccd_md           = &cccd_md;
    char_md.char_props.notify   = 1;
   
    
    // OUR_JOB: Step 2.B, Configure the attribute metadata
    ble_gatts_attr_md_t attr_md;
    memset(&attr_md, 0, sizeof(attr_md)); 
    attr_md.vloc        = BLE_GATTS_VLOC_STACK;   
    
    
    // OUR_JOB: Step 2.G, Set read/write security levels to our characteristic
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.write_perm);
    
    
    // OUR_JOB: Step 2.C, Configure the characteristic value attribute
    ble_gatts_attr_t    attr_char_value;
    memset(&attr_char_value, 0, sizeof(attr_char_value));        
    attr_char_value.p_uuid      = &char_uuid;
    attr_char_value.p_attr_md   = &attr_md;
    
    // OUR_JOB: Step 2.H, Set characteristic length in number of bytes
    attr_char_value.max_len     = 20;
    attr_char_value.init_len    = 20;
    uint8_t value[4]            = {0x00};
    attr_char_value.p_value     = value;

    // OUR_JOB: Step 2.E, Add our new characteristic to the service
    err_code = sd_ble_gatts_characteristic_add(p_gss->service_handle,
                                       &char_md,
                                       &attr_char_value,
                                       &p_gss->light_data_handles);
    APP_ERROR_CHECK(err_code);
    
    return NRF_SUCCESS;
    /**@snippet [Adding proprietary characteristic to S110 SoftDevice] */
}

static uint32_t light_config_char_add(ble_gss_t * p_gss, const ble_gss_init_t * p_gss_init)
{
    uint32_t   err_code = 0; // Variable to hold return codes from library and softdevice functions
    
    // OUR_JOB: Step 2.A, Add a custom characteristic UUID
    ble_uuid_t          char_uuid;
    ble_uuid128_t       base_uuid = BLE_UUID_GSS_BASE_UUID;
    char_uuid.uuid      = BLE_UUID_GSS_LIGHT_CONFIG_CHARACTERISTIC;
    sd_ble_uuid_vs_add(&base_uuid, &char_uuid.type);
    APP_ERROR_CHECK(err_code);
    
    // OUR_JOB: Step 2.F Add read/write properties to our characteristic
    ble_gatts_char_md_t char_md;
    memset(&char_md, 0, sizeof(char_md));
    char_md.char_props.read = 1;
    char_md.char_props.write = 1;

    
    // OUR_JOB: Step 3.A, Configuring Client Characteristic Configuration Descriptor metadata and add to char_md structure
    ble_gatts_attr_md_t cccd_md;
    memset(&cccd_md, 0, sizeof(cccd_md));
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.write_perm);
    cccd_md.vloc                = BLE_GATTS_VLOC_STACK;    
    char_md.p_cccd_md           = &cccd_md;
    char_md.char_props.notify   = 1;
   
    
    // OUR_JOB: Step 2.B, Configure the attribute metadata
    ble_gatts_attr_md_t attr_md;
    memset(&attr_md, 0, sizeof(attr_md)); 
    attr_md.vloc        = BLE_GATTS_VLOC_STACK;   
    
    
    // OUR_JOB: Step 2.G, Set read/write security levels to our characteristic
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.write_perm);
    
    
    // OUR_JOB: Step 2.C, Configure the characteristic value attribute
    ble_gatts_attr_t    attr_char_value;
    memset(&attr_char_value, 0, sizeof(attr_char_value));        
    attr_char_value.p_uuid      = &char_uuid;
    attr_char_value.p_attr_md   = &attr_md;
    
    // OUR_JOB: Step 2.H, Set characteristic length in number of bytes
    attr_char_value.max_len     = 20;
    attr_char_value.init_len    = 20;
    uint8_t value[4]            = {0x00};
    attr_char_value.p_value     = value;

    // OUR_JOB: Step 2.E, Add our new characteristic to the service
    err_code = sd_ble_gatts_characteristic_add(p_gss->service_handle,
                                       &char_md,
                                       &attr_char_value,
                                       &p_gss->light_config_handles);
    APP_ERROR_CHECK(err_code);
    
    return NRF_SUCCESS;
}

void ble_gss_on_ble_evt(ble_gss_t * p_gss, ble_evt_t * p_ble_evt)
{
    if ((p_gss == NULL) || (p_ble_evt == NULL))
    {
        return;
    }

    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GAP_EVT_CONNECTED:
            on_connect(p_gss, p_ble_evt);
						break;

        case BLE_GAP_EVT_DISCONNECTED:
            on_disconnect(p_gss, p_ble_evt);
            break;
				
				case BLE_GATTS_EVT_WRITE:
						on_write(p_gss, p_ble_evt);
						break;

        default:
            // No implementation needed.
            break;
    }
}

/**@brief Function for initiating our new service.
 *
 * @param[in]   p_our_service        Our Service structure.
 *
 */
void ble_gss_init(ble_gss_t * p_gss, const ble_gss_init_t * p_gss_init)
{
    uint32_t   err_code; // Variable to hold return codes from library and softdevice functions
	
	  p_gss->conn_handle = BLE_CONN_HANDLE_INVALID;
	  p_gss->xyz_config_handler   = p_gss_init->xyz_config_handler;
	  p_gss->mic_config_handler   = p_gss_init->mic_config_handler;
	  p_gss->bar_config_handler   = p_gss_init->bar_config_handler;
	  p_gss->light_config_handler = p_gss_init->light_config_handler;
	  p_gss->is_notification_enabled = false;

    // OUR_JOB: Declare 16-bit service and 128-bit base UUIDs and add them to the BLE stack
    ble_uuid_t        service_uuid;
    ble_uuid128_t     base_uuid = BLE_UUID_GSS_BASE_UUID;
    service_uuid.uuid = BLE_UUID_GSS_SERVICE;
    err_code = sd_ble_uuid_vs_add(&base_uuid, &service_uuid.type);
    APP_ERROR_CHECK(err_code);    

    // OUR_JOB: Add our service
    err_code = sd_ble_gatts_service_add(BLE_GATTS_SRVC_TYPE_PRIMARY,
                                        &service_uuid,
                                        &p_gss->service_handle);
    APP_ERROR_CHECK(err_code);
	
	    // Add the sensor data characteristic
    err_code = xyz_data_char_add(p_gss, p_gss_init);
    APP_ERROR_CHECK(err_code);
		
		err_code = xyz_config_char_add(p_gss, p_gss_init);
		APP_ERROR_CHECK(err_code);
		
    err_code = mic_data_char_add(p_gss, p_gss_init);
    APP_ERROR_CHECK(err_code);
		
		err_code = mic_config_char_add(p_gss, p_gss_init);
		APP_ERROR_CHECK(err_code);

    err_code = bar_data_char_add(p_gss, p_gss_init);
    APP_ERROR_CHECK(err_code);
		
		err_code = bar_config_char_add(p_gss, p_gss_init);
		APP_ERROR_CHECK(err_code);

    err_code = light_data_char_add(p_gss, p_gss_init);
    APP_ERROR_CHECK(err_code);
		
		err_code = light_config_char_add(p_gss, p_gss_init);
		APP_ERROR_CHECK(err_code);

    // Print messages to Segger Real Time Terminal
    // UNCOMMENT THE FOUR LINES BELOW AFTER INITIALIZING THE SERVICE OR THE EXAMPLE WILL NOT COMPILE.
   // SEGGER_RTT_WriteString(0, "Exectuing our_service_init().\n"); // Print message to RTT to the application flow
   // SEGGER_RTT_printf(0, "Service UUID: 0x%#04x\n", service_uuid.uuid); // Print service UUID should match definition BLE_UUID_OUR_SERVICE
   // SEGGER_RTT_printf(0, "Service UUID type: 0x%#02x\n", service_uuid.type); // Print UUID type. Should match BLE_UUID_TYPE_VENDOR_BEGIN. Search for BLE_UUID_TYPES in ble_types.h for more info
   // SEGGER_RTT_printf(0, "Service handle: 0x%#04x\n", p_our_service->service_handle); // Print out the service handle. Should match service handle shown in MCP under Attribute values
}

uint32_t ble_gss_xyz_measurement_send(ble_gss_t * p_gss, uint8_t * accel, uint8_t * gyro, uint8_t * mag)
{
		uint32_t err_code;
	
		uint8_t data[18] = {accel[0], accel[1], 
												accel[2], accel[3],
												accel[4], accel[5],
											  gyro[0], gyro[1], 
												gyro[2], gyro[3],
												gyro[4], gyro[5],
												mag[0], mag[1],
												mag[2], mag[3],
												mag[4], mag[5]
											 }; 

	// Send value if connected and notifying
	if (p_gss->conn_handle != BLE_CONN_HANDLE_INVALID)
	{
			uint16_t               len;
			uint16_t               hvx_len;
			ble_gatts_hvx_params_t hvx_params;

			len     = sizeof(data);
			hvx_len = len;

			memset(&hvx_params, 0, sizeof(hvx_params));

			hvx_params.handle = p_gss->xyz_data_handles.value_handle;
			hvx_params.type   = BLE_GATT_HVX_NOTIFICATION;
			hvx_params.offset = 0;
			hvx_params.p_len  = &hvx_len;
			hvx_params.p_data = data;

			err_code = sd_ble_gatts_hvx(p_gss->conn_handle, &hvx_params);
			if ((err_code == NRF_SUCCESS) && (hvx_len != len))
			{
					err_code = NRF_ERROR_DATA_SIZE;
			}
	}
	else
	{
			err_code = NRF_ERROR_INVALID_STATE;
	}

	return err_code;
}

uint32_t ble_gss_bar_measurement_send(ble_gss_t * p_gss, uint8_t * bar)
{
		uint32_t err_code;

		uint8_t data[8] = {bar[0], bar[1], 
											 bar[2], bar[3],
											 bar[4], bar[5],
											 bar[6], bar[7]
											 }; 
	// Send value if connected and notifying
	if (p_gss->conn_handle != BLE_CONN_HANDLE_INVALID)
	{
			uint16_t               len;
			uint16_t               hvx_len;
			ble_gatts_hvx_params_t hvx_params;

			len     = sizeof(data);
			hvx_len = len;

			memset(&hvx_params, 0, sizeof(hvx_params));

			hvx_params.handle = p_gss->bar_data_handles.value_handle;
			hvx_params.type   = BLE_GATT_HVX_NOTIFICATION;
			hvx_params.offset = 0;
			hvx_params.p_len  = &hvx_len;
			hvx_params.p_data = data;

			err_code = sd_ble_gatts_hvx(p_gss->conn_handle, &hvx_params);
			if ((err_code == NRF_SUCCESS) && (hvx_len != len))
			{
					err_code = NRF_ERROR_DATA_SIZE;
			}
	}
	else
	{
			err_code = NRF_ERROR_INVALID_STATE;
	}

	return err_code;
}

uint32_t ble_gss_bar_cal_send(ble_gss_t * p_gss, uint8_t * cal)
{
		uint32_t err_code;

		uint8_t data[20] = {cal[0], cal[1], 
											 cal[2], cal[3],
											 cal[4], cal[5],
											 cal[6], cal[7],
											 cal[8], cal[9], 
											 cal[10], cal[11],
											 cal[12], cal[13],
											 cal[14], cal[15],
											 cal[16], cal[17],
										   cal[18], cal[19]
											 }; 
	// Send value if connected and notifying
	if (p_gss->conn_handle != BLE_CONN_HANDLE_INVALID)
	{
			uint16_t               len;
			uint16_t               hvx_len;
			ble_gatts_hvx_params_t hvx_params;

			len     = sizeof(data);
			hvx_len = len;

			memset(&hvx_params, 0, sizeof(hvx_params));

			hvx_params.handle = p_gss->bar_config_handles.value_handle;
			hvx_params.type   = BLE_GATT_HVX_NOTIFICATION;
			hvx_params.offset = 0;
			hvx_params.p_len  = &hvx_len;
			hvx_params.p_data = data;

			err_code = sd_ble_gatts_hvx(p_gss->conn_handle, &hvx_params);
			if ((err_code == NRF_SUCCESS) && (hvx_len != len))
			{
					err_code = NRF_ERROR_DATA_SIZE;
			}
	}
	else
	{
			err_code = NRF_ERROR_INVALID_STATE;
	}

	return err_code;	
}

uint32_t ble_gss_light_measurement_send(ble_gss_t * p_gss, uint8_t * light)
{
		uint32_t err_code;
    
		uint8_t data[4] = {light[0], light[1], 
											 light[2], light[3]
											 }; 
	   
	// Send value if connected and notifying
	if (p_gss->conn_handle != BLE_CONN_HANDLE_INVALID)
	{
			uint16_t               len;
			uint16_t               hvx_len;
			ble_gatts_hvx_params_t hvx_params;

			len     = sizeof(data);
			hvx_len = len;

			memset(&hvx_params, 0, sizeof(hvx_params));

			hvx_params.handle = p_gss->light_data_handles.value_handle;
			hvx_params.type   = BLE_GATT_HVX_NOTIFICATION;
			hvx_params.offset = 0;
			hvx_params.p_len  = &hvx_len;
			hvx_params.p_data = data;

			err_code = sd_ble_gatts_hvx(p_gss->conn_handle, &hvx_params);
			if ((err_code == NRF_SUCCESS) && (hvx_len != len))
			{
					err_code = NRF_ERROR_DATA_SIZE;
			}
	}
	else
	{
			err_code = NRF_ERROR_INVALID_STATE;
	}

	return err_code;
}
