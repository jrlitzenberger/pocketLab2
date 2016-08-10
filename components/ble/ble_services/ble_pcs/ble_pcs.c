/* Copyright (c) 2012 Nordic Semiconductor. All Rights Reserved.
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

#include "ble_pcs.h"
#include "ble_srv_common.h"
#include "sdk_common.h"

#define BLE_UUID_PCS_PWM1_CHARACTERISTIC 0xAA22                      /**< The UUID of the PWM1 Characteristic. */
#define BLE_UUID_PCS_PWM2_CHARACTERISTIC 0xAA23                      /**< The UUID of the PWM2 Characteristic. */
#define BLE_UUID_PCS_PWM3_CHARACTERISTIC 0xAA24                      /**< The UUID of the PWM3 Characteristic. */

#define BLE_PCS_MAX_PWM1_CHAR_LEN        BLE_PCS_MAX_DATA_LEN        /**< Maximum length of the PWM1 Characteristic (in bytes). */
#define BLE_PCS_MAX_PWM2_CHAR_LEN        BLE_PCS_MAX_DATA_LEN        /**< Maximum length of the PWM2 Characteristic (in bytes). */
#define BLE_PCS_MAX_PWM3_CHAR_LEN        BLE_PCS_MAX_DATA_LEN			   /**< Maximum length of the PWM3 Characteristic (in bytes). */

#define PCS_BASE_UUID                  {{0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xB0, 0x00, 0x40, 0x51, 0x04, 0x00, 0x00, 0x00, 0xF0}} /**< Used vendor specific UUID. */

/**@brief Function for handling the @ref BLE_GAP_EVT_CONNECTED event from the S110 SoftDevice.
 *
 * @param[in] p_pcs     PWM Control Service structure.
 * @param[in] p_ble_evt Pointer to the event received from BLE stack.
 */
static void on_connect(ble_pcs_t * p_pcs, ble_evt_t * p_ble_evt)
{
    p_pcs->conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
}


/**@brief Function for handling the @ref BLE_GAP_EVT_DISCONNECTED event from the S110 SoftDevice.
 *
 * @param[in] p_pcs     PWM Control Service structure.
 * @param[in] p_ble_evt Pointer to the event received from BLE stack.
 */
static void on_disconnect(ble_pcs_t * p_pcs, ble_evt_t * p_ble_evt)
{
    UNUSED_PARAMETER(p_ble_evt);
    p_pcs->conn_handle = BLE_CONN_HANDLE_INVALID;
}


/**@brief Function for handling the @ref BLE_GATTS_EVT_WRITE event from the S110 SoftDevice.
 *
 * @param[in] p_pcs     PWM Control Service structure.
 * @param[in] p_ble_evt Pointer to the event received from BLE stack.
 */
static void on_write(ble_pcs_t * p_pcs, ble_evt_t * p_ble_evt)
{
    ble_gatts_evt_write_t * p_evt_write = &p_ble_evt->evt.gatts_evt.params.write;

    if ((p_evt_write->handle == p_pcs->pwm1_handles.value_handle) && (p_pcs->pwm1_handler != NULL))
    {
        p_pcs->pwm1_handler(p_pcs, p_evt_write->data, p_evt_write->len);
    }
    else if ((p_evt_write->handle == p_pcs->pwm2_handles.value_handle) && (p_pcs->pwm2_handler != NULL))
    {
        p_pcs->pwm2_handler(p_pcs, p_evt_write->data, p_evt_write->len);
    }
		else if ((p_evt_write->handle == p_pcs->pwm3_handles.value_handle) && (p_pcs->pwm3_handler != NULL))
		{
				p_pcs->pwm3_handler(p_pcs, p_evt_write->data, p_evt_write->len);
		}
    else
    {
        // Do Nothing. This event is not relevant for this service.
    }
}


/**@brief Function for adding PWM1 characteristic.
 *
 * @param[in] p_pcs       PWM Control Service structure.
 * @param[in] p_pcs_init  Information needed to initialize the service.
 *
 * @return NRF_SUCCESS on success, otherwise an error code.
 */
static uint32_t pwm1_char_add(ble_pcs_t * p_pcs, const ble_pcs_init_t * p_pcs_init)
{
    ble_gatts_char_md_t char_md;
    ble_gatts_attr_t    attr_char_value;
    ble_uuid_t          ble_uuid;
    ble_gatts_attr_md_t attr_md;

    memset(&char_md, 0, sizeof(char_md));

	  char_md.char_props.read          = 1;
    char_md.char_props.write         = 1;
    char_md.char_props.write_wo_resp = 1;
    char_md.p_char_user_desc         = NULL;
    char_md.p_char_pf                = NULL;
    char_md.p_user_desc_md           = NULL;
    char_md.p_cccd_md                = NULL;
    char_md.p_sccd_md                = NULL;

    ble_uuid.type = p_pcs->uuid_type;
    ble_uuid.uuid = BLE_UUID_PCS_PWM1_CHARACTERISTIC;

    memset(&attr_md, 0, sizeof(attr_md));

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.write_perm);

    attr_md.vloc    = BLE_GATTS_VLOC_STACK;
    attr_md.rd_auth = 0;
    attr_md.wr_auth = 0;
    attr_md.vlen    = 1;

    memset(&attr_char_value, 0, sizeof(attr_char_value));

    attr_char_value.p_uuid    = &ble_uuid;
    attr_char_value.p_attr_md = &attr_md;
    attr_char_value.init_len  = 1;
    attr_char_value.init_offs = 0;
    attr_char_value.max_len   = BLE_PCS_MAX_PWM1_CHAR_LEN;

    return sd_ble_gatts_characteristic_add(p_pcs->service_handle,
                                           &char_md,
                                           &attr_char_value,
                                           &p_pcs->pwm1_handles);
    /**@snippet [Adding proprietary characteristic to S110 SoftDevice] */
}


/**@brief Function for adding PWM2 characteristic.
 *
 * @param[in] p_pcs       PWM Control Service structure.
 * @param[in] p_pcs_init  Information needed to initialize the service.
 *
 * @return NRF_SUCCESS on success, otherwise an error code.
 */
static uint32_t pwm2_char_add(ble_pcs_t * p_pcs, const ble_pcs_init_t * p_pcs_init)
{
    ble_gatts_char_md_t char_md;
    ble_gatts_attr_t    attr_char_value;
    ble_uuid_t          ble_uuid;
    ble_gatts_attr_md_t attr_md;

    memset(&char_md, 0, sizeof(char_md));
	
		char_md.char_props.read          = 1;
    char_md.char_props.write         = 1;
    char_md.char_props.write_wo_resp = 1;
    char_md.p_char_user_desc         = NULL;
    char_md.p_char_pf                = NULL;
    char_md.p_user_desc_md           = NULL;
    char_md.p_cccd_md                = NULL;
    char_md.p_sccd_md                = NULL;

    ble_uuid.type = p_pcs->uuid_type;
    ble_uuid.uuid = BLE_UUID_PCS_PWM2_CHARACTERISTIC;

    memset(&attr_md, 0, sizeof(attr_md));

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.write_perm);

    attr_md.vloc    = BLE_GATTS_VLOC_STACK;
    attr_md.rd_auth = 0;
    attr_md.wr_auth = 0;
    attr_md.vlen    = 1;

    memset(&attr_char_value, 0, sizeof(attr_char_value));

    attr_char_value.p_uuid    = &ble_uuid;
    attr_char_value.p_attr_md = &attr_md;
    attr_char_value.init_len  = 1;
    attr_char_value.init_offs = 0;
    attr_char_value.max_len   = BLE_PCS_MAX_PWM2_CHAR_LEN;

    return sd_ble_gatts_characteristic_add(p_pcs->service_handle,
                                           &char_md,
                                           &attr_char_value,
                                           &p_pcs->pwm2_handles);
}

/**@brief Function for adding PWM3 characteristic.
 *
 * @param[in] p_pcs       PWM Control Service structure.
 * @param[in] p_pcs_init  Information needed to initialize the service.
 *
 * @return NRF_SUCCESS on success, otherwise an error code.
 */
static uint32_t pwm3_char_add(ble_pcs_t * p_pcs, const ble_pcs_init_t * p_pcs_init)
{
    ble_gatts_char_md_t char_md;
    ble_gatts_attr_t    attr_char_value;
    ble_uuid_t          ble_uuid;
    ble_gatts_attr_md_t attr_md;

    memset(&char_md, 0, sizeof(char_md));
	
		char_md.char_props.read    	 	   = 1;
    char_md.char_props.write         = 1;
    char_md.char_props.write_wo_resp = 1;
    char_md.p_char_user_desc         = NULL;
    char_md.p_char_pf                = NULL;
    char_md.p_user_desc_md           = NULL;
    char_md.p_cccd_md                = NULL;
    char_md.p_sccd_md                = NULL;

    ble_uuid.type = p_pcs->uuid_type;
    ble_uuid.uuid = BLE_UUID_PCS_PWM3_CHARACTERISTIC;

    memset(&attr_md, 0, sizeof(attr_md));

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.write_perm);

    attr_md.vloc    = BLE_GATTS_VLOC_STACK;
    attr_md.rd_auth = 0;
    attr_md.wr_auth = 0;
    attr_md.vlen    = 1;

    memset(&attr_char_value, 0, sizeof(attr_char_value));

    attr_char_value.p_uuid    = &ble_uuid;
    attr_char_value.p_attr_md = &attr_md;
    attr_char_value.init_len  = 1;
    attr_char_value.init_offs = 0;
    attr_char_value.max_len   = BLE_PCS_MAX_PWM3_CHAR_LEN;

    return sd_ble_gatts_characteristic_add(p_pcs->service_handle,
                                           &char_md,
                                           &attr_char_value,
                                           &p_pcs->pwm3_handles);
}


void ble_pcs_on_ble_evt(ble_pcs_t * p_pcs, ble_evt_t * p_ble_evt)
{
    if ((p_pcs == NULL) || (p_ble_evt == NULL))
    {
        return;
    }

    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GAP_EVT_CONNECTED:
            on_connect(p_pcs, p_ble_evt);
            break;

        case BLE_GAP_EVT_DISCONNECTED:
            on_disconnect(p_pcs, p_ble_evt);
            break;

        case BLE_GATTS_EVT_WRITE:
            on_write(p_pcs, p_ble_evt);
            break;

        default:
            // No implementation needed.
            break;
    }
}


uint32_t ble_pcs_init(ble_pcs_t * p_pcs, const ble_pcs_init_t * p_pcs_init)
{
    uint32_t      err_code;
    ble_uuid_t    ble_uuid;
    ble_uuid128_t pcs_base_uuid = PCS_BASE_UUID;

    VERIFY_PARAM_NOT_NULL(p_pcs);
    VERIFY_PARAM_NOT_NULL(p_pcs_init);

    // Initialize the service structure.
    p_pcs->conn_handle             = BLE_CONN_HANDLE_INVALID;
    p_pcs->pwm1_handler            = p_pcs_init->pwm1_handler;
	  p_pcs->pwm2_handler            = p_pcs_init->pwm2_handler;
	  p_pcs->pwm3_handler  	 	 	     = p_pcs_init->pwm3_handler;
    p_pcs->is_notification_enabled = false;

    /**@snippet [Adding proprietary Service to S110 SoftDevice] */
    // Add a custom base UUID.
    err_code = sd_ble_uuid_vs_add(&pcs_base_uuid, &p_pcs->uuid_type);
    VERIFY_SUCCESS(err_code);

    ble_uuid.type = p_pcs->uuid_type;
    ble_uuid.uuid = BLE_UUID_PCS_SERVICE;

    // Add the service.
    err_code = sd_ble_gatts_service_add(BLE_GATTS_SRVC_TYPE_PRIMARY,
                                        &ble_uuid,
                                        &p_pcs->service_handle);
    /**@snippet [Adding proprietary Service to S110 SoftDevice] */
    VERIFY_SUCCESS(err_code);

    // Add the RX Characteristic.
    err_code = pwm1_char_add(p_pcs, p_pcs_init);
    VERIFY_SUCCESS(err_code);

    // Add the TX Characteristic.
    err_code = pwm2_char_add(p_pcs, p_pcs_init);
    VERIFY_SUCCESS(err_code);
		
		// Add the duration Characteristic
		err_code = pwm3_char_add(p_pcs, p_pcs_init);
		VERIFY_SUCCESS(err_code);

    return NRF_SUCCESS;
}
