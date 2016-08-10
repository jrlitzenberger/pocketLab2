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

/**@file
 *
 * @defgroup ble_sdk_srv_pcs PWM Control Service
 * @{
 * @ingroup  ble_sdk_srv
 * @brief    PWM Contrl Service implementation.
 *
 * @details The PWM Control Service is a simple GATT-based service with TX and RX characteristics.
 *          Data received from the peer is passed to the application, and the data received
 *          from the application of this service is sent to the peer as Handle Value
 *          Notifications. This module demonstrates how to implement a custom GATT-based
 *          service and characteristics using the SoftDevice. The service
 *          is used by the application to send and receive ASCII text strings to and from the
 *          peer.
 *
 * @note The application must propagate SoftDevice events to the Nordic UART Service module
 *       by calling the ble_nus_on_ble_evt() function from the ble_stack_handler callback.
 */

#ifndef BLE_PCS_H__
#define BLE_PCS_H__

#include "ble.h"
#include "ble_srv_common.h"
#include <stdint.h>
#include <stdbool.h>

#define BLE_UUID_PCS_SERVICE 0xAA21                      /**< The UUID of the PWM Control Service. */
#define BLE_PCS_MAX_DATA_LEN (GATT_MTU_SIZE_DEFAULT - 3) /**< Maximum length of data (in bytes) that can be transmitted to the peer by the PWM Control module. */

/* Forward declaration of the ble_pcs_t type. */
typedef struct ble_pcs_s ble_pcs_t;

/**@brief PWM Control Service event handler type. */
typedef void (*ble_pcs_pwm1_handler_t) (ble_pcs_t * p_pcs, uint8_t * p_data, uint16_t length);
typedef void (*ble_pcs_pwm2_handler_t) (ble_pcs_t * p_pcs, uint8_t * p_data, uint16_t length);
typedef void (*ble_pcs_pwm3_handler_t) (ble_pcs_t * p_pcs, uint8_t * p_data, uint16_t length);

/**@brief PWM Control Service initialization structure.
 *
 * @details This structure contains the initialization information for the service. The application
 * must fill this structure and pass it to the service using the @ref ble_nus_init
 *          function.
 */
typedef struct
{
    ble_pcs_pwm1_handler_t pwm1_handler; /**< Event handler to be called for handling received data. */
	  ble_pcs_pwm2_handler_t pwm2_handler;
	  ble_pcs_pwm3_handler_t pwm3_handler;
} ble_pcs_init_t;

/**@brief PWM Control Service structure.
 *
 * @details This structure contains status information related to the service.
 */
struct ble_pcs_s
{
    uint8_t                  uuid_type;               /**< UUID type for PWM Control Service Base UUID. */
    uint16_t                 service_handle;          /**< Handle of PWM Control Service (as provided by the SoftDevice). */
    ble_gatts_char_handles_t pwm1_handles;            /**< Handles related to the pwm1 characteristic (as provided by the SoftDevice). */
    ble_gatts_char_handles_t pwm2_handles;            /**< Handles related to the pwm2 characteristic (as provided by the SoftDevice). */
    ble_gatts_char_handles_t pwm3_handles;            /**< Handles related to the pwm3 characterstic (as provided by the SoftDevice).  */
	  uint16_t                 conn_handle;             /**< Handle of the current connection (as provided by the SoftDevice). BLE_CONN_HANDLE_INVALID if not in a connection. */
    bool                     is_notification_enabled; /**< Variable to indicate if the peer has enabled notification of the RX characteristic.*/
    ble_pcs_pwm1_handler_t   pwm1_handler;            /**< Event handler to be called for handling received data. */
	  ble_pcs_pwm2_handler_t   pwm2_handler;
	  ble_pcs_pwm3_handler_t   pwm3_handler;
};

/**@brief Function for initializing the PWM Control Service.
 *
 * @param[out] p_pcs      PWM Control Service structure. This structure must be supplied
 *                        by the application. It is initialized by this function and will
 *                        later be used to identify this particular service instance.
 * @param[in] p_pcs_init  Information needed to initialize the service.
 *
 * @retval NRF_SUCCESS If the service was successfully initialized. Otherwise, an error code is returned.
 * @retval NRF_ERROR_NULL If either of the pointers p_pcs or p_pcs_init is NULL.
 */
uint32_t ble_pcs_init(ble_pcs_t * p_pcs, const ble_pcs_init_t * p_pcs_init);

/**@brief Function for handling the PWM Control Service's BLE events.
 *
 * @details The PWM Control Service expects the application to call this function each time an
 * event is received from the SoftDevice. This function processes the event if it
 * is relevant and calls the PWM Control Service event handler of the
 * application if necessary.
 *
 * @param[in] p_pcs       PWM Control Service structure.
 * @param[in] p_ble_evt   Event received from the SoftDevice.
 */
void ble_pcs_on_ble_evt(ble_pcs_t * p_pcs, ble_evt_t * p_ble_evt);

#endif // BLE_NUS_H__

/** @} */
