/******************************************************************************
* File Name:   main.c
*******************************************************************************
* (c) 2023, Cypress Semiconductor Corporation. All rights reserved.
*******************************************************************************
* This software, including source code, documentation and related materials
* ("Software"), is owned by Cypress Semiconductor Corporation or one of its
* subsidiaries ("Cypress") and is protected by and subject to worldwide patent
* protection (United States and foreign), United States copyright laws and
* international treaty provisions. Therefore, you may use this Software only
* as provided in the license agreement accompanying the software package from
* which you obtained this Software ("EULA").
*
* If no EULA applies, Cypress hereby grants you a personal, non-exclusive,
* non-transferable license to copy, modify, and compile the Software source
* code solely for use in connection with Cypress's integrated circuit products.
* Any reproduction, modification, translation, compilation, or representation
* of this Software except as specified above is prohibited without the express
* written permission of Cypress.
*
* Disclaimer: THIS SOFTWARE IS PROVIDED AS-IS, WITH NO WARRANTY OF ANY KIND,
* EXPRESS OR IMPLIED, INCLUDING, BUT NOT LIMITED TO, NONINFRINGEMENT, IMPLIED
* WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE. Cypress
* reserves the right to make changes to the Software without notice. Cypress
* does not assume any liability arising out of the application or use of the
* Software or any product or circuit described in the Software. Cypress does
* not authorize its products for use in any products where a malfunction or
* failure of the Cypress product may reasonably be expected to result in
* significant property damage, injury or death ("High Risk Product"). By
* including Cypress's product in a High Risk Product, the manufacturer of such
* system or application assumes all risk of such use and in doing so agrees to
* indemnify Cypress against all liability.
*******************************************************************************/
#include "cybsp.h"
#include "cyhal.h"
#include "cy_retarget_io.h"

/* Library for malloc and free */
#include "stdlib.h"

/* FreeRTOS */
#include <FreeRTOS.h>
#include <task.h>

/* btstack */
#include "wiced_bt_stack.h"

/* App utilities */
#include "app_bt_utils.h"

/* Include header files from BT configurator */
#include "cycfg_bt_settings.h"
#include "cycfg_gap.h"
#include "cycfg_gatt_db.h"


/*******************************************************************
 * Macros to assist development of the exercises
 ******************************************************************/
#define TASK_STACK_SIZE (4096u)
#define	TASK_PRIORITY 	(5u)
#define GPIO_INTERRUPT_PRIORITY (6u)

#define MOTOR_PWM_FREQUENCY (100u)
#define PWM_CLOCK_FREQUENCY (1000000u)
#define SERVO_PERIOD (5000u)

#define MOTOR_OFF   (0.01)

#define MOTOR_LEFT_FWD_PIN	CYBSP_D11
#define MOTOR_LEFT_REV_PIN	CYBSP_D10
#define MOTOR_RIGHT_FWD_PIN	CYBSP_D9
#define MOTOR_RIGHT_REV_PIN	CYBSP_D8

#define SERVO_PIN CYBSP_A4

#define ULTRASONIC_TRIG_PIN CYBSP_D13

#define OPTICAL_LEFT_PIN 	CYBSP_D7
#define OPTICAL_RIGHT_PIN 	CYBSP_D2

#define HALF_TURN (16)
#define HALF_TURN_SPEED (25)


/* Typdef for function used to free allocated buffer to stack */
typedef void (*pfn_free_buffer_t)(uint8_t *);

/*******************************************************************
 * Function Prototypes
 ******************************************************************/
/* Callback function for Bluetooth stack management type events */
static wiced_bt_dev_status_t app_bt_management_callback             (wiced_bt_management_evt_t event, wiced_bt_management_evt_data_t *p_event_data);

/* GATT Event Callback and Handler Functions */
static wiced_bt_gatt_status_t app_bt_gatt_event_callback            (wiced_bt_gatt_evt_t event, wiced_bt_gatt_event_data_t *p_event_data);

static wiced_bt_gatt_status_t app_bt_connect_event_handler          (wiced_bt_gatt_connection_status_t *p_conn_status);
static wiced_bt_gatt_status_t app_bt_server_event_handler           (wiced_bt_gatt_event_data_t *p_data);

static wiced_bt_gatt_status_t app_bt_write_handler                  (wiced_bt_gatt_event_data_t *p_data);
static wiced_bt_gatt_status_t app_bt_gatt_req_read_handler          (uint16_t conn_id, wiced_bt_gatt_opcode_t opcode,
                                                                    wiced_bt_gatt_read_t *p_read_req, uint16_t len_requested);
static wiced_bt_gatt_status_t app_bt_gatt_req_read_multi_handler    (uint16_t conn_id, wiced_bt_gatt_opcode_t opcode,
                                                                    wiced_bt_gatt_read_multiple_req_t *p_read_req, uint16_t len_requested);
static wiced_bt_gatt_status_t app_bt_gatt_req_read_by_type_handler  (uint16_t conn_id, wiced_bt_gatt_opcode_t opcode,
                                                                    wiced_bt_gatt_read_by_type_t *p_read_req, uint16_t len_requested);

/* Helper functions to find GATT database handles and allocate/free buffers for GATT operations */
static gatt_db_lookup_table_t 	*app_bt_find_by_handle(uint16_t handle);
static uint8_t 					*app_bt_alloc_buffer(uint16_t len);
static void 					app_bt_free_buffer(uint8_t *p_data);

/* Car functions */
void updateMotor(uint8_t direction, uint8_t speed);
void updateServo(int8 angle);
static void distanceMeasureTask(void * arg);
static void optical_left_isr(void *handler_arg, cyhal_gpio_event_t event);
static void optical_right_isr(void *handler_arg, cyhal_gpio_event_t event);

/*******************************************************************
 * Global/Static Variables
 ******************************************************************/

/* Enable RTOS aware debugging in OpenOCD */
volatile int uxTopUsedPriority;

uint16_t connection_id = 0;

/* Use one clock divider for all PWMs - 1MHz clock will give us 1us resolution */
cyhal_clock_t clock_pwm;

/* Motor PWMs */
cyhal_pwm_t motor_left_fwd;
cyhal_pwm_t motor_left_rev;
cyhal_pwm_t motor_right_fwd;
cyhal_pwm_t motor_right_rev;

/* PWM object for servo motor */
cyhal_pwm_t servo_pwm;

/* Wheel distances */
uint32_t leftDistance =  0;
uint32_t rightDistance = 0;

/* ISR callback data structures for HAL v3.X */
cyhal_gpio_callback_data_t optical_left_cb_data =
{
	.callback     = optical_left_isr,
	.callback_arg = NULL
};

cyhal_gpio_callback_data_t optical_right_cb_data =
{
	.callback     = optical_right_isr,
	.callback_arg = NULL
};

/*******************************************************************
 * Function Implementations
 ******************************************************************/

/*******************************************************************************
* Function Name: int main( void )
********************************************************************************/
int main(void)
{
    cy_rslt_t result ;

    /* Initialize the board support package */
    result = cybsp_init() ;
    if (result != CY_RSLT_SUCCESS)
    {
        CY_ASSERT(0);
    }

    /* Enable global interrupts */
    __enable_irq();

    /* Initialize retarget-io to use the debug UART port */
    cy_retarget_io_init(CYBSP_DEBUG_UART_TX, CYBSP_DEBUG_UART_RX,\
                        CY_RETARGET_IO_BAUDRATE);

    printf("**********Application Start*****************\n");

	/* Initialize clock divider for PWMs */
	cyhal_clock_allocate(&clock_pwm, CYHAL_CLOCK_BLOCK_PERIPHERAL_16BIT);
	cyhal_clock_set_frequency(&clock_pwm, PWM_CLOCK_FREQUENCY, NULL);
	if (!cyhal_clock_is_enabled(&clock_pwm))
	{
		cyhal_clock_set_enabled(&clock_pwm, true, true);
	}

    /* Initialize motors */
    cyhal_pwm_init(&motor_left_fwd,  CYBSP_D11, &clock_pwm);
    cyhal_pwm_init(&motor_left_rev,  CYBSP_D10, &clock_pwm);
    cyhal_pwm_init(&motor_right_fwd, CYBSP_D9,  &clock_pwm);
    cyhal_pwm_init(&motor_right_rev, CYBSP_D8,  &clock_pwm);
    cyhal_pwm_set_duty_cycle(&motor_left_fwd,  MOTOR_OFF, MOTOR_PWM_FREQUENCY); /* Off */
    cyhal_pwm_set_duty_cycle(&motor_left_rev,  MOTOR_OFF, MOTOR_PWM_FREQUENCY); /* Off */
    cyhal_pwm_set_duty_cycle(&motor_right_fwd, MOTOR_OFF, MOTOR_PWM_FREQUENCY); /* Off */
    cyhal_pwm_set_duty_cycle(&motor_right_rev, MOTOR_OFF, MOTOR_PWM_FREQUENCY); /* Off */
    cyhal_pwm_start(&motor_left_fwd);
    cyhal_pwm_start(&motor_left_rev);
    cyhal_pwm_start(&motor_right_fwd);
    cyhal_pwm_start(&motor_right_rev);

    /* Initialize PWM for servo motor and set angle to 0 degrees */
    cyhal_pwm_init(&servo_pwm, SERVO_PIN, &clock_pwm);
    updateServo(0);
    cyhal_pwm_start(&servo_pwm);

    /* Initialize pins and timer for ultrasonic sensor */
    /* Trigger output */
	cyhal_gpio_init(ULTRASONIC_TRIG_PIN, CYHAL_GPIO_DIR_OUTPUT, CYHAL_GPIO_DRIVE_STRONG, 0);
	/* Echo return */
	Cy_TCPWM_Counter_Init(ULTRASONIC_COUNT_HW, ULTRASONIC_COUNT_NUM, &ULTRASONIC_COUNT_config);
	Cy_TCPWM_Counter_Enable(ULTRASONIC_COUNT_HW, ULTRASONIC_COUNT_NUM);

	/* Initialize pins for optical wheel sensors */
	cyhal_gpio_init(OPTICAL_LEFT_PIN, CYHAL_GPIO_DIR_INPUT, CYHAL_GPIO_DRIVE_NONE, 0);
	//cyhal_gpio_register_callback(OPTICAL_LEFT_PIN, optical_left_isr, NULL); // HAL v1.X
	cyhal_gpio_register_callback(OPTICAL_LEFT_PIN, &optical_left_cb_data); // HAL v2.X

	cyhal_gpio_init(OPTICAL_RIGHT_PIN, CYHAL_GPIO_DIR_INPUT, CYHAL_GPIO_DRIVE_NONE, 0);
	//cyhal_gpio_register_callback(OPTICAL_RIGHT_PIN, optical_right_isr, NULL); // HAL v1.X
	cyhal_gpio_register_callback(OPTICAL_RIGHT_PIN, &optical_right_cb_data); // HAL v2.X

    /* Configure platform specific settings for the BT device */
    cybt_platform_config_init(&cybsp_bt_platform_cfg);

    /* Initialize stack and register the callback function */
    wiced_bt_stack_init (app_bt_management_callback, &wiced_bt_cfg_settings);

    /* Create task to measure distance every 250ms */
    xTaskCreate(distanceMeasureTask, (char *)"distanceMeasureTask", TASK_STACK_SIZE, 0, TASK_STACK_SIZE, NULL);

    /* Start the FreeRTOS scheduler */
    vTaskStartScheduler() ;

    /* Should never get here */
    CY_ASSERT(0) ;
}


/*******************************************************************************
* Function Name: wiced_bt_dev_status_t app_bt_management_callback(
* 					wiced_bt_management_evt_t event,
* 					wiced_bt_management_evt_data_t *p_event_data )
********************************************************************************/
static wiced_result_t app_bt_management_callback( wiced_bt_management_evt_t event, wiced_bt_management_evt_data_t *p_event_data )
{
	/* Start in error state so that any unimplemented states will return error */
    wiced_result_t result = WICED_BT_ERROR;
    wiced_bt_device_address_t bda = {0};

    printf("Bluetooth Management Event: 0x%x %s\n", event, get_bt_event_name(event));

    switch( event )
    {
		case BTM_ENABLED_EVT:								// Bluetooth Controller and Host Stack Enabled
			if( WICED_BT_SUCCESS == p_event_data->enabled.status )
			{
				printf( "Bluetooth Enabled\n" );

				/* Set the local BDA from the value in the configurator and print it */
				wiced_bt_set_local_bdaddr((uint8_t *)cy_bt_device_address, BLE_ADDR_PUBLIC);
				wiced_bt_dev_read_local_addr( bda );
				printf( "Local Bluetooth Device Address: ");
				print_bd_address(bda);

				/* Register GATT callback and initialize database*/
				wiced_bt_gatt_register( app_bt_gatt_event_callback );
				wiced_bt_gatt_db_init( gatt_database, gatt_database_len, NULL );

				/* Disable pairing */
				wiced_bt_set_pairable_mode( WICED_FALSE, WICED_FALSE );

				/* Set advertisement packet and begin advertising */
				wiced_bt_ble_set_raw_advertisement_data(CY_BT_ADV_PACKET_DATA_SIZE,
				                                        cy_bt_adv_packet_data);
				wiced_bt_start_advertisements( BTM_BLE_ADVERT_UNDIRECTED_HIGH, 0, NULL );


	            result = WICED_BT_SUCCESS;
			}
			else
			{
				printf( "Failed to initialize Bluetooth controller and stack\n" );
			}
			break;

		case BTM_PAIRING_IO_CAPABILITIES_BLE_REQUEST_EVT: 		// IO capabilities request
			result = WICED_BT_SUCCESS;
			break;

		case BTM_PAIRING_COMPLETE_EVT: 							// Pairing Complete event
			result = WICED_BT_SUCCESS;
			break;

		case BTM_ENCRYPTION_STATUS_EVT: 						// Encryption Status Event
			result = WICED_BT_SUCCESS;
			break;

		case BTM_SECURITY_REQUEST_EVT: 							// Security access
			result = WICED_BT_SUCCESS;
			break;

		case BTM_PAIRED_DEVICE_LINK_KEYS_UPDATE_EVT: 			// Save link keys with app
			result = WICED_BT_SUCCESS;
			break;

		case BTM_PAIRED_DEVICE_LINK_KEYS_REQUEST_EVT: 			// Retrieve saved link keys
            /* This must return WICED_BT_ERROR if bonding information is not stored in EEPROM */
			result = WICED_BT_ERROR;
			break;

		case BTM_LOCAL_IDENTITY_KEYS_UPDATE_EVT: 				// Save keys to NVRAM
			result = WICED_BT_SUCCESS;
			break;

		case  BTM_LOCAL_IDENTITY_KEYS_REQUEST_EVT: 				// Read keys from NVRAM
            /* This should return WICED_BT_SUCCESS if not using privacy. If RPA is enabled but keys are not
               stored in EEPROM, this must return WICED_BT_ERROR so that the stack will generate new privacy keys */
			result = WICED_BT_SUCCESS;
			break;

		case BTM_BLE_SCAN_STATE_CHANGED_EVT: 					// Scan State Change
			result = WICED_BT_SUCCESS;
			break;

		case BTM_BLE_ADVERT_STATE_CHANGED_EVT:					// Advertising State Change
            printf("Advertisement State Change: %s\n", get_bt_advert_mode_name(p_event_data->ble_advert_state_changed));
            result = WICED_BT_SUCCESS;
			break;

		default:
			break;
    }

    return result;
}


/*******************************************************************************
* Function Name: wiced_bt_gatt_status_t app_bt_gatt_event_callback(
* 					wiced_bt_gatt_evt_t event,
* 					wiced_bt_gatt_event_data_t *p_data )
********************************************************************************/
static wiced_bt_gatt_status_t app_bt_gatt_event_callback( wiced_bt_gatt_evt_t event, wiced_bt_gatt_event_data_t *p_event_data )
{
	/* Start in error state so that any unimplemented states will return error */
    wiced_bt_gatt_status_t status = WICED_BT_GATT_ERROR;

    /* Call the appropriate callback function based on the GATT event type, and pass the relevant event
     * parameters to the callback function */
    switch (event)
    {
    case GATT_CONNECTION_STATUS_EVT:
        status = app_bt_connect_event_handler(&p_event_data->connection_status);
        break;

    case GATT_ATTRIBUTE_REQUEST_EVT:
        status = app_bt_server_event_handler(p_event_data);
        break;

    case GATT_GET_RESPONSE_BUFFER_EVT: /* GATT buffer request, typically sized to max of bearer mtu - 1 */
        p_event_data->buffer_request.buffer.p_app_rsp_buffer = app_bt_alloc_buffer(p_event_data->buffer_request.len_requested);
        p_event_data->buffer_request.buffer.p_app_ctxt = (void *)app_bt_free_buffer;
        status = WICED_BT_GATT_SUCCESS;
        break;

    case GATT_APP_BUFFER_TRANSMITTED_EVT: /* GATT buffer transmitted event,  check \ref wiced_bt_gatt_buffer_transmitted_t*/
        {
            pfn_free_buffer_t pfn_free = (pfn_free_buffer_t)p_event_data->buffer_xmitted.p_app_ctxt;

            /* If the buffer is dynamic, the context will point to a function to free it. */
            if (pfn_free)
            {
                pfn_free(p_event_data->buffer_xmitted.p_app_data);
            }
            status = WICED_BT_GATT_SUCCESS;
        }
        break;

    default:
    	printf( "Unhandled GATT Event: 0x%x (%d)\n", event, event );
        status = WICED_BT_GATT_SUCCESS;
        break;
    }

    return status;
}


/*******************************************************************************
 * Function Name: app_bt_connect_event_handler
 *
 * Handles GATT connection status changes.
 *
 * Param:	p_conn_status  Pointer to data that has connection details
 * Return:	wiced_bt_gatt_status_t
 * See possible status codes in wiced_bt_gatt_status_e in wiced_bt_gatt.h
*******************************************************************************/
static wiced_bt_gatt_status_t app_bt_connect_event_handler(wiced_bt_gatt_connection_status_t *p_conn_status)
{
    wiced_bt_gatt_status_t status = WICED_BT_GATT_ERROR;

    if (NULL != p_conn_status)
    {
        if (p_conn_status->connected)
        {
           	printf("GATT_CONNECTION_STATUS_EVT: Connect BDA ");
           	print_bd_address(p_conn_status->bd_addr);
			printf("Connection ID %d\n", p_conn_status->conn_id );

			/* Handle the connection */
			connection_id = p_conn_status->conn_id;
        }
        else
        {
            printf("Disconnected : BDA " );
            print_bd_address(p_conn_status->bd_addr);
            printf("Connection ID '%d', Reason '%s'\n", p_conn_status->conn_id, get_bt_gatt_disconn_reason_name(p_conn_status->reason) );

			/* Handle the disconnection */
            connection_id = 0;

			/* Restart the advertisements */
			wiced_bt_start_advertisements( BTM_BLE_ADVERT_UNDIRECTED_HIGH, 0, NULL );
        }

        status = WICED_BT_GATT_SUCCESS;
    }

    return status;
}


/*******************************************************************************
 * Function Name: app_bt_server_event_handler
 *
 * Invoked when GATT_ATTRIBUTE_REQUEST_EVT occurs in GATT Event callback.
 *
 * Param:	p_data   				Pointer to BLE GATT request data
 * Return:	wiced_bt_gatt_status_t  BLE GATT status
*******************************************************************************/
static wiced_bt_gatt_status_t app_bt_server_event_handler(wiced_bt_gatt_event_data_t *p_data)
{
    wiced_bt_gatt_status_t status = WICED_BT_GATT_ERROR;
    wiced_bt_gatt_attribute_request_t   *p_att_req = &p_data->attribute_request;

    switch (p_att_req->opcode)
    {

		case GATT_REQ_READ: /* Attribute read notification (attribute value internally read from GATT database) */
		case GATT_REQ_READ_BLOB:
			status = app_bt_gatt_req_read_handler(p_att_req->conn_id, p_att_req->opcode,
																	   &p_att_req->data.read_req, p_att_req->len_requested);
			break;

		case GATT_REQ_READ_BY_TYPE:
			status = app_bt_gatt_req_read_by_type_handler(p_att_req->conn_id, p_att_req->opcode,
																   &p_att_req->data.read_by_type, p_att_req->len_requested);
			break;

		case GATT_REQ_READ_MULTI:
		case GATT_REQ_READ_MULTI_VAR_LENGTH:
			status = app_bt_gatt_req_read_multi_handler(p_att_req->conn_id, p_att_req->opcode,
															  &p_att_req->data.read_multiple_req, p_att_req->len_requested);
			break;

		case GATT_REQ_WRITE:
		case GATT_CMD_WRITE:
		case GATT_CMD_SIGNED_WRITE:
			status = app_bt_write_handler(p_data);
			if ((p_att_req->opcode == GATT_REQ_WRITE) && (status == WICED_BT_GATT_SUCCESS))
			{
				wiced_bt_gatt_write_req_t *p_write_request = &p_att_req->data.write_req;
				wiced_bt_gatt_server_send_write_rsp(p_att_req->conn_id, p_att_req->opcode, p_write_request->handle);
			}
			break;

		case GATT_REQ_PREPARE_WRITE:
			status = WICED_BT_GATT_SUCCESS;
			break;

		case GATT_REQ_EXECUTE_WRITE:
			wiced_bt_gatt_server_send_execute_write_rsp(p_att_req->conn_id, p_att_req->opcode);
			status = WICED_BT_GATT_SUCCESS;
			break;

		case GATT_REQ_MTU:
			/* Application calls wiced_bt_gatt_server_send_mtu_rsp() with the desired mtu */
			status = wiced_bt_gatt_server_send_mtu_rsp(p_att_req->conn_id,
													   p_att_req->data.remote_mtu,
													   wiced_bt_cfg_settings.p_ble_cfg->ble_max_rx_pdu_size);
			break;

		case GATT_HANDLE_VALUE_CONF: /* Value confirmation */
			break;

		case GATT_HANDLE_VALUE_NOTIF:
			break;

		default:
	    	printf( "Unhandled GATT Server Event: 0x%x (%d)\n", p_att_req->opcode, p_att_req->opcode );
			break;
    }

    return status;
}


/*******************************************************************************
 * Function Name: app_bt_write_handler
 *
 * Invoked when GATTS_REQ_TYPE_WRITE is received from the
 * client device. Handles "Write Requests" received from Client device.
 *
 * Param:	p_write_req   			Pointer to BLE GATT write request
 * Return:	wiced_bt_gatt_status_t  BLE GATT status
*******************************************************************************/
static wiced_bt_gatt_status_t app_bt_write_handler(wiced_bt_gatt_event_data_t *p_data)
{
	wiced_bt_gatt_write_req_t *p_write_req = &p_data->attribute_request.data.write_req;;

    wiced_bt_gatt_status_t status = WICED_BT_GATT_INVALID_HANDLE;

     for (int i = 0; i < app_gatt_db_ext_attr_tbl_size; i++)
     {
         /* Check for a matching handle entry */
         if (app_gatt_db_ext_attr_tbl[i].handle == p_write_req->handle)
         {
             /* Detected a matching handle in the external lookup table */
             if (app_gatt_db_ext_attr_tbl[i].max_len >= p_write_req->val_len)
             {
                 /* Value fits within the supplied buffer; copy over the value */
                 app_gatt_db_ext_attr_tbl[i].cur_len = p_write_req->val_len;
                 memset(app_gatt_db_ext_attr_tbl[i].p_data, 0x00, app_gatt_db_ext_attr_tbl[i].max_len);
                 memcpy(app_gatt_db_ext_attr_tbl[i].p_data, p_write_req->p_val, app_gatt_db_ext_attr_tbl[i].cur_len);

                 if (memcmp(app_gatt_db_ext_attr_tbl[i].p_data, p_write_req->p_val, app_gatt_db_ext_attr_tbl[i].cur_len) == 0)
                 {
                	 status = WICED_BT_GATT_SUCCESS;
                 }

                 switch ( p_write_req->handle )
                 {
                 	 // Add action when specified handle is written
                 	 case HDLC_CAR_MOTOR_VALUE:
                 		 updateMotor(app_car_motor[0], app_car_speed[0]);
                 		 break;
                 	 case HDLC_CAR_ANGLE_VALUE:
                 		 /*convert app_car_angle uint8_t array to float */
                 		 updateServo(app_car_angle[0]);
                 		 break;
                 }
             }
             else
             {
                 /* Value to write will not fit within the table */
            	 status = WICED_BT_GATT_INVALID_ATTR_LEN;
                 printf("Invalid attribute length during GATT write\n");
             }
             break;
         }
     }
     if (WICED_BT_GATT_SUCCESS != status)
     {
         printf("GATT write failed: %d\n", status);
     }

     return status;
}


/*******************************************************************************
 * Function Name: app_bt_gatt_req_read_handler
 *
 * This Function handles GATT read and read blob events
 *
 * Params: 	conn_id       			Connection ID
 * 			opcode        			BLE GATT request type opcode
 * 			p_read_req    			Pointer to read request containing the handle to read
 * 			len_requested 			Length of data requested
 * Return: 	wiced_bt_gatt_status_t  BLE GATT status
*******************************************************************************/
static wiced_bt_gatt_status_t app_bt_gatt_req_read_handler(uint16_t conn_id, wiced_bt_gatt_opcode_t opcode,
                                                              wiced_bt_gatt_read_t *p_read_req, uint16_t len_requested)
{
    gatt_db_lookup_table_t *puAttribute;
    uint16_t attr_len_to_copy, to_send;
    uint8_t *from;

    if ((puAttribute = app_bt_find_by_handle(p_read_req->handle)) == NULL)
    {
        wiced_bt_gatt_server_send_error_rsp(conn_id, opcode, p_read_req->handle, WICED_BT_GATT_INVALID_HANDLE);
        return WICED_BT_GATT_INVALID_HANDLE;
    }

    attr_len_to_copy = puAttribute->cur_len;

    if (p_read_req->offset >= puAttribute->cur_len)
    {
        wiced_bt_gatt_server_send_error_rsp(conn_id, opcode, p_read_req->handle, WICED_BT_GATT_INVALID_OFFSET);
        return WICED_BT_GATT_INVALID_OFFSET;
    }

	switch ( p_read_req->handle )
	{
		// Add action when specified handle is read
	}

    to_send = MIN(len_requested, attr_len_to_copy - p_read_req->offset);
    from = puAttribute->p_data + p_read_req->offset;
    return wiced_bt_gatt_server_send_read_handle_rsp(conn_id, opcode, to_send, from, NULL); /* No need for context, as buff not allocated */
}

/*******************************************************************************
 * Function Name: app_bt_gatt_req_read_by_type_handler
 *
 * Process read-by-type request from peer device
 *
 * Params:	conn_id       			Connection ID
 * 			opcode        			BLE GATT request type opcode
 * 			p_read_req    			Pointer to read request containing the handle to read
 * 			len_requested 			Length of data requested
 * Return:	wiced_bt_gatt_status_t	BLE GATT status
*******************************************************************************/
static wiced_bt_gatt_status_t app_bt_gatt_req_read_by_type_handler(uint16_t conn_id, wiced_bt_gatt_opcode_t opcode,
                                                       wiced_bt_gatt_read_by_type_t *p_read_req, uint16_t len_requested)
{
    gatt_db_lookup_table_t *puAttribute;
    uint16_t attr_handle = p_read_req->s_handle;
    uint8_t *p_rsp = app_bt_alloc_buffer(len_requested);
    uint8_t pair_len = 0;
    int used = 0;

    if (p_rsp == NULL)
    {
        wiced_bt_gatt_server_send_error_rsp(conn_id, opcode, attr_handle, WICED_BT_GATT_INSUF_RESOURCE);
        return WICED_BT_GATT_INSUF_RESOURCE;
    }

    /* Read by type returns all attributes of the specified type, between the start and end handles */
    while (WICED_TRUE)
    {
        attr_handle = wiced_bt_gatt_find_handle_by_type(attr_handle, p_read_req->e_handle, &p_read_req->uuid);

        if (attr_handle == 0)
            break;

        if ((puAttribute = app_bt_find_by_handle(attr_handle)) == NULL)
        {
            wiced_bt_gatt_server_send_error_rsp(conn_id, opcode, p_read_req->s_handle, WICED_BT_GATT_ERR_UNLIKELY);
            app_bt_free_buffer(p_rsp);
            return WICED_BT_GATT_INVALID_HANDLE;
        }

        {
            int filled = wiced_bt_gatt_put_read_by_type_rsp_in_stream(p_rsp + used, len_requested - used, &pair_len,
                                                                attr_handle, puAttribute->cur_len, puAttribute->p_data);
            if (filled == 0)
            {
                break;
            }
            used += filled;
        }

        /* Increment starting handle for next search to one past current */
        attr_handle++;
    }

    if (used == 0)
    {
        wiced_bt_gatt_server_send_error_rsp(conn_id, opcode, p_read_req->s_handle, WICED_BT_GATT_INVALID_HANDLE);
        app_bt_free_buffer(p_rsp);
        return WICED_BT_GATT_INVALID_HANDLE;
    }

	switch ( p_read_req->s_handle )
	{
		// Add action when specified handle is read
	}

    /* Send the response */
    wiced_bt_gatt_server_send_read_by_type_rsp(conn_id, opcode, pair_len, used, p_rsp, (void *)app_bt_free_buffer);

    return WICED_BT_GATT_SUCCESS;
}

/*******************************************************************************
 * Function Name: app_bt_gatt_req_read_multi_handler
 *
 * Process write read multi request from peer device
 *
 * Params:	conn_id       			Connection ID
 * 			opcode        			BLE GATT request type opcode
 * 			p_read_req    			Pointer to read request containing the handle to read
 * 			len_requested 			Length of data requested
 * Return:	wiced_bt_gatt_status_t  BLE GATT status
*******************************************************************************/
static wiced_bt_gatt_status_t app_bt_gatt_req_read_multi_handler(uint16_t conn_id, wiced_bt_gatt_opcode_t opcode,
                                                  wiced_bt_gatt_read_multiple_req_t *p_read_req, uint16_t len_requested)
{
    gatt_db_lookup_table_t *puAttribute;
    uint8_t *p_rsp = app_bt_alloc_buffer(len_requested);
    int used = 0;
    int xx;
    uint16_t handle = wiced_bt_gatt_get_handle_from_stream(p_read_req->p_handle_stream, 0);

    if (p_rsp == NULL)
    {
        wiced_bt_gatt_server_send_error_rsp(conn_id, opcode, handle, WICED_BT_GATT_INSUF_RESOURCE);
        return WICED_BT_GATT_INVALID_HANDLE;
    }

    /* Read by type returns all attributes of the specified type, between the start and end handles */
    for (xx = 0; xx < p_read_req->num_handles; xx++)
    {
        handle = wiced_bt_gatt_get_handle_from_stream(p_read_req->p_handle_stream, xx);
        if ((puAttribute = app_bt_find_by_handle(handle)) == NULL)
        {
            wiced_bt_gatt_server_send_error_rsp(conn_id, opcode, *p_read_req->p_handle_stream, WICED_BT_GATT_ERR_UNLIKELY);
            app_bt_free_buffer(p_rsp);
            return WICED_BT_GATT_ERR_UNLIKELY;
        }

        {
            int filled = wiced_bt_gatt_put_read_multi_rsp_in_stream(opcode, p_rsp + used, len_requested - used,
                                                        puAttribute->handle, puAttribute->cur_len, puAttribute->p_data);
            if (!filled)
            {
                break;
            }
            used += filled;
        }
    }

    if (used == 0)
    {
        wiced_bt_gatt_server_send_error_rsp(conn_id, opcode, *p_read_req->p_handle_stream, WICED_BT_GATT_INVALID_HANDLE);
        return WICED_BT_GATT_INVALID_HANDLE;
    }

	switch ( *p_read_req->p_handle_stream )
	{
		// Add action when specified handle is read
	}

    /* Send the response */
    wiced_bt_gatt_server_send_read_multiple_rsp(conn_id, opcode, used, p_rsp, (void *)app_bt_free_buffer);

    return WICED_BT_GATT_SUCCESS;
}


/*******************************************************************************
* Function Name: app_bt_find_by_handle
*
* Finds attribute location by handle
*
* Param:  handle    				handle to look up
* Return: gatt_db_lookup_table_t   	pointer to location containing handle data
********************************************************************************/
static gatt_db_lookup_table_t *app_bt_find_by_handle(uint16_t handle)
{
    int i;
    for (i = 0; i < app_gatt_db_ext_attr_tbl_size; i++)
    {
        if (app_gatt_db_ext_attr_tbl[i].handle == handle)
        {
            return (&app_gatt_db_ext_attr_tbl[i]);
        }
    }
    return NULL;
}


/*******************************************************************************
* Function Name: app_bt_alloc_buffer
*
* This Function allocates the buffer of requested length
*
* Param:  len			Length of buffer
* Return: uint8_t*      Pointer to allocated buffer
********************************************************************************/
static uint8_t *app_bt_alloc_buffer(uint16_t len)
{
    uint8_t *p = (uint8_t *)malloc(len);
    return p;
}

/*******************************************************************************
* Function Name: app_bt_free_buffer
*
* This Function frees the buffer requested
*
* Param:  p_data		Pointer to buffer to be freed
********************************************************************************/
static void app_bt_free_buffer(uint8_t *p_data)
{
    if (p_data != NULL)
    {
        free(p_data);
    }
}


/*******************************************************************************
* Function Name: updateMotor
*
* This function updates the motor speed/direction
*
********************************************************************************/
void updateMotor(uint8_t direction, uint8_t speed)
{

	/* Motor Directions */
	enum motor_direction_e
	{
		MOTOR_STOP,
		MOTOR_FWD_LEFT,
		MOTOR_FWD,
		MOTOR_FWD_RIGHT,
		MOTOR_ROT_LEFT,
		MOTOR_ROT_RIGHT,
		MOTOR_REV_LEFT,
		MOTOR_REV,
		MOTOR_REV_RIGHT,
		MOTOR_180,
	};

	/* Turn off all motors first before setting new values */
	cyhal_pwm_set_duty_cycle(&motor_left_fwd,  MOTOR_OFF, MOTOR_PWM_FREQUENCY);
	cyhal_pwm_set_duty_cycle(&motor_right_fwd, MOTOR_OFF, MOTOR_PWM_FREQUENCY);
	cyhal_pwm_set_duty_cycle(&motor_left_rev,  MOTOR_OFF, MOTOR_PWM_FREQUENCY);
	cyhal_pwm_set_duty_cycle(&motor_right_rev, MOTOR_OFF, MOTOR_PWM_FREQUENCY);

	switch (direction)
	{
		case MOTOR_FWD_LEFT:
			cyhal_pwm_set_duty_cycle(&motor_left_fwd,  (speed >> 1),	MOTOR_PWM_FREQUENCY);
			cyhal_pwm_set_duty_cycle(&motor_right_fwd, (speed),			MOTOR_PWM_FREQUENCY);
			break;

		case MOTOR_FWD:
			cyhal_pwm_set_duty_cycle(&motor_left_fwd,  (speed),			MOTOR_PWM_FREQUENCY);
			cyhal_pwm_set_duty_cycle(&motor_right_fwd, (speed),			MOTOR_PWM_FREQUENCY);
			break;

		case MOTOR_FWD_RIGHT:
			cyhal_pwm_set_duty_cycle(&motor_left_fwd,  (speed),			MOTOR_PWM_FREQUENCY);
			cyhal_pwm_set_duty_cycle(&motor_right_fwd, (speed >> 1),	MOTOR_PWM_FREQUENCY);
			break;

		case MOTOR_ROT_LEFT:
			cyhal_pwm_set_duty_cycle(&motor_left_rev,  (speed >> 1),	MOTOR_PWM_FREQUENCY);
			cyhal_pwm_set_duty_cycle(&motor_right_fwd, (speed >> 1),	MOTOR_PWM_FREQUENCY);
			break;

		case MOTOR_ROT_RIGHT:
			cyhal_pwm_set_duty_cycle(&motor_left_fwd,  (speed >> 1),	MOTOR_PWM_FREQUENCY);
			cyhal_pwm_set_duty_cycle(&motor_right_rev, (speed >> 1),	MOTOR_PWM_FREQUENCY);
			break;

		case MOTOR_REV_LEFT:
			cyhal_pwm_set_duty_cycle(&motor_left_rev,  (speed >> 1),	MOTOR_PWM_FREQUENCY);
			cyhal_pwm_set_duty_cycle(&motor_right_rev, (speed),			MOTOR_PWM_FREQUENCY);
			break;

		case MOTOR_REV:
			cyhal_pwm_set_duty_cycle(&motor_left_rev,  (speed),			MOTOR_PWM_FREQUENCY);
			cyhal_pwm_set_duty_cycle(&motor_right_rev, (speed),			MOTOR_PWM_FREQUENCY);
			break;

		case MOTOR_REV_RIGHT:
			cyhal_pwm_set_duty_cycle(&motor_left_rev,  (speed),			MOTOR_PWM_FREQUENCY);
			cyhal_pwm_set_duty_cycle(&motor_right_rev, (speed >> 1),	MOTOR_PWM_FREQUENCY);
			break;

		case MOTOR_180: /* This case rotates 180 degrees using encoders and then stops */
			/* Reset counters */
			leftDistance = 0;
			rightDistance = 0;
			/* Enable interrupts */
			cyhal_gpio_enable_event(OPTICAL_LEFT_PIN,  CYHAL_GPIO_IRQ_RISE, GPIO_INTERRUPT_PRIORITY, true);
			cyhal_gpio_enable_event(OPTICAL_RIGHT_PIN, CYHAL_GPIO_IRQ_RISE, GPIO_INTERRUPT_PRIORITY, true);
			/* Start motors */
			cyhal_pwm_set_duty_cycle(&motor_left_rev,  HALF_TURN_SPEED,	MOTOR_PWM_FREQUENCY);
			cyhal_pwm_set_duty_cycle(&motor_right_fwd, HALF_TURN_SPEED,	MOTOR_PWM_FREQUENCY);
			/* The ISRs will stop the motors and disable themselves when the turn is complete */
			break;
	}
}


/*******************************************************************************
* Function Name: updateServo
*
* This function updates the servo angle
*
********************************************************************************/
void updateServo(int8 angle)
{
	uint16_t pulse;

	/* Max range is -90 to +90 degrees */
	if(angle < -90)
	{
		angle = -90;
	}	if(angle > 90)
	{
		angle = 90;
	}

	/* Translate angle to pulse width */
	/* -90 degrees to +90 degrees <-> 2500us to 500us */
	pulse = (uint16_t)((float)angle * -11.1111 + 1500.0);

	/* Update pulse width */
	cyhal_pwm_set_period(&servo_pwm, SERVO_PERIOD, pulse);
}


/*******************************************************************************
* Function Name: distanceMeasure
*
* This function is a thread that measures distance using the ultrasonic sensor
*
********************************************************************************/
static void distanceMeasureTask(void * arg)
{
	while(1)
	{
		/* Sense distance using ultrasonic sensor */
		/* Reset and Start counter that will measure return signal pulse width */
		Cy_TCPWM_Counter_SetCounter(ULTRASONIC_COUNT_HW, ULTRASONIC_COUNT_NUM, 0);
		Cy_TCPWM_TriggerStart_Single(ULTRASONIC_COUNT_HW, ULTRASONIC_COUNT_NUM);

		/* Send 10us pulse */
		cyhal_gpio_write(ULTRASONIC_TRIG_PIN, 1);
		cyhal_system_delay_us(10);
		cyhal_gpio_write(ULTRASONIC_TRIG_PIN, 0);

		/* Wait for return pulse to complete before reading the count value
		 * Max count possible during this time is ~10000 */
		cyhal_system_delay_ms(10);

		/* Read count value of return signal  - shifting right by 7 (divide by 128) gives a distance result approximately in inches */
		app_car_distance[0] = (uint8_t)(Cy_TCPWM_Counter_GetCounter(ULTRASONIC_COUNT_HW, ULTRASONIC_COUNT_NUM) >> 7);

		/*Send notification if they are enabled and we are connected */
		if( connection_id ) /* Check if we have an active connection */
		{
			/* Check to see if the client has asked for notifications */
			 if( app_car_distance_client_char_config[0] & GATT_CLIENT_CONFIG_NOTIFICATION )
			 {
				 wiced_bt_gatt_server_send_notification( connection_id,
						 HDLC_CAR_DISTANCE_VALUE,
						 app_car_distance_len,
						 app_car_distance,
						 NULL);
			 }
		 }

		/* Wait to start next measurement */
		vTaskDelay(pdMS_TO_TICKS(250));
	}
}

static void optical_left_isr(void *handler_arg, cyhal_gpio_event_t event)
{
  	leftDistance++;
	if(leftDistance >= HALF_TURN)
	{
		cyhal_gpio_enable_event(OPTICAL_LEFT_PIN, CYHAL_GPIO_IRQ_RISE, GPIO_INTERRUPT_PRIORITY, false);
		cyhal_pwm_set_duty_cycle(&motor_left_rev,  MOTOR_OFF, MOTOR_PWM_FREQUENCY);
	}
}

static void optical_right_isr(void *handler_arg, cyhal_gpio_event_t event)
{
	rightDistance++;
	if(rightDistance >= HALF_TURN)
	{
		cyhal_gpio_enable_event(OPTICAL_RIGHT_PIN, CYHAL_GPIO_IRQ_RISE, GPIO_INTERRUPT_PRIORITY, false);
		cyhal_pwm_set_duty_cycle(&motor_right_fwd, MOTOR_OFF, MOTOR_PWM_FREQUENCY);
	}
}

/* [] END OF FILE */
