/*
 * BLE_process.c
 *
 *  Created on: 27 Feb 2021
 *      Author: jzapn
 */
#define INC_BLE_PROCESS_H_

#include "../inc/BLE_process.h"
#include "cycfg_ble.h"
#include "cy_pdl.h"
#include "cy_retarget_io.h"
#include "cybsp.h"
#include "cyhal.h"
#include "../inc/feeder.h"

/*******************************************************************************
 * Macros
 ********************************************************************************/
#define BLESS_INTR_PRIORITY       (1u)
#define STRING_BUFFER_SIZE (80)
#define TICK 1//ms
cyhal_rtc_t rtc_obj;

/******************************************************************************
 * Global Variables
 *****************************************************************************/
/* Variables to hold GATT notification byte count or GATT write byte count */
static uint32_t gatt_write_rx_bytes;
static uint32_t notif_tx_bytes;

/* Connection handle to identify the connected peer device */
static cy_stc_ble_conn_handle_t conn_handle;
/* Variable to store connection parameters after GAP connection */
static cy_stc_ble_gap_connected_param_t conn_param;
/* Flags */
/* To indicate if GATT Client has subscribed for notifications or not */
static bool notify_flag;
/* To indicate if BLE stack is busy or free */
static bool stack_free = true;
/* Variable to store information about custom data to be sent as notification to GATT Client */
static cy_stc_ble_gatt_handle_value_pair_t custom_data;
/* Variable to store MTU size for active BLE connection */
static uint16_t att_mtu_size = CY_BLE_GATT_MTU;

/* Pointer to store a single attribute information present in the GATT DB of the server */
static cy_stc_ble_gatt_write_param_t *wrReqParam;
static void update_gatt_db_notif(cy_stc_ble_gatt_write_param_t *write_param);
static void update_gatt_db_write(cy_stc_ble_gatt_write_param_t *write_param);
void ble_stack_event_handler(uint32_t event, void * eventParam);
static void ble_init(void);
static void bless_interrupt_handler(void);
void UpdateTime(void);
void GetTime(void);

uint32_t ledtimer = 0;
time_t dispense_tmr = 0;
/** This structure is used to hold the machine state */
struct FeederState feeder;

void ble_feeder_init(void) {
	/* Configure BLE */
	ble_init();
	/* Init RTC */
	cy_rslt_t rslt;
	rslt = cyhal_rtc_init(&rtc_obj);
	if (CY_RSLT_SUCCESS != rslt) {
		CY_ASSERT(0);
	}
	/* Configure deep sleep wakeup timer */
	//wakeup_timer_init();
	/* Enable global interrupts */
	__enable_irq();
	dispense_tmr += TICK;
}

/****
 * machine task
 *
 *
 */
void feeder_process(void) {
	/* Send command to process BLE events */

	ble_process();
	switch (feeder.state) {
	case IDLE:
		if (feeder.new_state) {
			feeder.new_state = false;

		}
		if (feeder.timestamp == 0) {
			feeder.state = RTC_OofS;
		}
		break;
	case FEED:
		if (feeder.new_state) {
			dispense_tmr = feeder.timestamp;
			//start counter
			Cy_TCPWM_TriggerStart_Single(motCount_HW, motCount_NUM);
			//start Motor
			Cy_TCPWM_TriggerStart_Single(motTrig_HW, motTrig_NUM);
			//turns off when count is reached
			feeder.new_state = false;
		}

		break;
	case RTC_OofS:
		if (feeder.new_state) {
			feeder.new_state = false;
		}
		break;
	case ERROR:
		if (feeder.new_state) {
			feeder.new_state = false;
		}
		break;
	}

}

/*******************************************************************************
 * Function Name: ble_init
 ********************************************************************************
 * Summary:
 *  This function initializes the BLE and registers IAS callback function.
 *
 *******************************************************************************/
static void ble_init(void) {
	static const cy_stc_sysint_t bless_isr_config = {
	/* The BLESS interrupt */
	.intrSrc = bless_interrupt_IRQn,

	/* The interrupt priority number */
	.intrPriority = BLESS_INTR_PRIORITY };

	/* Hook interrupt service routines for BLESS */
	(void) Cy_SysInt_Init(&bless_isr_config, bless_interrupt_handler);

	/* Store the pointer to blessIsrCfg in the BLE configuration structure */
	cy_ble_config.hw->blessIsrConfig = &bless_isr_config;

	/* Registers the generic callback functions  */
	Cy_BLE_RegisterEventCallback(ble_stack_event_handler);

	/* Initializes the BLE host */
	Cy_BLE_Init(&cy_ble_config);

	/* Enables BLE */
	Cy_BLE_Enable();

	/* Enables BLE Low-power mode (LPM)*/
	Cy_BLE_EnableLowPowerMode();

}

/*******************************************************************************
 * Function Name: ble_findme_process
 ********************************************************************************
 * Summary:
 *  This function processes the BLE events and configures the device to enter
 *  low power mode as required.
 *
 *******************************************************************************/
void ble_process(void) {
	/* Enter low power mode. The call to enter_low_power_mode also causes the
	 * device to enter hibernate mode if the BLE stack is shutdown.
	 */
	/* Variable used to store the return values of BLE APIs */
	cy_en_ble_api_result_t ble_api_result;

	// enter_low_power_mode();
	/* Cy_BLE_ProcessEvents() allows the BLE stack to process pending events */
	Cy_BLE_ProcessEvents();
	if (ledtimer > 100000) {
		ledtimer = 0;
		/* Update CYBSP_USER_LED1 to indicate current BLE status */
		if (CY_BLE_ADV_STATE_ADVERTISING == Cy_BLE_GetAdvertisementState()) {
			Cy_GPIO_Inv(LED1_PORT, LED1_NUM);
			ledtimer = 50000;
		} else if (CY_BLE_CONN_STATE_CONNECTED
				== Cy_BLE_GetConnectionState(conn_handle)) {
			Cy_GPIO_Inv(LED1_PORT, LED1_NUM);
			//Cy_GPIO_Write(LED1_PORT, LED1_NUM, CYBSP_LED_STATE_ON);
		} else {
			Cy_GPIO_Write(LED1_PORT, LED1_NUM, CYBSP_LED_STATE_OFF);
		}

	}
	ledtimer += 1;

	/* notify_flag is used to indicate if notifications are enabled by
	 * GATT client */
	if (notify_flag) {
		if (stack_free) {
			/* Send notification */
			ble_api_result = Cy_BLE_GATTS_SendNotification(&conn_handle,
					&custom_data);

			if (ble_api_result == CY_BLE_SUCCESS) {
				/* Increment notification byte count only if notification
				 * was sent successfully */
				notif_tx_bytes += custom_data.value.len;

				/* Switch ON LED2 to show data TX */
				Cy_GPIO_Write(LED0_PORT, LED0_NUM, CYBSP_LED_STATE_ON);

			} else {
				/* Sending notifications failed. Switch OFF LED2 to show there is no data TX */
				Cy_GPIO_Write(LED0_PORT, LED0_NUM, CYBSP_LED_STATE_OFF);

			}
		} else {
			/* Stack is busy. Switch OFF LED2 to show data TX stopped */
			Cy_GPIO_Write(LED0_PORT, LED0_NUM, CYBSP_LED_STATE_OFF);

		}
	}
//epoch 1614804922
	if (feeder.trigger == 1) {
		UpdateTime();
		feeder.trigger = 0;
	} else if (feeder.trigger == 2) {
		GetTime();
		feeder.trigger = 0;
	}

//time_t test;
//	test = GetTime();
}
/******************************************************************************
 * Function Name: bless_interrupt_handler
 *******************************************************************************
 * Summary:
 *  Wrapper function for handling interrupts from BLESS.
 *
 ******************************************************************************/
static void bless_interrupt_handler(void) {
	Cy_BLE_BlessIsrHandler();
}

void ble_stack_event_handler(uint32 event, void * eventParam) {
	/* Variable used to store the return values of BLE APIs */
	cy_en_ble_api_result_t ble_api_result;
	uint32_t tempval = 0;
	/* 'RGBledData[]' is an array to store 4 bytes of RGB LED data*/

	switch (event) {
	case CY_BLE_EVT_STACK_ON:
		iprintf("BLE Stack Event : CY_BLE_EVT_STACK_ON");
		/* Start Advertisement and enter discoverable mode */
		ble_api_result = Cy_BLE_GAPP_StartAdvertisement(
		CY_BLE_ADVERTISING_FAST,
		CY_BLE_PERIPHERAL_CONFIGURATION_0_INDEX);
		if (ble_api_result == CY_BLE_SUCCESS) {
			iprintf("BLE Advertisement started successfully");
		} else {
			printf("BLE Advertisement API, errorcode = 0x%X", ble_api_result);
		}
		break;
		/* This event indicates BLE stack status. This event is used to handle
		 * data throttling which may occur due to continuous notification being
		 * sent */
	case CY_BLE_EVT_STACK_BUSY_STATUS: {
		/* Variable to store status of the stack */
		cy_stc_ble_l2cap_state_info_t stack_status;
		stack_status = *(cy_stc_ble_l2cap_state_info_t*) eventParam;

		if (stack_status.flowState == CY_BLE_STACK_STATE_BUSY) {
			/* If stack is busy, stop notifications */
			stack_free = false;
		} else {
			/* If stack is free, start notifications */
			stack_free = true;
		}
		break;
	}

		/***********************************************************************
		 *                          GATT Events                                 *
		 ***********************************************************************/
		/* This event is generated at the GAP Peripheral end after connection
		 * is completed with peer Central device */
	case CY_BLE_EVT_GATT_CONNECT_IND: {
		conn_handle = *((cy_stc_ble_conn_handle_t*) eventParam);

		printf("BLE Stack Event : CY_BLE_EVT_GATT_CONNECT_IND");
		printf("GATT connected\r\n");
		break;
	}


	case CY_BLE_EVT_GAP_DEVICE_DISCONNECTED:
		/* This event is generated at GAP disconnection. */
		/* Restart advertisement */
		notify_flag = false;
		stack_free = true;
		Cy_BLE_GAPP_StartAdvertisement(CY_BLE_ADVERTISING_FAST,
		CY_BLE_PERIPHERAL_CONFIGURATION_0_INDEX);
		break;
		/* This event is generated at the GAP Peripheral end after connection
		 * is completed with peer Central device */
//	case CY_BLE_EVT_GAP_DEVICE_CONNECTED:
//		break;
		/* This event is generated when a 'write command' is received from GATT
		 * client device */
	case CY_BLE_EVT_GATTS_WRITE_CMD_REQ: {
		wrReqParam = (cy_stc_ble_gatt_write_param_t*) eventParam;

		/* Write the value received from GATT Client device into GATT Server
		 * database */
		update_gatt_db_write(wrReqParam);
		break;
	}
	case CY_BLE_EVT_GATTS_WRITE_REQ:
//	case CY_BLE_EVT_GATTS_EXEC_WRITE_REQ: //receive Write
		iprintf("BLE Stack Event : CY_BLE_EVT_GATTS_WRITE_REQ");
		wrReqParam = (cy_stc_ble_gatt_write_param_t*) eventParam;

		/* Send response to GATT Client device */
		ble_api_result = Cy_BLE_GATTS_WriteRsp(conn_handle);

		if (CY_BLE_TIME_TIMESTAMP_CHAR_HANDLE
				== wrReqParam->handleValPair.attrHandle) {
			/* Store RGB LED data in local array */
			//feeder.timestamp = *(wrReqParam->handleValPair.value.val);
			memcpy(&tempval, wrReqParam->handleValPair.value.val,
					(size_t) 0x04u);
			feeder.timestamp = tempval;
			UpdateTime();
		}
		if (CY_BLE_TRIGGER_FEEDQTY_CHAR_HANDLE
				== wrReqParam->handleValPair.attrHandle) {
			/* Store RGB LED data in local array */
			feeder.feedQty = *(wrReqParam->handleValPair.value.val);

		}
		if (CY_BLE_TRIGGER_MANUALTRIG_CHAR_HANDLE
				== wrReqParam->handleValPair.attrHandle) {
			/* Store RGB LED data in local array */
			feeder.trigger = *(wrReqParam->handleValPair.value.val);

		}
		if (CY_BLE_TRIGGER_TRIGSPD_CHAR_HANDLE
				== wrReqParam->handleValPair.attrHandle) {
			/* Store RGB LED data in local array */
			feeder.spd = *(wrReqParam->handleValPair.value.val);

		}
		/* Send the response to the write request received. */

		/* Send response to GATT Client device */
		Cy_BLE_GATTS_WriteRsp(conn_handle);

		/* Update the attribute with new values received from the client
		 * device i.e., In this case, enable or disable notification  */
		update_gatt_db_notif(wrReqParam);
		break;
	default:
		break;
	}
}

void UpdateTime(void) {
	/* Local variables to store calculated color components */
	cy_rslt_t rslt;
	struct tm *date_time;
	date_time = localtime(&feeder.timestamp);
//date_time->tm_year -= 100;
	rslt = cyhal_rtc_write(&rtc_obj, date_time);
	if (CY_RSLT_SUCCESS != rslt) {
		CY_ASSERT(0);
	}
	//GetTime();
}

void GetTime(void) {
	/* Local variables to store calculated color components */
	cy_rslt_t rslt;

	rslt = cyhal_rtc_read(&rtc_obj, &feeder.date_time);
	if (CY_RSLT_SUCCESS != rslt) {
		CY_ASSERT(0);
	}
	feeder.timestamp = mktime(&feeder.date_time);
}

/**********************************************************************************
 * Function Name: void update_gatt_db_notif(cy_stc_ble_gatt_write_param_t *write_param)
 ***********************************************************************************
 * Summary: This function updates the CCCD attribute of GATT Notify
 *          characteristic present in the custom service
 *
 * Parameters:
 *  cy_stc_ble_gatt_write_param_t *write_param: parameter which holds information
 *                                    about attribute handle and attribute value
 *
 * Return:
 *  None
 **********************************************************************************/
static void update_gatt_db_notif(cy_stc_ble_gatt_write_param_t *write_param) {
	cy_en_ble_gatt_err_code_t gattErrorCode;

	printf("Info: Attribute handle: 0x%X",
			write_param->handleValPair.attrHandle);
	printf("Info: Attribute Value: 0x%X",
			write_param->handleValPair.value.val[0]);

	notif_tx_bytes = 0u;
	gatt_write_rx_bytes = 0u;

	/* Write into the identified attribute in GATT database */
	gattErrorCode = Cy_BLE_GATTS_WriteAttributeValuePeer(&conn_handle,
			&(write_param->handleValPair));
	if (gattErrorCode == CY_BLE_GATT_ERR_NONE) {
		/* If the attribute is CCCD for notification characteristic */
		if (write_param->handleValPair.attrHandle
				== CY_BLE_TRIGGER_MANUALTRIG_CHAR_HANDLE) {
			/* If notifications are enabled */
			if (write_param->handleValPair.value.val[0]) {
				printf("Notifications Enabled\r\n");
				notify_flag = true;
				/* Set the custom notification data using the MTU size */
				/* Packet length = (ATT_MTU_SIZE - ATT_OPCODE(1 byte) - ATT_HEADER(2 bytes))
				 *               = (ATT_MTU_SIZE - 3)*/
				custom_data.value.len = (att_mtu_size - 3u);
				custom_data.value.val = &feeder.trigger;
				custom_data.attrHandle = CY_BLE_TRIGGER_MANUALTRIG_CHAR_HANDLE;
			}
			/* If notifications are disabled */
			else {
				printf("Notifications Disabled\r\n");
				notify_flag = false;
				/* Switch OFF LED2 to show notifications stopped */
				Cy_GPIO_Write(LED0_PORT, LED0_NUM, CYBSP_LED_STATE_OFF);

			}

		}
	}
	/* If the operation is not successful */
	else {
		printf("GATT Write API, errorcode = 0x%X", gattErrorCode);
	}
}

/**********************************************************************************
 * Function Name: void update_gatt_db_write(cy_stc_ble_gatt_write_param_t *write_param)
 ***********************************************************************************
 * Summary: This function updates the value attribute of GATT Write
 *          characteristic present in the custom service
 *
 * Parameters:
 *  cy_stc_ble_gatt_write_param_t *write_param: Parameter which holds information
 *                              about attribute handle and value.
 * Return:
 *  None
 **********************************************************************************/
static void update_gatt_db_write(cy_stc_ble_gatt_write_param_t *write_param) {
	cy_en_ble_gatt_err_code_t gattErrorCode;

	/* Write the attribute value into GATT database of server */
	gattErrorCode = Cy_BLE_GATTS_WriteAttributeValuePeer(&conn_handle,
			&(write_param->handleValPair));

	/* Check for successful write operation */
	if (gattErrorCode == CY_BLE_GATT_ERR_NONE) {
		/* If data is written into value attribute of GATT write characteristic */
		if (write_param->handleValPair.attrHandle
				== CY_BLE_TRIGGER_MANUALTRIG_CHAR_HANDLE) {
			/* Copy the data to a variable for further processing in application
			 * layer if required */
			memcpy(custom_data.value.val, write_param->handleValPair.value.val,
					write_param->handleValPair.value.len);
			/* Increment count of gatt write bytes received */
			gatt_write_rx_bytes += write_param->handleValPair.value.len;
		}
	} else {
		printf("GATT Write API, errorcode = 0x%X ", gattErrorCode);
	}
}
