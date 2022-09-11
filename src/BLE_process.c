/*
 * BLE_process.c
 *
 *  Created on: 27 Feb 2021
 *      Author: jzapn
 */
#define INC_BLE_PROCESS_H_

#include "BLE_process.h"
#include <system.h>
#include "cycfg_ble.h"
#include "cy_pdl.h"
//#include "cy_retarget_io.h"
#include "cybsp.h"
#include "cyhal.h"
#include "ui.h"
#include "stdio.h"
#include <string.h>
/*******************************************************************************
 * Macros
 ********************************************************************************/
#define BLESS_INTR_PRIORITY (1u)
#define STRING_BUFFER_SIZE (80)
#define NOTIFY_CCCD_SIZE (02u)

cyhal_rtc_t rtc_obj;

/** This structure is used to hold the machine state */
extern DEVICE_CFG_t dev;
/******************************************************************************
 * Global Variables
 *****************************************************************************/

/* Connection handle to identify the connected peer device */
static cy_stc_ble_conn_handle_t conn_handle;

/* Variable to store information about custom data to be sent as notification to GATT Client */
static cy_stc_ble_gatt_handle_value_pair_t custom_data;
/* Flags */
/* To indicate if GATT Client has subscribed for notifications or not */
static bool notify_flag;
/* To indicate if BLE stack is busy or free */
static bool stack_free = true;

/* Variable to store MTU size for active BLE connection */
static uint16_t att_mtu_size = CY_BLE_GATT_MTU;

/* Pointer to store a single attribute information present in the GATT DB of the server */
static cy_stc_ble_gatt_write_param_t *wrReqParam;
static cy_stc_ble_gatt_write_param_t tempwrReqParam;
static void update_gatt_db_notif(cy_stc_ble_gatt_write_param_t *write_param);
static void update_gatt_db_write(cy_stc_ble_gatt_write_param_t *write_param);
void ble_stack_event_handler(uint32_t event, void *eventParam);
static void ble_init(void);
static void bless_interrupt_handler(void);
static void ble_start_advertisement(void);
static void ble_ias_callback(uint32 event, void *eventParam);
/* Variable used to store the return values of BLE APIs */
cy_en_ble_api_result_t ble_api_result;
uint32_t ledtimer = 0;

void ble_feeder_init(void)
{
	/* Configure BLE */
	ble_init();
	/* Init RTC */
	cy_rslt_t rslt;
	rslt = cyhal_rtc_init(&rtc_obj);
	if (CY_RSLT_SUCCESS != rslt)
	{
		CY_ASSERT(0);
	}
	/* Configure deep sleep wakeup timer */
	// wakeup_timer_init();
	/* Enable global interrupts */
	__enable_irq();
}

/*******************************************************************************
 * Function Name: ble_init
 ********************************************************************************
 * Summary:
 *  This function initializes the BLE and registers IAS callback function.
 *
 *******************************************************************************/
static void ble_init(void)
{
	static const cy_stc_sysint_t bless_isr_config = {
		/* The BLESS interrupt */
		.intrSrc = bless_interrupt_IRQn,

		/* The interrupt priority number */
		.intrPriority = BLESS_INTR_PRIORITY};

	/* Hook interrupt service routines for BLESS */
	(void)Cy_SysInt_Init(&bless_isr_config, bless_interrupt_handler);

	/* Store the pointer to blessIsrCfg in the BLE configuration structure */
	cy_ble_config.hw->blessIsrConfig = &bless_isr_config;

	/* Registers the generic callback functions  */
	Cy_BLE_RegisterEventCallback(ble_stack_event_handler);

	/* Initializes the BLE host */
	Cy_BLE_Init(&cy_ble_config);

	/* Enables BLE */
	Cy_BLE_Enable();

	/* Register IAS event handler */
	Cy_BLE_IAS_RegisterAttrCallback(ble_ias_callback);

	/* Enables BLE Low-power mode (LPM)*/
	// Cy_BLE_EnableLowPowerMode();
}

/*******************************************************************************
 * Function Name: ble_findme_process
 ********************************************************************************
 * Summary:
 *  This function processes the BLE events and configures the device to enter
 *  low power mode as required.
 *
 *******************************************************************************/
void ble_task(void)
{

	/* Enter low power mode. The call to enter_low_power_mode also causes the
	 * device to enter hibernate mode if the BLE stack is shutdown.
	 */

	// enter_low_power_mode();
	/* Cy_BLE_ProcessEvents() allows the BLE stack to process pending events */
	Cy_BLE_ProcessEvents();
	if (ledtimer > 100000)
	{
		ledtimer = 0;
		/* Update CYBSP_USER_LED1 to indicate current BLE status */
		if (CY_BLE_ADV_STATE_ADVERTISING == Cy_BLE_GetAdvertisementState())
		{
			Cy_GPIO_Inv(LED1_PORT, LED1_NUM);
			ledtimer = 50000;
		}
		else if (CY_BLE_CONN_STATE_CONNECTED == Cy_BLE_GetConnectionState(conn_handle))
		{
			Cy_GPIO_Inv(LED1_PORT, LED1_NUM);
			//uartprint("connected");
			// Cy_GPIO_Write(LED1_PORT, LED1_NUM, CYBSP_LED_STATE_ON);
		}
		else
		{
			Cy_GPIO_Write(LED1_PORT, LED1_NUM, CYBSP_LED_STATE_OFF);
		}
	}
	ledtimer += 1;

	if (notify_flag)
	{
		if (stack_free)
		{
			/* Send notification */
			ble_api_result = Cy_BLE_GATTS_SendNotification(&conn_handle,
														   &custom_data);

			if (ble_api_result == CY_BLE_SUCCESS)
			{
			}
		}
	}
	if (dev.trigger == 1 && dev.state == IDLE)
	{
		dev.trigger = 0;

		tempwrReqParam.handleValPair.value.len = 1u;
		tempwrReqParam.handleValPair.value.actualLen = 0u;
		tempwrReqParam.handleValPair.value.val = &dev.trigger;
		tempwrReqParam.handleValPair.attrHandle =
			CY_BLE_TRIGGER_MANUALTRIG_CHAR_HANDLE;
		tempwrReqParam.connHandle = conn_handle;
		notify_flag = true;
		update_gatt_db_notif(&tempwrReqParam);
	}
}
/******************************************************************************
 * Function Name: bless_interrupt_handler
 *******************************************************************************
 * Summary:
 *  Wrapper function for handling interrupts from BLESS.
 *
 ******************************************************************************/
static void bless_interrupt_handler(void)
{
	Cy_BLE_BlessIsrHandler();
}

void ble_stack_event_handler(uint32 event, void *eventParam)
{

	uint32_t tempval = 0;
	/* 'RGBledData[]' is an array to store 4 bytes of RGB LED data*/

	switch (event)
	{
	case CY_BLE_EVT_STACK_ON:
	{
		uartprint("BLE Stack Event : CY_BLE_EVT_STACK_ON\n\r");
		/* Start Advertisement and enter discoverable mode */
		ble_api_result = Cy_BLE_GAPP_StartAdvertisement(CY_BLE_ADVERTISING_FAST,
														CY_BLE_PERIPHERAL_CONFIGURATION_0_INDEX);
		if (ble_api_result == CY_BLE_SUCCESS)
		{
			uartprint("BLE Advertisement started successfully\n\r");
		}
		else
		{
			// printf("BLE Advertisement API, errorcode = 0x%X", ble_api_result);
			uartprint("BLE Advertisement API, errorcode = \n\r");
		}
		break;
	}
	/* This event indicates BLE stack status. This event is used to handle
	 * data throttling which may occur due to continuous notification being
	 * sent */
	/* This event is received when there is a timeout */
	case CY_BLE_EVT_TIMEOUT:
	{
		/* Reason for Timeout */
		cy_en_ble_to_reason_code_t reason_code =
			((cy_stc_ble_timeout_param_t *)eventParam)->reasonCode;

		switch (reason_code)
		{
		case CY_BLE_GAP_ADV_TO:
		{
			uartprint("[INFO] : Advertisement timeout event \r\n");
			break;
		}
		case CY_BLE_GATT_RSP_TO:
		{
			uartprint("[INFO] : GATT response timeout\r\n");
			break;
		}
		default:
		{
			uartprint("[INFO] : BLE timeout event\r\n");
			break;
		}
		}
		break;
	}
		/* This event indicates completion of Set LE event mask */
	case CY_BLE_EVT_LE_SET_EVENT_MASK_COMPLETE:
	{
		uartprint("[INFO] : Set LE mask event mask command completed\r\n");
		break;
	}

	/* This event indicates set device address command completed */
	case CY_BLE_EVT_SET_DEVICE_ADDR_COMPLETE:
	{
		uartprint("[INFO] : Set device address command has completed \r\n");
		break;
	}

	/* This event indicates set Tx Power command completed */
	case CY_BLE_EVT_SET_TX_PWR_COMPLETE:
	{
		uartprint("[INFO] : Set Tx power command completed\r\n");
		break;
	}

	/* This event indicates BLE Stack Shutdown is completed */
	case CY_BLE_EVT_STACK_SHUTDOWN_COMPLETE:
	{
		uartprint("[INFO] : BLE shutdown complete\r\n");
		break;
	}
	case CY_BLE_EVT_STACK_BUSY_STATUS:
	{
		/* Variable to store status of the stack */
		cy_stc_ble_l2cap_state_info_t stack_status;
		stack_status = *(cy_stc_ble_l2cap_state_info_t *)eventParam;
		uartprint("[INFO] : BLE BUSY_STATUS\r\n");
		if (stack_status.flowState == CY_BLE_STACK_STATE_BUSY)
		{
			/* If stack is busy, stop notifications */
			stack_free = false;
		}
		else
		{
			/* If stack is free, start notifications */
			stack_free = true;
		}
		break;
	}
		/**********************************************************************
		 * GAP events
		 *********************************************************************/

		/* This event is generated at the GAP Peripheral end after connection
		 * is completed with peer Central device
		 */
	case CY_BLE_EVT_GAP_DEVICE_CONNECTED:
	{
		uartprint("[INFO] : GAP device connected \r\n");
		break;
	}
		/* This event is triggered instead of 'CY_BLE_EVT_GAP_DEVICE_CONNECTED',
		 * if Link Layer Privacy is enabled in component customizer
		 */
	case CY_BLE_EVT_GAP_ENHANCE_CONN_COMPLETE:
	{
		printf("[INFO] : GAP enhanced connection complete \r\n");
		break;
	}

	case CY_BLE_EVT_GAP_DEVICE_DISCONNECTED:
	{
		/* This event is generated at GAP disconnection. */
		/* Restart advertisement */
		notify_flag = false;
		stack_free = true;
		Cy_BLE_GAPP_StartAdvertisement(CY_BLE_ADVERTISING_FAST,
									   CY_BLE_PERIPHERAL_CONFIGURATION_0_INDEX);
		uartprint("GAP discconnected\r\n");
		ble_start_advertisement();
		break;
	}

		/* This event indicates that the GAP Peripheral device has
		 * started/stopped advertising
		 */
	case CY_BLE_EVT_GAPP_ADVERTISEMENT_START_STOP:
	{
		if (CY_BLE_ADV_STATE_ADVERTISING == Cy_BLE_GetAdvertisementState())
		{
			printf("[INFO] : BLE advertisement started\r\n");
		}
		else
		{
			printf("[INFO] : BLE advertisement stopped\r\n");

			Cy_BLE_Disable();
		}
		break;
	}

		/***********************************************************************
		 *                          GATT Events                                 *
		 ***********************************************************************/
		/* This event is generated at the GAP Peripheral end after connection
		 * is completed with peer Central device */
	case CY_BLE_EVT_GATT_CONNECT_IND:
	{
		conn_handle = *((cy_stc_ble_conn_handle_t *)eventParam);

		uartprint("BLE Stack Event : CY_BLE_EVT_GATT_CONNECT_IND\r\n");
		uartprint("GATT connected\r\n");
		break;
	}
		/* This event is generated at the GAP Peripheral end after disconnection */
	case CY_BLE_EVT_GATT_DISCONNECT_IND:
	{
		uartprint("[INFO] : GATT device disconnected\r\n");
		break;
	}
		/* This event indicates that the 'GATT MTU Exchange Request' is received */
	case CY_BLE_EVT_GATTS_XCNHG_MTU_REQ:
	{
		uartprint("[INFO] : GATT MTU Exchange Request received \r\n");
		break;
	}

		/* This event received when GATT read characteristic request received */
	case CY_BLE_EVT_GATTS_READ_CHAR_VAL_ACCESS_REQ:
	{
		uartprint("[INFO] : GATT read characteristic request received \r\n");
		break;
	}

		/* This event is generated at the GAP Peripheral end after connection
		 * is completed with peer Central device */
		//	case CY_BLE_EVT_GAP_DEVICE_CONNECTED:
		//		break;
		/* This event is generated when a 'write command' is received from GATT
		 * client device */

	case CY_BLE_EVT_GATTS_WRITE_CMD_REQ:
	{
		//wrReqParam = (cy_stc_ble_gatt_write_param_t *)eventParam;
		uartprint("[INFO] :'write command' is received from GATT \r\n");
		/* Write the value received from GATT Client device into GATT Server
		 * database */
		//update_gatt_db_write(wrReqParam);
		break;
	}
	case CY_BLE_EVT_GATTS_WRITE_REQ:
	{
		//	case CY_BLE_EVT_GATTS_EXEC_WRITE_REQ: //receive Write
		uartprint("BLE Stack Event : CY_BLE_EVT_GATTS_WRITE_REQ");
		wrReqParam = (cy_stc_ble_gatt_write_param_t *)eventParam;

		/* Send response to GATT Client device */
		ble_api_result = Cy_BLE_GATTS_WriteRsp(conn_handle);

		if (CY_BLE_TIME_TIMESTAMP_CHAR_HANDLE == wrReqParam->handleValPair.attrHandle)
		{
			/* Store RGB LED data in local array */
			// dev.timestamp = *(wrReqParam->handleValPair.value.val);
			memcpy(&tempval, wrReqParam->handleValPair.value.val,
				   (size_t)0x04u);
			dev.timestamp = tempval;
			UpdateTime();
		}
		if (CY_BLE_TRIGGER_FEEDQTY_CHAR_HANDLE == wrReqParam->handleValPair.attrHandle)
		{
			/* Store RGB LED data in local array */
			dev.feedQty = *(wrReqParam->handleValPair.value.val);
		}
		if (CY_BLE_TRIGGER_MANUALTRIG_CHAR_HANDLE == wrReqParam->handleValPair.attrHandle)
		{
			/* Store RGB LED data in local array */
			dev.trigger = *(wrReqParam->handleValPair.value.val);
		}
		if (CY_BLE_TRIGGER_TRIGSPD_CHAR_HANDLE == wrReqParam->handleValPair.attrHandle)
		{
			/* Store RGB LED data in local array */
			dev.spd = *(wrReqParam->handleValPair.value.val);
		}
		/* Send the response to the write request received. */

		/* Send response to GATT Client device */
		Cy_BLE_GATTS_WriteRsp(conn_handle);
		update_gatt_db_notif(wrReqParam);
		/* Update the attribute with new values received from the client
		 * device i.e., In this case, enable or disable notification  */
		// update_gatt_db_notif(wrReqParam);
		break;
	}
	case CY_BLE_TRIGGER_FEEDQTY_CLIENT_CHARACTERISTIC_CONFIGURATION_DESC_HANDLE:
	  uartprint("trig char");
	  break;
	case CY_BLE_TIME_TIMESTAMP_CHAR_HANDLE:
		  uartprint("tmstamp char");
		  break;

	default:
	{
		// printf("[INFO] : BLE Event 0x%lX\r\n", (unsigned long) event);
	  char msg[10];
	  sprintf(msg, "[INFO] : BLE Event other %lu", (unsigned long)event);
	 		 	      uartprint(msg);
		//uartprint("[INFO] : BLE Event other");
	}
	}
}
void UpdateTime(void)
{
	/* Local variables to store calculated color components */
	cy_rslt_t rslt;
	struct tm *date_time;
	date_time = localtime(&dev.timestamp);
	// date_time->tm_year -= 100;
	rslt = cyhal_rtc_write(&rtc_obj, date_time);
	if (CY_RSLT_SUCCESS != rslt)
	{
		CY_ASSERT(0);
	}
}

void GetTime(void)
{
	/* Local variables to store calculated color components */
	cy_rslt_t rslt;

	rslt = cyhal_rtc_read(&rtc_obj, &dev.date_time);
	if (CY_RSLT_SUCCESS != rslt)
	{
		CY_ASSERT(0);
	}
	dev.timestamp = mktime(&dev.date_time);
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
static void update_gatt_db_notif(cy_stc_ble_gatt_write_param_t *write_param)
{
	cy_en_ble_gatt_err_code_t gattErrorCode;

	//	printf("Info: Attribute handle: 0x%X", write_param->handleValPair.attrHandle);
	//	printf("Info: Attribute Value: 0x%X", write_param->handleValPair.value.val[0]);

	/* Write into the identified attribute in GATT database */
	gattErrorCode = Cy_BLE_GATTS_WriteAttributeValuePeer(&conn_handle,
														 &(write_param->handleValPair));
	if (gattErrorCode == CY_BLE_GATT_ERR_NONE)
	{
		/* If the attribute is CCCD for notification characteristic */
		if (write_param->handleValPair.attrHandle == CY_BLE_TRIGGER_MANUALTRIG_CLIENT_CHARACTERISTIC_CONFIGURATION_DESC_HANDLE)
		{
			/* If notifications are enabled */
			if (write_param->handleValPair.value.val[0])
			{
				/* Custom data value for BLE notification. */
				uint8_t custom_notif_data[CY_BLE_GATT_MTU] = {0};
				uartprint("Notifications Enabled\r\n");
				notify_flag = true;
				/* Set the custom notification data using the MTU size */
				/* Packet length = (ATT_MTU_SIZE - ATT_OPCODE(1 byte) - ATT_HEADER(2 bytes))
				 *               = (ATT_MTU_SIZE - 3)*/
				custom_data.value.len = (att_mtu_size - 3u);
				custom_data.value.val = custom_notif_data;
				custom_data.attrHandle =
					CY_BLE_TRIGGER_MANUALTRIG_CHAR_HANDLE;
			}

			/* If notifications are disabled */
			else
			{
				uartprint("Notifications Disabled\r\n");
				notify_flag = false;
				/* Switch OFF LED2 to show notifications stopped */

				// xQueueSend(led_cmdQ, &led_status, (TickType_t) 0);
			}
			/* Start 1 second timer to calculate throughput */
			//	xTimerStart(timer_handle, (TickType_t) 0);
		}
	}
	/* If the operation is not successful */
	else
	{
		printf("GATT Write API, errorcode ");
		//		uartprint("GATT Write API, errorcode = 0x%X", gattErrorCode);
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
static void update_gatt_db_write(cy_stc_ble_gatt_write_param_t *write_param)
{
	cy_en_ble_gatt_err_code_t gattErrorCode;

	/* Write the attribute value into GATT database of server */
	gattErrorCode = Cy_BLE_GATTS_WriteAttributeValuePeer(&conn_handle,
														 &(write_param->handleValPair));

	/* Check for successful write operation */
	if (gattErrorCode == CY_BLE_GATT_ERR_NONE)
	{
		/* If data is written into value attribute of GATT write characteristic */
		if (write_param->handleValPair.attrHandle == CY_BLE_TRIGGER_MANUALTRIG_CHAR_HANDLE)
		{
			/* Copy the data to a variable for further processing in application
			 * layer if required */
			memcpy(custom_data.value.val,
				   write_param->handleValPair.value.val,
				   write_param->handleValPair.value.len);
			/* Increment count of gatt write bytes received */
			//	gatt_write_rx_bytes += write_param->handleValPair.value.len;
		}
	}
	else
	{
		//		printf("GATT Write API, errorcode = 0x%X ", gattErrorCode);
		uartprint("GATT Write API, errorcode = ");
	}
}

/******************************************************************************
 * Function Name: ble_start_advertisement
 *******************************************************************************
 * Summary:
 *  This function starts the advertisement.
 *
 ******************************************************************************/
static void ble_start_advertisement(void)
{
	cy_en_ble_api_result_t ble_api_result;

	if ((CY_BLE_ADV_STATE_ADVERTISING != Cy_BLE_GetAdvertisementState()) &&
		(Cy_BLE_GetNumOfActiveConn() < CY_BLE_CONN_COUNT))
	{
		ble_api_result = Cy_BLE_GAPP_StartAdvertisement(
			CY_BLE_ADVERTISING_FAST,
			CY_BLE_PERIPHERAL_CONFIGURATION_0_INDEX);

		if (CY_BLE_SUCCESS != ble_api_result)
		{
			printf("[ERROR] : Failed to start advertisement \r\n");
		}
	}
}

/*******************************************************************************
* Function Name: ble_ias_callback
********************************************************************************
* Summary:
*  This is an event callback function to receive events from the BLE, which are
*  specific to Immediate Alert Service.
*
* Parameters:
*  uint32 event:      event from the BLE component
*  void* eventParams: parameters related to the event
*
*******************************************************************************/
void ble_ias_callback(uint32 event, void *eventParam)
{
    /* Alert Level Characteristic write event */
    if(event == CY_BLE_EVT_IASS_WRITE_CHAR_CMD)
    {
        /* Read the updated Alert Level value from the GATT database */
   //     Cy_BLE_IASS_GetCharacteristicValue(CY_BLE_IAS_ALERT_LEVEL,
    //                                       sizeof(alert_level), &alert_level);
    }

    /* Remove warning for unused parameter */
    (void)eventParam;
}
