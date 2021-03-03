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

/* Connection handle to identify the connected peer device */
static cy_stc_ble_conn_handle_t conn_handle;

/* Pointer to store a single attribute information present in the GATT DB of the server */
static cy_stc_ble_gatt_write_param_t *wrReqParam;
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
	dispense_tmr+=TICK;
}

/****
 * machine task
 *
 *
 */
void feeder_process(void) {
	ble_process();
	switch (feeder.state) {
	case IDLE:
		if (feeder.timestamp == 0) {
			feeder.state = RTC_OofS;
		}
		break;
	case FEED:
		if (feeder.new_state) {
			dispense_tmr=feeder.timestamp;
			//start counter
			Cy_TCPWM_TriggerStart_Single(motCount_HW, motCount_NUM);
			//start Motor
			Cy_TCPWM_TriggerStart_Single(motTrig_HW, motTrig_NUM);
			//turns off when count is reached
		}


		break;
	case RTC_OofS:

		break;
	case ERROR:
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
	// enter_low_power_mode();
	/* Cy_BLE_ProcessEvents() allows the BLE stack to process pending events */
	Cy_BLE_ProcessEvents();
	if (ledtimer > 100000) {
		/* Update CYBSP_USER_LED1 to indicate current BLE status */
		if (CY_BLE_ADV_STATE_ADVERTISING == Cy_BLE_GetAdvertisementState()) {
			Cy_GPIO_Inv(LED1_PORT, LED1_NUM);
		} else if (CY_BLE_CONN_STATE_CONNECTED
				== Cy_BLE_GetConnectionState(conn_handle)) {
			Cy_GPIO_Write(LED1_PORT, LED1_NUM, CYBSP_LED_STATE_ON);
		} else {
			Cy_GPIO_Write(LED1_PORT, LED1_NUM, CYBSP_LED_STATE_OFF);
		}
		ledtimer = 0;
	}
	ledtimer += 1;
//epoch 1614804922

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
		ble_api_result = Cy_BLE_GAPP_StartAdvertisement(CY_BLE_ADVERTISING_FAST,
				CY_BLE_PERIPHERAL_CONFIGURATION_0_INDEX);
		if (ble_api_result == CY_BLE_SUCCESS) {
			iprintf("BLE Advertisement started successfully");
		} else {
			printf("BLE Advertisement API, errorcode = 0x%X", ble_api_result);
		}
		break;

	case CY_BLE_EVT_GAP_DEVICE_DISCONNECTED:
		/* This event is generated at GAP disconnection. */
		/* Restart advertisement */
		Cy_BLE_GAPP_StartAdvertisement(CY_BLE_ADVERTISING_FAST,
				CY_BLE_PERIPHERAL_CONFIGURATION_0_INDEX);
		break;
		/* This event is generated at the GAP Peripheral end after connection
		 * is completed with peer Central device */
//	case CY_BLE_EVT_GAP_DEVICE_CONNECTED:
//		break;
	case CY_BLE_EVT_GATTS_WRITE_REQ:
//	case CY_BLE_EVT_GATTS_EXEC_WRITE_REQ: //receive Write
		iprintf("BLE Stack Event : CY_BLE_EVT_GATTS_WRITE_REQ");
		wrReqParam = (cy_stc_ble_gatt_write_param_t*) eventParam;

		/* Send response to GATT Client device */
		Cy_BLE_GATTS_WriteRsp(conn_handle);

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
		break;
	default:
		break;
	}
}

struct tm *date_time;

void UpdateTime(void) {
	/* Local variables to store calculated color components */
	cy_rslt_t rslt;

	date_time = localtime(&feeder.timestamp);
	date_time->tm_year -= 100;
	rslt = cyhal_rtc_write(&rtc_obj, date_time);
	if (CY_RSLT_SUCCESS == rslt) {

	}
	GetTime();
}

void GetTime(void) {
	/* Local variables to store calculated color components */
	cy_rslt_t rslt;

	rslt = cyhal_rtc_read(&rtc_obj, &feeder.date_time);
	if (CY_RSLT_SUCCESS == rslt) {

	}
	feeder.timestamp = mktime(date_time);
}
void UpdateRGBcharacteristic(uint64* timestampData, uint8 timestamplen,
		uint16 attrHandle) {

}
