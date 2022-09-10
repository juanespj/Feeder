
/*
 * feeder.c
 *
 *  Created on: 27 Feb 2021
 *      Author: jzapn
 */
#define INC_FEEDER_H_
#include "BLE_process.h"
#include "cycfg_ble.h"
#include "cy_pdl.h"
#include "cybsp.h"
#include "cyhal.h"
#include "system.h"
#include "cyhal_gpio.h"
#include "ui.h"
/** This structure is used to hold the machine state */
DEVICE_CFG_t dev;
time_t dispense_tmr = 0;

/****
 * machine task
 *
 *
 */
void sys_task(void) {
	/* Send command to process BLE events */
	//GetTime();

	dev.new_state = false;

	if (dev.prev_state != dev.state) {
	    dev.new_state = true;
	}
	dev.prev_state = dev.state;
	switch (dev.state) {

	case IDLE:
		if (dev.new_state) {
			Cy_GPIO_Write(LED0_PORT, LED0_NUM, CYBSP_LED_STATE_OFF);
		}
		if (dev.trigger == 2) {
		    dev.state = FEED;
		}
		if(BTN_getPressed(USRBTN)){
		   		Cy_GPIO_Write(LED0_PORT, LED0_NUM, CYBSP_LED_STATE_ON);
		   		uartprint("PSoC Feeder \r\n\n");
			dev.state = FEED;
		}
		uartprint("idle \r\n");
		break;
	case FEED:
		GetTime();
		if (dev.new_state) {
			dispense_tmr = dev.timestamp;
			//	Cy_TCPWM_Block_SetPeriod(motCount_HW, motCount_NUM, feeder.feedQty);
			//start counter
			Cy_TCPWM_TriggerStart_Single(motCount_HW, motCount_NUM);

			//	Cy_TCPWM_Block_SetCounter(motTrig_HW, motTrig_NUM, feeder.spd);
			//start Motor
			Cy_TCPWM_TriggerStart_Single(motTrig_HW, motTrig_NUM);
			//turns off when count is reached
			Cy_GPIO_Write(LED0_PORT, LED0_NUM, CYBSP_LED_STATE_ON); /*  start the PWM */
		}
		/* Get all the enabled pending interrupts */
		uint32_t interrupts = Cy_TCPWM_GetInterruptStatusMasked(motCount_HW,
		motCount_NUM);
		if (0UL != (CY_TCPWM_INT_ON_TC & interrupts)) {
			/* Handle the Terminal Count event */
		    dev.state = IDLE;
			Cy_GPIO_Write(LED0_PORT, LED0_NUM, CYBSP_LED_STATE_OFF);
			/* Clear the interrupt */
			Cy_TCPWM_ClearInterrupt(motCount_HW, motCount_NUM, interrupts);
		}

		if (dev.timestamp - dispense_tmr > 2) {	//seconds
			Cy_TCPWM_TriggerStopOrKill_Single(motTrig_HW, motTrig_NUM);
			dev.state = IDLE;
			Cy_GPIO_Write(LED0_PORT, LED0_NUM, CYBSP_LED_STATE_OFF);
			dev.trigger = 1;
		}
		break;
	case RTC_OofS:
		if (dev.new_state) {
			if (dev.timestamp == 0) {
			    dev.state = RTC_OofS;
			    dev.timestamp = 1614804922;
				UpdateTime();
				GetTime();
				dev.state = IDLE;
			}
		}
		break;
	case ERROR:
		if (dev.new_state) {

		}
		break;
	}

}

