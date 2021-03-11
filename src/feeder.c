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
#include "feeder.h"
#include "cyhal_gpio.h"
/** This structure is used to hold the machine state */
FeederState feeder;
time_t dispense_tmr = 0;
/**
 * @fn bool BTN_getPressed ( BTN_NAMES_t button )
 * @param button the button name to read
 * @return true if an unacknowledged press has occurred
 *
 * This function will check the acknowledged flag for a given button
 * and return the appropriate value. If a press has been acknowledged
 * (by calling this function) then then the current press is not
 * returned.
 *
 * Presses are persistent until read!
 */
bool BTN_getPressed(BTN_NAMES_t button) {
	bool result = false;
	if (feeder.bttns[button].isPressed) {
		//------------------------------------------------------
		// only return the press if not acknowledged
		//------------------------------------------------------
		if (!feeder.bttns[button].isAcknowledged) {
			result = true;
			feeder.bttns[button].isAcknowledged = true;
		}
	}

	return result;
}

/**
 * @fn bool BTN_getHeld ( BTN_NAMES_t button )
 * @param button the name of the button to read
 * @return true if the button is held
 *
 * This function will return the value of the held flag. Releasing the
 * button clears the flag. There is no time base on held events.
 */
bool BTN_getHeld(BTN_NAMES_t button) {
	return feeder.bttns[button].isHeld;
}

void BTN_task(void) {

	uint8_t i = 0;
	//--------------------------------------------
	// get the values from the GPIO
	feeder.bttns[MAIN_BTN].sense = cyhal_gpio_read(BTN);
	feeder.bttns[USRBTN].sense = !Cy_GPIO_Read(USR_BTN_PORT, USR_BTN_NUM);

	//--------------------------------------------
	// software debounce for press / hold events.
	//--------------------------------------------
	for (i = 0; i < BTN_MAX; i++) {
		if (feeder.bttns[i].sense == 1) {
			if (feeder.bttns[i].timer < BTN_HOLD_TIME) {
				feeder.bttns[i].timer += BTN_TICK_TIME;
			} else {
				feeder.bttns[i].isHeld = true;
			}

			if (feeder.bttns[i].timer > BTN_PRESS_MSEC) {
				if (!feeder.bttns[i].isAcknowledged) {
					feeder.bttns[i].isPressed = true;
				}
			}
		} else {
			feeder.bttns[i].timer = 0;
			feeder.bttns[i].isHeld = false;

			if (feeder.bttns[i].isAcknowledged == true) {
				feeder.bttns[i].isPressed = false;
				feeder.bttns[i].isAcknowledged = false;
			}
		}

	}
}

/****
 * machine task
 *
 *
 */
void feeder_task(void) {
	/* Send command to process BLE events */
	//GetTime();
	if (BTN_getPressed(MAIN_BTN)) {
		Cy_GPIO_Write(LED0_PORT, LED0_NUM, CYBSP_LED_STATE_ON);
		//	feeder.state = FEED;
	} else {
		Cy_GPIO_Write(LED0_PORT, LED0_NUM, CYBSP_LED_STATE_OFF);
	}

	feeder.new_state = false;

	if (feeder.prev_state != feeder.state) {
		feeder.new_state = true;
	}
	feeder.prev_state = feeder.state;
	switch (feeder.state) {

	case IDLE:
		if (feeder.new_state) {

		}
		if (feeder.trigger == 2) {
			feeder.state = FEED;

		}
		break;
	case FEED:
		GetTime();
		if (feeder.new_state) {
			dispense_tmr = feeder.timestamp;
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
			feeder.state = IDLE;
			Cy_GPIO_Write(LED0_PORT, LED0_NUM, CYBSP_LED_STATE_OFF);
			/* Clear the interrupt */
			Cy_TCPWM_ClearInterrupt(motCount_HW, motCount_NUM, interrupts);
		}

		if (feeder.timestamp - dispense_tmr > 2) {	//seconds
			Cy_TCPWM_TriggerStopOrKill_Single(motTrig_HW, motTrig_NUM);
			feeder.state = IDLE;
			Cy_GPIO_Write(LED0_PORT, LED0_NUM, CYBSP_LED_STATE_OFF);
			feeder.trigger = 1;
		}
		break;
	case RTC_OofS:
		if (feeder.new_state) {
			if (feeder.timestamp == 0) {
				feeder.state = RTC_OofS;
				feeder.timestamp = 1614804922;
				UpdateTime();
				GetTime();
				feeder.state = IDLE;
			}
		}
		break;
	case ERROR:
		if (feeder.new_state) {

		}
		break;
	}

}
