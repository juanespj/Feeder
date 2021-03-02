/*
 * feeder.c
 *
 *  Created on: 27 Feb 2021
 *      Author: jzapn
 */
#define INC_FEEDER_H_
#include "cycfg_ble.h"
#include "cy_pdl.h"
#include "cybsp.h"
#include "cyhal.h"
#include "feeder.h"

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

	if (feeder.instance[button].isPressed) {
		//------------------------------------------------------
		// only return the press if not acknowledged
		//------------------------------------------------------
		if (!feeder.instance[button].isAcknowledged) {
			result = true;
			feeder.instance[button].isAcknowledged = true;
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
	return feeder.instance[button].isHeld;
}

void BTN_task(void) {
	uint32_t i = 0;

	//--------------------------------------------
	// get the values from the GPIO
	feeder.instance[MAIN_BTN].sense = !Cy_GPIO_Read(BTN_PORT, BTN_NUM);
	feeder.instance[USRBTN].sense = !Cy_GPIO_Read(USR_BTN_PORT, USR_BTN_NUM);

	//--------------------------------------------
	// software debounce for press / hold events.
	//--------------------------------------------
	for (i = 0; i < BTN_MAX; i++) {
		if (feeder.instance[i].sense == 1) {
			if (feeder.instance[i].timer < BTN_HOLD_TIME) {
				feeder.instance[i].timer += BTN_TICK_TIME;
			} else {
				feeder.instance[i].isHeld = true;
			}

			if (feeder.instance[i].timer > BTN_PRESS_MSEC) {
				if (!feeder.instance[i].isAcknowledged) {
					feeder.instance[i].isPressed = true;
				}
			}
		} else {
			feeder.instance[i].timer = 0;
			feeder.instance[i].isHeld = false;

			if (feeder.instance[i].isAcknowledged == true) {
				feeder.instance[i].isPressed = false;
				feeder.instance[i].isAcknowledged = false;
			}
		}
	}
}
