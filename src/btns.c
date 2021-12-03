/* ========================================
 *
 * Copyright YOUR COMPANY, THE YEAR
 * All Rights Reserved
 * UNPUBLISHED, LICENSED SOFTWARE.
 *
 * CONFIDENTIAL AND PROPRIETARY INFORMATION
 * WHICH IS THE PROPERTY OF your company.
 *
 * ========================================
*/
#include "btns.h"
#include "cy_pdl.h"
#include "cycfg.h"

BTN_INSTANCE_t bttns[BTN_MAX];
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
void BTN_INIT(void){

	Cy_GPIO_Pin_Init(USR_BTN_PORT, USR_BTN_NUM,&USR_BTN_config);
	Cy_GPIO_Pin_Init(BTN_PORT, BTN_NUM,&BTN_config);
}
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
uint8_t BTN_getPressed(BTN_NAMES_t button) {
	bool result = false;
	if (bttns[button].isPressed) {
		//------------------------------------------------------
		// only return the press if not acknowledged
		//------------------------------------------------------
		if (!bttns[button].isAcknowledged) {
			result = true;
			bttns[button].isAcknowledged = true;
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
uint8_t BTN_getHeld(BTN_NAMES_t button) {
	return bttns[button].isHeld;
}

void BTN_task(void) {

	uint8_t i = 0;
	//--------------------------------------------
	// get the values from the GPIO
	bttns[MAIN_BTN].sense = !Cy_GPIO_Read(BTN_PORT, BTN_NUM);
	bttns[USRBTN].sense = !Cy_GPIO_Read(USR_BTN_PORT, USR_BTN_NUM);

	//--------------------------------------------
	// software debounce for press / hold events.
	//--------------------------------------------
	for (i = 0; i < BTN_MAX; i++) {
		if (bttns[i].sense == 1) {
			if (bttns[i].timer < BTN_HOLD_TIME) {
				bttns[i].timer += BTN_TICK_TIME;
			} else {
				bttns[i].isHeld = true;
			}

			if (bttns[i].timer > BTN_PRESS_MSEC) {
				if (!bttns[i].isAcknowledged) {
					bttns[i].isPressed = true;
				}
			}
		} else {
			bttns[i].timer = 0;
			bttns[i].isHeld = false;

			if (bttns[i].isAcknowledged == true) {
				bttns[i].isPressed = false;
				bttns[i].isAcknowledged = false;
			}
		}

	}
}


/* [] END OF FILE */

