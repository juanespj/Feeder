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

#ifndef BTNS_H_
#define BTNS_H_

#include "stdint.h"
#include "time.h"


#define BTN_CHECK_MSEC	5	// Read hardware every 5 msec
#define BTN_PRESS_MSEC	10	// Stable time before registering pressed
#define BTN_HOLD_TIME	100	// Stable time before registering released
#define BTN_TICK_TIME 5

typedef enum BTN_NAMES_t {
	MAIN_BTN, USRBTN, BTN_MAX,
} BTN_NAMES_t;

typedef struct BTN_INSTANCE_t
{
	uint8_t isPressed : 1;
	uint8_t isHeld : 1;
	uint8_t isAcknowledged : 1;
	uint32_t timer;
	uint8_t sense : 1;
} BTN_INSTANCE_t;

uint8_t BTN_getPressed(BTN_NAMES_t button);
uint8_t BTN_getHeld(BTN_NAMES_t button);
void BTN_task(void);
void feeder_task(void);
void BTN_INIT(void);

#else

/** This structure is used to hold the machine state */
extern void BTN_task(void);
extern uint8_t BTN_getPressed(BTN_NAMES_t button);
extern uint8_t BTN_getHeld(BTN_NAMES_t button) ;
extern void BTN_INIT(void);
#endif

/* [] END OF FILE */
