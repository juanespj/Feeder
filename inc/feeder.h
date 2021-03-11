/* Index values in array where respective color coordinates
 * are saved */
#include "stdint.h"
#include "stdbool.h"
#include "time.h"

#define BTN_CHECK_MSEC	5	// Read hardware every 5 msec
#define BTN_PRESS_MSEC	10	// Stable time before registering pressed
#define BTN_HOLD_TIME	100	// Stable time before registering released
#define BTN_TICK_TIME 5
#define FEEDER_TICK 1//ms
typedef struct BTN_INSTANCE_t {
	bool isPressed;
	bool isHeld;
	bool isAcknowledged;
	uint32_t timer;
	bool sense;
} BTN_INSTANCE_t;

typedef enum BTN_NAMES_t {
	MAIN_BTN, USRBTN, BTN_MAX,
} BTN_NAMES_t;

typedef enum FEEDER_STATES_t {
	IDLE, RTC_OofS, FEED, ERROR,
} FEEDER_STATES_t;

typedef struct FeederState {
	FEEDER_STATES_t state;
	FEEDER_STATES_t prev_state;
	bool new_state;
	struct tm date_time;
	time_t timestamp;
	uint8_t feedQty;
	uint8_t trigger;
	uint8_t spd;
	BTN_INSTANCE_t bttns[BTN_MAX];
} FeederState;

#ifndef INC_FEEDER_H_

	bool BTN_getPressed(BTN_NAMES_t button);
	bool BTN_getHeld(BTN_NAMES_t button);
	void BTN_task(void );
	void feeder_task(void );

#else

	/** This structure is used to hold the machine state */
	extern void BTN_task(void );
	extern bool BTN_getPressed(BTN_NAMES_t button);
	extern bool BTN_getHeld(BTN_NAMES_t button);
	extern void feeder_task(void );
#endif

