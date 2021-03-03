
/* Index values in array where respective color coordinates
 * are saved */
#define BTN_CHECK_MSEC	5	// Read hardware every 5 msec
#define BTN_PRESS_MSEC	10	// Stable time before registering pressed
#define BTN_HOLD_TIME	100	// Stable time before registering released
#define BTN_TICK_TIME 5
typedef struct BTN_INSTANCE_t{
	bool		isPressed;
	bool		isHeld;
	bool		isAcknowledged;
	uint32_t	timer;
	bool		sense;
}BTN_INSTANCE_t;

typedef enum BTN_NAMES_t{
	MAIN_BTN,
	USRBTN,
	BTN_MAX,
}BTN_NAMES_t;

typedef enum FEEDER_STATES_t{
	IDLE,
	RTC_OofS,
	FEED,
	ERROR,
}FEEDER_STATES_t;

struct FeederState
{
	FEEDER_STATES_t state;
	bool new_state;
	struct tm date_time;
	time_t timestamp;
	uint8_t feedQty;
	uint16_t trigger;
	uint8_t spd;
	BTN_INSTANCE_t instance[BTN_MAX];
} ;



extern struct FeederState feeder;
#ifndef INC_FEEDER_H_

void BTN_task(void);
bool BTN_getPressed(BTN_NAMES_t button) ;
bool BTN_getHeld(BTN_NAMES_t button);

void BTN_task(void);



#else
/** This structure is used to hold the machine state */
extern void BTN_task(void);
extern bool BTN_getPressed(BTN_NAMES_t button) ;
extern bool BTN_getHeld(BTN_NAMES_t button);

#endif

