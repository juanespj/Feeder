/* Index values in array where respective color coordinates
 * are saved */
#include "stdint.h"
#include "stdbool.h"
#include "time.h"
#include "btns.h"

#define SYS_TICK 1//ms

typedef enum DEVICE_STATE_t
{
  IDLE, RTC_OofS, FEED, ERROR,
} DEVICE_STATE_t;

typedef struct DEVICE_CFG_t
{
  DEVICE_STATE_t state;
  DEVICE_STATE_t prev_state;
  uint8_t new_state:1;
  struct tm date_time;
  time_t timestamp;
  uint8_t feedQty;
  uint8_t trigger;
  uint8_t spd;
  uint8_t msgidx;
  uint8_t INvar;
} DEVICE_CFG_t;


#ifndef INC_FEEDER_H_
void
sys_task (void);

#else
	/** This structure is used to hold the machine state */
	extern void sys_task(void );
#endif

