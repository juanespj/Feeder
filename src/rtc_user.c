
/* Header files */
#include "cycfg.h"
#include "rtc_user.h"
#include "ui.h"
#include <stdio.h>

#define RTC_ACCESS_RETRY    (500u)
#define RTC_RETRY_DELAY     (5u)     /* 5 msec   */

/* Date and time on start */
#define RTC_INITIAL_TIME_SEC	(0UL)
#define RTC_INITIAL_TIME_MIN	(0UL)
#define RTC_INITIAL_TIME_HOUR	(0UL)
#define RTC_INITIAL_DATE_DOW	(CY_RTC_MONDAY)
#define RTC_INITIAL_DATE_DOM	(1UL)
#define RTC_INITIAL_DATE_MONTH 	(1UL)
#define RTC_INITIAL_DATE_YEAR	(18UL)
#define RTC_HOUR_FORMAT			(CY_RTC_24_HOURS)

// Variable used for storing days in the week
char_t day_str[CY_RTC_DAYS_PER_WEEK][4] = {
		"SUN", "MON", "TUE", "WED", "THU", "FRI", "SAT"};

cy_en_rtc_status_t Rtc_Init(void)
{
	/* Variable used to store return status of RTC API */
	cy_en_rtc_status_t rtcApiStatus;

	/* Initial date and time */
	cy_stc_rtc_config_t initialDateTime = {
		.sec = 	RTC_INITIAL_TIME_SEC,
		.min = 	RTC_INITIAL_TIME_MIN,
		.hour = RTC_INITIAL_TIME_HOUR,
		.hrFormat = RTC_HOUR_FORMAT,
		.date = RTC_INITIAL_DATE_DOM,
		.month = RTC_INITIAL_DATE_MONTH,
		.year = RTC_INITIAL_DATE_YEAR,
		.dayOfWeek = RTC_INITIAL_DATE_DOW
	};

	uint32_t rtcAccessRetry = RTC_ACCESS_RETRY;

	/* RTC block doesn't allow to access, when synchronizing the user registers
	 * and the internal actual RTC registers. It will return RTC_BUSY value, if
	 * it is not available to update the configuration values. Needs to retry,
	 * if it doesn't return CY_RTC_SUCCESS.
	 */
	do
	{
		rtcApiStatus = Cy_RTC_Init(&initialDateTime);
		rtcAccessRetry--;
		Cy_SysLib_Delay(RTC_RETRY_DELAY);
	}while((rtcApiStatus != CY_RTC_SUCCESS) && (rtcAccessRetry != 0));

	return rtcApiStatus;
}

/* [] END OF FILE */
