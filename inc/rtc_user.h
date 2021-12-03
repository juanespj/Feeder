/* Include guard */
#ifndef SOURCE_RTC_USER_H_
#define SOURCE_RTC_USER_H_

extern char_t day_str[CY_RTC_DAYS_PER_WEEK][4];

cy_en_rtc_status_t Rtc_Init(void);

#endif /* SOURCE_RTC_USER_H_ */
