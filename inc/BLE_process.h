/*
 * BLE_process.h
 *
 *  Created on: 27 Feb 2021
 *      Author: jzapn
 */
	#include "stdint.h"

#ifndef INC_BLE_PROCESS_H_

	void ble_feeder_init(void);
	void UpdateTime(void);
	void GetTime(void);
	void ble_task(void);
#else
	extern void UpdateTime(void);
	extern void GetTime(void);
	extern void feeder_process(void) ;
	extern void ble_feeder_init(void);
	extern void ble_process(void);
	extern void ble_task(void);

#endif /* INC_BLE_PROCESS_H_ */
