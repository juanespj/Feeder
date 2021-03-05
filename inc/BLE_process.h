/*
 * BLE_process.h
 *
 *  Created on: 27 Feb 2021
 *      Author: jzapn
 */

#ifndef INC_BLE_PROCESS_H_

#include "stdint.h"
/* RGB LED Characteristic data length*/

#define RGB_CHAR_DATA_LEN 4

void ble_feeder_init(void);
void ble_process(void);
void feeder_process(void) ;

#else
extern void feeder_process(void) ;
extern void ble_feeder_init(void);
extern void ble_process(void);
#endif /* INC_BLE_PROCESS_H_ */
