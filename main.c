/******************************************************************************
 * File Name: main.c
 *
 * Description: This is source code for the PSoC 6 MCU with BLE Find Me code
 *              example.
 *
 * Related Document: README.md
 *
 *******************************************************************************
 * Copyright 2019-2021, Cypress Semiconductor Corporation (an Infineon company) or
 * an affiliate of Cypress Semiconductor Corporation.  All rights reserved.
 *
 * This software, including source code, documentation and related
 * materials ("Software") is owned by Cypress Semiconductor Corporation
 * or one of its affiliates ("Cypress") and is protected by and subject to
 * worldwide patent protection (United States and foreign),
 * United States copyright laws and international treaty provisions.
 * Therefore, you may use this Software only as provided in the license
 * agreement accompanying the software package from which you
 * obtained this Software ("EULA").
 * If no EULA applies, Cypress hereby grants you a personal, non-exclusive,
 * non-transferable license to copy, modify, and compile the Software
 * source code solely for use in connection with Cypress's
 * integrated circuit products.  Any reproduction, modification, translation,
 * compilation, or representation of this Software except as specified
 * above is prohibited without the express written permission of Cypress.
 *
 * Disclaimer: THIS SOFTWARE IS PROVIDED AS-IS, WITH NO WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING, BUT NOT LIMITED TO, NONINFRINGEMENT, IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE. Cypress
 * reserves the right to make changes to the Software without notice. Cypress
 * does not assume any liability arising out of the application or use of the
 * Software or any product or circuit described in the Software. Cypress does
 * not authorize its products for use in any products where a malfunction or
 * failure of the Cypress product may reasonably be expected to result in
 * significant property damage, injury or death ("High Risk Product"). By
 * including Cypress's product in a High Risk Product, the manufacturer
 * of such system or application assumes all risk of such use and in doing
 * so agrees to indemnify Cypress against all liability.
 *******************************************************************************/

/******************************************************************************
 * Header files
 ******************************************************************************/
//#include "cy_pdl.h"
#include <system.h>
#include "cy_retarget_io.h"
#include "cybsp.h"
#include "cyhal.h"
#include "main.h"
#include "BLE_process.h"
#include "cycfg_ble.h"
#include "cyhal_gpio.h"
#include "ui.h"
#include "btns.h"
#define TIME_AT_RESET           (00u),   /* Seconds    */\
                                (00u),   /* Minutes    */\
                                (00u),   /* Hours      */\
                                (01u),   /* Date       */\
                                (01u),   /* Month      */\
                                (17u)    /* Year 20xx  */

void Init(void) {
	cy_rslt_t result;

	/* Initialize the device and board peripherals */
	result = cybsp_init();

	/* Board init failed. Stop program execution */
	if (result != CY_RSLT_SUCCESS) {
		CY_ASSERT(0);
	}

	/* Initialize the User LEDs */
	result |= Cy_GPIO_Pin_Init(LED0_PORT, LED0_PIN, &LED0_config);
	result |= Cy_GPIO_Pin_Init(LED1_PORT, LED1_PIN, &LED1_config);
	result |= Cy_GPIO_Pin_Init(LED2_PORT, LED2_PIN, &LED2_config);

	BTN_INIT();
	/* GPIO init failed. Stop program execution */
	if (result != CY_RSLT_SUCCESS) {
		CY_ASSERT(0);
	}

	if (CY_TCPWM_SUCCESS
			!= Cy_TCPWM_PWM_Init(motTrig_HW, motTrig_NUM, &motTrig_config)) {
		/* Handle possible errors */
	}
	/* Enable the initialized PWM */
	Cy_TCPWM_PWM_Enable(motTrig_HW, motTrig_NUM);

	if (CY_TCPWM_SUCCESS
			!= Cy_TCPWM_Counter_Init(motCount_HW, motCount_NUM,
					&motCount_config)) {
		/* Handle possible errors */
	}
	/* Enable the initialized PWM */
	Cy_TCPWM_Counter_Enable(motCount_HW, motCount_NUM);
	UI_init();
	///START BLE
	ble_feeder_init();
}


char read_data[10];
int main(void) {
	Init();

	for (;;) {
   	BTN_task();

		ble_task();
		sys_task();

	}

}

/* END OF FILE */

