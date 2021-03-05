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
#include "cy_pdl.h"
#include "cy_retarget_io.h"
#include "cybsp.h"
#include "cyhal.h"
#include "main.h"
#include "feeder.h"
#include "BLE_process.h"
#include "cycfg_ble.h"

#define TIME_AT_RESET           (00u),   /* Seconds    */\
                                (00u),   /* Minutes    */\
                                (00u),   /* Hours      */\
                                (01u),   /* Date       */\
                                (01u),   /* Month      */\
                                (17u)    /* Year 20xx  */



void InitializeSystem(void) {
	cy_rslt_t result;

	/* Initialize the device and board peripherals */
	result = cybsp_init();

	/* Board init failed. Stop program execution */
	if (result != CY_RSLT_SUCCESS) {
		CY_ASSERT(0);
	}

	/* Initialize retarget-io to use the debug UART port */
	result = cy_retarget_io_init(CYBSP_DEBUG_UART_TX, CYBSP_DEBUG_UART_RX,
	CY_RETARGET_IO_BAUDRATE);
	/* retarget-io init failed. Stop program execution */
	if (result != CY_RSLT_SUCCESS) {
		CY_ASSERT(0);
	}

	/* Initialize the User LEDs */

	result |= Cy_GPIO_Pin_Init(LED0_PORT, LED0_PIN, &LED0_config);
	result |= Cy_GPIO_Pin_Init(LED1_PORT, LED1_PIN, &LED1_config);
	result |= Cy_GPIO_Pin_Init(LED2_PORT, LED2_PIN, &LED2_config);

	result |= Cy_GPIO_Pin_Init(BTN_PORT, BTN_PIN, &BTN_config);
	result |= Cy_GPIO_Pin_Init(BTN_PORT, BTN_PIN, &BTN_config);

	result |= Cy_GPIO_Pin_Init(USR_BTN_PORT, USR_BTN_PIN, &USR_BTN_config);
	result |= Cy_GPIO_Pin_Init(BTN_PORT, BTN_PIN, &BTN_config);

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

	///START BLE
	ble_feeder_init();
}

int main(void) {
	InitializeSystem();
	/* \x1b[2J\x1b[;H - ANSI ESC sequence for clear screen */
	printf("\x1b[2J\x1b[;H");
	printf("PSoC 6 Feeder\r\n\n");

	for (;;) {
		feeder_process();
		BTN_task();
		if (BTN_getPressed(MAIN_BTN)) {
			/* Insert logic for High pin state */
			Cy_GPIO_Write(LED0_PORT, LED0_NUM, CYBSP_LED_STATE_ON); /*  start the PWM */

		} else {
			/* Insert logic for Low pin state */
			//	Cy_TCPWM_TriggerStopOrKill_Single(motTrig_HW, motTrig_NUM);
			Cy_GPIO_Write(LED0_PORT, LED0_NUM, CYBSP_LED_STATE_OFF);
		}
	}
}

/* END OF FILE */

