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
#include "ui.h"
#include "stdio.h"
#include <string.h>
#include <stdlib.h>
#include <ctype.h>
#include <cyhal.h>
#include "cybsp.h"
#include "system.h"
#include "cy_pdl.h"
#include "cycfg.h"

extern DEVICE_CFG_t dev;

void Isr_UART(void);
uint8_t uart_error; /* UART error flag */

void UI_init(void)
{
	cy_stc_scb_uart_context_t UART_context;
	cy_en_scb_uart_status_t init_status;
	cy_stc_sysint_t UART_INT_cfg = { .intrSrc = UART_IRQ, .intrPriority = 7u, };
	Cy_SysInt_Init(&UART_INT_cfg, &Isr_UART);
	NVIC_EnableIRQ(UART_INT_cfg.intrSrc);

	/* Start UART operation */
	init_status = Cy_SCB_UART_Init(UART_HW, &UART_config, &UART_context);
	if (init_status != CY_SCB_UART_SUCCESS) {
		//     handle_error();
	}
	Cy_SCB_UART_Enable(UART_HW);

	/* Transmit header to the terminal */
	/* \x1b[2J\x1b[;H - ANSI ESC sequence for clear screen */
	uartprint(CLEAR_SCN);
	uartprint(UART_CURSORHOME);
	uartprint("PSoC Feeder \r\n\n");
	__enable_irq();
}

/** 
 * Serial CMD process.
 * maps the input character to the corresponding command
 * or input
 * change device State
 * @param[in] char inChar 
 */
void serialEvent(char inChar)
{
	static uint8_t inpos = 0;
	char msg[15];
	static char input_arr[15] = "\0";
	inpos += 1;
	// Serial.print(inChar); //SERIAL Repeat
	switch (inChar)
	{
	case 'p':
	{                    // toggle stream on
		 uint32_t tempinput = (uint32_t)atoi(input_arr);
		 sprintf(msg, "P %lu", tempinput);
		 	      uartprint(msg);
		memset(input_arr, 0, sizeof input_arr);
		dev.msgidx = 0;
		inpos = 0;
		/* code */
		break;
	}
	case 'h':  {           // help
		UARTCMD(CLEAR_SCN); // cursor to home command
		UARTCMD(CURSOR_HOME);
		// UI guide
		uartprint("*********** COMMANDS GUIDE ***********\n\r");
		uartprint(" b#\t'b1' xxx\n\r");

		uartprint("**************************************\n\r");
		break;
	}
	default:{
	    if (isdigit(inChar) || (inChar == '.'))
	    {
	      input_arr[dev.msgidx] = inChar;
	      dev.msgidx++;
	    }
	    else if (inChar != '\0') // unsupported charachters discard cmd
	    {
	      sprintf(msg, "%c? wtf u doin", inChar);
	      uartprint(msg);
	      dev.msgidx = 0;
	      memset(input_arr, '\0', sizeof input_arr);
	    }
	    break;
	}
	if (inChar == '\r' || inChar == ';') // END of message process data
	  {
	    	    // update new setting
	    switch (dev.INvar)
	    {
	    case 'g':
	    {
	      uint32_t tempinput = (uint32_t)atoi(input_arr);

	      break;
	    }
	    }
	  }
}}

void printheader(void)
{
	/*
	 char msg[15];
	 UARTCMD(CLEAR_SCN); // cursor to home command
	 UARTCMD(CURSOR_HOME);
	 //UI guide
	 uartprint("\e[32m*********** COMMANDS GUIDE ***********\n\r");
	 // uartprint(" e###\tEn/Dis B opt PWM 0-256\n\r");
	 uartprint(" w\tEn/Dis PID control (follow with r)\n\r");
	 uartprint(" d \t Enable Coil B\n\r");
	 uartprint(" k\tSTOP/Kill \n\r");
	 uartprint(" o\tReset Terminal\n\r");
	 uartprint(" p\tPrint Values\n\r");
	 uartprint(" r\tStart Ramp-Up/Control Sequence\n\r");

	 uartprint(" s\tToggle Static or Stream Terminal\n\r");
	 uartprint(" v\tShow version info\n\r");
	 uartprint("***************************************\e[37m\n\r");

	 uartprint("\e[44mI[mA]\tVres\tON_time\tsys_time\ttemp\tSP\n\r");
	 UARTCMD(CURSOR_HOME);
	 UARTCURSORPOS(18u, 0u);
	 uartprint("\e[49mINPUT:");
	 */
}

void printmsg(void)
{
	/* char msg[20];
	 if (rig.printEN)
	 {
	 sprintf(msg, "%u,%u", curr, vres);
	 uartprint(msg);
	 }*/

}
#define BUFFER_SIZE 10

void Isr_UART(void)
{
	uint32 intrSrcRx;
	uint32 intrSrcTx;

	/* Get RX interrupt sources */
	intrSrcRx = Cy_SCB_UART_GetRxFifoStatus(UART_HW);
	Cy_SCB_UART_ClearRxFifoStatus(UART_HW, intrSrcRx);
	serialEvent((char) Cy_SCB_UART_Get(UART_HW));
	/* Get TX interrupt sources */
	intrSrcTx = Cy_SCB_UART_GetTxFifoStatus(UART_HW);
	Cy_SCB_UART_ClearTxFifoStatus(UART_HW, intrSrcTx);

	/* RX overflow or RX underflow or RX overflow occured */
	uart_error = 1;
}

/* [] END OF FILE */
