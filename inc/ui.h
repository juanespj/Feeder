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

#if !defined(UI_H)
#define UI_H
#include <cyhal.h>
#include "cybsp.h"
#include "cy_pdl.h"
#include "cycfg.h"
//HAL UART

#define BAUD_RATE       115200
#define UART_DELAY      10u
#define TX_BUF_SIZE     10
#define RX_BUF_SIZE	10
#define INT_PRIORITY    3
#define DATA_BITS_8     8
#define STOP_BITS_1     1

#define uartprint(x) (Cy_SCB_UART_PutString(UART_HW, x))
    
//https://www.csie.ntu.edu.tw/~r92094/c++/VT100.html
#define UARTCMD(value) uartprint(value)
#define UARTCURSORPOS(x, y)           \
  {                                   \
    sprintf(msg, "\33[%u;%uf", x, y); \
    uartprint(msg);          \
  }

#define CLEARLINE "\33[K"
#define CURSOR_HOME "\33[H"
#define CLEAR_SCN "\33[2J\33[H" //cursor to top left
#define UART_CLEARSCRN "\33[2J" //Clear entire screen
#define UART_CURSORHOME "\33[H" //cursor to top left

#define UART_CLEARLINE "\33[2K" ///from http://www.climagic.org/mirrors/VT100_Escape_Codes.html
#define UART_CLEAR_EOL "\33[K"
#define UART_MOVEUP "\033[A"
#define UART_DHTOP "\33#3"  //Double-height letters, top half
#define UART_DHBOT "\33#4"  //Double-height letters, bottom half
#define UART_SWSH "\33#5"   //Single width, single height letters
#define UART_DWSH "\33#6"   //Double width, single height letters
#define UART_FBOLD "\33[1m" //Font to bold
#define UART_FCLEAR "\33[m" //Font clear format

#define UART_FRED "\033[0;31m" /*/https://xdevs.com/guide/color_serial/ \
                               //https://misc.flogisoft.com/bash/tip_colors_and_formatting*/

void printheader(void);
void serialEvent(char inChar);
void printmsg(void);
void UI_init(void);
void uart_event_handler(void* handler_arg, cyhal_uart_event_t event);
#elif

extern void UI_init(void);
extern void printmsg(void);
extern void printheader(void);
extern void uart_event_handler(void* handler_arg, cyhal_uart_event_t event);
#endif
