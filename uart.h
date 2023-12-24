#include <stdint.h>
#include <stdbool.h>
#include "timer.h"
#include "lcd.h"
#include <inc/tm4c123gh6pm.h>
#include "driverlib/interrupt.h"


#ifndef UART_H
#define UART_H

// initialize uart
void uart_init(void);

// send a single character
void uart_sendChar(char data);

// receive a character, non-blocking
char uart_receive(void);

// send a string over UART
void uart_sendString(char str[]);

// initialize the proper interrupts
void uart_interrupt_init();

// Interrupt handler declaration (create the ISR)
void uart1_handler();


#endif
