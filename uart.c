#include "uart.h"
#include "Timer.h"



volatile char data_char;
volatile char stop = 0;

/**
 * Configures UArt 1.
 * UArt 1 is one of 7 UArt modules on the microcontroller.
 * Each has its own send and receive pins.
 *
 */
void uart_init(void) {
	// baud rate: 115200, 8 data bits, 1 stop bit, no parity
	static char initialized = 0;
	if (initialized) {
		return;
	}

	// 1: enable uart
	SYSCTL_RCGCUART_R |= 0x02; // enable bit 1


	// 2: GPIO
	SYSCTL_RCGCGPIO_R |= 0b10;
	timer_waitMillis(1);            // Small delay before accessing device after turning on clock
	GPIO_PORTB_AFSEL_R |= 0b11;
	GPIO_PORTB_PCTL_R &= 0x11;     // Force 0's in the desired locations
	GPIO_PORTB_PCTL_R |= 0x11;     // Force 1's in the desired locations
	GPIO_PORTB_DEN_R |= 0b11;
	GPIO_PORTB_DIR_R &= 0b10;      // Force 0's in the desired locations (pin 0 is U1Rx, input)
	GPIO_PORTB_DIR_R |= 0b10;

	// 3: disable uart with UARTCTL
	UART1_CTL_R &= 0xFFFFFFFE; // set bit 0 to 0

	// 4: UARTIBRD (BRD = Baud Rate Divisor)
	// int portion = 8
	UART1_IBRD_R = 8;

	// 5: UARTFBRD
	// fractional portion = 44
	UART1_FBRD_R = 44;

	// 6: UARTLCRH
	// bit 0 (normal use), 1 (parity off), 3 (1 stop bit), 4 (FIFO off) = 0, bits 6:5 (8 bits) = 1
	UART1_LCRH_R = 0b1100000;

	// 7: UARTCC: use the system clock = 0x0
	UART1_CC_R &= 0xFFFFFFFE;

	// 8: UARTDMACTL: Disable Direct Memory Access request error handling (bit 2) and for Tx(bit 1) and Rx(bit 0)
	// optional
	UART1_DMACTL_R = 0;

	// : Re-enable uart by setting bit 0 back to 1
	UART1_CTL_R |= 0x01;

	initialized = 1;
}

void uart_sendChar(char data) {
	// Make sure the data register is empty there is a flag on the uart flag register
	// 		also flags for other stuff

	// Condition to check if the transmit register is empty (bit 7 == 1)
	if (UART1_FR_R & 0x80) {
		// 1. write ASCII value to Uart data register
		UART1_DR_R = data;
		// Data is sent automatically

		// do we need to reset flags?
	}


	//
}

char uart_receive(void) {
	// ensure the receive register is full (bit 6 == 1)
	// If this register is full, we have data

	int start_time = timer_getMillis();
	int curr_time = start_time;
	int end_time = start_time + 500;
	while (curr_time < end_time) {
		if (UART1_FR_R & 0x40){
			return UART1_DR_R & 0xFF;
			// do we need to change any flags?
		}
		curr_time = timer_getMillis();
	}
//	while (timer_getMillis) {
//		if (UART1_FR_R & 0x40){
//			return UART1_DR_R & 0xFF;
//			// do we need to change any flags?
//		}
//		timer_waitMillis(50);
//		++cycles;
//	}
	return '\0';
}

void uart_sendString(char str[]) {
    int length = strlen(str);
    int i = 0;
    for (i = 0; i < length; i++){
    	uart_sendChar(str[i]);
    	timer_waitMillis(1);
    }
}

void uart_interrupt_init() {
	// disable UART
	UART1_CTL_R &= 0xFFFFFFFE; // set bit 0 to 0

	// Unmask bit 4 in the UART1_IM_R register
	// to enable the UART receive interrupt mask.
	// This interrupt is sent when the RXRIS bit int the
	// UART1_RIS_R register is set to 1.
	UART1_IM_R |= 0x10;

	// enable UART1
	UART1_CTL_R |= 1;

	// enable interrupts for UART1 by enabling bit 6 on the NVIC_EN0_R
	NVIC_EN0_R |= 0x40;

	// Bind the interrupt to the handler.
	IntRegister(INT_UART1, uart1_handler);
}

/**
 * Configured to handle when a character is received.
 */
void uart1_handler() {
	// clear interrupt status register by writing a 1 to the RXIC bit UART1_ICR_R register
	UART1_ICR_R |= 0x10;

	// upadate the character variable with the received character
	data_char = UART1_DR_R & 0xFF;
//	if (data_char == 'x' && stop == 1){
//		stop = 0;
//	} else if (data_char == 'x' && stop == 0) {
//		stop = 1;
//	}
	if (data_char == 'x') {
		stop = 1; // 1 signals a stop- cancel certain operations.
	}
}
