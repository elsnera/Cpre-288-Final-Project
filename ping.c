#include "ping.h"
#include "driverlib/interrupt.h"

enum status{LOW, HIGH, DONE};
volatile enum status current_status = LOW;
volatile int rising_edge_time;
volatile int falling_edge_time;
int num_overflows;

/**
 * Initialize input-edge timer mode on the GPTM module and communicate
 * with the PING sensor via Port B pin 3. The LED will flash when it's on.
 * ALSO INITIALIZES INTERRUPTS for capturing duration of a ping scan.
 *
 */
void ping_initialize(void) {

	SYSCTL_RCGCTIMER_R |= 0x8; // enable clock functions to timer3.

	// configure GPIO pin ***reconfigure for part 2***
	SYSCTL_RCGCGPIO_R |= 0x2; // Port B enable
	GPIO_PORTB_DIR_R |= 0x8; // Pin 3 as output (1)
	GPIO_PORTB_DEN_R |= 0x8; // Enable digital functions (1)
	GPIO_PORTB_AFSEL_R |= 0x8; // write 1 to enable alternate function
	GPIO_PORTB_PCTL_R &= ~0xF000; // set bits 11:8 to 0x7
	GPIO_PORTB_PCTL_R |= 0x7000;

	// configure Timer 3B, port B pin 3
	TIMER3_CTL_R &= ~0x100; // disable timer
	// Input edge-time mode on, 16 bit mode
//	TIMER3_CFG_R &= 0x0;
	TIMER3_CFG_R |= 0x4;
	// Configure mode register
	TIMER3_TBMR_R &= ~0x10; // count-down mode (bit 4 = 0), capture mode (bit 3 == 0), edge time (bit 2 == 1), capture mode (1:0 == 0x3)
	TIMER3_TBMR_R |= 0x7;
	// Capture both edges (set bits 11:10 to 0x3)
	TIMER3_CTL_R &= ~0xF00;
	TIMER3_CTL_R |= 0xC00;
	// PR register to 0xFF (as specified)
	TIMER3_TBPR_R = 0xFF;
	// Initialize ILR to 0xFFFF (as specified)
	TIMER3_TBILR_R = 0xFFFF;

	// mask interrupts
	TIMER3_IMR_R |= 0x400;

	// Enable timer: bit 8 = 1;
	TIMER3_CTL_R |= 0x100;

	// NVIC
	NVIC_EN1_R |= 0x10; // bit 4 = 1 to enable interrupts for T3B, #36


	// BIND INTERRUPT TO HANDLER
	IntRegister(INT_TIMER3B, ping_handler);
	IntMasterEnable();
}

/**
 *
 */
int ping_read(void) {
	// ***Setting a pin to low or high is the same as writing a 0 or 1 to it's
	// corresponding bit/location in the PORT's data register.***

	TIMER3_CTL_R &= ~0x100;
	TIMER3_IMR_R &= ~0x400; // Mask interrupt by writing 0 to its bit.

	// Send trigger signal as GPIO output
	GPIO_PORTB_AFSEL_R &= ~0x08; // disable alternate functions
	GPIO_PORTB_DIR_R |= 0x08; // Pin 3 as output (1)
	GPIO_PORTB_DATA_R &= ~0X08; // set signal to 0
	GPIO_PORTB_DATA_R |= 0X08; // set signal to 1

	// wait 5 microseconds
	timer_waitMicros(5);

	// convert to timer input capture
	GPIO_PORTB_DATA_R &= ~0X08; // set signal to 0
	GPIO_PORTB_AFSEL_R |= 0x08; // enable alternate functions
	GPIO_PORTB_DIR_R &= ~0x08; // Pin 3 as input (0)

	// clear, then unmask timer interrupt: bit 10 to enable capture mode event
	TIMER3_ICR_R |= 0x400;
	TIMER3_IMR_R |= 0x400;
	TIMER3_CTL_R |= 0x100; // turn timer on again
	//TIMER3_ICR_R |= 0x400; // maybe do before next line


	// Get both edge times using interrupt handler.
	while (current_status != DONE) {
		// wait for the interrupts
	}
	// Compute pulse width
	current_status = LOW;
	// Detect overflows
	if (rising_edge_time < falling_edge_time) {
		++num_overflows;
		return  0xFFFFFF - falling_edge_time + rising_edge_time;
		}

	else {
		return rising_edge_time - falling_edge_time; // return the pulse width in clock cycles
	}

}
// UNUSED?
//void ping_interrupt_init(void) {
//	// Both edges should trigger interrupts (need duration of signal)
//	// The initial configuration sets up both edges.
//	// unmask interrupts
//	TIMER3_IMR_R |= 0x400;
//	// bind interrupts to handler
//	IntRegister(INT_TIMER3B, ping_handler);
//}

/**
 * Captures the times of the rising and falling edges.
 */
void ping_handler(void) {
	// clear interrupt status
	if (current_status == LOW) {
		rising_edge_time = TIMER3_TBR_R & 0xFFFFFF; // 16 bits + 8 bit prescale = 24 bits
		current_status = HIGH;
	}
	else if (current_status == HIGH) {
				falling_edge_time = TIMER3_TBR_R & 0xFFFFFF;
				current_status = DONE;
			}
	TIMER3_ICR_R |= 0x400;
}
