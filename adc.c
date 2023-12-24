#include "adc.h"

/**
 * Initializes ADC 0 to use Analog Input 10, PB4, on Sample Sequencer 0.
 */
void adc_init() {
	// Enable clock to ADC 0.
	SYSCTL_RCGCADC_R |= 0x1;

	// Setup GPIO Port B pin
	SYSCTL_RCGCGPIO_R |= 0x02;
	GPIO_PORTB_AFSEL_R |= 0x10;
	GPIO_PORTB_DIR_R &= ~0x10;
	GPIO_PORTB_DEN_R &= ~0x10;
	GPIO_PORTB_AMSEL_R |= 0x10;
	GPIO_PORTB_ADCCTL_R &= 0x00;

	//GPIO_PORTB_PCTL_R &= 0xFFF0FFFF;

	// Disable ADC0 SS0 in preparation for configuration.
	ADC0_ACTSS_R &= 0xFFFFFFFE;

	// Configure ADC0 SS0 to be triggered by setting the SS0 bit in the ADCPSSI
	// register in the main program or read function (set 3:0 to 0).
	// The bits 3:0 in this ADCEMUX register correspond to SS0.
	ADC0_EMUX_R &= 0xFFFFFFF0;

	// If I understand correctly, our sample sequence will be a single reading from
	// the IR sensor each time we collect. Bits 3:0 will be configured to request
	// Analog Input 10 as our first sample and no others.
	ADC0_SSMUX0_R &= 0xA; // clear all other bits
	ADC0_SSMUX0_R |= 0xA; // Initialize the first MUX reading to AIN10

	// Configure bits 3:0 to handle our single sample in the sequence reading from
	// the IR voltage register. It is the end of sequence, but will not read from the temp
	// sensor, use interrupts, or use differential samples.
	ADC0_SSCTL0_R &= ~0x6;
	ADC0_SSCTL0_R |= 0x6;
	//ADC0_SSCTL0_R |= 0x20000000;
	//ADC0_SSCTL0_R &= ~0x20000000;

	// Use the hardware averager to average the results of 16 samples.
	ADC0_SAC_R &= ~0x04;
	ADC0_SAC_R |= 0x04;

	// Enable ADC0 SS0
	ADC0_ACTSS_R |= 0x1;
}

/**
 * Take a sample from the IR sensor and convert it to a digital signal.
 * Results from the conversion are found in the ADCSSFIFO0 register.
 * The IR sensor is always taking readings once initialized. This function
 * tells the ADC to grab the reading, convert it, and put it in the FIFO.
 * Next, it returns the 12-bit value as an integer.
 */
int adc_read() {
	// first: Perform the ADC conversion.
	ADC0_PSSI_R |= 1;
	ADC0_PSSI_R &= 0xF7FFFFFF; // do not use synchronous wait
	// wait for the conversion to complete
	while (~ADC0_RIS_R & 0x1) {}
	// clear Interrupt
	ADC0_ISC_R |= 1;
	// return the reading if complete.
	return ADC0_SSFIFO0_R & 0xFFF;
}



