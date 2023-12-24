#include "ping.h"
#include "driverlib/interrupt.h"
#include "button.h"
#include "math.h"
#include "servo.h"
#include "lcd.h"
#include "ping.h"
#include "adc.h"

int current_degrees;
int right_calibration_value;
int left_calibration_value;
extern volatile int button_num;

/**
 * Initialize servo motor controls.
 */
void servo_init(void) {

	// Configure Port B pin 5 for timer use.
	SYSCTL_RCGCGPIO_R |= 0x02; // Port B enable
	GPIO_PORTB_DIR_R |= 0x20; // Pin 5 as output (1)
	GPIO_PORTB_DEN_R |= 0x20; // Enable digital functions (1)
	GPIO_PORTB_AFSEL_R |= 0x20; // write 1 to enable alternate function
	GPIO_PORTB_PCTL_R &= ~0xF00000; // set corresponding bits to 0x7
	GPIO_PORTB_PCTL_R |= 0x700000;

	// Configure timer 1B as follows:
	// PWM mode, 16 bits + 8 bit prescale extension, count down, periodic mode
	SYSCTL_RCGCTIMER_R |= 0x02; // Timer 1 clock services enable
	TIMER1_CTL_R &= ~0x100; // Disable timer 1B
	TIMER1_CFG_R |= 0x4; // Set to 16 bit timer configuration.
	// Configure mode register: bit 4 = 0 (count down), bit 3 = 1 (PWM), bit 2 = 0 (edge-count), bits 1:0 to 0b10 (periodic).
	TIMER1_TBMR_R &= ~0x15; // Force 0 in bits 4, 2, and 1.
	TIMER1_TBMR_R |= 0xA; // Force 1 in bits 3 and 1.
	TIMER1_CTL_R &= ~0x4000; // Output is not inverted.

	TIMER1_TBPR_R = 0x4; // Configure 8-bit prescaler
	// NO interrupts.
	TIMER1_TBILR_R = 0xE200;// Configure 16 bit load

	// Configure 4 bits of prescale for the match register to point it to 90 degrees.
//	TIMER1_TBPMR_R = 0x4;
//
//	TIMER1_TBMATCHR_R = 0x8440;

	set_match_registers(296000); // 90 degrees

	TIMER1_CTL_R |= 0x100; // Enable timer 1B to point to 90 degrees.
	TIMER1_CTL_R &= ~0x100; // Disable timer 1B to free the servo.
}

/**
 * Moves the servo a certain number of degrees. May not be consistent across all
 * cyBots (color me surprised).
 *
 * PARAM: degrees = angle the servo should point to.
 *
 * RETURN: optionally, return the value in the MATCH_R register
 */
int servo_move(float degrees) {
	/**
	 * How this works: the servo is fed a pulse of a certain width. The width
	 * determines the angle it moves to. PWM will periodically send out a pulse
	 * with that pulse width. The timer match register determines the pulse width.
	 *
	 * No matter how long the pulses are sent, the servo will only move to that
	 * angle since it is specified by the pulse width. Our code has no way of knowing
	 * what the current angle of the motor is, so we have just wait for a some time
	 * until it should be done with its movement.
	 */

	// 1. Compute the match value for that number of degrees and load it into the
	// match register.
	//TIMER1_CTL_R &= ~0x100; // Disable timer 1B
	set_match_registers(round(320000 - right_calibration_value - ((left_calibration_value - right_calibration_value) / 180) * degrees));

	// 2. Have the PWM start sending pulses.
	//TIMER1_CTL_R |= 0x100; // Enable timer 1B

	// 3. Since the PWM/timer system is independent of the CPU,
	// wait a small amount of time for the servo to have completed its movement.
	// This will depend on how far it is moving from its current position.
	// May want to use a global variable to store the last position.
	// Err towards the short end.

	// 4. Set current_degrees to represent the current position.
	if (degrees > current_degrees) {
		current_degrees += degrees - current_degrees;
	} else if (degrees < current_degrees) {
		current_degrees -= current_degrees - degrees;
	}

	return TIMER1_TBMATCHR_R + (TIMER1_TBPMR_R << 16);
}

void set_match_registers(int match_value) {
	TIMER1_CTL_R &= ~0x100;
	TIMER1_TBPMR_R = match_value >> 16;
	TIMER1_TBMATCHR_R = match_value & 0xFFFF;
	TIMER1_CTL_R |= 0x100;
}

void servo_calibrate(void) {
	// initialize match value to... ?
	// move the servo to what it thinks zero is
	//servo_move(0);
	short turn_direction = 1; // 1 = clockwise, -1 = counterclockwise.

	int match_val = TIMER1_TBMATCHR_R + (TIMER1_TBPMR_R << 16);
	int btn = button_num;
	while (1) {
		btn = button_num;
		switch (btn) {
		case 1: // move servo 1 degree
			match_val += turn_direction * 89;
			set_match_registers(match_val);
			lcd_clear();
			lcd_printf("%d", 320000 - match_val);
			break;

		case 2: // move servo 5 degrees
			match_val += turn_direction * 89 * 5;
			set_match_registers(match_val);
			lcd_clear();
			lcd_printf("%d", 320000 - match_val);
			break;

		case 3: // toggle between clockwise and counterclockwise
			turn_direction *= -1;
			if (turn_direction == 1) {
				lcd_printf("Angle now DEcreasing");
			}
			else if (turn_direction == -1) {
				lcd_printf("Angle now INcreasing");
			}
			break;

		case 4: // Move to 0 degrees in clockwise mode or 180 degrees in CCW mode.
			if (turn_direction == 1) { // CW
				match_val = 304000;
			}

			else if (turn_direction == -1) { // CCW
				match_val = 288000;
			}
			set_match_registers(match_val);
			lcd_clear();
			lcd_printf("%d", 320000 - match_val);
			break;
		}
		timer_waitMillis(100);
	}



}
