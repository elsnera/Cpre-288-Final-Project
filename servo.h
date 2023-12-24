#include <stdint.h>
#include <stdbool.h>
#include "timer.h"
#include "lcd.h"
#include <inc/tm4c123gh6pm.h>
#include "driverlib/interrupt.h"

#ifndef SERVO_H
#define SERVO_H

/**
 * Initialize servo motor controls.
 */
void servo_init(void);

/**
 * Moves the servo a certain number of degrees. May not be consistent across all
 * cyBots (color me surprised).
 *
 * PARAM: degrees = angle the servo should point to.
 *
 * RETURN: optionally, return the value in the MATCH_R register
 */
int servo_move(float degrees);

void set_match_registers(int match_value);

void servo_calibrate(void);


#endif
