#include <stdint.h>
#include <stdbool.h>
#include "timer.h"
#include "lcd.h"
#include <inc/tm4c123gh6pm.h>
#include "driverlib/interrupt.h"

#ifndef PING_H
#define PING_H

void ping_initialize(void);

int ping_read(void);

void ping_handler(void);

#endif
