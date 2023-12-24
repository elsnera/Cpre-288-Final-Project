/**
 * Header file for functions that configure
 * Analog Digital Converter.
 */

#include <stdint.h>
#include <stdbool.h>
#include "timer.h"
#include "lcd.h"
#include <inc/tm4c123gh6pm.h>
#include "driverlib/interrupt.h"

#ifndef ADC_H
#define ADC_H


void adc_init();

int adc_read();

#endif
