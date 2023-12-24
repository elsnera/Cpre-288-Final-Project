/**
 * movement.h
 *
 * Contains functionality to move the robot forward a specific distance
 * and turn the robot a specific number of degrees.
 *
 * @author Jacob Carnesi
 * @author Marcus Barker
 *
 * @date 9/7/2023
 *
 */

#ifndef MOVEMENT_H
#define MOVEMENT_H

#include "open_interface.h"
#include "uart.h"
#include "lcd.h"

void move_forward(oi_t *sensor, int centimeters, int speed);

double move_with_detection(oi_t *sensor, int centimeters, int speed);

void move_with_detection_after_bump(oi_t *sensor);

void reverse(oi_t *sensor, int centimeters, int speed);

void turn_right(oi_t *sensor, double degrees);

void turn_left(oi_t *sensor, double degrees);

void left_impact(oi_t *sensor);

void right_impact(oi_t *sensor);

void cliff_data_processor(oi_t *sensor);

void cliff_front_sensors(oi_t *sensor, signed short side);

void cliff_side_sensors(oi_t *sensor, signed short side);

#endif
