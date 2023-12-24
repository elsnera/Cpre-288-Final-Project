/**
 * Movement functions for the Roomba.
 */

#include "movement.h"

extern volatile char stop;
short obstacle_error = 0;
//short hit_2_obstacles = 0; // potentially just for testing.

/**
 * Move the robot forwards a certain distance.
 *
 * @param *sensor -
 * @param centimeters - the desired distance the robot will move forward
 * @param speed - desired speed
 */
void move_forward(oi_t *sensor, int centimeters, int speed) {

    double distance_traveled = 0;
    int millimeters = 10 * centimeters;

    oi_setWheels(speed, speed);
    while (distance_traveled < millimeters) {
        oi_update(sensor);
        distance_traveled += sensor->distance;
    }

    oi_setWheels(0,0);

}

double move_with_detection(oi_t *sensor, int centimeters, int speed) {
    double distance_traveled = 0;
    int millimeters = 10 * centimeters;

    oi_setWheels(speed, speed);
    while (distance_traveled < millimeters && !stop && obstacle_error == 0) {
       oi_update(sensor);
       distance_traveled += sensor->distance;
       if (sensor->bumpLeft){
           left_impact(sensor);
           distance_traveled -= 150;
           oi_setWheels(speed, speed);
       }

       if (sensor->bumpRight){
           right_impact(sensor);
           distance_traveled -= 150;
           oi_setWheels(speed, speed);
       }
       cliff_data_processor(sensor);
       if (obstacle_error == 1) {
    	   break;
       }
    }
    stop = 0; // reset stop if triggered.
    oi_setWheels(0,0);
    return distance_traveled / 10; // Distance in cm.
}

void move_with_detection_after_bump(oi_t *sensor) {
	obstacle_error = 0;

    double distance_traveled = 0;
    int millimeters = 150;

    int speed = 200;
    oi_setWheels(speed, speed);

    while (distance_traveled < millimeters) {
       oi_update(sensor);
       distance_traveled += sensor->distance;
       if (sensor->bumpLeft || sensor->bumpRight){
    	   reverse(sensor, 5, 250);
    	   turn_left(sensor, 174);
    	   oi_setWheels(0,0);
    	   obstacle_error = 1;
    	   return;
       }
       cliff_data_processor(sensor);
       if (obstacle_error == 1) {
    	   //hit_2_obstacles = 1;
    	   // Check for holes or boundaries.
    	   oi_setWheels(0,0);
    	   //uart_sendString("Hit a boundary.\r\n");
    	   lcd_printf("Hit a boundary.\r\n");
    	   obstacle_error = 1;
    	   return;
       }
    }
    oi_setWheels(-100, -100);
    oi_setWheels(0,0);
    return;
}


/**
 * Move the robot backwards a certain distance.
 *
 * @param *sensor -
 * @param centimeters - the desired distance the robot will move
 * @param speed - desired speed
 */
void reverse(oi_t *sensor, int centimeters, int speed) {
    double distance_traveled = 0;
   int millimeters = 10 * centimeters;

   oi_setWheels(-speed, -speed);
   while (distance_traveled < millimeters) {
       oi_update(sensor);
       distance_traveled -= sensor->distance;
   }

   oi_setWheels(0,0);
}

/**
 * Turns the robot a certain number of degrees clockwise.
 * Tends to overshoot, even at low speed.
 *
 * @param *sensor
 * @param degrees - the desired number of degrees the robot will turn
 */
void turn_right(oi_t *sensor, double degrees) {
    double angle_turned = 0;

    oi_setWheels(-100, 100); // correct wheel difference for clockwise.

    while (angle_turned < degrees) {
        oi_update(sensor);
        angle_turned -= sensor->angle; // This data may need some adjustment.
    }

    //oi_setWheels(0,0);
    // move back a degree to prevent drift
    oi_setWheels(100, -100);
    angle_turned = 0;
	while (angle_turned < 0) {
		oi_update(sensor);
		angle_turned += sensor->angle;
	}

	oi_setWheels(0,0);
}

/**
 * Turns the robot a certain number of degrees counterclockwise.
 * Tends to overshoot, even at low speed.
 *
 * @param *sensor
 * @param degrees - the desired number of degrees the robot will turn
 */
void turn_left(oi_t *sensor, double degrees) {
    double angle_turned = 0;

    oi_setWheels(100, -100); // correct wheel difference for clockwise.

    while (angle_turned < degrees) {
        oi_update(sensor);
        angle_turned += sensor->angle; // This data may need some adjustment.
    }

    //oi_setWheels(0,0);
    // move back a degree to prevent drift
    oi_setWheels(-100, 100);
    angle_turned = 0;
	while (angle_turned < 0) {
		oi_update(sensor);
		angle_turned -= sensor->angle;
	}

	oi_setWheels(0,0);
}

/**
 * When we are driving manually, the robot will simply back up from an obstacle.
 * The function also sets the obstacle_error flag to prevent more driving.
 */
void left_impact(oi_t *sensor) {
    reverse(sensor, 10, 300);
//    turn_right(sensor,87);
//    move_with_detection_after_bump(sensor);
//    if (obstacle_error == 0) { // did not hit a boundary
//    	turn_left(sensor, 87);
//    }
    obstacle_error = 1;
	uart_sendString("Hit short obstacle on left side. Reversed 5cm.\r\n");
	lcd_clear();
	lcd_printf("Hit on left side.\r\n");

}

/**
 * When we are driving manually, the robot will simply back up from an obstacle.
 * The function also sets the obstacle_error flag to prevent more driving.
 */
void right_impact(oi_t *sensor) {
    reverse(sensor, 10, 300);
//    turn_left(sensor,87);
//    move_with_detection_after_bump(sensor);
//    if (obstacle_error == 0) { // did not hit a boundary
//    	turn_right(sensor, 87);
//    }
    obstacle_error = 1;
	uart_sendString("Hit short obstacle on right side. Reversed 5cm.\r\n");
	lcd_clear();
	lcd_printf("Hit on right side.\r\n");
}

/**
 * Used to determine if the robot has detected a hole or boundary. Both are handled in
 * the same way: if one of the side sensors detects it, the robot will turn slightly off
 * its original course to correct. If either of the front sensors detects it, the robot
 * will square up with the obstacle and back away from it.
 *
 * Sends a message to the remote client and LCD screen.
 */
void cliff_data_processor(oi_t *sensor) {
	//int hit = 0;
	// front left detected a hole or boundary
	if (sensor->cliffFrontLeftSignal > 2700 || sensor->cliffFrontLeftSignal < 100) {
		lcd_clear();
		lcd_puts("FL cliff triggered.");
		cliff_front_sensors(sensor, -1);
	}
	// front right detected a hole or boundary
	else if (sensor->cliffFrontRightSignal > 2700 || sensor->cliffFrontRightSignal < 100) {
		lcd_clear();
		lcd_puts("FR cliff triggered.");
		cliff_front_sensors(sensor, 1);
	}
	// left side detected a hole or boundary
	else if (sensor->cliffLeftSignal > 2700 || sensor->cliffLeftSignal < 100) {
		cliff_side_sensors(sensor, -1);
	}
	// right side detected a hole or boundary
	else if (sensor->cliffRightSignal > 2700 || sensor->cliffRightSignal < 100) {
		cliff_side_sensors(sensor, 1);
	}

//	lcd_clear();
//	lcd_puts("Cliff data processed.");
}

/**
 * Will square up the cybot with a hole or boundary and back away from it.
 * Turn the appropriate direction until the two front sensor values are approximately equal.
 * Side represents left with -1 and right with 1.
 */
void cliff_front_sensors(oi_t *sensor, signed short side) {
	oi_setWheels(0, 0); // stop moving forwards.
	// left
	if (side == -1) {
		while (sensor->cliffFrontRightSignal < 2700 && sensor->cliffFrontRightSignal > 100 && !stop) {
			oi_update(sensor);
			lcd_clear();
			lcd_printf("FL:%d, FR:%d\nL:%d, R:%d", sensor->cliffFrontLeftSignal, sensor->cliffFrontRightSignal,
					   sensor->cliffLeftSignal, sensor->cliffRightSignal);
			turn_left(sensor, 3);
		}

		if (sensor->cliffFrontLeftSignal <= 100) {
			uart_sendString("Located hole with front left sensor.\r\n");
		}
		else {
			uart_sendString("Located area boundary with front left sensor.\r\n");
		}
	}
	// right
	else if (side == 1) {
		while (sensor->cliffFrontLeftSignal < 2700 && sensor->cliffFrontLeftSignal > 100 && !stop) {
			oi_update(sensor);
			lcd_clear();
			lcd_printf("FL:%d, FR:%d\nL:%d, R:%d", sensor->cliffFrontLeftSignal, sensor->cliffFrontRightSignal,
						sensor->cliffLeftSignal, sensor->cliffRightSignal);
			turn_right(sensor, 3);
		}

		if (sensor->cliffFrontLeftSignal <= 100) {
			uart_sendString("Located hole with front right sensor.\r\n");
		}
		else {
			uart_sendString("Located area boundary with front right sensor.\r\n");
		}

	}

	if (!stop) {
		//uart_sendString("Backed away.\r\n");
		reverse(sensor, 10, 250);
	}
	stop = 0;
	obstacle_error = 1;
	return;
}

/**
 * If a left OR right side cliff sensor is triggered, this will perform an adjustment to right the course.
 * Side represents left with -1 and right with 1.
 */
void cliff_side_sensors(oi_t *sensor, signed short side) {
	// left side
	if (side == -1) {
		turn_right(sensor, 9);
		if (sensor->cliffLeftSignal <= 100) {
			uart_sendString("Located and avoided hole with left side sensor by turning 10 deg.\r\n");
		}
		else {
			uart_sendString("Located and avoided area boundary with left side sensor by turning 10 deg.\r\n");
		}

	}
	// right side
	else if (side == 1) {
		turn_left(sensor, 9);
		if (sensor->cliffRightSignal <= 100) {
			uart_sendString("Located and avoided hole with right side sensor by turning 10 deg.\r\n");
		}
		else {
			uart_sendString("Located and avoided area boundary with right side sensor by turning 10 deg.\r\n");
		}
	}
	obstacle_error = 1;
	return;
}




