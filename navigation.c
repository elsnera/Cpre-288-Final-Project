#include "navigation.h"

// buttons
extern volatile int button_event;
extern volatile int button_num;

// Uart
extern volatile char data_char;
extern volatile char stop;

// ping
extern volatile enum status current_status;
extern volatile int rising_edge_time;
extern volatile int falling_edge_time;
extern int num_overflows;

// servo
extern int right_calibration_value;
extern int left_calibration_value;

void control_navigation(oi_t sensor_data) {
//	int putty_cmd;       // Variable to get bytes from Client
//		putty_cmd = 1;
//		object_t smallest;
//		movePlan_t current_plan;
//		char data_buffer[50];  // Buffer to store data
//		mode = 1;
//		while (1) {
//			uart_sendString("Manual Mode\r\n");
//			while (mode == 1) { // this is manual mode
//				putty_cmd = uart_receive();
//				switch (putty_cmd) {
//					case 'w':
//					   move_forward(sensor_data, 5, 250);
//					   break;
//					case 'a':
//					   turn_left(sensor_data, 5);
//					   break;
//					case 's':
//					   reverse(sensor_data, 5, 250);
//					   break;
//					case 'd':
//					   turn_right(sensor_data, 5);
//					   break;
//					case 'm':
//						IR_and_ping_scan(1);
//						break;
//					case '_':
//						uart_sendString("Open the pod bay doors Toonces!\r\n");
//						break;
//
//				}
//			}
//			uart_sendString("Autonomous Mode\r\n");
//			current_plan.plan_pending = 0; // otherwise this could be volatile
//			while (mode == 0) { // autonomous mode
//				putty_cmd = uart_receive();
//				//uart_sendString("auto\r\n");
//
//				// if (h and has not formulated plan)
//				// scan and make plan
//				// set flag to true
//				if (putty_cmd == 'h' && current_plan.plan_pending == 0) {
//					smallest = IR_and_ping_scan(1);
//					current_plan = make_plan(smallest);
//					if (current_plan.turn_direction == -1) {
//						sprintf(data_buffer, "Object detected %d degrees to the right and %.00f centimeters away.\r\n", current_plan.heading, current_plan.distance);
//						uart_sendString(data_buffer);
//					}
//					else if (current_plan.turn_direction == 1){
//						sprintf(data_buffer, "Object detected %d degrees to the left and %.00f centimeters away.\r\n", current_plan.heading, current_plan.distance);
//						uart_sendString(data_buffer);
//					}
//					else if (current_plan.turn_direction == 0) {
//						sprintf(data_buffer, "No objects detected. Moving %.00f centimeters forward.\r\n", current_plan.distance);
//						uart_sendString(data_buffer);
//					}
//				}
//				else if (putty_cmd == 'h' && current_plan.plan_pending == 1) {
//					// execute plan
//					if (current_plan.turn_direction == 0) {
//						//move_forward(sensor_data, current_plan.distance, 100);
//						double distance_traveled = 0;
//						int distance_mm = 10 * current_plan.distance;
//						oi_setWheels(100, 100);
//						while (distance_traveled < distance_mm) {
//						   oi_update(sensor_data);
//						   distance_traveled += sensor_data->distance;
//						   if (sensor_data->bumpLeft){
//							   left_impact(sensor_data);
//							   oi_setWheels(0,0);
//							   uart_sendString("Collision has occurred with the left bumper.\r\n");
//							   uart_sendString("Toonces has performed evasive maneuvers right.\r\n");
//							   break;
//						   }
//						   if (sensor_data->bumpRight){
//							   right_impact(sensor_data);
//							   oi_setWheels(0,0);
//							   uart_sendString("Collision has occurred with the right bumper.\r\n");
//							   uart_sendString("Toonces has performed evasive maneuvers left.\r\n");
//							   break;
//						   }
//						}
//						oi_setWheels(0,0);
//
//					} else {
//						navigate_to_smallest_obj(sensor_data, current_plan);
//					}
//					// replan
//					smallest = IR_and_ping_scan(1);
//					current_plan = make_plan(smallest);
//					if (current_plan.turn_direction == -1) {
//						sprintf(data_buffer, "Object detected %d degrees to the right and %.00f centimeters away.\r\n", current_plan.heading, current_plan.distance);
//						uart_sendString(data_buffer);
//					}
//					else if (current_plan.turn_direction == 1){
//						sprintf(data_buffer, "Object detected %d degrees to the left and %.00f centimeters away.\r\n", current_plan.heading, current_plan.distance);
//						uart_sendString(data_buffer);
//					}
//					else if (current_plan.turn_direction == 0) {
//						sprintf(data_buffer, "No objects detected. Moving %.00f centimeters forward.\r\n", current_plan.distance);
//						uart_sendString(data_buffer);
//					}
//				}
//
//				// else if (h and has formulated plan)
//				// execute plan, set formulated plan to false
//			}
//		}
}

void IR_calibrate() {
	// the Roomba has to be ON to use the sensors
	//servo_move(90); // ensure the scanner is facing the right direction.
	int IR_total = 0;
	short scan_size = 8; // was 10
	char data_buffer[8];
	short i = 0;
	short scan_real_distance = 10; // starts at 10 cm
	lcd_printf("Calibrating_IR");
	while(1) {
		if (button_num) {
			for (i = 0; i < scan_size; ++i) {
				IR_total += adc_read();
			}
			IR_total /= scan_size;

			sprintf(data_buffer, "%d,%u\r\n", IR_total, scan_real_distance);
			scan_real_distance += 2;

			uart_sendString(data_buffer);
			lcd_clear();
			lcd_puts(data_buffer);
			button_num = 0;
			timer_waitMillis(1000);
		}
	}
}

int scan_IR(float degrees) {
	servo_move(degrees);
	timer_waitMillis(60);
	// example distance formula: IR_distance = 1 [varies] * pow(10, 6 [varies]) * pow((IR_raw_sum / (float)num_scans), -1.49 [varies]);
	return adc_read();
}

float scan_ping(float degrees) {
	servo_move(degrees);
	timer_waitMillis(60);
	return ping_read() * 0.0010625;
}

object_t IR_and_ping_scan() {
    int current_angle = 0; // currently scanned angle
    int starting_angle = 0; // starting angle of an object
    float previous_distance = 100; // a dummy previous distance so our algorithm works
    char scanning_object = 0; // true or false value
    short object_num = 0; // number of objects found
    object_t found_objects[10]; // array of object_t's for the objects found
    char data_buffer[35]; // data sent to PuTTy
    short i = 0; // iterator
    short num_scans = 3; // number of scans per angle
    int IR_raw_sum = 0; // sum of raw IR values
	float IR_distance = 0; // averaged raw IR value converted to cm using formula
	float obj_distance = 0; // distance to an object in the struct
	int obj_mid_angle = 0; // angle to scan the middle of an object


	uart_sendString("Degrees\t\tIR Distance (cm)\n\r");
//	else if (stop == 0) { // do not print individual readings in autonomous mode.
//		uart_sendString("Scanning 180 degrees...\n\r");
//	}

	servo_move(0);
	timer_waitMillis(150);
    // perform scan
   while (current_angle <= 180 && !stop) {
	   IR_raw_sum = 0;
	   IR_distance = 0;
	   for (i = 0; i < num_scans; i++) {
		   //cyBOT_Scan(current_angle, &current_reading);
		   IR_raw_sum += scan_IR(current_angle);
	   }

       // Convert IR raw value into an averaged distance in centimeters using
	   // the line of best fit for the calibration of that robot.
       //IR_distance = 2 * pow(10, 8)* pow((IR_raw_sum / (float)num_scans), -2.115); // cybot 8
	   // IR_distance = 197305 * pow((IR_raw_sum / (float)num_scans), -1.249); // cybot 14

	   //IR_distance = pow(10, 8) * pow((IR_raw_sum / (float)num_scans), -2.075); // cybot 1
	   IR_distance = 877514 * pow((IR_raw_sum / (float)num_scans), -1.45); // cybot 1

	   sprintf(data_buffer, "%d,%.00f,\n\r", current_angle, IR_distance);
	   uart_sendString(data_buffer);
       // The cyBot can only provide useful data at distances under 50 cm.
       // if we get a reading under 50 cm, save that angle as a potential object.
       if (scanning_object) {
    	   // If no longer scanning object
           if (abs(IR_distance - previous_distance) > 10 || current_angle == 180) {
        	   // and the object was at least 4 degrees of travel (throw out bad readings)
               if (abs(starting_angle - current_angle) > 4) { // was 14
            	   // the robot found an object
            	   obj_mid_angle = (starting_angle + current_angle) / 2.0;
            	   servo_move(obj_mid_angle);
            	   timer_waitMillis(100);

                   found_objects[object_num].start_angle = starting_angle;
                   found_objects[object_num].end_angle = current_angle;
                   found_objects[object_num].obj_number = object_num + 1;
                   // Scan middle of object with ping sensor to obtain distance
                   for (i = 0; i < num_scans; i++) {
					   obj_distance += scan_ping(obj_mid_angle);
				   }
                   servo_move(current_angle);
                   found_objects[object_num].distance = obj_distance / num_scans;
                   obj_distance = 0;
                   ++object_num;
               }
               scanning_object = 0;

           }
       } else { // we could detect a new object
           // check if we are detecting a new object
           // If the change is also more than 10 centimeters, we have a new object. (save the previously detected object)
           if (IR_distance < 70 /*&& abs(IR_distance - previous_distance) > 10*/) {
               starting_angle = current_angle;
               scanning_object = 1;
           }
       }


       previous_distance = IR_distance;

       current_angle += 2;
   }
   uart_sendString("Object#\tAngle\tDistance (cm)\tWidth (deg)\tLinear Width (cm)\n\r");

   short index_smallest_obj = -1;
   float smallest_linear_width = 0;
   int width;
   float linear_width, duhh;
   object_t no_object_detected;
   no_object_detected.distance = 50;
   no_object_detected.obj_number = -1;
   no_object_detected.start_angle = 90;
   no_object_detected.end_angle = 90;
   // go through all the objects and find the smallest one
   for (i = 0; i < object_num; i++) {
       width = found_objects[i].end_angle - found_objects[i].start_angle;
       duhh = width * M_PI / 180.0;
       //why = fabs(uhh);
       linear_width = 2.0 * found_objects[i].distance * fabs(sin(0.5 * duhh));
       if (i == 0) {
           index_smallest_obj = 0;
           smallest_linear_width = linear_width;
       }
       else if (linear_width < smallest_linear_width) {
           smallest_linear_width = linear_width;
           index_smallest_obj = i;
       }
    	   sprintf(data_buffer, "%u\t,%d,\t%.00f,\t\t%d,\t\t%.00f,\n\r", found_objects[i].obj_number, found_objects[i].start_angle, found_objects[i].distance, width, linear_width);
    	   uart_sendString(data_buffer);
   }

   uart_sendString("OBJECT COMPLETE\n");

   servo_move(0);
   // Point to the smallest object, if we identified one.
   if (index_smallest_obj > -1) {
       //int angle = (found_objects[index_smallest_obj].end_angle - found_objects[index_smallest_obj].start_angle) / 2  + found_objects[index_smallest_obj].start_angle;
       //servo_move(angle);
       // Return the smallest object.
       return found_objects[index_smallest_obj];
   }
   else {
	   return no_object_detected;
   }
}

movePlan_t make_plan(object_t destination) {
	movePlan_t plan;
	// did not detect any objects, will always make a plan to move forward ~25 cm.
	if (destination.obj_number == -1) {
		plan.distance = destination.distance;
		plan.heading = 0;
		plan.turn_direction = 0;
		plan.plan_pending = 1;
		return plan;
	}
	plan.distance = destination.distance;
	float center_degrees = (destination.start_angle + destination.end_angle) / 2;
	signed char left_or_right; // L = +1, R = -1
	float theta_rads;
	if (center_degrees > 90) {
		theta_rads = (center_degrees - 90) * M_PI / 180;
		left_or_right = 1;
	}
	else {
		theta_rads = (90 - center_degrees) * M_PI / 180;
		left_or_right = -1;
	}
	plan.turn_direction = left_or_right;
	// compute the angle the robot needs to turn, which is different from the measured angle
	int sensor_offset = 11; // it's 11 cm from the sensor to the center of the robot
	double opp = destination.distance * sin(theta_rads);
	double adj_theta = destination.distance * cos(theta_rads);
	double adj_phi = adj_theta + sensor_offset;
	center_degrees = atan(opp / adj_phi) * 180 / M_PI;
	plan.heading = center_degrees;
	plan.plan_pending = 1;

	return plan;
}

void navigate_to_smallest_obj(oi_t *sensor, movePlan_t plan) {
	// turn to smallest object
	if (stop == 0) { // do not print individual readings in autonomous mode.
		uart_sendString("Moving to object...\n\r");
	}
	// case 1: turn left
	if (plan.turn_direction == 1) {
		turn_left(sensor, (int)(plan.heading * 0.65));
		//turn_right(sensor, correction_angle); // correction for overshot angles
	}
	// case 2: turn right
	else if (plan.turn_direction == -1) {
		turn_right(sensor, (int)(plan.heading * 0.88));
		//turn_left(sensor, 5); // correction for overshooting angles
	}

	// scan and orient to object again

	object_t new_scan = IR_and_ping_scan();
	movePlan_t adjustments = make_plan(new_scan);
	// case 1: turn left
	if (adjustments.turn_direction == 1) {
		turn_left(sensor, (int)(adjustments.heading * 0.65));
		//turn_right(sensor, correction_angle); // correction for overshot angles
	}
	// case 2: turn right
	else if (adjustments.turn_direction == -1) {
		turn_right(sensor, (int)(adjustments.heading * 0.84));
		//turn_left(sensor, 5); // correction for overshooting angles
	}

	// drive to object (allegedly)
	scan_ping(90);
	double distance_traveled = 0;
	int distance_mm = 10 * (plan.distance - 10);
	oi_setWheels(40, 40);
	while (scan_ping(90) > 10 && distance_traveled < distance_mm && stop == 0) {
		oi_update(sensor);
		if (sensor->bumpLeft){
		   left_impact(sensor);
		   // may want to move forwards again to be even with the barricade
		   oi_setWheels(0,0);
		   uart_sendString("Collision has occurred with the left bumper.\r\n");
		   uart_sendString("Toonces has performed evasive maneuvers right.\r\n");
		   break;
	   }
	   if (sensor->bumpRight){
		   right_impact(sensor);
		   oi_setWheels(0,0);
		   uart_sendString("Collision has occurred with the right bumper.\r\n");
		   uart_sendString("Toonces has performed evasive maneuvers left.\r\n");
		   break;
	   }
		distance_traveled += sensor->distance;
	}
	oi_setWheels(0,0);
	stop = 0;
}


void display_cliff_values(oi_t *sensor) {
	while (1) {
	    	oi_update(sensor);
	    	// white will send sensor values > 2700
	    	// black is < 1200
	    	// actual hole is < 120

	    	lcd_printf("FL:%d, FR:%d\nL:%d, R:%d", sensor->cliffFrontLeftSignal, sensor->cliffFrontRightSignal,
					   sensor->cliffLeftSignal, sensor->cliffRightSignal);
	    	timer_waitMillis(1000);
	    }
}

void random_drive(oi_t *sensor, int speed) {
	char data_buffer[50];
	sprintf(data_buffer, "L\tFL\tFR\tR\n\r");
	uart_sendString(data_buffer);
	oi_setWheels(speed, speed);
	double time = 600000; // 10 minutes
	double elapsed_time = 0;
	while (elapsed_time < time) {
		oi_update(sensor);
//    	sprintf(data_buffer, "%d\t%d\t%d\t%d\n\r", sensor_data->cliffLeftSignal, sensor_data->cliffFrontLeftSignal,
//					sensor_data->cliffFrontRightSignal, sensor_data->cliffRightSignal);
//    	uart_sendString(data_buffer);
		if (sensor->bumpLeft){
		   left_impact(sensor);
		   //distance_traveled -= 150;
		   oi_setWheels(speed, speed);
	   }

	   if (sensor->bumpRight){
		   right_impact(sensor);
		   //distance_traveled -= 150;
		   oi_setWheels(speed, speed);
	   }
	   cliff_data_processor(sensor);
//		   if (hit_2_obstacles == 1) {
//			   break;
//		   }
	   oi_setWheels(speed, speed);
	   elapsed_time += timer_getMillis();
	}
	oi_setWheels(0, 0);
}
