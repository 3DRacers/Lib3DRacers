#ifndef THREEDRACERS_1_2_0_H
#define THREEDRACERS_1_2_0_H

//SETTINGS

#define MOTOR_MODE_PH_EN  	true   // false: Phase/Enable mode
#define MOTOR_MODE_PIN  	12
#define MOTOR_PIN1			5   // DRV883c IN PH 1
#define MOTOR_PIN2      	8   // DRV883c IN EN 2    
#define SOFTWARE_PWM    	true
#define MOTOR_RUN_METHOD    1 //LOW for forward/reverse coasts, HIGH for forward/reverse brake

#define LED_PIN 			11
#define SENSOR_PIN 			A1
#define SERVO_PIN 			3

#endif