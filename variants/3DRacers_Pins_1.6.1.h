#ifndef THREEDRACERS_VARIANT_H
#define THREEDRACERS_VARIANT_H

//SETTINGS

#define MOTOR_MODE_PH_EN    true   // true: Phase/Enable mode, false: IN/IN mode
#define MOTOR_MODE_PIN  	12
#define MOTOR_PIN1      	4   // DRV883c IN PH A
#define MOTOR_PIN2      	6   // DRV883c IN EN A (PWM)   
//#define MOTOR_PIN1      	9   // DRV883c IN PH B <- NB: 9, 10 PWM disabled by Servo library, moved Servo to Interrupt 3 (pins 4 and 5 now PWM disabled)
//#define MOTOR_PIN2      	10   // DRV883c IN EN B (PWM)
#define SOFTWARE_PWM    	false
#define MOTOR_RUN_METHOD    0 //ONLY in IN/IN mode: 0 for forward/reverse coasts, 1 for forward/reverse brake

#define LED_PIN 			11
#define SENSOR_PIN 			A5
#define SENSOR_LED_PIN 		A4
#define SERVO_PIN 			3
#define BLE_RESET			5

#define IDENTITY_PIN		8

#endif

