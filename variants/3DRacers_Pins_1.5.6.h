#ifndef THREEDRACERS_1_5_6_H
#define THREEDRACERS_1_5_6_H

//SETTINGS

#define MOTOR_MODE_PH_EN    true   // false: Phase/Enable mode
#define MOTOR_MODE_PIN  	12
//#define MOTOR_PIN1      	5   // DRV883c IN PH B <- pin rotto sulla 1.5.6, in futuro ok ma cmq senza PWM sul PH
//#define MOTOR_PIN2      	4   // DRV883c IN EN B    
#define MOTOR_PIN1      	9   // DRV883c IN PH A <- 9, 10 PWM disabled by Servo library -__-°
#define MOTOR_PIN2      	10   // DRV883c IN EN A   
#define SOFTWARE_PWM    	false
#define MOTOR_RUN_METHOD    0 //0 for forward/reverse coasts, 1 for forward/reverse brake

#define LED_PIN 			11
#define SENSOR_PIN 			A5
#define SENSOR_LED_PIN 		A4
#define SERVO_PIN 			3
#define BLE_RESET			2

#endif

