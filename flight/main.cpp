// main.c

#define F_CPU 8000000UL

#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>

#include "MPU6050.hpp"

// SERVO PORTC
#define SERVO_PIN 3

// LED0 PORTD
#define LED0_PIN 5
#define HEARTBEAT 6

uint16_t interval_counter = 0;

int main()
{
	// Counter for the interval
	uint16_t servo_angle = 105;
	uint8_t pulse_active = 0;

	// Set Servo output
	DDRC |= (1 << SERVO_PIN);
	// Set LED output
	DDRD |= (1 << LED0_PIN) | (1 << HEARTBEAT);

	// Setup 16-bit timer for CTC prescale = 1
	TCCR1B |= (1 << CS10) | (1 << WGM12);
	OCR1A	= 80;	// every 0.01ms
	TIMSK1 |= (1 << OCIE1A);

	sei();	// Global interrupts on

	// Init the IMU
	MPU6050 imu;
	imu.start();

	while(1) 
	{
		imu.readData();

		// Every 20ms
		if (interval_counter >= 2000)
		{
			pulse_active |= (1 << 0);
			PORTD ^= (1 << HEARTBEAT);
			interval_counter = 0;

			// Servo sweep test
			if (pulse_active & (1 << 1)) {
				servo_angle -= 4;
				if (servo_angle < 100) pulse_active &= ~(1 << 1);
			} else {
				servo_angle += 4;
				if (servo_angle > 260) pulse_active |= (1 << 1);
			}
		}

		if (pulse_active & (1 << 0))
		{
			// Mid-point = 185
			// Calculate the servo angle 


			// Turn the servo signal on
			PORTC |= (1 << SERVO_PIN);
			if (interval_counter >= servo_angle)
			{
				PORTC &= ~(1 << SERVO_PIN);
				pulse_active &= ~(1 << 0);
			}
		}
	}

	return 0;
}

// Timer Interrupt
ISR(TIMER1_COMPA_vect)
{
	++interval_counter;
}
