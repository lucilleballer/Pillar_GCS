// main.cpp to run the autonomous flight controller on
// the Narwhal for the Pillar CropCopter

#define F_CPU 8000000UL

#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include "inc/UART.h"
#include "inc/UAVTalk.h"

// SERVO PORTC
#define SERVO_PIN 3

// LED0 PORTD
#define LED0_PIN 5
#define HEARTBEAT 6
#define INPUT_PWM1 0

uint16_t interval_counter = 0;

int main()
{
	// Set Servo output
	DDRC |= (1 << SERVO_PIN);
	// Set LED output
	DDRD |= (1 << LED0_PIN) | (1 << HEARTBEAT);
	// Set PWM inputs
	DDRB &= ~(1 << INPUT_PWM1);
	// Deactivate PWM input pull-up resistors
	PORTB &= ~(1 << INPUT_PWM1);

	// Setup 16-bit timer for CTC prescale = 1
	TCCR1B |= (1 << CS10) | (1 << WGM12);
	OCR1A	= 80;	// every 0.01ms
	TIMSK1 |= (1 << OCIE1A);

	sei();	// Global interrupts on

	// UAVTalk object to read the UAVObjects
	UAVTalk uavtalk;

	// PWM Variables for control signals
	uint8_t pwm_inputs = 0;		// Indicates which inputs are counting
	uint16_t pwm1_counter = 0;	// Counter for PWM1

	while(1) 
	{
		// Detect when the inputs go high
		if (PINB & (1 << INPUT_PWM1)) {
			// Start PWM counter
			pwm_inputs |= (1 << INPUT_PWM1);
		}

		// Detect when the inputs go low
		if (!(PINB & (1 << INPUT_PWM1))) {
			// Stop the PWM counter
			pwm_inputs &= ~(1 << INPUT_PWM1);

			// Send the length to UART and reset counter
			// Counter * 0.01ms = time in ms
			UART::writeByte(pwm1_counter);
			pwm1_counter = 0;
		}

		// Increment the PWM counters every 0.01ms
		if (pwm_inputs & (1 << INPUT_PWM1)) ++pwm1_counter;
	}

	return 0;
}

// Timer Interrupt
ISR(TIMER1_COMPA_vect)
{
	++interval_counter;
}
