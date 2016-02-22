/*
 * ds1621_test_mega168.c
 * Author: Max Brueggemann
 *
 * The array temperatures[] contains the sensor readings.
 * Interpretation of values (examples): 231 => 23.1°C
 *										-45 => -4.5°C
 *							 			-862 => sensor not reachable
 */ 

#define F_CPU 20000000L
#include <avr/io.h>
#include <avr/interrupt.h>
#include "ds1621avr.h"

volatile int16_t temperatures[SensorTopAdr+1];

void timer0100us_start(void) {
	TCCR0B|=(1<<CS01);
	TIMSK0|=(1<<TOIE0);
}

ISR(TIMER0_OVF_vect) { 
	tickTWItimeoutCounter();
}

int main() {
	
	sei();
	
	timer0100us_start();
	
	PORTC |= (1<<PORTC4)|(1<<PORTC5);
	outputDataPtr=temperatures; //make the pointer point to the desired array
	ds1621twi_init();
	
	while (1)
	{
		ds1621StateMachine();
	}
}