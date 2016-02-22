#ifndef DS1621avr_H
#define DS1621avr_H
#endif
/************************************************************************
Title:    A state machine based DS1621 library
Author:   Max Brueggemann
Hardware: any AVR with built-in TWI, tested on Atmega 88/168 at 20Mhz
License:  GNU General Public License 

LICENSE:
    Copyright (C) 2015 Max Brueggemann

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.
    
************************************************************************/

/** 
 *  @code #include <DS1621avr.h> @endcode
 * 
 *  @brief Interrupt-Free TWI library to get readings from Maxim Integrated's 
 *  DS1621 TWI temperature sensors.
 *
 *  Since the DS1621 can only perform about 2 conversions per seconds, it
 *  seemed to be a good idea to assign the TWI communication with 
 *  a rather low priority within the avr.
 *  
 *
 *  @author Max Brueggemann
 */

#define F_CPU 20000000L

/**
 * @brief    Highest sensor address. Also defines how many sensors
 *           are present on the bus. Set to 0 for single sensor
 *           operation with Address = 0.
 */
#define SensorTopAdr 7

/**
 * @brief    Configures the TWI. Call this function only once.
 */
extern void ds1621twi_init(void);

/**
 * @brief	Needs to be called whenever possible. Preferably in the
 *			main while. Do NOT call within ISRs! (or declare all variables 
 *			in ds1621avr.c volatile) :)
 */
extern void ds1621StateMachine(void);

/**
 * @brief   
 *			Pointer to the array you want this library to write its data to.
 *			The array will contain the temperature readings in two's complement.
 *          -862 means that the addressed sensor is not present on the bus.
 */
volatile int16_t *outputDataPtr;

/**
 * @brief    Call about every 100µs using a timer ISR.
 */
extern void tickTWItimeoutCounter(void);
