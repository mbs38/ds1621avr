/*************************************************************************
Title:    A state machine based DS1621 library
Author:   Max Brueggemann
Hardware: any AVR with built-in TWI, tested on Atmega 88/168 at 20Mhz
License:  GNU General Public License 
          
DESCRIPTION:
    Refer to the header file ds1621avr.h.
    
USAGE:
    Refer to the header file ds1621avr.h.
                    
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
                        
*************************************************************************/

#include <avr/io.h>
#include <compat/twi.h>
#include "ds1621avr.h"

#define TWIActRequested 0x01
#define TWIDirection 0x02
#define TWIIssuingStartCond 0x04
#define TWIWritingOrReading 0x08
#define TWIfinished 0x10
#define TWIIssuingStopCond 0x20
#define TWIWritingAddress 0x40
#define TWIerror 0x80
#define DS1621_Write  0x90
#define DS1621_Read   0x91
#define DS1621_startOneShot 0xEE
#define DS1621_readTemperature 0xAA
#define DS1621_readCountRemain 0xA8
#define DS1621_readSlope 0xA9
#define TWItimeoutThreshold 30

#define SCL_CLOCK  100000L

#if defined(__AVR_ATmega168PA__)|(__AVR_ATmega88PA__)

#else
 #error "no TWI definitions for this MCU available"
#endif

uint16_t tempReadCounterA = 0;
uint8_t tempReadCounterB = 0;
int16_t temperature = 0x00;
volatile int16_t *outputDataPtr; 
uint8_t sensorCount = 0;
uint8_t TWIdataArray[2];
uint8_t DSStaMaStates = 1;
int16_t temperatureRawHighbyte=0x00;
int16_t count_Remain=0x00;
int16_t count_Per_C = 0;
volatile uint8_t twiTimeoutCounter = 0;
uint8_t twiState = 0;

void startCondition(void) {
	TWCR = (1<<TWINT) | (1<<TWSTA) | (1<<TWEN);
}

void stopCondition(void) {
	TWCR = (1<<TWINT) | (1<<TWEN) | (1<<TWSTO);
}

void tickTWItimeoutCounter(void) { 
	if ((twiTimeoutCounter>0) && (twiTimeoutCounter<255)) twiTimeoutCounter++;
}

void stopTWItimeoutCounter(void) {
	twiTimeoutCounter = 0;
}

void startRestartTWItimeoutCounter(void) {
	twiTimeoutCounter = 1;
}

void enterTWIerrorState(void) { /* In case of an error a Stop Condition has to be created */
	twiState=TWIerror|TWIIssuingStopCond;
	stopCondition();
}

void twiHandler(void) { /* the sequence is always: 1. Start Condition 2. Address+RW-Flag 3. Read one byte 4. Stop Condition */
	if (twiState)
	{
		uint8_t   twst = 0;
		
		if ((twiState&~TWIDirection) == TWIActRequested)
		{
			TWCR = 0x00;
			twiState|=TWIIssuingStartCond;
			startRestartTWItimeoutCounter();
			startCondition();
		}
		
		else if ((twiState&TWIIssuingStartCond) && (TWCR & (1<<TWINT))) /* doing write/read if Stop Condition was successful */
		{
			
			twst = TW_STATUS & 0xF8; /* read state, unmask prescaler bits */
			if ( (twst != TW_START) && (twst != TW_REP_START)) enterTWIerrorState(); else {
				/* no errors => send address */
				twiState&=~(TWIIssuingStartCond);
				twiState|=TWIWritingAddress;
				TWDR = TWIdataArray[0]; /* send device address */
				startRestartTWItimeoutCounter();
				TWCR = (1<<TWINT) | (1<<TWEN);
			}
		}
		
		else if ((twiState & TWIWritingAddress) && (TWCR & (1<<TWINT))) /* address has been written, ack has been received */
		{
			twst = TW_STATUS & 0xF8;
			if ( (twst != TW_MT_SLA_ACK) && (twst != TW_MR_SLA_ACK) ) enterTWIerrorState(); else {
				twiState&=~TWIWritingAddress;
				twiState|=TWIWritingOrReading;
				startRestartTWItimeoutCounter();
				if (!(twiState&TWIDirection)) TWDR = TWIdataArray[1]; /* read if TWIDirection bit is set */
				TWCR = (1<<TWINT) | (1<<TWEN); /* ReadNoAck */
			}
		}
		
		else if ((twiState & TWIWritingOrReading) && (TWCR & (1<<TWINT))) /* stop reading/writing of date */
		{
			twiState&=~TWIWritingOrReading;
			if (twiState & TWIDirection)
			{
				TWIdataArray[1]=TWDR;
			}
			twiState|=TWIIssuingStopCond;
			startRestartTWItimeoutCounter();
			stopCondition();
		}
		
		else if ((twiState&TWIIssuingStopCond) && !(TWCR & (1<<TWSTO))) /* end */
		{
			stopTWItimeoutCounter();
			if (!(twiState&TWIerror)) twiState=TWIfinished; /* There has been no error. */
			else twiState&=~TWIIssuingStopCond;
		}
		
		else if ((twiState&~(TWIfinished|TWIerror)) && (twiTimeoutCounter>TWItimeoutThreshold)) /* create Stop Condition if a timeout has occured */
		{
			enterTWIerrorState();
		}
	}
}


void sensorWrite(uint8_t sensorAdr) {
	TWIdataArray[0]=DS1621_Write | ((sensorAdr&0x07)<<1);
	twiState=TWIActRequested;
}

void sensorRead(uint8_t sensorAdr) {
	TWIdataArray[0]=DS1621_Read | ((sensorAdr&0x07)<<1);
	twiState=TWIActRequested|TWIDirection;
}

uint8_t getTWIendState(void) {
	return (twiState&(TWIfinished|TWIerror));
}


void ds1621twi_init(void)
{
	TWSR = 0;                         /* no prescaler */
	TWBR = ((F_CPU/SCL_CLOCK)-16)/2;
}

void ds1621StateMachine(void)
{
	twiHandler();
	if (DSStaMaStates == 1)
	{
		TWIdataArray[1]=DS1621_startOneShot;
		sensorWrite(sensorCount);
		DSStaMaStates = 2;
	}
	if (getTWIendState()) switch(DSStaMaStates) {
		case 2:
		{
			TWIdataArray[1]=DS1621_readTemperature;
			sensorWrite(sensorCount);
			DSStaMaStates = 3;
		}
		break;
		case 3:
		{
			sensorRead(sensorCount);
			DSStaMaStates = 4;
		}
		break;
		case 4:
		{
			temperatureRawHighbyte = TWIdataArray[1];
			if (temperatureRawHighbyte&(1<<7)) temperatureRawHighbyte|=0xFF00;
			TWIdataArray[1]=DS1621_readCountRemain;
			sensorWrite(sensorCount);
			DSStaMaStates = 5;
		}
		break;
		case 5:
		{
			sensorRead(sensorCount);
			DSStaMaStates = 6;
		}
		break;
		case 6:
		{
			count_Remain = 100 * TWIdataArray[1];
			TWIdataArray[1] = DS1621_readSlope;
			sensorWrite(sensorCount);
			DSStaMaStates = 7;
		}
		break;
		case 7:
		{
			sensorRead(sensorCount);
			DSStaMaStates = 8;
		}
		break;
		case 8:
		{
			count_Per_C = TWIdataArray[1];
			*(outputDataPtr+sensorCount) = ((temperatureRawHighbyte * 100) -25 + ( (count_Per_C*100 - count_Remain)/count_Per_C ))/10; /* Let's do some fixed point arithmetics */
			twiState = 0;
			DSStaMaStates = 1;
			sensorCount++;
			if (sensorCount>SensorTopAdr) {
				sensorCount=0;
				DSStaMaStates=1; /* Stop TWI */
				tempReadCounterA = 1; /* Restart the timer */
			}
		}
	}
}
