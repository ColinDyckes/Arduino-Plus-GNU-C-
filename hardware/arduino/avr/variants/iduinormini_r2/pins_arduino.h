/*
  pins_arduino.h - Pin definition functions for Arduino
  Part of Arduino - http://www.arduino.cc/

  Copyright (c) 2007 David A. Mellis

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General
  Public License along with this library; if not, write to the
  Free Software Foundation, Inc., 59 Temple Place, Suite 330,
  Boston, MA  02111-1307  USA

  $Id: wiring.h 249 2007-02-03 16:52:51Z mellis $
 */

/*
	This version of pins_arduino.h is for the iduino r1
	SMeshLink 2014 Aug 10
 */


#ifndef Pins_Arduino_h
#define Pins_Arduino_h

#ifndef iduinorfa1
#define iduinorfa1
#endif


#include <avr/pgmspace.h>
#define NUM_DIGITAL_PINS 35
#define NUM_ANALOG_INPUTS 7
#define analogInputToDigitalPin(p) (((p) < 6) ? 24-(p)  : \
		((p) == 6) ? 30 : \
				((p) == 5) ? 19 :-1)
#define digitalPinHasPWM(p) ((p) == 12 ||(p) == 13 ||(p) == 15 ||(p) == 16 ||(p) == 17 ||(p) == 28)

const static uint8_t SS = 8;
const static uint8_t MOSI = 10;
const static uint8_t MISO = 11;
const static uint8_t SCK = 9;

const static uint8_t SDA = 4;
const static uint8_t SCL = 3;

const static uint8_t LED = 25;
const static uint8_t LED1 = 25;
const static uint8_t LED2 = 26;
const static uint8_t LED3 = 27;

const static uint8_t A0 = 24;
const static uint8_t A1 = 23;
const static uint8_t A2 = 22;
const static uint8_t A3 = 21;
const static uint8_t A4 = 20;
const static uint8_t A5 = 19;
const static uint8_t A6 = 30;

const static uint8_t RFTX = 26;
const static uint8_t RFRX = 27
		;
//const static uint8_t BATMON = 35;
#define LEDGREEN 25
#define LEDYELLOW 26
#define LEDRED 27
// A majority of the pins are NOT PCINTs, SO BE WARNED (i.e. you cannot use them as receive pins)
// Only pins available for RECEIVE (TRANSMIT can be on any pin):
// Pins:  8, 9, 10, 11, 12, 13, 28,32 ,33

#define digitalPinToPCICR(p) ( ((((p) >= 8) && ((p) <= 13)) || \
		((p) == 28) || ((p) == 32) || ((p) == 33)) ? \
				(&PCICR) : ((uint8_t *)0) )

#define digitalPinToPCICRbit(p) ( ((p) == 33) ? 1 : 0 )

#define digitalPinToPCMSK(p) ( ((((p) >= 8) && ((p) <= 13)) || ((p) == 28) || ((p) == 32)) ? (&PCMSK0) : \
		( ((p) == 33) ? (&PCMSK1) : \
				((uint8_t *)0) ) )

#define digitalPinToPCMSKbit(p) ( ((p) == 8) ? 0 : \
		( ((p) == 9) ? 1 : \
				( ((p) == 10) ? 2 : \
						( ((p) == 11) ? 3 : \
								( ((p) == 12) ? 4 : \
										( ((p) == 13) ? 7 : \
												( ((p) == 28) ? 6 : \
														( ((p) == 32) ? 5 : \
																( ((p) == 33) ? 0 : \
																		0 ) ) ) ) ) ) ) ))

#ifdef ARDUINO_MAIN

const uint16_t PROGMEM port_to_mode_PGM[] = {
		NOT_A_PORT,
		NOT_A_PORT,
		(uint16_t)&DDRB,
		NOT_A_PORT,
		(uint16_t)&DDRD,
		(uint16_t)&DDRE,
		(uint16_t)&DDRF,
		(uint16_t)&DDRG,
		NOT_A_PORT,
		NOT_A_PORT,
		NOT_A_PORT,
		NOT_A_PORT,
		NOT_A_PORT,
};

const uint16_t PROGMEM port_to_output_PGM[] = {
		NOT_A_PORT,
		NOT_A_PORT,
		(uint16_t)&PORTB,
		NOT_A_PORT,
		(uint16_t)&PORTD,
		(uint16_t)&PORTE,
		(uint16_t)&PORTF,
		(uint16_t)&PORTG,
		NOT_A_PORT,
		NOT_A_PORT,
		NOT_A_PORT,
		NOT_A_PORT,
		NOT_A_PORT,
};

const uint16_t PROGMEM port_to_input_PGM[] = {
		NOT_A_PIN,
		NOT_A_PIN,
		(uint16_t)&PINB,
		NOT_A_PIN,
		(uint16_t)&PIND,
		(uint16_t)&PINE,
		(uint16_t)&PINF,
		(uint16_t)&PING,
		NOT_A_PIN,
		NOT_A_PIN,
		NOT_A_PIN,
		NOT_A_PIN,
		NOT_A_PIN,
};

const uint8_t PROGMEM digital_pin_to_port_PGM[] = {
		// PORTLIST
		// -------------------------------------------
		PG	, // PG 0 ** 0 **
		PG	, // PG 2 ** 1 **
		PG	, // PG 5 ** 2 **
		PD	, // PD 0 ** 3 **  	SCL
		PD	, // PD 1 ** 4 **	SDA
		PD	, // PD 2 ** 5 ** 	RXD1
		PD	, // PD 3 ** 6 ** 	TXD1
		PD	, // PD 4 ** 7 **
		PB	, // PB 0 ** 8 ** 	SPI_SS
		PB	, // PB 1 ** 9 ** 	SPI_CLK
		PB	, // PB 2 ** 10 **  SPI_MOSI
		PB	, // PB 3 ** 11 **  SPI_MISO

		PB	, // PB 4 ** 12 **  PWM
		PB	, // PB 7 ** 13 **  PWM
		PE	, // PE 2 ** 14 **
		PE	, // PE 3 ** 15 **  PWM
		PE	, // PE 4 ** 16 **  PWM
		PE	, // PE 5 ** 17 **  PWM·Ö
		PE	, // PE 6 ** 18 **
		PF	, // PF 5 ** 19 **  ADC5
		PF	, // PF 4 ** 20 **  ADC4
		PF	, // PF 3 ** 21 **  ADC3
		PF	, // PF 2 ** 22 **  ADC2
		PF	, // PF 1 ** 23 **  ADC1
		PF ,  // PF 0 ** 24 **  ADC0

		PD ,  // PD 5 ** 25 **   LED1
		PD	, // PD 6 ** 26 ** 	 LED2
		PD ,  // PD 7 ** 27 **   LED3
		PB	, // PB 6 ** 28 **   DS2411
		PG	, // PG 1 ** 29 **   Tempature Rsistor Power
		PF	, // PF 6 ** 30 **   Tempature Rsistor ADC6
		PE	, // PE 7 ** 31 **   User Key
		PB	, // PB 5 ** 32 **   LIS3DH INT
		PE	, // PE 0 ** 33 **  USART0_RX CP2104
		PE	, // PE 1 ** 34 **  USART0_TX CP2104
//		PF	, // PF 7 ** 35 **  not connect
//      PG	, // PG 3 ** 36 ** 32.768
//		PF ,  // PG 4 ** 37 ** 32.8768



};

const uint8_t PROGMEM digital_pin_to_bit_mask_PGM[] = {
		// PORTLIST
				// -------------------------------------------
		_BV( 0 )	, // PG 0 ** 0 **
		_BV( 2 )	, // PG 2 ** 1 **
		_BV( 5 )	, // PG 5 ** 2 **
		_BV( 0 )	, // PD 0 ** 3 **  	SCL
		_BV( 1 )	, // PD 1 ** 4 **	SDA
		_BV( 2 )	, // PD 2 ** 5 ** 	RXD1
		_BV( 3 )	, // PD 3 ** 6 ** 	TXD1
		_BV( 4 )	, // PD 4 ** 7 **
		_BV( 0 )	, // PB 0 ** 8 ** 	SPI_SS
		_BV( 1 )	, // PB 1 ** 9 ** 	SPI_CLK
		_BV( 2 )	, // PB 2 ** 10 **  SPI_MOSI
		_BV( 3 )	, // PB 3 ** 11 **  SPI_MISO

		_BV( 4 )	, // PB 4 ** 12 **
		_BV( 7 )	, // PB 7 ** 13 **
		_BV( 2 )	, // PE 2 ** 14 **
		_BV( 3 )	, // PE 3 ** 15 **
		_BV( 4 )	, // PE 4 ** 16 **
		_BV( 5 )	, // PE 5 ** 17 **
		_BV( 6 )	, // PE 6 ** 18 **
		_BV( 5 )	, // PF 5 ** 19 **  ADC5
		_BV( 4 )	, // PF 4 ** 20 **  ADC4
		_BV( 3 )	, // PF 3 ** 21 **  ADC3
		_BV( 2 )	, // PF 2 ** 22 **  ADC2
		_BV( 1 )	, // PF 1 ** 23 **  ADC1
		_BV( 0 )	,  // PF 0 ** 24 **  ADC0

		_BV( 5 ) 	,  // PD 5 ** 25 **   LED1
		_BV( 6 )	, // PD 6 ** 26 ** 	 LED2
		_BV( 7 ) 	,  // PD 7 ** 27 **   LED3
		_BV( 6 )	, // PB 6 ** 28 **   DS2411
		_BV( 1 )	, // PG 1 ** 29 **   Tempature Rsistor Power
		_BV( 6 )	, // PF 6 ** 30 **   Tempature Rsistor ADC
		_BV( 7 )	, // PE 7 ** 31 **   User Key
		_BV( 5 )	, // PB 5 ** 32 **   LIS3DH INT
		_BV( 0 )	, // PE 0 ** 33 **  USART0_RX CP2104
		_BV( 1 )	, // PE 1 ** 34 **  USART0_TX CP2104
		//		PF	, // PF 7 ** 35 **  not connect
		//      PG	, // PG 3 ** 36 ** 32.768
		//		PF ,  // PG 4 ** 37 ** 32.8768

};

const uint8_t PROGMEM digital_pin_to_timer_PGM[] = {
		// TIMERS
		// -------------------------------------------
		NOT_ON_TIMER	, // PG 0 ** 0 **
		NOT_ON_TIMER	, // PG 2 ** 1 **
		NOT_ON_TIMER	, // PG 5 ** 2 **
		NOT_ON_TIMER	, // PD 0 ** 3 **  	SCL
		NOT_ON_TIMER	, // PD 1 ** 4 **	SDA
		NOT_ON_TIMER	, // PD 2 ** 5 ** 	RXD1
		NOT_ON_TIMER	, // PD 3 ** 6 ** 	TXD1
		NOT_ON_TIMER	, // PD 4 ** 7 **
		NOT_ON_TIMER	, // PB 0 ** 8 ** 	SPI_SS
		NOT_ON_TIMER	, // PB 1 ** 9 ** 	SPI_CLK
		NOT_ON_TIMER	, // PB 2 ** 10 **  SPI_MOSI
		NOT_ON_TIMER	, // PB 3 ** 11 **  SPI_MISO

		TIMER2A	, // PB 4 ** 12 **  PWM
		TIMER1C	, // PB 7 ** 13 **  PWM
		NOT_ON_TIMER	, // PE 2 ** 14 **
		TIMER3A			, // PE 3 ** 15 **  PWM
		TIMER3B			, // PE 4 ** 16 **  PWM
		TIMER3C			, // PE 5 ** 17 **  PWM
		NOT_ON_TIMER	, // PE 6 ** 18 **
		NOT_ON_TIMER	, // PF 5 ** 19 **  ADC5
		NOT_ON_TIMER	, // PF 4 ** 20 **  ADC4
		NOT_ON_TIMER	, // PF 3 ** 21 **  ADC3
		NOT_ON_TIMER	, // PF 2 ** 22 **  ADC2
		NOT_ON_TIMER	, // PF 1 ** 23 **  ADC1
		NOT_ON_TIMER 	,  // PF 0 ** 24 **  ADC0

		NOT_ON_TIMER ,  // PD 5 ** 25 **   LED1
		NOT_ON_TIMER	, // PD 6 ** 26 ** 	 LED2
		NOT_ON_TIMER ,  // PD 7 ** 27 **   LED3
		TIMER0B	, // PB 6 ** 28 **   DS2411
		NOT_ON_TIMER	, // PG 1 ** 29 **   Tempature Rsistor Power
		NOT_ON_TIMER	, // PF 6 ** 30 **   Tempature Rsistor ADC
		NOT_ON_TIMER	, // PE 7 ** 31 **   User Key
		NOT_ON_TIMER	, // PB 5 ** 32 **   LIS3DH INT
		NOT_ON_TIMER	, // PE 0 ** 33 **  USART0_RX CP2104
		NOT_ON_TIMER	, // PE 1 ** 34 **  USART0_TX CP2104
		//		PF	, // PF 7 ** 35 **  not connect
		//      PG	, // PG 3 ** 36 ** 32.768
		//		PF ,  // PG 4 ** 37 ** 32.8768

};	

#endif

#endif
