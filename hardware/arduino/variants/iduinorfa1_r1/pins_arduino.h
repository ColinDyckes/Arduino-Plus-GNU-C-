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
	SMeshLink 2012 Aug 10
*/


#ifndef Pins_Arduino_h
#define Pins_Arduino_h

#ifndef iduinorfa1
#define iduinorfa1
#endif


#include <avr/pgmspace.h>

#define NUM_DIGITAL_PINS            36
#define NUM_ANALOG_INPUTS           8
#define analogInputToDigitalPin(p)  ((p < 8) ? (p) + 14 : -1)
#define digitalPinHasPWM(p)         ((p) == 3 ||(p) == 5 ||(p) == 6 ||(p) == 8 ||(p) == 9 ||(p) == 22 ||(p) == 23) ||(p) == 35)

const static uint8_t SS   = 10;
const static uint8_t MOSI = 11;
const static uint8_t MISO = 12;
const static uint8_t SCK  = 13;

const static uint8_t SDA = 24;
const static uint8_t SCL = 25;
const static uint8_t LEDGREEN = 27;
const static uint8_t LEDYELLOW = 28;
const static uint8_t LEDRED = 29;

const static uint8_t A0 = 14;
const static uint8_t A1 = 15;
const static uint8_t A2 = 16;
const static uint8_t A3 = 17;
const static uint8_t A4 = 18;
const static uint8_t A5 = 19;
const static uint8_t A6 = 8;
const static uint8_t A7 = 9;

// A majority of the pins are NOT PCINTs, SO BE WARNED (i.e. you cannot use them as receive pins)
// Only pins available for RECEIVE (TRANSMIT can be on any pin):
// Pins:   10, 11, 12, 13, 20, 22, 23

#define digitalPinToPCICR(p)    ( (((p) >= 10) && ((p) <= 13)) || \
                                  ((p) == 22 || (p) == 23) ? (&PCICR) : ((uint8_t *)0) )

#define digitalPinToPCICRbit(p) ( 0 )

#define digitalPinToPCMSK(p)    ( ((((p) >= 10) && ((p) <= 13)) || ((p) == 20 ||  (p) == 22 || (p) == 23 )) ? (&PCMSK0) : \
                                 ((uint8_t *)0) )
//zigduino D10 use pb6 iduino use p0
/*#define digitalPinToPCMSKbit(p) ( ((p) == 23) ? 6 : \
                                ( ((p) == 8) ? 4 : \
                                ( ((p) == 9) ? 7 : \
                                ( ((p) == 10) ? 0 : \
                                ( ((p) == 11) ? 2 : \
                                ( ((p) == 12) ? 3 : \
                                ( ((p) == 13) ? 1 : \
                                ( ((p) == 22) ? 5 : \
                                0 ) ) ) ) ) ) ) )
 */
#define digitalPinToPCMSKbit(p) ( ((p) == 23) ? 6 : \
                                ( ((p) == 10) ? 0 : \
                                ( ((p) == 11) ? 2 : \
                                ( ((p) == 12) ? 3 : \
                                ( ((p) == 13) ? 1 : \
                                ( ((p) == 22) ? 5 : \
                                0 ) ) ) ) ) )

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
	PD	, // PD 2 ** 0 ** D0:RXD1:INT2
	PD	, // PD 3 ** 1 ** D1:TXD1:INT3	
	PE	, // PE 6 ** 2 ** D2:INT6:T3
	PE	, // PE 5 ** 3 ** D3:PWM3:INT5:Timer3 OUTPUTC
	PE	, // PE 2 ** 4 ** D4:AIN0:XCK0
	PE	, // PE 3 ** 5 ** D5:PWM5:AIN1:Timer3 OUTPUTA
	PE	, // PE 4 ** 6 ** D6:PWM6:INT4:Timer3 OUTPUTB
	PE	, // PE 7 ** 7 ** D7:INT7:ICP3:CLK0

	PF	, // PF 6 ** 8 ** D8:ADC6:TDO
	PF	, // PF 7 ** 9 ** D9:ADC7:TDI
	PB	, // PB 0 ** 10 ** D10:SPI_SSN:PCINT0
	PB	, // PB 2 ** 11 ** D11:SPI_MOSI:PCINT2
	PB	, // PB 3 ** 12 ** D12:SPI_MISO:PCINT3
	PB	, // PB 1 ** 13 ** D13:SPI_SCK:PCINT1

	PF	, // PF 0 ** 14 ** A0	
	PF	, // PF 1 ** 15 ** A1	
	PF	, // PF 2 ** 16 ** A2	
	PF	, // PF 3 ** 17 ** A3	
	PF	, // PF 4 ** 18 ** A4	
	PF	, // PF 5 ** 19 ** A5
	
	PB	, // PB 4 ** 20 ** D20:PCINT4:PCINT4:Timer2 OUTPUTA VCC_ENC_CONTROL LOW ENABLE
	PF	, // PB 7 ** 21 ** D21:PCINT7:LIS3DH INT
	PB	, // PB 5 ** 22 ** PCINT5:PWM:Timer1 OUTPUTA:SPI_CS_SD
	PB	, // PB 6 ** 23 ** PCINT6:PWM:Timer1 OUTPUTB:DS2411
	PD	, // PD 0 ** 24 ** I2C_SCL:INT0
	PD	, // PD 1 ** 25 ** I2C_SDA:INT1
	PD	, // PD 4 ** 26 ** ICP1:RF212_IRQ
	PD	, // PD 5 ** 27 ** SCK1:LED
	PD	, // PD 6 ** 28 ** T1:LED
	PD	, // PD 7 ** 29 ** T0:LED
	PE	, // PE 0 ** 30 ** USART0_RX:PCINT8:USB
	PE	, // PE 1 ** 31 ** USART0_TX:USB
	PG	, // PG 0 ** 32 ** RF212_SLP
	PG	, // PG 1 ** 33 ** RF212_RST
	PG	, // PG 2 ** 34 ** SPI_CS_ENC
	PG	, // PG 5 ** 35 ** VCC_SD_CONTROL LOW ENABLE
	
	
};

const uint8_t PROGMEM digital_pin_to_bit_mask_PGM[] = {
	// PIN IN PORT		
	// -------------------------------------------		
	_BV(2)	, // PD 2 ** 0 **
	_BV(3)	, // PD 3 ** 1 **
	_BV(6)	, // PE 6 ** 2 **
	_BV(5)	, // PE 5 ** 3 **
	_BV(2)	, // PE 2 ** 4 **
	_BV(3)	, // PE 3 ** 5 **
	_BV(4)	, // PE 4 ** 6 **
	_BV(7)	, // PE 7 ** 7 **
	_BV(6)	, // PF 6 ** 8 **
	_BV(7)	, // PF 7 ** 9 **
	_BV(0)	, // PB 0 ** 10 **
	_BV(2)	, // PB 2 ** 11 **
	_BV(3)	, // PB 3 ** 12 **
	_BV(1)	, // PB 1 ** 13 **
	_BV(0)	, // PF 0 ** 14 **
	_BV(1)	, // PF 1 ** 15 **
	_BV(2)	, // PF 2 ** 16 **
	_BV(3)	, // PF 3 ** 17 **
	_BV(4)	, // PF 4 ** 18 **
	_BV(5)	, // PF 5 ** 19 **
	
	_BV(4)	, // PB 4 ** 20 **
	_BV(7)	, // PB 7 ** 21 **
	_BV(5)	, // PB 5 ** 22 **
	_BV(6)	, // PB 6 ** 23 **
	_BV(0)	, // PD 0 ** 24 **
	_BV(1)	, // PD 1 ** 25 **
	_BV(4)	, // PD 4 ** 25 **
	_BV(5)	, // PD 5 ** 27 **
	_BV(6)	, // PD 6 ** 28 **
	_BV(7)	, // PD 7 ** 29 **
	_BV(0)	, // PE 0 ** 30 **
	_BV(1)	, // PE 1 ** 31 **
	_BV(0)	, // PG 0 ** 32 **
	_BV(1)	, // PG 1 ** 33 **
	_BV(2)	, // PG 2 ** 34 **
	_BV(5)	, // PG 5 ** 35 **
};

const uint8_t PROGMEM digital_pin_to_timer_PGM[] = {
	// TIMERS		
	// -------------------------------------------	
	NOT_ON_TIMER	, // PD 2 ** 0 **
	NOT_ON_TIMER	, // PD 3 ** 1 **
	NOT_ON_TIMER	, // PE 6 ** 2 **
	TIMER3C	, 		  // PE 5 ** 3 **
	NOT_ON_TIMER	, // PE 2 ** 4 **
	TIMER3A	, 		  // PE 3 ** 5 **
	TIMER3B	, 		  // PE 4 ** 6 **
	NOT_ON_TIMER	, // PE 7 ** 7 **
	NOT_ON_TIMER	, // PF 6 ** 8 **
	NOT_ON_TIMER	, // PF 7 ** 9 **
	NOT_ON_TIMER	, // PB 0 ** 10 **
	NOT_ON_TIMER	, // PB 2 ** 11 **
	NOT_ON_TIMER	, // PB 3 ** 12 **
	NOT_ON_TIMER	, // PB 1 ** 13 **
	NOT_ON_TIMER	, // PF 0 ** 14 **
	NOT_ON_TIMER	, // PF 1 ** 15 **
	NOT_ON_TIMER	, // PF 2 ** 16 **
	NOT_ON_TIMER	, // PF 3 ** 17 **
	NOT_ON_TIMER	, // PF 4 ** 18 **
	NOT_ON_TIMER	, // PF 5 ** 19 **
	NOT_ON_TIMER	, // PB4 ** 20 **
	NOT_ON_TIMER	, // PB7 ** 21 **
	NOT_ON_TIMER	, // PB 5 ** 22 **
	NOT_ON_TIMER	, // PB 6 ** 23 **
	NOT_ON_TIMER	, // PD 0 ** 24 **
	NOT_ON_TIMER	, // PD 1 ** 25 **
	NOT_ON_TIMER	, // PD 4 ** 26 **
	NOT_ON_TIMER	, // PD 5 ** 27 **
	NOT_ON_TIMER	, // PD 6 ** 28 **
	NOT_ON_TIMER	, // PD 7 ** 29 **
	NOT_ON_TIMER	, // PE 0 ** 30 **
	NOT_ON_TIMER	, // PE 1 ** 31 **
	NOT_ON_TIMER	, // PG 0 ** 32 **
	NOT_ON_TIMER	, // PG 1 ** 33 **
	NOT_ON_TIMER	, // PG 2 ** 34 **
	NOT_ON_TIMER	, // PG 5 ** 35 **
	
};	

#endif

#endif
