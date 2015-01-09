/*
 * mm1acc.c
 *
 *  Created on: Jan 15, 2014
 *      Author: dirkjan
 */

#include <avr/io.h>
#include <avr/interrupt.h>
#include "mm1acc.h"

// Application specific defines
#define PORTINT0 PORTD
#define DDRINT0 DDRD
#define PININT0 PIND
#define PINT0 PD2

// Defaults
#define _GIMSK EIMSK
#define _MCUCR EICRA
#define _TIMSK TIMSK0
#define TIMER_OVF_vect TIMER0_OVF_vect
#define TIMER_COMPA_vect TIMER0_COMPA_vect

// Device specific defines:
#if defined( __AVR_ATtiny2313__ )

#define PORTINT0 PORTB
#define DDRINT0 DDRB
#define PININT0 PINB
#define PINT0 PB2

#define _GIMSK GIMSK
#define _MCUCR MCUCR
#define TIMER_OVF_vect TIMER0_OVF_vect
#define TIMER_COMPA_vect TIMER0_COMPA_vect
#define _TIMSK TIMSK
#endif

#if defined( __AVR_ATtiny24__ ) | \
defined( __AVR_ATtiny44__ ) | \
defined( __AVR_ATtiny84__ )

#define PORTINT0 PORTB
#define DDRINT0 DDRB
#define PININT0 PINB
#define PINT0 PB2

#define _GIMSK GIMSK
#define _MCUCR MCUCR
#define TIMER_OVF_vect TIM0_OVF_vect
#define TIMER_COMPA_vect TIM0_COMPA_vect
#define _TIMSK TIMSK0
#endif

volatile uint8_t newdata = 0;
volatile uint8_t bits = 0;
volatile uint8_t bit = 0;
volatile uint8_t byte = 0;
volatile uint8_t old_byte = 0;

acc_data data1;
acc_data data2;

volatile acc_data* current_data;
volatile acc_data* previous_data;
volatile acc_data* swap_data;

uint8_t period;

volatile state_t state = NONE;
volatile uint8_t dcc_buffer[MAX_DCC_BYTES];

volatile uint8_t overflow = 0;

#define data_buf GPIOR1

ISR(INT0_vect) {
	// Rising edge => reset timer
	TCCR0B |= (1 << CS01);
	period = TCNT0;
	TCNT0 = 0;
	//	if ((period < 200 ) | (period > 215)) // Invalid bit time (F_CPU = 16 MHz)
	if (overflow == 0 && (period > (99 * (F_CPU/8000000))) && (period < (108 * (F_CPU/8000000)))) // Valid MM1 bit time (F_CPU =  8 MHz)
	{
//		if (bits == 18) {
			state = MM1;
//		};
	} else if (overflow == 0 && (period > (108 * (F_CPU/8000000)) ) && (period < (123 * (F_CPU/8000000)))) { // DCC 1 bit time (F_CPU =  8 MHz)
		PIND = (1 << PD6); // PD6
		if (state == NONE) { // Beginning of DCC Preamble
			state = DCC_PREAMBLE;
			bits = 0;
			byte = 0;
			data_buf |= (1 << bits);
		} else if (state > DCC_VALID) {
			if (bits > 7 && state == DCC_BYTE) { // DCC Packet end bit;
				PIND = (1 << PD3); // PD6
				dcc_buffer[byte] = data_buf;
				data_buf = 0;
				bits = 0;
				old_byte = byte;
				state = DCC_VALID;
				PIND = (1 << PD3); // PD6
			} else { // Normal DCC Packet bit
				PIND = (1 << PD5);
				data_buf |= (1 << bits);
				bits++;
				PIND = (1 << PD5);
			};
		} else {
			state = NONE;
			bits = 0;
			PIND = (1 << PD6); // PD6
			overflow = 0;
			return;
		};
		PIND = (1 << PD6); // PD6
	} else if (overflow == 1 && (period > (70 * (F_CPU/8000000)) ) && (period < (120 * (F_CPU/8000000))))  {  // DCC 0 bit time (F_CPU =  8 MHz)
		PIND = (1 << PD4); // PD6
		if (state == DCC_PREAMBLE && bits > 6 && data_buf == 255) { // We have recieved the packet start bit after 8 preamble bits;
			state = DCC_BYTE;
			data_buf = 0;
			bits = 0;
			byte = 0;
			PIND = (1 << PD4); // PD6
			overflow = 0;
			return;
		} else if (state == DCC_BYTE) {
			if (bits > 7) { // && state == DCC_BYTE) ? // DCC byte stop bit
				dcc_buffer[byte] = data_buf;
				data_buf = 0;
				byte++;
				bits= 0;
				state = DCC_BYTE;
			} else { // normal databit
				PIND = (1 << PD5);
				bits++;
				PIND = (1 << PD5);
			}
		} else { //Should not happen
			state = NONE;
			bits = 0;
			byte = 0;
			data_buf = 0;
		}
		PIND = (1 << PD4); // PD6
	} else if (state == MM1) {
		state = NONE;
		bits = 0;
		overflow = 0;
		return;
	};
	overflow = 0;
	
#ifdef DEBUG
	PININT0 = (1 << PD4); // PD4
#endif
}

ISR(TIMER_OVF_vect) {
#if F_CPU == 16000000
#warning USING Overflow
	if (overflow == 0){
		TCNT0 = 0;
		overflow = 1;
		return;
	}
	overflow = 0;
#endif
	// Timer overflowed => we have a pause
	if (bits == 18 && state == MM1) { // New complete byte recieved
#ifdef DEBUG
		PIND = (1 << PD6); // PD6
#endif
		swap_data = previous_data;
		previous_data = current_data;
		current_data = swap_data;
		current_data->address = 0;
		current_data->port = 0;
		current_data->function = 0;
		newdata = 1;
		state = MM1_VALID;
#ifdef DEBUG
		PIND = (1 << PD6); // PD6
#endif
	};
	bits = 0;
	state = NONE;
	TCCR0B &= !(1 << CS01);
}

ISR(TIMER_COMPA_vect) {
#ifdef DEBUG
	PIND = (1 << PD5); // PD5
#endif
	if (state > MM1_VALID) // DCC, so we're checking length
		return;
	
	// Its 52 ms after a rising edge, we need to check the value of the INT0 line again
	bit = (PININT0 & (1 << PINT0)) ? 1 : 0;
	if ((bits < 8) & bit) { // Still in the address part
		current_data->address |= (bit << (7 - bits));
	} else if (bits > 15) {
		current_data->function = bit;
	} else {
		current_data->port |= (bit << ((bits/2) - 5));
	}
	bits++;
#ifdef DEBUG
	PIND ^= (1 << PD5); // PD5
#endif
}

void mm1acc_init() {
	/* Setup 8 bit timer
	 * =================
	 * We need a resolution of a couple of milliseconds, around 4 would be brilliant
	 * Setting the prescaler to /8 gives us 1 ms on 8 MHz
	 * On 16 MHz we could go to /64, giving 4 ms...
	 */
	// Setup the clock source, prescaler to /8 => CS00:02 = 2
	TCCR0A  = 0;
	TCCR0B |= (1 << CS01);
	
	// Initialize timer value
	TCNT0 = 0;
	
	// Setup the the output compare unit to fire mid-way a bit
	//	OCR0A = 104; // F_CPU = 16 000 000
	OCR0A = 52 * (F_CPU/8000000);  // F_CPU =  8 000 000
	
	// Setup the interrupt for the output compare unit
	// Setup the interrupt for the overflow
	_TIMSK = (1 << OCIE0A) | (1 << TOIE0);
	
	/* Setup rising edge interrupt on INT0
	 * ===================================
	 */
	_MCUCR = ((1 << ISC01) | (1 << ISC00));
	_GIMSK |= (1 << INT0);
	
	/* Setup data buffers
	 * ==================
	 */
	current_data = &data1;
	previous_data = &data2;
	
	/* Setup pin direction
	 * ===================
	 */
	
	// INT0 as input:
	DDRINT0 &= ~(1 << PINT0);
	
#ifdef DEBUG
	// PD4, 5, 6 (arduino pin 4, 5, 6) as output for debug
	DDRD |= 	 ((1<<PD4) | (1 <<PD5) | (1 << PD6));
#endif
}

state_t mm1acc_check(acc_data* mm1acc_data) {
	if (state == MM1_VALID) {
		/*		mm1acc_data->address = previous_data->address;
		 mm1acc_data->port = previous_data->port;
		 mm1acc_data->function = previous_data->function;*/
		*mm1acc_data = *previous_data;
		state = NONE;
		return MM1_VALID;
	}
	return state;
}

state_t dccacc_check(uint8_t* data, uint8_t* bytes) {
	if (state == DCC_VALID) {
		uint8_t i = 0;
		for (i; i <= old_byte; i++)
			data[i] = dcc_buffer[i];
		*bytes = old_byte;
		state = NONE;
		return DCC_VALID;
	}
	*data = data_buf;
	*bytes = byte;
	return state;
}

