/*_____________________________________________________________________________
 │                                                                            |
 │ COPYRIGHT (C) 2022 Mihai Baneu                                             |
 │                                                                            |
 | Permission is hereby  granted,  free of charge,  to any person obtaining a |
 | copy of this software and associated documentation files (the "Software"), |
 | to deal in the Software without restriction,  including without limitation |
 | the rights to  use, copy, modify, merge, publish, distribute,  sublicense, |
 | and/or sell copies  of  the Software, and to permit  persons to  whom  the |
 | Software is furnished to do so, subject to the following conditions:       |
 |                                                                            |
 | The above  copyright notice  and this permission notice  shall be included |
 | in all copies or substantial portions of the Software.                     |
 |                                                                            |
 | THE SOFTWARE IS PROVIDED  "AS IS",  WITHOUT WARRANTY OF ANY KIND,  EXPRESS |
 | OR   IMPLIED,   INCLUDING   BUT   NOT   LIMITED   TO   THE  WARRANTIES  OF |
 | MERCHANTABILITY,  FITNESS FOR  A  PARTICULAR  PURPOSE AND NONINFRINGEMENT. |
 | IN NO  EVENT SHALL  THE AUTHORS  OR  COPYRIGHT  HOLDERS  BE LIABLE FOR ANY |
 | CLAIM, DAMAGES OR OTHER LIABILITY,  WHETHER IN AN ACTION OF CONTRACT, TORT |
 | OR OTHERWISE, ARISING FROM,  OUT OF OR IN CONNECTION WITH THE SOFTWARE OR  |
 | THE USE OR OTHER DEALINGS IN THE SOFTWARE.                                 |
 |____________________________________________________________________________|
 |                                                                            |
 |  Author: Mihai Baneu                           Last modified: 10.Dec.2022  |
 |                                                                            |
 |___________________________________________________________________________*/

// redefine the desired frequency to 1 MHz
#undef F_CPU
#define F_CPU 1000000UL

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>

#define MIN_PULSE_WIDTH       544     // the shortest pulse sent to a servo  
#define MAX_PULSE_WIDTH      2400     // the longest pulse sent to a servo 

typedef enum {
    loop_move_servo_1 = 0,
    loop_move_servo_2,
    loop_hold,
    loop_break,
    loop_end
} servo_loop_state_t;

typedef struct {
    servo_loop_state_t type;
    union {
        uint16_t start;
        uint16_t duty_servo_1;
    };
    union {
        uint16_t end;
        uint16_t duty_servo_2;
    };
    int16_t inc;
    uint16_t delay;
    uint16_t hold;
} servo_state_t;

void delay_ms(int __ms)
{
	double __tmp ;

	uint16_t __ticks;
	__tmp = ((F_CPU) / 4e3) * __ms;
	if (__tmp < 1.0)
		__ticks = 1;
	else if (__tmp > 65535)
	{
		//	__ticks = requested delay in 1/10 ms
		__ticks = (uint16_t) (__ms * 10.0);
		while(__ticks)
		{
			// wait 1/10 ms
			_delay_loop_2(((F_CPU) / 4e3) / 10);
			__ticks --;
		}
		return;
	}
	else
		__ticks = (uint16_t)__tmp;
	_delay_loop_2(__ticks);
}

void setup_1mhz_clock()
{
    cli();
    CLKPR = 0b10000000;
    CLKPR = 0b00000100;   // 0100 = 16 MHz
    sei();
}

void setup_heart_beat()
{
    // set PB5 as output
    DDRB |= _BV(DDB5);
    PORTB |= _BV(PORTB5);

    // set timer 0 as heartbeat
    TCCR0A = 0b00000010;
    TCCR0B = 0b00000101;  // 1MHz / 1024
    OCR0A  = 244;
    TIMSK0 = 0b00000010;
}

ISR(TIMER0_COMPA_vect)
{
   if (bit_is_set(PINB, PINB5)) {
       PORTB &= ~_BV(PORTB5);
   } else {
       PORTB |= _BV(PORTB5);
   }
}

void setup_servos()
{
    // set PB1 and PB2 as output
    DDRB |= (_BV(DDB1) | _BV(DDB2));

    TCCR1A = 0b10100010;
    TCCR1B = 0b00011001;   // 1MHz / 1
    ICR1   = 1000*20;      // TOP
    OCR1A  = MIN_PULSE_WIDTH;
    OCR1B  = MIN_PULSE_WIDTH;
}

void setup_switch()
{
    // set PD2 as input
    DDRD &= ~_BV(DDD2);
    PORTD |= _BV(PORTD2);
}

static const servo_state_t servo_state_vector[] = 
{
    { loop_move_servo_1,   MIN_PULSE_WIDTH, MAX_PULSE_WIDTH,  1000, 1, 2000},
    { loop_move_servo_2,   MIN_PULSE_WIDTH, MAX_PULSE_WIDTH,  1000, 1, 2000},
    { loop_hold,           MAX_PULSE_WIDTH, MAX_PULSE_WIDTH,     0, 0, 1000},
    { loop_move_servo_1,   MAX_PULSE_WIDTH, MIN_PULSE_WIDTH, -1000, 1, 2000},
    { loop_move_servo_2,   MAX_PULSE_WIDTH, MIN_PULSE_WIDTH, -1000, 1, 2000},
    { loop_break,                        0,               0,     0, 0, 1000},

    { loop_move_servo_1,   MIN_PULSE_WIDTH, MAX_PULSE_WIDTH,     1, 1, 1000},
    { loop_move_servo_2,   MIN_PULSE_WIDTH, MAX_PULSE_WIDTH,     1, 1, 1000},
    { loop_hold,           MAX_PULSE_WIDTH, MAX_PULSE_WIDTH,     0, 0, 1000},
    { loop_move_servo_1,   MAX_PULSE_WIDTH, MIN_PULSE_WIDTH,    -1, 2, 1000},
    { loop_move_servo_2,   MAX_PULSE_WIDTH, MIN_PULSE_WIDTH,    -1, 2, 1000},
    { loop_end,                          0,               0,     0, 0, 1000},
};
static uint16_t servo_state_index = 0;

int execute_state(const servo_state_t *state)
{
    int go_next = 0;
    uint16_t duty_cycle;

    switch (state->type)
    {
        case loop_move_servo_1:
            duty_cycle = state->start;
            OCR1A  = duty_cycle;
            if (state->inc > 0) {
                while (duty_cycle < state->end) {
                    OCR1A  = duty_cycle;
                    duty_cycle += state->inc;
                    delay_ms(state->delay);
                }
            } else {
                while (duty_cycle > state->end) {
                    OCR1A  = duty_cycle;
                    duty_cycle += state->inc;
                    delay_ms(state->delay);
                }
            }
            OCR1A  = state->end;
            go_next = 1;
            servo_state_index++;
            break;

        case loop_move_servo_2:
            duty_cycle = state->start;
            OCR1B  = duty_cycle;
            if (state->inc > 0) {
                while (duty_cycle < state->end) {
                    OCR1B  = duty_cycle;
                    duty_cycle += state->inc;
                    delay_ms(state->delay);
                }
            } else {
                while (duty_cycle > state->end) {
                    OCR1B  = duty_cycle;
                    duty_cycle += state->inc;
                    delay_ms(state->delay);
                }
            }
            OCR1B = state->end;
            go_next = 1;
            servo_state_index++;
            break;

        case loop_hold:
            OCR1A = state->duty_servo_1;
            OCR1B = state->duty_servo_2;
            go_next = 1;
            servo_state_index++;
            break;

        case loop_break:
            go_next = 0;
            servo_state_index++;
            break;

        case loop_end:
            go_next = 0;
            servo_state_index = 0;
            break;
        
        default:
            break;
    }

    delay_ms(state->hold);
    return go_next;
}

int main()
{
    setup_1mhz_clock();
    setup_heart_beat();
    setup_servos();
    setup_switch();

    servo_state_index = 0;
    while (1) {
        if (!(PIND & _BV(PORTD2))) {
            while (execute_state(servo_state_vector + servo_state_index)) {
            }
        }
        delay_ms(10);
    }
}
