// ***********************************************************
// Project:
// Author:
// Module description:
// ***********************************************************

//                                                                 ___________
//		                                                          |           |
//		PortCB BitX (control)------------------------------Orange-|  SERVO X  |
//		                                                          |           |
//		Servo Supply (+5V)------------------------------------Red-| Tower Pro |
//		                                                          |   SG-50   |
//		Ground -----o-----------/\/\/\/\/-------------o-----Brown-|  I<300mA  |
//		            |              1 Ohm              |           |___________|
//		            |    ||                           |
//		            \----||---------o----/\/\/\/\/----/  Note: No sense ckt for
//		                 ||0.1 uF   |    100K Ohm              Servos 7 thru B.
//		                            |                          Brown connected
//		PortA BitX (sense)----------/                          directly to Gnd
//

//========================= SERVO CONTROL LEAD TIMING =========================
//
// All servos at maximum (counter-clockwise):
//
//      |<---------------- 20 mS --------------->|
//      |                                     ->||<-- 0.8 mS to spare
//  0 __XXXXX____________________________________XXXXX_________________________
//  1 _______XXXXX____________________________________XXXXX____________________
//  2 ____________XXXXX____________________________________XXXXX_______________
//  3 _________________XXXXX____________________________________XXXXX__________
//  4 ______________________XXXXX____________________________________XXXXX_____
//  5 ___________________________XXXXX____________________________________XXXXX
//  6 ________________________________XXXXX____________________________________
//  7 _____________________________________XXXXX_______________________________
//  8 __XXXXX____________________________________XXXXX_________________________
//  9 _______XXXXX____________________________________XXXXX____________________
//  A ____________XXXXX____________________________________XXXXX_______________
//  B _________________XXXXX____________________________________XXXXX__________
//
// Various positions:
//
//      |<---------------- 20 mS --------------->|
//  0 __XXX______________________________________XXX___________________centered
//  1 _____X________________________________________X___________________minimum
//  2 ______XXXXX____________________________________XXXXX______________maximum
//  3 ___________XX_______________________________________XXX________increasing
//  4 _____________XX________________________________________X_______decreasing
//  9 ________________________________________________________disabled(limit=0)
//  6 _________________X________________________________________XX_____________
//  7 __________________X_________________________________________X____________
//  8 __XXXXX____________________________________XXXXX_________________________
//  9 _______XXXXX____________________________________XXXXX____________________
//  A ____________XXXXX____________________________________XXXXX_______________
//  B _________________XXXXX____________________________________XXXXX__________
//
//      Note: The interrupt which ends the ServoB pulse _can_ occur after the
//          interrupt ending the Servo7 pulse (which also increments "tocks").


#include <avr/io.h>
#include <avr/eeprom.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>

const char firmwareID[] PROGMEM = "StaggeringServoJumble V0.02";
const char prompt[] PROGMEM = "\r\n> ";

#define FOSC 12000000			// Clock Speed
#define BAUD 19200
#define UBRR FOSC/16/BAUD-1
#define MAX_TIMER_COUNT 30000			// timer ticks for 20 ms  (@12MHz/8)
#define PULSE_MIN_COUNT  900			//                0.6 ms
#define PULSE_MAX_COUNT 3600			//                2.4 ms (*8 = 19.2)
#define CENTER_COUNT    2250			//                1.5 ms
#define MAX_SENSE_LIMIT 255

#define SERVOS_PER_LEG	3
#define NUMBER_LEGS		4
#define NUMBER_SERVOS 	(SERVOS_PER_LEG*NUMBER_LEGS)

enum { HIP, KNEE, ROTATE };	// joint defines

#define SERVO(joint,leg) (((joint)*NUMBER_LEGS)+(leg))

struct leg_position_tag {
	int8_t joint[SERVOS_PER_LEG];
	} leg_position[10];
EEMEM struct leg_position_tag eeleg_position[10];

volatile int count[NUMBER_SERVOS];		// pulse width (position) in timer ticks
volatile int target[NUMBER_SERVOS];		// target position
volatile uint8_t rate[NUMBER_SERVOS];	// speed - add to count until target
volatile uint8_t sense[NUMBER_SERVOS];	// current sensed (varies with load & speed)
volatile uint8_t limit[NUMBER_SERVOS];	// current limit - 0 disables servo
int offset[NUMBER_SERVOS];				// 0th order correction for each joint
EEMEM int eeoffset[NUMBER_SERVOS];		// non-volitile storage

volatile uint8_t servoCA;				// servo for which pulse is being generated
int waitcountCA;						// loaded into timer 1 compare register A
volatile uint8_t tocks;					// incremented at 50/sec


ISR(TIMER1_COMPA_vect) {

	PORTC = 0;	// end active pulse
	DDRA = 0;	// stop any sense voltage dump
	uint8_t servo = servoCA + 1;
	if (servo >= 8) {
		tocks++;
		if (tocks == 25) {	// 1/2 second
			PORTD ^= 1<<PD4;	// blink LED
			tocks = 0;
			}
		servoCA = 255;
		// set up to wait for end of 20 millisecond period
		OCR1A = MAX_TIMER_COUNT - 1;
		TCCR1B |= 8;	// clear timer on match
		waitcountCA = 0;
	} else {
		servoCA = servo;
		if (limit[servo]) PORTC |= 1<<servo;	// start next pulse
		// start current sense conversion for this servo
		// channel will already be set
		ADCSRA = 0xCF;		// no need to read-modify-write
		// conversion MUST complete before minimum pulse end
		// because that's where the interrupt for the end is set.
		// Shouldn't be an issue - 13/93Khz=0.14mS < 0.6ms
	}
}

ISR(ADC_vect) {

	uint8_t servo = servoCA;
	sense[servo] = ADCL;
	if (ADCH) sense[servo] = 0xFF;	// overflow at 255mV
	// begin dump of this channel's sense voltage (for rest of servo pulse)
	DDRA |= 1<<servo;
	// compute and set interrupt for end of servo pulse
	int cnt = count[servo];
	int tgt = target[servo];
	if ( /* sense[servo]<limit[servo] && */ cnt!=tgt) {
		// advance count toward target
		if (cnt<tgt) {
			cnt += rate[servo];
			if (cnt>tgt) cnt = tgt;
		} else {
			cnt -= rate[servo];
			if (cnt<tgt) cnt = tgt;
		}
		count[servo] = cnt;
	} else rate[servo] = 0;
	waitcountCA += cnt;
	OCR1A = waitcountCA;
	TCCR1B &= 0xF7;	// do not clear timer on match
	// set channel for next servo
	servo = (servo + 1) & 7;
	ADMUX = ( 0xC0 | servo );
}

uint8_t servoB;
int waitcountB;

ISR(TIMER1_COMPB_vect) {

	PORTB = 0;		// end pulse
	uint8_t servo = servoB + 1;
	if (servo >= 4) {
		servo = 255;
		// set up to wait for end of 20 millisecond period
		waitcountB = 600;		// offset start to keep ints from constantly overlapping
	} else {
		if (limit[8+servo]) PORTB = 1<<servo;	// start next pulse
		// compute and set interrupt for end of servo pulse
		int cnt = count[8+servo];
		int tgt = target[8+servo];
		if ( cnt!=tgt) {
			// advance count toward target
			if (cnt<tgt) {
				cnt += rate[8+servo];
				if (cnt>tgt) cnt = tgt;
			} else {
				cnt -= rate[8+servo];
				if (cnt<tgt) cnt = tgt;
			}
			count[8+servo] = cnt;
		} else rate[8+servo] = 0;
		waitcountB += cnt;
	}
	servoB = servo;
	OCR1B = waitcountB;
}

static inline uint8_t rx_rdy(void) {

	return UCSR0A & (1<<RXC0);
}

static inline uint8_t getch(void) {

	while (!(UCSR0A & (1<<RXC0)));
	return UDR0;
}

static inline void putch(uint8_t c) {

	while (!(UCSR0A & (1<<UDRE0)));
	UDR0 = c;
}

static int getint(void) {

	int x = 0;
	uint8_t c;
	while ( (c=getch()) <= '9' ) {
		if ( c == '-' ) x = -x;
		if ( c < '0' ) break;
		putch(c);
		x = (10*x) + c - '0';
	}
	putch(c);
	return x;
}

static void putint(int i) {

	if (i<0) {
		putch('-');
		i = -i;
	}
	if (i>=10) putint(i/10);
	putch('0' + (i%10));
}

void putstr_P(const char *pstr) {

	uint8_t i;
	uint8_t c;
	for (i = 0; (c=pgm_read_byte(&pstr[i]))!=0; i++) putch(c);
}

static void move( uint8_t s, int pos, int time ) {

	if (s & 1) pos = -pos;		// change direction for odd legs
	if (s & 2) pos = -pos;		// and again for rear legs
	pos = (pos<<4) + CENTER_COUNT + offset[s];
	if (pos > PULSE_MAX_COUNT) pos = PULSE_MAX_COUNT;
	if (pos < PULSE_MIN_COUNT) pos = PULSE_MIN_COUNT;
	int delta = count[s] - pos;
	if (delta < 0) delta = -delta;
	delta /= time;
	if (delta <= 0) delta = 1;
	if (delta > 255) delta = 255;
	target[s] = pos;
	rate[s] = delta;
}

void moveLeg( uint8_t leg, uint8_t pos, int time ) {

	move( HIP*NUMBER_LEGS+leg, leg_position[pos].joint[HIP], time );
	move( KNEE*NUMBER_LEGS+leg, leg_position[pos].joint[KNEE], time );
	move( ROTATE*NUMBER_LEGS+leg, leg_position[pos].joint[ROTATE], time );
}

uint8_t notdone(uint8_t leg) {

	return rate[HIP*NUMBER_LEGS+leg] | rate[KNEE*NUMBER_LEGS+leg] | rate[ROTATE*NUMBER_LEGS+leg];
}

uint8_t notalldone(void) {

	uint8_t s = NUMBER_SERVOS, x = 0;

	while (s--) {
		x |= rate[s];
	}
	return x;
}

void store(void) {

	eeprom_write_block( offset, &eeoffset,
							NUMBER_SERVOS*sizeof(int) );
	eeprom_write_block( leg_position, &eeleg_position,
							10*sizeof(struct leg_position_tag) );
}

void recall(void) {

	eeprom_read_block( offset, &eeoffset,
							NUMBER_SERVOS*sizeof(int) );
	eeprom_read_block( leg_position, &eeleg_position,
							10*sizeof(struct leg_position_tag) );
}

int main(void) {

	uint8_t c,i,j;
	uint8_t movetime = 48;	// in tocks

	//initialize UART
	UBRR0 = UBRR;
	UCSR0B = (1<<RXEN0)|(1<<TXEN0);
	UCSR0C = 3<<UCSZ00;

	// set up LED on PD4
	DDRD |= 1<<PD4;

	// set up ADC for servo current sensing
	DIDR0 = 0xFF;		// Digital input disable all of PORTC
	PORTA = 0;
	DDRC = 0;
	ADMUX = 0xC0;		// 1.1Vref, right-adjust, channel=0
	ADCSRA = 0x8F;		// ADC enabled, noAuto, intEnabled, FOSC/128->93.75KHz

	// set up servos
	PORTB &= 0xF0;		// clear servo bits
	PORTC = 0;
	DDRB |= 0x0F;	// set servo bits to output
	DDRC = 0xFF;
	recall();								// recall offsets
	i = NUMBER_SERVOS;
	while (i--) {
		count[i] = CENTER_COUNT;
		target[i] = CENTER_COUNT;
		rate[i] = 0;
		limit[i] = 1;
	}

	waitcountCA = 0;
	servoCA = 255;  // ready to start clock interrupts
	OCR1A = 0;  // next count (0) generates interrupt
	// enable the timer1 output compare A & B interrupt
	TIMSK1 = (1<<OCIE1A) | (1<<OCIE1B);
	sei();
	TCCR1B = 2;			// start timer1 ( CK/8 )

	// small delay to let XBee startup
	while (!tocks);
	while (tocks);
	
	putstr_P(firmwareID);

	while (1) {
		putstr_P(prompt);
		c = getch();
		putch(c);
	   	switch (c) {
	   	case 's':	// status
	   		for ( c = 0; c < NUMBER_SERVOS; c++ ) {
	   			putch('\r');
	   			putch('\n');
	   			putch('\t');
	   			putint(c);
	   			putch('\t');
	   			putint(offset[c]);
	   			putch('\t');
	   			putint(count[c]);
	   			putch('\t');
	   			putint(target[c]);
	   			putch('\t');
	   			putint(rate[c]);
	   			putch('\t');
	   			putint(limit[c]);
	   			putch('\t');
	   			putint(sense[c]);
	   		}
	   		break;
	   	case 'p':	// pose all legs in leg_position %1
	   		c = getch();
	   		putch(c);
	   		c -= '0';
	   		if (c>=10) break;
            moveLeg(0,c,movetime);
            moveLeg(1,c,movetime);
            moveLeg(2,c,movetime);
            moveLeg(3,c,movetime);
	   		break;
	   	case 'P': {	// pose all legs as hip%1,knee%2,rotate%3 and store in leg_pos%4
				int hip = getint();
				int knee = getint();
				int rot = getint();
				move(0,hip,movetime);
				move(1,hip,movetime);
				move(2,hip,movetime);
				move(3,hip,movetime);
				move(4,knee,movetime);
				move(5,knee,movetime);
				move(6,knee,movetime);
				move(7,knee,movetime);
				move(8,rot,movetime);
				move(9,rot,movetime);
				move(10,rot,movetime);
				move(11,rot,movetime);
				c = getch();
				putch(c);
				c -= '0';
				if (c>=10) break;
				leg_position[c].joint[HIP] = hip;
				leg_position[c].joint[KNEE] = knee;
				leg_position[c].joint[ROTATE] = rot;
			}
	   		break;
        case 'w':	// walk until keyin
            while (!rx_rdy()) {
				c = (c+1)&3;			// phase
				for (i=0; i<4; i++) {	// leg phase - 3 is for the leg being lifted
					// figure which leg
					j = ( i + ( c ? (c==3?1:c+1) : 0 ) ) & 3;
					if (i!=3) {		// simple move
						if (!(c&1)) {
							if ( i==1 ) moveLeg(j,3,movetime);
							else moveLeg(j,1,movetime);
						}
						else if ( (c+i==1) || (c+i==5) ) moveLeg(j,0,movetime);
							else moveLeg(j,2,movetime);
					} else { 		// retrace
						uint8_t t = movetime/3;
						if (j&2) {		// rear leg
							moveLeg(j,5,t);
							while (notdone(j)) if (rx_rdy()) break;
							moveLeg(j,4,t);
							while (notdone(j)) if (rx_rdy()) break;
							moveLeg(j,3,t);
						} else {		// front leg
							moveLeg(j,4,t);
							while (notdone(j)) if (rx_rdy()) break;
							moveLeg(j,5,t);
							while (notdone(j)) if (rx_rdy()) break;
							moveLeg(j,0,t);
						}
					}
				}
                while (notalldone()) if (rx_rdy()) break;
            }
            break;
        case 'W':	// sway side to side until keyin
			c = 0;
            while (!rx_rdy()) {
				c ^= 1;			// phase
				for (j=0; j<4; j++) {	// leg
					moveLeg(j,((j&1)^c)?8:9,movetime);
				}
                while (notalldone()) if (rx_rdy()) break;
            }
            break;
	   	case 'o':	// set servo%1 offset = %2
	   		c = getch();
	   		putch(c);
	   		c -= '0';
	   		if (c>=NUMBER_SERVOS) break;
	   		offset[c] = getint();
			break;
	   	case 't':	// set movetime = %1
	   		movetime = getint();
			break;
	   	case 'e':
	   		store();		// store offsets & positions
	   		break;
	   	case 'r':
	   		recall();	// recall offsets & positions
	   		break;
	   	case '0':	// zero all servos
	   		c = NUMBER_SERVOS;
	   		while (c--) {
	   			move( c, 0, movetime );
	   		}
	   		break;
		case 'v':	// version
			putstr_P(firmwareID);
			break;
		case '>':	// non-echoing control mode
			c = 0;
			while (!c) {
				putch('.');
				if ((i=getch()) == ' ') break;	// exit mode without error
				switch (i) {
				case 'M':	// set servo motions: mn(spt)... number(servo,pos,time)
					if ( (c=getch()) >= NUMBER_SERVOS ) break;
					for (; c; c--) {
						if ( (i=getch()) >= NUMBER_SERVOS ) {
							c = 1;
							break;
						}
						int8_t pos = getch();	// for sign ext and arg order
						move( i, pos, getch());
					}
					break;
				case 'w':	// wait all done
					while (notalldone()) if (rx_rdy()) { c=1; break; }
					break;
				default:
					c = 1;
				}
				if (c) putch('?');
			}
			break;
	   	default:
	   		putch('?');
	   	}
	}
}

