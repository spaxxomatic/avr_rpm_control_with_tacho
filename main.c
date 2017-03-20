
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/signal.h>
#include <avr/delay.h>

#include <stdlib.h>
#include <avr/eeprom.h>
#include <avr/pgmspace.h>	

#include "../lib/lcd.h"
	
#define F_CPU    8000000      //Interner Taktgenerator auf 8MHz gestellt
                              //FUSE-BIT CKSEL2 und CKSEL3 gesetzt

#define MAX_STEPS 8
#define TWI_SLAVE_ADDR 0xF0 


volatile uint8_t a=0; //Pulsverhältnis zu 240
uint8_t druckdauer = 0;
uint8_t iButton = 0;
uint8_t iPrevBtn = 0;
volatile uint8_t dbg_presc;

#define FULLWAvEFREQ 2
//steps ergibt sich aus der Steuerfrequenz geteilt durch Anzahl 0-Crosses (100 Hz für 50 Hz Netz)
#define STEPS FULLWAvEFREQ/100 

volatile uint8_t bSave;
volatile uint8_t iLevel;

volatile uint16_t tacho_ticks;
volatile uint16_t integrator;

volatile uint8_t cnt_phase;
volatile uint8_t tacho_phase;

uint8_t soll_freq; 

#define SETPRESCALER TCCR0B|=(1<<CS01) 
#define RESETPRESCALER TCCR0B|=0  

//debug defines
#define debug
#ifdef debug
	#define debug_zerocross
	//#define debug_remotepulse
	//#define debug_overflow  
#endif
 

#define TRIAC_ON PORTB &=~ _BV(PINB2)
#define TRIAC_OFF PORTB |= _BV(PINB2)

#define IN_TACHO PD0
#define TACHO_PULSE_ON bit_is_set(PIND, IN_TACHO)
#define TACHO_PULSE_OFF bit_is_clear(PIND, IN_TACHO)

#define IN_KEY_RPM_UP PD4
#define IN_KEY_RPM_DN PD5

#define KEY_RPM_UP bit_is_set(PIND, IN_KEY_RPM_UP)
#define KEY_RPM_DN  bit_is_set(PIND, IN_KEY_RPM_DN)

volatile uint16_t cnt_periode;
volatile uint16_t soll_periode;
volatile uint16_t ist_periode;

#define RPM_MAX 6000 
#define MIN_PERIODE 1000/(6000/60) //in ms, 10 ms for a max RPM of 6000
#define RPM_MIN 50 
#define MIN_PERIODE 1000/(50/60) //in ms, ex. 1000 ms for a min RPM of 60

#define SIG_TACHO SIG_INT1
 
SIGNAL(SIG_TACHO){ //interrupt handler for incoming tachometer raising edge
	//tacho_phase = cnt_phase;
	tacho_ticks ++;
	ist_periode = cnt_periode; //save value 
	cnt_periode = 0;
}

/*
	if TACHO_PULSE_ON {
		cnt_tacho ++;
	}
	
	if (ist_rpm > soll_rpm) {
		integrator ++;
	}else{
		integrator --;
	}

*/

SIGNAL (SIG_OVERFLOW0) //hi freq timer 10000 Hz, increments the phase counter and triggers the freq loop
{
	cli();
	cnt_periode ++;	 
	sei();   
}

volatile uint8_t triac_trigger_delay; 

SIGNAL (SIG_OUTPUT_COMPARE0A) //phase-delay trigger handler
{
	cli();
	static uint8_t divider = 0;
	if (divider >= triac_trigger_delay )
	{
		TIMSK &=~ (1<<OCIE0A); //disable this interrupt because it will be set in INT0
		TRIAC_ON;
		divider = 0;
	}else {divider ++;}
	sei();
}

#define DEF_OCR0A 9
#define PARAM_DELTA_THESHOLD_LOW 30 
//this param is the max delta between periode_ist and periode_soll. If the delta exceeds this param, the output will be 100% on (full throtle)

SIGNAL (SIG_INT0) //mains zero cross detector, the "regelschleife"
{ 	
	static int16_t delta;
	#ifdef debug_zerocross //let the dbg LED blink to visualise interupt presence
		static uint8_t cnt;	
		cnt++;
		if (cnt <= iLevel)
		{
			PORTB |= (1<<PB2);		
		}else{
			PORTB &=~  (1<<PB2);		
		}
		if (cnt >= MAX_STEPS ) cnt = 0;	
	#endif 	
	delta = ist_periode - soll_periode;
	if (delta < 0){ // too fast, stop pumping energy 
		TRIAC_OFF; 
	}else{
	// compute fill factor (pwm factor -> trigger delay) from the actual deviation from expected value
	  if (delta > PARAM_DELTA_THESHOLD_LOW){ // we're far away from target, so we go full throttle
		TRIAC_ON; //trigger triac 
	  }	else { // we're near target, reduce power 
		triac_trigger_delay = delta;
		//set the timer 0, triac will be triggered on timer expire 
        TCNT0 = 0; //reset timer register
        OCR0A = DEF_OCR0A;
        TIMSK |= (1<<OCIE0A); // enable compare interrupt , start counter OCR0 which will trigger the triac after the timer expires
	  }	
	}
}


void ioinit(void)
{
	
	
	PORTD = (1 << PD0) | (1 << PD1)| (1 << PD3) | (1 << PD4) | (1 << PD5); // PULLUP für inputs
	//DDRD = (1 << IN_TACHO) | (1 << IN_DK1) | (1 << IN_DK3) | (1 << IN_DK4);    // Eingänge aktivieren
	DDRB = (1 << PB0) | (1 << PB1) | (1 << PB2) | (1 << PB3) | (1 << PB4) ;    // Ausgänge aktivieren

	/*Vorteiler
	CS13 CS12 CS11 CS10
	 0    0    0    1   CK
	 0    0    1    0   CK/2
	 0    0    1    1   CK/4
	 0    1    0    1   CK/16
	 0    1    1    1   CK/64 
	*/

	//set interrupt mask
	//INT0 - zero cross detector
	PCMSK |= (1<<PIND3) | (1<<PIND2);
	MCUCR =  (1<<ISC11) | (1<<ISC10) | (1<<ISC01) | (1<<ISC00); //activate int0 and int1 on rising  edge
    //MCUCR =  (1<<ISC01) | (1<<ISC00); //activate int0 on rising  edge
	//MCUCR = (1<<ISC11) | (1<<ISC10);
	//TIMSK |= (1<<TOIE0); // enable overflow interrupt for counter 0 
    TIMSK |= (1<<OCIE0A); // enable compare interrupt for counter 0  
	GIMSK  |= (1<<INT1) | (1<<INT0);
	
	lcd_init(LCD_DISP_ON_CURSOR_BLINK);
	
	sei();
	

}

int userkey;
uint8_t soll_rpm;
char c_rpm[5] ;

void set_soll_periode(int8_t op){
	if(soll_rpm < RPM_MAX && soll_rpm > RPM_MIN){
    soll_rpm += op;
	soll_periode = 60000UL/soll_rpm; //in ms
	lcd_gotoxy(4,1);
	itoa( soll_rpm, c_rpm, 10);
	lcd_puts(c_rpm);
    lcd_gotoxy(7,1);
	lcd_puts("000 RPM");
	}
}

int main(void)
{
	iLevel = MAX_STEPS ; 
	userkey = 0;
	ioinit();
	lcd_gotoxy(0,1); lcd_puts("INIT");
//TWI_Slave_Initialise(TWI_SLAVE_ADDR);
   for(;;) 
  {
	if (KEY_RPM_UP) //user pressed RPM UP
	{
		set_soll_periode(1);
	}
	if (KEY_RPM_DN) //user pressed RPM DN
	{
		set_soll_periode(-1);
	}
	_delay_ms(255);
   }
   
}


