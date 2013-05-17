// Clock program for DX 32*8 LED Matrix + HT1632C + ATmega8; 
// DrJones 2012
//
// Clock with Tics for JY-MCU 3208 by Michael LeBlanc, NSCAD University
// http://generaleccentric.net
// March 2013
//
// Version 002 adds typography for letters
// Version 003 adds randomize
// Version 004 replaces manual brightness control with Child Safe (swearwords off) mode,
// corrects randomization functions and uses CdS cell on AD0 for auto brightness control
//
// button1: adjust time forward, keep pressed for a while for fast forward
// button2: adjust time backward, keep pressed for a while for fast backward
// button3: toggle "Child Safe" mode




#define F_CPU 1000000

#include <avr/pgmspace.h>
#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>
#include <avr/eeprom.h>

#include <stdlib.h>
#include <stdint.h>	       // needed for uint8_t

#define byte uint8_t
#define word uint16_t
   
unsigned int n;				/* for random number function */
int childsafe = 0;


PROGMEM byte bigdigits[10][6] = {
  {0b01111110,0b10000001,0b10000001,0b10000001,0b10000001,0b01111110},	// 0
  {0b00000000,0b00000000,0b10000010,0b11111111,0b10000000,0b00000000},	// 1
  {0b11100010,0b10010001,0b10010001,0b10001001,0b10001001,0b10000110},	// 2
  {0b01000001,0b10000001,0b10001001,0b10001001,0b10010101,0b01100011},	// 3
  {0b00110000,0b00101000,0b00100100,0b00100010,0b11111111,0b00100000},	// 4 
  {0b01001111,0b10001001,0b10001001,0b10001001,0b10001001,0b01110001}, 	// 5
  {0b01111110,0b10001001,0b10001001,0b10001001,0b10001001,0b01110000},	// 6 
  {0b00000001,0b00000001,0b11100001,0b00010001,0b00001001,0b00000111}, 	// 7
  {0b01110110,0b10001001,0b10001001,0b10001001,0b10001001,0b01110110},	// 8 
  {0b00001110,0b10010001,0b10010001,0b10010001,0b10010001,0b01111110}, 	// 9
};

PROGMEM byte letters[8][6] = {
  {0b00010000,0b10101000,0b10101000,0b10101000,0b10101000,0b01000000},	// s
  {0b11111111,0b00001000,0b00001000,0b00001000,0b00001000,0b11110000},	// h
  {0b00000000,0b00000000,0b00000000,0b11111010,0b00000000,0b00000000},	// i
  {0b00000000,0b00001000,0b01111110,0b10001000,0b10000000,0b01000000},	// t
  {0b00000000,0b00001000,0b11111110,0b00001001,0b00001001,0b00000010},	// f 
  {0b01111000,0b10000000,0b10000000,0b10000000,0b01000000,0b11111000}, 	// u
  {0b01110000,0b10001000,0b10001000,0b10001000,0b10001000,0b01000000},	// c 
  {0b11111111,0b00010000,0b00010000,0b00101000,0b01000100,0b10000000}, 	// k
};

PROGMEM byte control[5][6] = {
  {0b00000000,0b00111110,0b01000001,0b01000001,0b01000001,0b00100010},	// C
  {0b00100110,0b01001001,0b01001001,0b01001001,0b00110010,0b00000000},	// S
  {0b00000000,0b00111110,0b01000001,0b01000001,0b00111110,0b00000000},	// O
  {0b01111111,0b00000110,0b00001000,0b00110000,0b01111111,0b00000000},	// N
  {0b01111111,0b00001001,0b00000000,0b01111111,0b00001001,0b00000000},	// F 
};




//pins and macros

#define HTport   PORTB
#define HTddr    DDRB
#define HTstrobe 3
#define HTclk    4
#define HTdata   5

#define HTclk0    HTport&=~(1<<HTclk)
#define HTclk1    HTport|= (1<<HTclk)
#define HTstrobe0 HTport&=~(1<<HTstrobe)
#define HTstrobe1 HTport|= (1<<HTstrobe)
#define HTdata0   HTport&=~(1<<HTdata)
#define HTdata1   HTport|= (1<<HTdata)
#define HTpinsetup() do{  HTddr |=(1<<HTstrobe)|(1<<HTclk)|(1<<HTdata); HTport|=(1<<HTstrobe)|(1<<HTclk)|(1<<HTdata);  }while(0)
        // set as output and all high
 


#define key1 ((PIND&(1<<7))==0)
#define key2 ((PIND&(1<<6))==0)
#define key3 ((PIND&(1<<5))==0)
#define keysetup() do{ DDRD&=0xff-(1<<7)-(1<<6)-(1<<5); PORTD|=(1<<7)+(1<<6)+(1<<5); }while(0)  //input, pull up


byte leds[32];  //the screen array, 1 byte = 1 column, left to right, lsb at top. 


#define HTstartsys   0b100000000010 //start system oscillator
#define HTstopsys    0b100000000000 //stop sytem oscillator and LED duty    <default
#define HTsetclock   0b100000110000 //set clock to master with internal RC  <default
#define HTsetlayout  0b100001000000 //NMOS 32*8 // 0b100-0010-ab00-0  a:0-NMOS,1-PMOS; b:0-32*8,1-24*16   default:ab=10
#define HTledon      0b100000000110 //start LEDs
#define HTledoff     0b100000000100 //stop LEDs    <default
#define HTsetbright  0b100101000000 //set brightness b=0..15  add b<<1  //0b1001010xxxx0 xxxx:brightness 0..15=1/16..16/16 PWM
#define HTblinkon    0b100000010010 //Blinking on
#define HTblinkoff   0b100000010000 //Blinking off  <default
#define HTwrite      0b1010000000   // 101-aaaaaaa-dddd-dddd-dddd-dddd-dddd-... aaaaaaa:nibble adress 0..3F   (5F for 24*16)

//ADRESS: MSB first
//DATA: LSB first     transferring a byte (msb first) fills one row of one 8*8-matrix, msb left, starting with the left matrix
//timing: pull strobe LOW, bits evaluated at rising clock edge, strobe high
//commands can be queued: 100-ccccccccc-ccccccccc-ccccccccc-... (ccccccccc: without 100 at front)
//setup: cast startsys, setclock, setlayout, ledon, brightness+(15<<1), blinkoff



int ADCsingleREAD(uint8_t adctouse)
{
    int ADCval;

    ADMUX = adctouse;         		// use #1 ADC
    ADMUX |= (1 << REFS0);    		// use AVcc as the reference
    ADMUX &= ~(1 << ADLAR);   		// clear for 10 bit resolution
    
    ADCSRA|= (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0);    // 128 prescale for 8Mhz
    ADCSRA |= (1 << ADEN);    		// Enable the ADC

    ADCSRA |= (1 << ADSC);    		// Start the ADC conversion

    while(ADCSRA & (1 << ADSC));   	// Thanks T, this line waits for the ADC to finish 


    ADCval = ADCL;
    ADCval = (ADCH << 8) + ADCval;    // ADCH is read so ADC can be updated again

    return ADCval;
}


void HTsend(word data, byte bits) {  //MSB first
  word bit=((word)1)<<(bits-1);
  while(bit) {
    HTclk0;
    if (data & bit) HTdata1; else HTdata0;
    HTclk1;
    bit>>=1;
  }
}

void HTcommand(word data) {
  HTstrobe0;
  HTsend(data,12);
  HTstrobe1;
}

void HTsendscreen(void) {
  HTstrobe0;
  HTsend(HTwrite,10);
  for (byte mtx=0;mtx<4;mtx++)  	//sending 8x8-matrices left to right, rows top to bottom, MSB left
    for (byte row=0;row<8;row++) {  //while leds[] is organized in columns for ease of use.
      byte q=0;
      for (byte col=0;col<8;col++)  q = (q<<1) | ( (leds[col+(mtx<<3)]>>row)&1 ) ;
      HTsend(q,8);
    }
  HTstrobe1;
}


void HTsetup() {  //setting up the display
  HTcommand(HTstartsys);
  HTcommand(HTledon);
  HTcommand(HTsetclock);
  HTcommand(HTsetlayout);
  HTcommand(HTsetbright+(8<<1));
  HTcommand(HTblinkoff);
}

void HTbrightness(byte b) {
  HTcommand(HTsetbright + ((b&15)<<1) );
}

int gen_rand(void)
{
   unsigned int m;
   m = rand() % n;
   return(m);
}

//------------------------------------------------------------------------------------- CLOCK ------------------


volatile byte sec=5;
byte sec0=200, minute, hour, day, month; word year;


inline void clocksetup() {  // CLOCK, interrupt every second
  ASSR |= (1<<AS2);    //timer2 async from external quartz
  TCCR2=0b00000101;    //normal,off,/128; 32768Hz/256/128 = 1 Hz
  TIMSK |= (1<<TOIE2); //enable timer2-overflow-int
  sei();               //enable interrupts
}


// CLOCK interrupt
ISR(TIMER2_OVF_vect) {     //timer2-overflow-int
  sec++;
}



void incsec(byte add) {
  sec+=add;
  while (sec>=60) { 
    sec-=60;  minute++;
    while (minute>=60) {
      minute -= 60;  hour++;
      while (hour >=24) {
        hour-=24;  day++;
      }//24hours
    }//60min
  }//60sec
}

void decsec(byte sub) {
  while (sub>0) {
    if (sec>0) sec--; 
    else {
      sec=59; 
      if (minute>0) minute--; 
      else {
        minute=59; 
        if (hour>0) hour--;
        else {hour=23;day--;}
      }//hour
    }//minute
    sub--;
  }//sec
}

byte clockhandler(void) {
  if (sec==sec0) return 0;   //check if something changed
  sec0=sec;
  incsec(0);  //just carry over
  return 1;
}


//-------------------------------------------------------------------------------------- clock render ----------


void renderclock(void) {
  byte col=0;
  leds[col++]=0;	// add a 1 column space on the left
  for (byte i=0;i<6;i++) leds[col++]=pgm_read_byte(&bigdigits[hour/10][i]);
  leds[col++]=0;
  for (byte i=0;i<6;i++) leds[col++]=pgm_read_byte(&bigdigits[hour%10][i]);
  leds[col++]=0;
  if (sec%2) {leds[col++]=0b00000000;leds[col++ ]=0b00100100;} else {leds[col++]=0b00100100; leds[col++]=0b00000000;}
  leds[col++]=0;
  for (byte i=0;i<6;i++) leds[col++]=pgm_read_byte(&bigdigits[minute/10][i]);
  leds[col++]=0;
  for (byte i=0;i<6;i++) leds[col++]=pgm_read_byte(&bigdigits[minute%10][i]);
  leds[col++]=0;	// add a 1 column space on the right
} 

void rendershit(void) {
  byte col=0;
  leds[col++]=0; 	// make space on the left
  leds[col++]=0;
  leds[col++]=0;
  for (byte i=0;i<6;i++) leds[col++]=pgm_read_byte(&letters[0][i]);  	// s
  leds[col++]=0;	// add a little space between the letters
  leds[col++]=0;
  for (byte i=0;i<6;i++) leds[col++]=pgm_read_byte(&letters[1][i]);		// h
  leds[col++]=0;
  for (byte i=0;i<5;i++) leds[col++]=pgm_read_byte(&letters[2][i]);		// i
  leds[col++]=0;
  for (byte i=0;i<6;i++) leds[col++]=pgm_read_byte(&letters[3][i]);		// t
  leds[col++]=0;
  leds[col++]=0;
}

void renderfuck(void) {
  byte col=0;
  leds[col++]=0; 	// make space on the left
  leds[col++]=0;
  for (byte i=0;i<6;i++) leds[col++]=pgm_read_byte(&letters[4][i]);
  leds[col++]=0;
  for (byte i=0;i<6;i++) leds[col++]=pgm_read_byte(&letters[5][i]);
  leds[col++]=0;
  for (byte i=0;i<6;i++) leds[col++]=pgm_read_byte(&letters[6][i]);
  leds[col++]=0;
  for (byte i=0;i<6;i++) leds[col++]=pgm_read_byte(&letters[7][i]);
  leds[col++]=0;
  leds[col++]=0; 	// add padding on right
  leds[col++]=0;
}

void rendercs_on(void) {
  byte col=0;
  leds[col++]=0; 	// make space on the left
  for (byte i=0;i<6;i++) leds[col++]=pgm_read_byte(&control[0][i]);
  leds[col++]=0;
  for (byte i=0;i<6;i++) leds[col++]=pgm_read_byte(&control[1][i]);
  leds[col++]=0;
  leds[col++]=0;
  for (byte i=0;i<6;i++) leds[col++]=pgm_read_byte(&control[2][i]);
  for (byte i=0;i<6;i++) leds[col++]=pgm_read_byte(&control[3][i]);
  leds[col++]=0;
  leds[col++]=0;
  leds[col++]=0; 	// add padding on right
  leds[col++]=0;
}

void rendercs_off(void) {
  byte col=0;
  leds[col++]=0; 	// make space on the left
  for (byte i=0;i<6;i++) leds[col++]=pgm_read_byte(&control[0][i]);
  leds[col++]=0;
  for (byte i=0;i<6;i++) leds[col++]=pgm_read_byte(&control[1][i]);
  leds[col++]=0;
  leds[col++]=0;
  for (byte i=0;i<6;i++) leds[col++]=pgm_read_byte(&control[2][i]);
  for (byte i=0;i<6;i++) leds[col++]=pgm_read_byte(&control[4][i]);
  leds[col++]=0;
  leds[col++]=0;
  leds[col++]=0; 	// add padding on right
  leds[col++]=0;
}


byte changing, bright=3;

int main(void) {  //==================================================================== main ==================

  HTpinsetup();
  HTsetup();
  keysetup();
  clocksetup();

  for (byte i=0;i<32;i++) leds[i]=0b01010101<<(i%2);  HTsendscreen();


  hour=12;minute=00;

  while(1){ 

  bright = (ADCsingleREAD(0) - 80) / 62; 	// set the display brightness
  HTbrightness(bright);	

         if (key1) {if (changing>250) incsec(20); else {changing++; incsec(1);} }
    else if (key2) {if (changing>250) decsec(20); else {changing++; decsec(1);} }
 	else if (key3) {if (!changing) {
		changing=1; 
		if (childsafe == 0) {
			rendercs_on(); childsafe = 1;} else {rendercs_off(); childsafe = 0;}
		HTsendscreen(); 
		_delay_ms(2000);} //	wait 2 seconds before reverting to clock
	}
    else changing=0;

    if(clockhandler()) { renderclock(); HTsendscreen(); }
	if (childsafe == 0) {
		int shtfck = gen_rand();
		if((shtfck > 20) && (shtfck < 25)) { rendershit(); HTsendscreen(); _delay_ms(70); renderclock(); HTsendscreen(); }
		if((shtfck > 0) && (shtfck < 5)) { renderfuck(); HTsendscreen(); _delay_ms(70); renderclock(); HTsendscreen(); }
	}
  }
  return(0);
}//main
