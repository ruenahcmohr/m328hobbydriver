/*

avr @ 16Mhz
9600 baud

 SERVO 0 ON B0
 1 ON       B1
 2 ON       D2
 3 ON       D3
 4 ON       D4
 5 ON       D5
 6 ON       D6
 7 ON       D7

packet:
[255][servo #][MSB][LSB]
where msb:lsb <=24000

pulse 0.5ms -> 2ms
update 50Hz

-----

servo number 

[ AAAB BCZZ ]
A = servo address, 0-7
B = servo board,   0-3
C = cache flag     0 = immediate, 1 = cached
Z = unused         at 9600 baud, there is only time to update 31 servos

treat AAABB as a single number.

*/
#include <avr/io.h>
#include "usart.h"
#include "binary.h"
#include <avr/interrupt.h>


// ** due to optimizations, don't use values other than these ***

// servos 0-7
#define BASEADDRESS 0

// servos 8-15
//#define BASEADDRESS 8

// servos 16-23
//#define BASEADDRESS 16

// servos 23-31
//#define BASEADDRESS 24



#define OUTPUT  1
#define INPUT   0

#define SetBit(BIT, PORT)     PORT |= (1<<BIT)
#define ClearBit(BIT, PORT)   PORT &= ~(1<<BIT)
#define IsHigh(BIT, PORT)    (PORT & (1<<BIT)) != 0
#define IsLow(BIT, PORT)     (PORT & (1<<BIT)) == 0
#define InBoundsI(v, l, h)        ((v) >= (h)) ? (0) : ((v) <= (l)) ? (0) : (1)


#define stopCounter()  TCCR1B = b00011000;
#define startCounter() TCCR1B = b00011001;

void countersInit();


volatile unsigned int position[8];
volatile unsigned char idx;

int main( void ) {
 
  unsigned char pkt[2];
  unsigned char ptr;
  unsigned int temp;
  unsigned char data;
  unsigned int cache[8];

    // set up directions 
  DDRB = (OUTPUT << PB0 | OUTPUT << PB1 | INPUT << PB2 |INPUT << PB3 |INPUT << PB4 |INPUT << PB5 |INPUT << PB6 |INPUT << PB7);
  DDRD = (INPUT << PD0 | INPUT << PD1 |OUTPUT << PD2 |OUTPUT << PD3 |OUTPUT << PD4 |OUTPUT << PD5 |OUTPUT << PD6 |OUTPUT << PD7);        

  ptr = 0;
  USART_Init( 103 );
  countersInit();
  idx = 7;
  pkt[0] = pkt[1] = 0;

  for (temp = 0; temp < 8; temp++) { position[temp] = 16000;  cache[temp] = 16000; }
  
  sei();
  
  while(1) {
      data = USART_Receive();
      if (0) {
      } else if (ptr == 0) {
        if (data == 255) ptr++;
      } else if (ptr == 1) {
        pkt[0] = data;        
        if (InBoundsI((data & 31), BASEADDRESS, (BASEADDRESS+7)) || (data == 0xFF) ){  // if this is for us, carry on
          ptr++;
        } else {
          ptr = 20;   // otherwise, ignore the rest of the packet
        }
      } else if (ptr == 2) {
        pkt[1] = data;
        ptr++;
      } else if (ptr == 3) {
          ptr = 0;
          temp = pkt[1];
          temp = (temp<<8);
          temp = temp | data;      
          if (temp > 31000) temp = 31000;
        if (pkt[0] < 32) {
          // immediate move                              
          position[(pkt[0] & 0x07)] = temp;
        } else if ( pkt[0] < 64) {
          // cache position
          cache[(pkt[0] & 0x07)] = temp;
        } else if (pkt[0] == 0xFF) {
          // enguage cached positions
          position[0] = cache[0];
          position[1] = cache[1];
          position[2] = cache[2];
          position[3] = cache[3];
          position[4] = cache[4];
          position[5] = cache[5];
          position[6] = cache[6];
          position[7] = cache[7];
        }
      } else if (ptr == 20) {  // ignoring data
        ptr++;
      } else if (ptr == 21) { // ignoring data
        ptr = 0;
      }
      
      
                    
  }
  
}

// 16 bit T/C 0
//   variable .5ms to 2ms  counter 1
ISR( TIMER1_COMPA_vect ) {  

  PORTB = 0x00;
  PORTD = 0x00;
  stopCounter();
  
}


//   8 bit T/C
//   every 2.5ms  counter 2
ISR( TIMER0_COMPA_vect) {
  
  idx++;
  if (idx == 8) idx = 0;
  
  OCR1A = 8000 + position[idx];  
  
  TCNT1 = 0;
  
  if (idx < 2) {
   SetBit(idx, PORTB);  
  } else {
   SetBit(idx, PORTD);
  }
  
  startCounter();

}


void countersInit() {

  // counter 1A is used to time out the pwm pulse, a count of 8000 produces the min pulse duration.
  // counter 1A ( 16 bit)  to fast pwm, no output control,  interrupt enabled, clock source off.
  // 
  // fast pwm: WGM = 1111   cs off: CS = 000 
  
  TCCR1A = b00000000; // [ COM1A1 | COM1A0 | COM1B1 | COM1B0 |       |        | WGM11   | WGM10  ]
  TCCR1B = b00001000; // [ ICNC1  | ICES1  |        | WGM13  | WGM12 | CS12   | CS11    | CS10   ]
  TIMSK1 = b00000010; // [        |        | ICIE1  |        |       | OCIE1B | OCIE1A  | TOIE1  ]
  
  
  
  // counter 0A is used to start pwm cycles
  // counter 0A ( 8 bit ) to CTC cdiv=256 ocr0A = 156, interrupt enabled, clock source on.
  // 
  // CTC: WGM = 010 cdiv=256: CS=100 
  
  TCCR0A = b00000010;  // [ COM0A1 | COM0A0 | COM0B1 | COM0B0 |       |      | WGM01 | WGM00 ]
  TCCR0B = b00000100;  // [ FOC0A  | FOC0B  |        |        | WGM02 | CS02 | CS01  | CS00  ]
  
  OCR0A  = 156;  

  TIMSK0  = b00000010; // [        |        |        |        |       | OCIE0B | OCIE0A  | TOIE0  ]
}


































