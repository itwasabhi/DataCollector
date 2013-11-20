//Abhishek Sriraman
//netID: as2587
//11/7/2013

//Written for Arduino Uno, Atmega328p
//This code was made to read the frequency of a Flow Sensor plugged in pin pD2

#define F_CPU 16000000UL
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>

double thresh_freq =20; //Minimum frequency to register data from Flow Sensor
int sampleFreq = 5; //Requested Sample Frequency
double alpha = 0.75; //Smoothing Coefficient; Time constant for exponential average

volatile double mainAvg;
bool firstVal;
unsigned int thresh_ticks;
volatile int pseudoScalar = 0;
int pseudoStart;

volatile bool FLOW_ERROR;

//Reads Timer1 Value
unsigned int TIM16_ReadTCNT1(void){
  unsigned char sreg;
  unsigned int i;
  sreg = SREG;
  cli();
  i = TCNT1;
  SREG = sreg;

  return i;
}

//Resets Timer1 Value
void TIM16_RESET(void){
  unsigned char sreg;
  sreg = SREG;
  cli();
  TCNT1 = 0;
  SREG = sreg;
}

//Adds a new value to an exponential average
void add_to_avg(double new_val){
  if(firstVal){
    mainAvg = new_val;
    firstVal = false;
  }
  else{
    mainAvg = alpha*new_val + (1-alpha)*mainAvg;
  }
}

//When Timer reaches the maximum value (based on threshold frequency)
ISR(TIMER1_COMPA_vect){
  FLOW_ERROR = true;
  add_to_avg(0.0);

}

//When input pin goes HIGH, 
ISR(INT0_vect){ 

  if(FLOW_ERROR){
    FLOW_ERROR= false;
  }
  else{
    volatile unsigned int curr_ticks = TIM16_ReadTCNT1();
    volatile double inst_freq = 1/((curr_ticks)*(.000004));
    add_to_avg(inst_freq);
  } 

  TIM16_RESET();
}

//Sample Frequency ISR:
ISR(TIMER0_COMPA_vect){
  if(pseudoScalar==1){
    //PRINT THE AVERAGE
    pseudoScalar = pseudoStart;
  }
  else{
    pseudoScalar = pseudoScalar-1;
  }
}

void init_sampleFq(void){
  pseudoStart = ceil((16000000.0/(1024*256))/sampleFreq);
  float aFq = 1/(0.016384*(pseudoStart));
  pseudoScalar = pseudoStart;
  //PRINT ACTUAL SAMPLE FREQUENCY. 
}

int main(void){
  thresh_ticks = (double) (1.0/thresh_freq)*(16000000/64); //12500 ticks at 20hz thresh
  firstVal = true;
  init_sampleFq();

  DDRD &= ~(1<<2); //Set PD2 as Input                             c

  //Timer 0 Setup:
  TCCR0A |= (1<<1); //xxxxxx10 "Mode 2"                           c
  TCCR0A &= ~(1<<0); //                                           c
  TCCR0B &= ~(1<<3); //xxxx0xxx "Mode 2"                          c
  TCCR0B |= (1<<2)|(1<<0); //xxxxxVVV Set Prescale on timer 0     c
  OCR0A = 0xFF; //Set to 255                                      c
  TIMSK0 |=(1<<1); //xxxxxx1x Enable Output Compare Match A       c

  //Timer 1 Setup:
  TCCR1B |= (1<<0)|(1<<1)|(1<<3);//Set Prescale on timer 1 to 64 and set to CTC
  OCR1A = thresh_ticks;
  TIMSK1 |= (1<<1);

  EIMSK |= 1; //Enable External Interrupt
  EICRA |= (1<<0) | (1<<1); //Enable Interrupt on INT0 on the Rising Edge: EICRA = xxxxxx11


  sei();

  while(1){
    //Loop Through
  }

}
