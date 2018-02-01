#include "avr/pgmspace.h" // PROGMEM and pgm_read_byte_near()
#include "avr/io.h"

#define REFCLK 3923 // TODO: measure me and adjust
#define STEP_FREQ 0.065

// output frequency
#define FREQ_MIN    25   
#define FREQ_MAX    50   

// slope time (s)
#define UP_TIME     10
#define DOWN_TIME   10

#define DEADTIME    2   // us 

// pins for 3 phases: Alpha, Bravo, Charlie, and trigger for CRO.
// ATMega 168 / 328 valid PWM pins are 3,5,6,9,10,11. Non-default values have
// not been tested

#define PWM_UH 6    //OC0A   
#define PWM_UL 5    //OC0B

#define PWM_VH 9    //OC1A
#define PWM_VL 10   //OC1B

#define PWM_WH 11   //OC2A
#define PWM_WL 3    //OC2B

#define TRIG   7     // default:  7

#define ONOFF 2

/*******************************************************************************
 * Should not need to configure anything below here ***************************/

/*
 * LUT for sin(t) generated (by spreadsheet) using this formula:
 * x = ROUND( (255/2) * (1+SIN((2*PI()) * (t/256))) )
 *
 * where 0 <= t <= 255 ; keeping one period of sine in nvram
 *
 * Keeping LUT is way faster than calculating all the time
 */
const byte sin256[] PROGMEM =
{
//  0   1   2   3   4   5   6   7   8   9   a   b   c   d   e   f
  128,131,134,137,140,143,146,149,152,155,158,162,165,167,170,173, // 00:0f
  176,179,182,185,188,190,193,196,198,201,203,206,208,211,213,215, // 10:1f
  218,220,222,224,226,228,230,232,234,235,237,238,240,241,243,244, // 20:2f
  245,246,248,249,250,250,251,252,253,253,254,254,254,255,255,255, // 30:3f
  255,255,255,255,254,254,254,253,253,252,251,250,250,249,248,246, // 40:4f
  245,244,243,241,240,238,237,235,234,232,230,228,226,224,222,220, // 50:5f
  218,215,213,211,208,206,203,201,198,196,193,190,188,185,182,179, // 60:6f
  176,173,170,167,165,162,158,155,152,149,146,143,140,137,134,131, // 70:7f
  128,124,121,118,115,112,109,106,103,100, 97, 93, 90, 88, 85, 82, // 80:8f
   79, 76, 73, 70, 67, 65, 62, 59, 57, 54, 52, 49, 47, 44, 42, 40, // 90:9f
   37, 35, 33, 31, 29, 27, 25, 23, 21, 20, 18, 17, 15, 14, 12, 11, // a0:af
   10,  9,  7,  6,  5,  5,  4,  3,  2,  2,  1,  1,  1,  0,  0,  0, // b0:bf
    0,  0,  0,  0,  1,  1,  1,  2,  2,  3,  4,  5,  5,  6,  7,  9, // c0:cf
   10, 11, 12, 14, 15, 17, 18, 20, 21, 23, 25, 27, 29, 31, 33, 35, // d0:df
   37, 40, 42, 44, 47, 49, 52, 54, 57, 59, 62, 65, 67, 70, 73, 76, // e0:ef
   79, 82, 85, 88, 90, 93, 97,100,103,106,109,112,115,118,121,124  // f0:ff
};

// phase offsets (n * 255 / 3)
#define U_OFFSET 0   // (0x00)
#define V_OFFSET 85  // (0x55)
#define W_OFFSET 170 // (0xaa)
//#define W_OFFSET 128 // (0x80)

// variables in ISR 
volatile double TUN_K; // tuning coefficient       
volatile int i1s;

void setup(){
  pinMode(PWM_UH, OUTPUT);
  pinMode(PWM_UL, OUTPUT);
  pinMode(PWM_VH, OUTPUT);
  pinMode(PWM_VL, OUTPUT);
  pinMode(PWM_WH, OUTPUT);
  pinMode(PWM_WL, OUTPUT);
  pinMode(TRIG, OUTPUT);
  pinMode(ONOFF, INPUT); 
  setup_timers(); // this also disables interrupts
}

void loop(){ 
  static byte freq = 0; // output frequency  
  byte onoff = 0;
  byte freqStep = 0;

  if(i1s>3922){
    onoff = digitalRead(ONOFF);
    if(onoff&&freq<FREQ_MAX){
      if(freq<FREQ_MIN) freq = FREQ_MIN;
      else{
        freqStep = (FREQ_MAX - FREQ_MIN) / (UP_TIME);
        freq+=freqStep;
      }
    }
    if(!onoff&&freq>0){
      if(freq<=FREQ_MIN) freq = 0;
      else{
        freqStep = (FREQ_MAX - FREQ_MIN) / (DOWN_TIME);
        freq-=freqStep;
      }
    }
    i1s = 0;
  }
  TUN_K = freq*STEP_FREQ;
  
}


/**
 * Set prescaler to 1, PWM mode to phase correct PWM
 * @see https://www.arduino.cc/en/Tutorial/SecretsOfArduinoPWM
 * @see http://www.atmel.com/images/Atmel-8271-8-bit-AVR-Microcontroller-ATmega48A-48PA-88A-88PA-168A-168PA-328-328P_datasheet_Complete.pdf
 * @see datasheet section 15.7.4 "Phase Correct PWM Mode"
 */
void setup_timers(){
  OCR0A = 0;
  OCR0B = 255;
  OCR1A = 0;
  OCR1B = 255;
  OCR2A = 0;
  OCR2B = 255;
  
  /*** Timer 0 ***/
  /* Timer 0 Clock Prescaler to no prescaling (CS12:0 = 001)
  * Section 16.11.2 Table 16-5 */
  TCCR0B &= ~(_BV(CS02));
  TCCR0B |= _BV(CS01);
  TCCR0B &= ~(_BV(CS00));
  
  /* Timer 0 Set WGM to Phase Correct PWM 8 bit (WGM0 = 001)
  * Section 16.11.1, Table 16-4 */
  TCCR0B &= ~(_BV(WGM02));
  TCCR0A &= ~(_BV(WGM01));
  TCCR0A |= _BV(WGM00);
  
  /* Timer 0 Set Compare Output Mode (COM0A = 11 ; COM0B = 10)
  * Section 16.11.1, Table 16-2 */
  TCCR0A &= ~(_BV(COM0A0));
  TCCR0A |= _BV(COM0A1);
  TCCR0A |= _BV(COM0B1) | _BV(COM0B0);
  
  /*** Timer 1 ***/
  /* Timer 1 Clock Prescaler to 8 (CS12:0 = 010)
  * Section 16.11.2 Table 16-5 */
  TCCR1B &= ~(_BV(CS12));
  TCCR1B |= _BV(CS11);
  TCCR1B &= ~(_BV(CS10));
  /* Timer 1 Set WGM to Phase Correct PWM 8 bit (WGM13:0 = 0001)
  * Section 16.11.1, Table 16-4 */
  TCCR1B &= ~(_BV(WGM13));
  TCCR1B &= ~(_BV(WGM12));
  TCCR1A &= ~(_BV(WGM11));
  TCCR1A |= _BV(WGM10);
  
  /* Timer 1 Set Compare Output Mode (COM1A = 11 ; COM1B = 10)
  * Section 16.11.1, Table 16-2 */
  TCCR1A &= ~(_BV(COM1A0));
  TCCR1A |= _BV(COM1A1);
  TCCR1A |= _BV(COM1B1) | _BV(COM1B0);
  /*** Timer 2 ***/
  /* Timer 2 Clock Prescaler to 8 (CS22:0 = 010)
  * Section 18.11.2 */
  TCCR2B &= ~(_BV(CS22));
  TCCR2B |= _BV(CS21);
  TCCR2B &= ~(_BV(CS20));
  /* Timer2 Set WGM to Phase Correct PWM (WGM = 001)
  * Section 18.7.4 */
  TCCR2B &= ~(_BV(WGM22));
  TCCR2A &= ~(_BV(WGM21));
  TCCR2A |= _BV(WGM20);
  
  /* Timer 2 Set compare output mode (COM2A = 11 ; COM2B = 10)
  * Section 18.11.1 */
  TCCR2A &= ~(_BV(COM2A0));
  TCCR2A |= _BV(COM2A1);
  TCCR2A |= _BV(COM2B1) | _BV(COM2B0);
  // Timer 2 NOTE: we can leave FOC2B:A unset also, bits 4 and 5 are reserved
  
  /* Enable Timer 2
  * See Section 18.11.6 */
  TIMSK2 |= _BV(TOIE2); 
}


/**
 * Timer 2 interrupt - 255us
 */
ISR(TIMER2_OVF_vect){
  static byte icnt = 0;
  i1s++;
  //PORTD |= _BV(TRIG);  // Trigger CRO
  icnt++;
  if(TUN_K>0){
    OCR0A = pgm_read_byte_near(sin256 + (uint8_t)((icnt*TUN_K + U_OFFSET)));
    if(OCR0A + DEADTIME>255) OCR0B = 255;
    else OCR0B = OCR0A + DEADTIME;
    OCR1A = pgm_read_byte_near(sin256 + (uint8_t)((icnt*TUN_K + V_OFFSET)));
    if(OCR1A + DEADTIME>255) OCR1B = 255;
    else OCR1B = OCR1A + DEADTIME;
    OCR2A = pgm_read_byte_near(sin256 + (uint8_t)((icnt*TUN_K + W_OFFSET)));
    if(OCR2A + DEADTIME>255) OCR2B = 255;
    else OCR2B = OCR2A + DEADTIME;
  }else{
    OCR0A = 0;
    OCR0B = 255;
    OCR1A = 0;
    OCR1B = 255;
    OCR2A = 0;
    OCR2B = 255;
  }
  //PORTD &= ~(_BV(TRIG)); // Reset CRO trigger
}
