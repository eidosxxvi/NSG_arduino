// Program that displays encoder values of 2 wheel diff. drive to serial
#include "Arduino.h"
// Input pin defines
#define L_MOT_A 11 // PCINT0_vect
#define L_MOT_B 12 // PCINT0_vect
#define R_MOT_A 4  // PCINT2_vect
#define R_MOT_B 6  // PCINT2_vect
#define L_CUR   A1
#define R_CUR   A5

// Output pin defines
#define L_MOT_DIR 7
#define L_MOT_PWM 9
#define R_MOT_DIR 5
#define R_MOT_PWM 10

#define CP_ROT 1120 // counts per rotation

int inPins[4] = {L_MOT_A, L_MOT_B, R_MOT_A, R_MOT_B};
int outPins[4] = {L_MOT_PWM, L_MOT_DIR, R_MOT_PWM, R_MOT_DIR};

volatile int left_count = 0;
volatile int right_count = 0;

// the bytes will be like registers storing the previous states which will be compared
volatile byte left_state = 0;
volatile byte right_state = 0;

// Define ISRs
ISR(PCINT0_vect)
{
    // Reason why this wasn't working was because the pin #
    // does not correspond to the port pin #
    if(PINB & bit(3))
        left_state = (left_state | 0b10) & 0b1111;
    else
        left_state &= 0b1101;

    if(PINB & bit(4))
        left_state = (left_state | 0b01) & 0b1111;
    else
        left_state &= 0b1110;

    switch(left_state)
    {
        case 1:
        case 7:
        case 14:
        case 8:
            left_count++;
        break;
        case 4:
        case 2:
        case 11:
        case 13:
            left_count--;
    } // end case

    left_state <<= 2;
} // end PCINT0_vect

ISR(PCINT2_vect)
{
    if(PIND & bit(R_MOT_A))
        right_state = (right_state | 0b10) & 0b1111;
    else
        right_state &= 0b1101;

    if(PIND & bit(R_MOT_B))
        right_state = (right_state | 0b01) & 0b1111;
    else
        right_state &= 0b1110;

    switch(right_state)
    {
        case 1:
        case 7:
        case 14:
        case 8:
            right_count++;
        break;
        case 4:
        case 2:
        case 11:
        case 13:
            right_count--;
    } // end case

    right_state <<= 2;
} // end PCINT2_vect

void setup()
{
    Serial.begin(9600);

    //Flag setting for registers 
    PCICR |= bit(PCIE2) | bit(PCIE0);
    PCIFR |= bit(PCIF2) | bit(PCIF0);
    PCMSK2 |= bit(PCINT20) | bit(PCINT22);
    PCMSK0 |= bit(PCINT3) | bit(PCINT4);
    
    // Change the ADC clock prescaler
    ADCSRA &= 0b11111000;

    // Change the clock prescaler, CS10, CS11, and CS12.
    TCCR1B &= ~( bit(CS12) | bit(CS11) | bit(CS10) ); // reset clock prescaler
    TCCR1B |= bit(CS10);                              // enable 1x scaling 
    
    for(int i = 0; i < 4; i++)
    {
        pinMode(inPins[i], INPUT);     // set which pins are inputs
        pinMode(outPins[i], OUTPUT);   // set which pins are outputs
        digitalWrite(outPins[i], LOW); // set output pins low
    } // end for

    pinMode(L_CUR, INPUT);
    pinMode(R_CUR, INPUT);
} // end setup

int starttime, interval;
double left_current;
double right_current;
int state = 0;
void loop()
{   
    if(state < 4)
    {   
        Serial.print("Left Current: ");
        starttime = micros();
        left_current = analogRead(L_CUR);
        interval = micros()-starttime;
        left_current = (left_current-515)/1023.0 * 73.3;
        Serial.println(left_current);
        Serial.print("Sample time: ");
        Serial.println(interval);

        Serial.print("Right Current: ");
        starttime = micros();
        right_current = analogRead(R_CUR);
        interval = micros()-starttime;
        right_current = (right_current-519)/1023.0 * 73.3;
        Serial.println(right_current);
        Serial.print("Sample time: ");
        Serial.println(interval);
    }
    switch(state)
    {
        case 0:
            analogWrite(L_MOT_PWM, 255);
            analogWrite(R_MOT_PWM, 255);
            state = 1;
        break;

        case 1:
            if(abs(left_count) > 10*CP_ROT)
                analogWrite(L_MOT_PWM, 0);
            if(abs(right_count) > 10*CP_ROT)
                analogWrite(R_MOT_PWM, 0);
            if(abs(left_count)> 10*CP_ROT && abs(left_count) > 10*CP_ROT)
                state = 2;
        break;

        case 2:
            Serial.print("Left encoder: ");
            Serial.println(left_count);
            Serial.print("Right encoder: ");
            Serial.println(right_count);
            state = 3;
        break;

        case 3:
            Serial.println("Done");
            state = 4;
    } // end switch
} // end loop
