
#include "FreqPeriod.h"

//Definitions
#define MOTOR_INITIALIZATION_DELAY 4000 //Delay for motor to recognize ESC

#define ENGINE_NO_RUN_THRESHOLD 200 //Threshold for starter to begin operation
#define START_RPM_THRESH0LD 1000 //Threshold for engine to be started

#define RPM_TIME 60000000 //Time for RPM Division (in millionths of second)

// Pin Configuration Section
const int startPin=5;        //Switch that keeps genset on
const int escPowerOn=2;      //Powers up ESC
const int escToGenerator=3;  //HIGH: ESC powers the generator. LOW: Generator send power to rectifier.
const int throttlePin=4;
//const int tachPin=9;

//Initial Sensor Values (All overwritten when programme begins)
//int sensorValue=0;    //Tone Wheel Output (HIGH or LOW)
int startState=LOW;     //Start Button Output (HIGH or LOW)


//int tachState=0;
//int lastTachState=0;
//int oldTimeStamp=0;
//int newTimeStamp=0;

//int timeStep=0;
//int rpm=0;



void setup() 
{
  // put your setup code here, to run once:
    Serial.begin(9600);
    pinMode(startPin, INPUT);
    pinMode(escPowerOn, OUTPUT);
    pinMode(escToGenerator, OUTPUT);
    pinMode(throttlePin, OUTPUT);
    //pinMode(tachPin, INPUT); 
}

void loop() 
{
    // START SECTION
    // This portion of the script outlines several conditions for
    // starting and stopping the genset, as well as keeping it on.
    // It will do so by diverting power to the motor via relays,
    // and by maintaining a constant RPM with servo control.
    //digitalWrite(escToGenerator, LOW);
        if(startState==LOW)
    {
      Serial.print("nyukka");
    }
    startState = digitalRead(startPin);
    if(startState==LOW)
    {
      Serial.print("nyukka");
    }
    
    //if ((StartState=HIGH) && (rpm < ENGINE_NO_RUN_THRESHOLD))
    if ((startState==HIGH))
    {
        Serial.print("ON");
        digitalWrite(escToGenerator, HIGH);    // Switch Relays over to power motor
        delay(MOTOR_INITIALIZATION_DELAY/4);   // Wait for ESC/Motor Response
        digitalWrite(escPowerOn, HIGH);        // Switch on ESC
        //analogWrite(throttlePin,250);          // Upper Bound ESC Calibration
        for (int i=0; i <= 200; i++)
        {
        digitalWrite(4, HIGH);
        delayMicroseconds(2300); // Approximately 10% duty cycle @ 1KHz
        digitalWrite(4, LOW);
        delayMicroseconds(20000-2300);
        } 
                   
        for (int i=0; i <= 100; i++)
        {
        digitalWrite(4, HIGH);
        delayMicroseconds(700); // Approximately 10% duty cycle @ 1KHz
        digitalWrite(4, LOW);
        delayMicroseconds(20000-700);
        }        
            
        for (int i=0; i <= 200; i++)
        {
        digitalWrite(4, HIGH);
        delayMicroseconds(2300); // Approximately 10% duty cycle @ 1KHz
        digitalWrite(4, LOW);
        delayMicroseconds(20000-2300);
        }
        //delay(MOTOR_INITIALIZATION_DELAY);     // Wait for ESC/Motor Response
        //analogWrite(throttlePin,155);          // Lower Bound ESC Calibration
        //delay(MOTOR_INITIALIZATION_DELAY);     // Wait for ESC/Motor Response
        //analogWrite(throttlePin,250);          // PWM at full calibrated power
        //delay(10000);
    }
    
    if ((startState==LOW))
    {
        digitalWrite(escPowerOn, LOW);        // Switch off ESC
        digitalWrite(escToGenerator, LOW);    // Swittch Relays over to diode bridge
    } 
    //else if ((StartState=HIGH) && (rpm > START_RPM_THRESH0LD))
    //{     
    //    digitalWrite(escToGenerator, LOW);
    //    analogWrite(throttlePin,127);  // PWM at 1 millisecond pulses (50% duty cycle) for 0% throttle
    //}
    
    
    // TACH READ SECTION
    // This section is responsible for reading out and interpreting
    // the rpm from the tone wheel output.
    
    //lastTachState=tachState;
  
    //sensorValue = digitalRead(tachPin);

    //if (sensorValue==HIGH)
    //    tachState=1;
    
    //if (sensorValue==LOW)
    //    tachState=0;
      
    //newTimeStamp=micros();
    
    //if ((tachState != lastTachState) && (tachState==1))
    //{
    //    timeStep = newTimeStamp - oldTimeStamp;
    //    rpm = RPM_TIME/timeStep;
    //    oldTimeStamp=newTimeStamp;
    //}
    
    //Serial.println(rpm);   
    
}
