#include <Servo.h> 
#include <TimerOne.h>

//
volatile uint16_t timeFromLastPulse =0;//holds value since last pulse
volatile uint8_t stopped_flag = 0; 
volatile uint8_t timer_interrupt_flag = 0;
volatile uint16_t lowPulseTime=0;

//Create object for servo library
Servo engine_throttle_servo;
Servo choke_servo;
Servo esc_throttle_servo;

void initialize_servo()//function to initialize servo motors
{
  cli();
  // INIT SERVOS
  engine_throttle_servo.attach(9);
  choke_servo.attach(10);
  esc_throttle_servo.attach(11);
  sei();
}
void initialize_timers()
{
  cli();//stop interrupts

  // INIT TIMER1 INTERRUPT
  Timer1.initialize(500000);
  Timer1.attachInterrupt(sendSerial_Timer_ISR);

  // INIT TIMER5
  //this timer is set to input capture mode. When it receives a rising edge on pin ICP5 
  //aka pin 48, it will copy the current value of the timer into another register.
  //We use that time for calculating the speed of the engine
  
  //set timer to normal mode
  TCCR5A = 0; 
  //set Input Capture on rising edge
  //set clock source to /64, prescaler must be active
  TCCR5B = 0;
  TCCR5B = 0 | 1 << ICNC5 | 1 << ICES5 | 1 << CS52 | 0 << CS51 | 0 << CS50;//set

  //enable interrupts
  TIMSK5 = 0 | 1 << ICIE5 | 1 << TOIE5;  

  sei();//reenable interrupts
}

#define Esc_Gen_Load A1
#define Gen_Esc_Rotor A2
#define Esc_Power A3
#define Ecu_Power A0
void setup() {

initialize_servo();
initialize_timers();

Serial.begin(115200);
pinMode(13, OUTPUT);

pinMode(Esc_Gen_Load, OUTPUT);
pinMode(Gen_Esc_Rotor, OUTPUT);
pinMode(Esc_Power, OUTPUT);
pinMode(Ecu_Power, OUTPUT);

}

void loop() {

digitalWrite(Esc_Gen_Load,HIGH);  
delay(500);
digitalWrite(Esc_Gen_Load,LOW);  
delay(500);
if (timer_interrupt_flag)
  {
    uint32_t tach_speed;
    if (!stopped_flag&&timeFromLastPulse>0)
    {
      tach_speed = 3750000 / timeFromLastPulse;
    }
    else 
    {
      tach_speed = 0;
    }
    Serial.print(timeFromLastPulse);
    Serial.print("   ");
    Serial.println(tach_speed);
    timer_interrupt_flag = 0;
/*
    PORTB |= (1<<PB5);    // test signal  120RPM
    delay(10);         
    PORTB &= ~(1<<PB5);
    */
  }
}

//ISRs

ISR(TIMER5_CAPT_vect)  // PULSE DETECTED 
{
  if(PINL & (1<<PL1))//check to see if rising or falling edge
  {
    TCNT5 = 0;// restart timer for next revolution

    if(!stopped_flag) //from stop, dont calculate speed from first pulse
    {
      timeFromLastPulse = ICR5;//H << 8 + ICR5L;      // save duration of last revolution   
    }

    stopped_flag=0; //if it was stopped, this pulse means it has started again, next pulse will report speed
    PORTB ^= 1 << PORTB7; //blink

    TCCR5B &= ~(1 << ICES5); //set the ICP to trigger on the falling edge
  }
  else
  {
    lowPulseTime = ICR5;//records time at start of low pulse, doesnt reset timer
    TCCR5B |= 1 << ICES5; //set the ICP to trigger on the rising edge
  }  
    
  
  
}

ISR(TIMER5_OVF_vect)    // counter overflow/timeout. Basically this will happen if the engine is stopped
{ 
  stopped_flag = 1;
  timeFromLastPulse = 0xFFFF;
  PORTB ^= 1 << PORTB7; //blink
}     // engine stopped


void sendSerial_Timer_ISR() //trigger every certain amount of time
{
  timer_interrupt_flag = 1;
}
