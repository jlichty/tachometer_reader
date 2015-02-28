#include <BufferedSerial.h>
#include <ByteBuffer.h>
#include <TimerOne.h>
#include <Servo.h> 

#define clock_speed 16000000
BufferedSerial serial = BufferedSerial(256, 256);
ByteBuffer send_buffer;

//Use Arduino MEGA
volatile uint16_t timeFromLastPulse =0;//holds value since last pulse
volatile uint8_t stopped_flag = 0; 
volatile uint8_t timer_interrupt_flag = 0;
volatile uint16_t lowPulseTime=0;
volatile uint8_t servo_angle=0;
volatile uint32_t tach_speed=0;

Servo servo;
const uint8_t UPPER_SERVO_TH = 150;
const uint8_t LOWER_SERVO_TH = 130;

void initialize_servo() {
  cli();
  servo.attach(9);
  sei();
}

void initialize_timers() {
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


/*Currently not being used
  // INIT TIMER4
  //this timer is set to input capture mode. When it receives a falling edge on pin ICP5 
  //aka pin 48, it will copy the current value of the timer into another register.
  //We use this for making sure the sensor is not underneath the pickup
  
  //set timer to normal mode
  TCCR5A = 0; 
  //set Input Capture on rising edge
  //set clock source to /64, prescaler must be active
  TCCR5B = 0;
  TCCR5B = 0 | 1 << ICNC5 | 1 << ICES5 | 1 << CS52 | 0 << CS51 | 0 << CS50;//set
  //enable interrupts
  TIMSK5 = 0 | 1 << ICIE5 | 1 << TOIE5;
*/

//Setting up external interrupt. Dont judge me, its being used for timing so it belongs here
        DDRD = 1<<PD2;    // Set PD2 as input (Using for interupt INT2)
  PORTD = 1<<PD2;   // Enable PD2 pull-up resistor
 
        EICRA |= 1 << ISC21 | 0 << ISC20; // sets an interrupt on the falling edge on INT2

  EIMSK |= 1 << INT2;
  //MCUCR = 1<<ISC01 | 1<<ISC00;  // Trigger INT0 on rising edge
     
  sei();//reenable interrupts
}

ISR(INT2_vect) {
  lowPulseTime = TCNT5;
}

ISR(TIMER5_CAPT_vect) { // PULSE DETECTED!  (interrupt automatically triggered, not called by main program)
  TCNT5 = 0;  // restart timer for next revolution
  if(ICR5 - lowPulseTime > 6)
  {
    
    if(!stopped_flag) //from stop, dont calculate speed from first pulse
    {
      timeFromLastPulse = ICR5;//H << 8 + ICR5L;      // save duration of last revolution   
    }
    stopped_flag=0;
    PORTB ^= 1 << PORTB7; //blink
  }
  else
  {
    stopped_flag=1;
  }
}

ISR(TIMER5_OVF_vect) { // counter overflow/timeout
  stopped_flag = 1;
  timeFromLastPulse = 0xFFFF;
  PORTB ^= 1 << PORTB7; //blink
}     // engine stopped

void sendSerial_Timer_ISR() { //trigger every certain amount of time
  timer_interrupt_flag = 1;
}

void handlePacket(ByteBuffer* packet) {
  send_buffer.clear();
  servo_angle = min(max((uint8_t)packet->getFloat(),UPPER_SERVO_TH),LOWER_SERVO_TH);
}

void setup() {
  initialize_servo();
  initialize_timers();
  
  // INITIALIZE BUFFEREDSERIAL
  //serial.init(0, 115200);
  serial.init(0, 9600);
  serial.setPacketHandler(handlePacket);
  send_buffer.init(64);

  /*
  Serial.begin(115200);
  pinMode(13, OUTPUT);
  */
 
  PORTL|=(1<<PL1);     //pullup enabled on ICP5
  DDRL&=~(1<<PL1);     //ICR5 as input
  DDRB |= (1<<PB5); //pin 11 as output
}

const uint32_t setpoint_speed = 7400;
const float Kp = 1;
const float LSF = 0.002;

void loop() {
  serial.update();
  
 if (!stopped_flag && timeFromLastPulse>0) {
    tach_speed = 3750000 / timeFromLastPulse;
    servo_angle = servo_angle + (int8_t)(Kp*LSF*(setpoint_speed - tach_speed));
    servo_angle = min(max(servo_angle,LOWER_SERVO_TH),UPPER_SERVO_TH);
    //servo.write(servo_angle);
  }
  
  if (timer_interrupt_flag) {
    
    // WRITE TO BUFFEREDSERIAL
    send_buffer.clear();
    send_buffer.putFloat((float)tach_speed);
    send_buffer.putFloat((float)servo_angle);
    serial.sendSerialPacket( &send_buffer );
    /*
    Serial.print(timeFromLastPulse);
    Serial.print("   ");
    Serial.println(tach_speed);
    */
    timer_interrupt_flag = 0;
/*
    PORTB |= (1<<PB5);    // test signal  120RPM
    delay(10);         
    PORTB &= ~(1<<PB5);
    */
  }
  /*
    uint32_t tach_speed=0;
    tach_speed = 937500 / timeFromLastPulse;
    Serial.println(tach_speed);
    
 
    delay(425);  
*/
  //servo.write(servo_angle); 
  
}
