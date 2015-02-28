#include <BufferedSerial.h>
#include <ByteBuffer.h>
#include <TimerOne.h>
#include <Servo.h> 

#define clock_speed 16000000
BufferedSerial serial = BufferedSerial(256, 256);
ByteBuffer send_buffer;

//Use Arduino MEGA
volatile uint16_t timeFromLastPulse = 0;//holds value since last pulse
volatile uint8_t stopped_flag = 0; 
volatile uint16_t lowPulseTime = 0;

//***PID***
volatile uint32_t tach_speed = 0;
uint32_t setpoint_speed = 7400;
const float Kp = 1;
const float LSF = 0.002;
const uint32_t UPPER_RPM_TH = 8000;
const uint32_t LOWER_RPM_TH = 500;

const uint32_t timer1_duration = 200000; // 20ms
volatile uint8_t serial_write_counter = 0;
volatile uint8_t controller_update_flag = 0;
const uint8_t serial_write_threshold = 1000000/timer1_duration - 1;

//Create object for servo library
Servo engine_throttle_servo;
volatile uint8_t servo_angle=0;
const uint8_t UPPER_SERVO_TH = 150;
const uint8_t LOWER_SERVO_TH = 130;

Servo choke_servo;
Servo esc_throttle_servo;

void initialize_servo() { //function to initialize servo motors
  cli();
  // INIT SERVOS
  engine_throttle_servo.attach(9);
  choke_servo.attach(10);
  esc_throttle_servo.attach(11);
  sei();
}

void initialize_timers() {
  cli();//stop interrupts

  // INIT TIMER1 INTERRUPT
  Timer1.initialize(timer1_duration);
  Timer1.attachInterrupt(setFlags_Timer_ISR);

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
  
  // INITIALIZE BUFFEREDSERIAL
  //serial.init(0, 115200);
  serial.init(0, 9600);
  serial.setPacketHandler(handlePacket);
  send_buffer.init(64);
  
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
  
  serial.update();
  if (controller_update_flag) {
    controller_update_flag = 0;
    if (!stopped_flag && timeFromLastPulse>0) {
      tach_speed = 3750000 / timeFromLastPulse;
      servo_angle = servo_angle + (uint8_t)(Kp*LSF*(setpoint_speed - tach_speed));
      servo_angle = min(max(servo_angle,LOWER_SERVO_TH),UPPER_SERVO_TH);
      //servo.write(servo_angle);
    } else {
      tach_speed = 0;
    }
  }
  
  if (serial_write_counter >= serial_write_threshold) {
    serial_write_counter = 0;
    
    // WRITE TO BUFFEREDSERIAL
    send_buffer.clear();
    send_buffer.putFloat((float)tach_speed);
    send_buffer.putFloat((float)servo_angle);
    serial.sendSerialPacket( &send_buffer );
  }
}

//ISRs

ISR(TIMER5_CAPT_vect) { // PULSE DETECTED 
  if(PINL & (1<<PL1)) { //check to see if rising or falling edge
    TCNT5 = 0;// restart timer for next revolution

    if(!stopped_flag) { //from stop, dont calculate speed from first pulse
      timeFromLastPulse = ICR5;//H << 8 + ICR5L;      // save duration of last revolution   
    }

    stopped_flag=0; //if it was stopped, this pulse means it has started again, next pulse will report speed
    PORTB ^= 1 << PORTB7; //blink

    TCCR5B &= ~(1 << ICES5); //set the ICP to trigger on the falling edge
  } else {
    lowPulseTime = ICR5;//records time at start of low pulse, doesnt reset timer
    TCCR5B |= 1 << ICES5; //set the ICP to trigger on the rising edge
  }  
}

ISR(TIMER5_OVF_vect) { // counter overflow/timeout. Basically this will happen if the engine is stopped 
  stopped_flag = 1;
  timeFromLastPulse = 0xFFFF;
  PORTB ^= 1 << PORTB7; //blink
}     // engine stopped


void setFlags_Timer_ISR() { // trigger every certain amount of time
  controller_update_flag = 1;
  serial_write_counter += 1;
}

void handlePacket(ByteBuffer* packet) {
  send_buffer.clear();
  int protocol = (int)packet->getFloat();
  if (protocol == 1) {
    servo_angle = (uint8_t)packet->getFloat();
    servo_angle = min(max(servo_angle,LOWER_SERVO_TH),UPPER_SERVO_TH);
  } else if (protocol == 2) {
    setpoint_speed = (uint32_t)packet->getFloat();
    setpoint_speed = min(max(setpoint_speed,LOWER_RPM_TH),UPPER_RPM_TH);
  }
}
