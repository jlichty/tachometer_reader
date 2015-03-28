#include <BufferedSerial.h>
#include <ByteBuffer.h>
#include <TimerOne.h>
#include <Servo.h> 

#define clock_speed 16000000
BufferedSerial serial = BufferedSerial(256, 256);
ByteBuffer send_buffer;
bool serial_mode = 0; // 1 = buffered serial, 0 = normal ASCII serial
bool throttle_servo_initialized = 0;

//Use Arduino MEGA
volatile uint16_t timeFromLastPulse = 0;//holds value since last pulse
volatile uint8_t stopped_flag = 0; 
volatile uint16_t lowPulseTime = 0;
volatile uint32_t current_time = 0;

//relay pins
#define Esc_Gen_Load 35 //HIGH: ESC powers the generator. LOW: Generator send power to rectifier.
#define Gen_Esc_Rotor 37
#define Esc_Power 39
#define Ecu_Power 41

//throttle pins
#define engine_throttle_pin A6
#define choke_pin A7
#define escThrottlePin A8
#define DEBUG_LED 13

//fans
#define cooling_fan A13

//***START SEQUENCE***
bool servos_initialized = false;
volatile uint8_t vehicleState = 0;
const uint16_t callibration_delay = 5000;
#define MOTOR_INITIALIZATION_DELAY 4000 //Delay for motor to recognize ESC
const uint16_t startup_RPM_threshold = 1000;
Servo start_sequence_servo;
volatile bool start_state=0;
volatile uint32_t start_state_time;

//***TIMER ONE***
const uint16_t timer1_duration = 2000; // 2ms
volatile uint16_t serial_write_counter = 0;
const uint16_t serial_write_threshold = (1000000/10)/timer1_duration - 1; // 10 writes/sec

//***PID***
volatile bool throttle_controller_enabled = false;
volatile uint16_t throttle_controller_counter = 0;
volatile uint16_t throttle_controller_threshold = (200000)/timer1_duration - 1;
volatile uint16_t last_RPM_average = 0;
volatile uint64_t running_average_RPM = 0;
volatile uint16_t RPM_measure_count = 0;
volatile uint16_t RPM_measure_threshold = 10;
volatile uint32_t tach_speed = 0;
uint16_t setpoint_speed = 7500;
const float Kp = 1.0;
const float LSF = 1.0/25; // 0.04
float Kp_LSF = Kp*LSF;
const uint16_t UPPER_RPM_TH = 8000;
const uint16_t LOWER_RPM_TH = 500;
float SETPOINT = 0;
float AVERAGE = 0;
float THROTTLE = 0;
float NEW_THROTTLE = 0;

//maximum servo microseconds for engine throttle servo is 1420, this is mapped to min throttle
//minimum servo microseconds for engine throttle servo is 840, this is mapped to max throttle
//Create object for servo library
Servo engine_throttle_servo;
Servo choke_servo;
volatile uint16_t throttle_servo_angle = 0;
volatile uint16_t choke_servo_angle=0;
const uint16_t UPPER_THROTTLE_SERVO_TH = 840;
const uint16_t LOWER_THROTTLE_SERVO_TH = 1420;
const uint16_t UPPER_CHOKE_SERVO_TH = 150;
const uint16_t LOWER_CHOKE_SERVO_TH = 130;

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



void setup() {
  
  pinMode(DEBUG_LED,OUTPUT);
  digitalWrite(DEBUG_LED,LOW);
  
  initialize_timers();
  
  if (serial_mode == 1) {
    // INITIALIZE BUFFEREDSERIAL
    //serial.init(0, 115200);
    serial.init(0, 9600);
    serial.setPacketHandler(handlePacket);
    send_buffer.init(32*5);
  } else if (serial_mode == 0) {
    Serial.begin(9600);
  }
  
  pinMode(Ecu_Power, OUTPUT);
  pinMode(Esc_Power, OUTPUT);
  pinMode(Gen_Esc_Rotor, OUTPUT);
  pinMode(Esc_Gen_Load, OUTPUT);
  pinMode(escThrottlePin, OUTPUT);
  
  if (serial_mode == 0) {
    vehicleState = 1; //without using pc script, this is necessary to start to sequence
  } else if (serial_mode == 1) { 
    vehicleState = 0;
  }
  start_state = 1;
}

void loop() {
  if (serial_mode == 1) { serial.update(); }
  
  // START SECTION
  // This portion of the script outlines several conditions for
  // starting and stopping the genset, as well as keeping it on.
  // It will do so by diverting power to the motor via relays,
  // and by maintaining a constant RPM with servo control.
  if (vehicleState == 1) {
    
    if (!servos_initialized) {
      servos_initialized = true;
      
      //cli();
      
      start_sequence_servo.writeMicroseconds(900);
      start_sequence_servo.attach(escThrottlePin);
      choke_servo.attach(choke_pin);
      
      //sei();
    }
    
    digitalWrite(Esc_Gen_Load, HIGH);    // Switch Relays over to power motor
    digitalWrite(Gen_Esc_Rotor, HIGH);    // Switch Relays over to power motor
    digitalWrite(Ecu_Power, HIGH);
    delay(MOTOR_INITIALIZATION_DELAY/4);   // Wait for ESC/Motor Response
    digitalWrite(Esc_Power, HIGH);        // Switch on ESC
    
    delay(5000);
    
    // 3% PWM           
    start_sequence_servo.writeMicroseconds(900);
    delay(callibration_delay);       
    
    vehicleState = 2;    
  }
  
  //Turn on ESC to max throttle, this state will overcome initial resistance and go up to ~500RPM
  if (vehicleState == 2) {
    
    if(start_state == 1) {
      //PORTB ^= 1 << PORTB7; //blink
      start_state_time = current_time;
      start_state = 0;
    }
    
    digitalWrite(DEBUG_LED,start_state);
    start_sequence_servo.writeMicroseconds(2200);
    if (!stopped_flag && timeFromLastPulse>0) {
        tach_speed = 3750000 / timeFromLastPulse;
        if ((tach_speed > 500) && (current_time-start_state_time > 1000)){ //delay to allow motor to catch
          start_state = 1;
          vehicleState = 3;
        }
      } else {
        tach_speed = 0;
      }  
  }
  
  //Reduce ESC to ~half throttle until the engine takes over
  if (vehicleState == 3) { 
    
    if(start_state == 1) {
      //PORTB ^= 1 << PORTB7; //blink
      start_state_time = current_time;
      start_state = 0;
    }
    
    start_sequence_servo.writeMicroseconds(1500);
    if (!stopped_flag && timeFromLastPulse>0) {
        tach_speed = 3750000 / timeFromLastPulse;
        if (tach_speed > 2100  && (current_time-start_state_time > 1400)){//startup_RPM_threshold) {
          start_sequence_servo.writeMicroseconds(900);
          digitalWrite(Esc_Power, LOW);
          digitalWrite(Esc_Gen_Load, LOW);
          digitalWrite(Gen_Esc_Rotor, LOW);
          vehicleState = 4;
        }
      } else {
        tach_speed = 0;
      }  
  }
  
  if (vehicleState == 4) {
    if (throttle_servo_initialized == 0) {
      throttle_servo_initialized = 1;
      
      //throttle_servo_angle = LOWER_THROTTLE_SERVO_TH;
      throttle_servo_angle = 1350;
      engine_throttle_servo.writeMicroseconds(throttle_servo_angle);
      //sei();
      engine_throttle_servo.attach(engine_throttle_pin);
      //cli();

      engine_throttle_servo.writeMicroseconds(throttle_servo_angle);
      delay(5000);
      
      throttle_controller_enabled = true;
    }
    
    if (throttle_servo_initialized == 1) {
      engine_throttle_servo.writeMicroseconds(throttle_servo_angle);
    }
    
    if (!stopped_flag && timeFromLastPulse>0) {
      // measure the current RPM
      tach_speed = 3750000 / timeFromLastPulse;
      
      RPM_measure_count += 1;
      running_average_RPM += tach_speed;
        
      if (RPM_measure_count >= RPM_measure_threshold) {
        last_RPM_average = running_average_RPM/RPM_measure_threshold;
        RPM_measure_count = 0;
        running_average_RPM = 0;
      }
      
      if (throttle_controller_counter >= throttle_controller_threshold) {
        throttle_controller_counter = 0;
        SETPOINT = setpoint_speed;
        AVERAGE = last_RPM_average;
        THROTTLE = throttle_servo_angle;
        NEW_THROTTLE = THROTTLE - Kp_LSF*(SETPOINT - AVERAGE);
        //throttle_servo_angle = (uint16_t)(NEW_THROTTLE); 
        //throttle_servo_angle = throttle_servo_angle - (uint16_t)(Kp_LSF*(setpoint_speed - last_RPM_average));
        throttle_servo_angle = min(max(throttle_servo_angle,UPPER_THROTTLE_SERVO_TH),LOWER_THROTTLE_SERVO_TH);
      }
      if (serial_mode == 0) { Serial.println("done"); }
    } else {
      tach_speed = 0;
    }
  }
  
  if (vehicleState == 5) {
    // kill everything
    digitalWrite(Esc_Gen_Load, LOW);
    digitalWrite(Gen_Esc_Rotor, LOW);
    digitalWrite(Ecu_Power, LOW);
    digitalWrite(Esc_Power, LOW);
    
    start_sequence_servo.write(0);
    engine_throttle_servo.writeMicroseconds(LOWER_THROTTLE_SERVO_TH);
    choke_servo.writeMicroseconds(LOWER_CHOKE_SERVO_TH);
    
    delay(2000);
      
    //cli();
    
    start_sequence_servo.detach();
    engine_throttle_servo.detach();
    choke_servo.detach();
    
    //sei();
   
    servos_initialized = false;
    vehicleState = 0; 
  }
 
  if (serial_write_counter >= serial_write_threshold) {
    serial_write_counter = 0;
    if (serial_mode == 1) {
      // WRITE TO BUFFEREDSERIAL
      send_buffer.clear();
      send_buffer.putFloat((float)tach_speed);
      send_buffer.putFloat((float)last_RPM_average);
      send_buffer.putFloat((float)throttle_servo_angle);
      send_buffer.putFloat((float)vehicleState);
      send_buffer.putFloat((float)setpoint_speed);
      send_buffer.putFloat((float)throttle_controller_threshold);
      send_buffer.putFloat((float)Kp_LSF);
      serial.sendSerialPacket( &send_buffer );
    } else if (serial_mode == 0) {
      tach_speed = 3750000 / timeFromLastPulse;
      Serial.print(current_time);
      Serial.print(" ");
      Serial.print(stopped_flag);
      Serial.print(" ");
      Serial.print(vehicleState);
      Serial.print(" ");
      Serial.println(tach_speed);
    }
  } 
}

//ISRs

ISR(TIMER5_CAPT_vect) { // PULSE DETECTED 

  
  //digitalWrite(13,HIGH); 
//PORTB ^= 1 << PORTB7; //blink
  if(PINL & (1<<PL1)) { //check to see if rising or falling edge
    TCNT5 = 0;// restart timer for next revolution
    
    if(!stopped_flag) { //from stop, dont calculate speed from first pulse
      timeFromLastPulse = ICR5;//H << 8 + ICR5L;      // save duration of last revolution   
    }

    stopped_flag=0; //if it was stopped, this pulse means it has started again, next pulse will report speed
    

    TCCR5B &= ~(1 << ICES5); //set the ICP to trigger on the falling edge
  } else {
    
    lowPulseTime = ICR5;//records time at start of low pulse, doesnt reset timer
    TCCR5B |= 1 << ICES5; //set the ICP to trigger on the rising edge
  }  
}

ISR(TIMER5_OVF_vect) { // counter overflow/timeout. Basically this will happen if the engine is stopped 
  stopped_flag = 1;
  timeFromLastPulse = 0xFFFF;
  //PORTB ^= 1 << PORTB7; //blink
}     // engine stopped

void setFlags_Timer_ISR() { // trigger every certain amount of time
  serial_write_counter += 1;
  current_time++;
  
  if (throttle_controller_enabled) {
    throttle_controller_counter += 1;
  }
}

void handlePacket(ByteBuffer* packet) {
  send_buffer.clear();
  int protocol = (int)packet->getFloat();
  if (protocol == 1) {
    throttle_servo_angle = (uint16_t)packet->getFloat();
    throttle_servo_angle = min(max(throttle_servo_angle,UPPER_THROTTLE_SERVO_TH),LOWER_THROTTLE_SERVO_TH);
  } else if (protocol == 2) {
    choke_servo_angle = (uint16_t)packet->getFloat();
    choke_servo_angle = min(max(choke_servo_angle,LOWER_CHOKE_SERVO_TH),UPPER_CHOKE_SERVO_TH);
  } else if (protocol == 3) {
    setpoint_speed = (uint16_t)packet->getFloat();
    setpoint_speed = min(max(setpoint_speed,LOWER_RPM_TH),UPPER_RPM_TH);
  } else if (protocol == 4) {
    vehicleState = (uint8_t)packet->getFloat();
    vehicleState = min(max(vehicleState,0),5);
  } else if (protocol == 5) {
    throttle_controller_threshold = (packet->getFloat())/timer1_duration - 1;
  } else if (protocol == 6) {
    Kp_LSF = packet->getFloat();
  }
}
