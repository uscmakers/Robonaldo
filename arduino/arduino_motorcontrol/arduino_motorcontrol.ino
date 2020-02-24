#include <ros.h>
#include <robonaldo/motor_speeds.h>
#include <robonaldo/beam_break.h>
#include <avr/interrupt.h>
#include <avr/io.h>


const int LMOTOR = 10;  //can be 10 or 13  Motor 1 = left, motor 2 = right
const int RMOTOR = 9;  //can be 9 or 4
const float TCCR2B_FREQ = 245.10f;
const float TCCR0B_FREQ = 244.14f;
const int BBPOUT=13; //might be unnecessary if 5V pin is open 
const int BBPIN=4;

volatile unsigned char timer_reached1sec = 0;
unsigned char lastState = 0;

ros::NodeHandle n;

void messageCb(const robonaldo::motor_speeds& motor_speed_msg) {
  setLeftMotorSpeed(motor_speed_msg.left_speed);
  setRightMotorSpeed(motor_speed_msg.right_speed);
  resetTimer();
  timer_reached1sec = 0;
}

ros::Subscriber<robonaldo::motor_speeds> sub("motor_control", &messageCb);
ros::Publisher<robonaldo::beam_break> beam_pub("beam_breaker");

void setup() {
  // initialize digital pin LED_BUILTIN as an output.
  pinMode(LMOTOR, OUTPUT);
  pinMode(RMOTOR, OUTPUT);
  pin_init();
  
  //break beam code
  pinMode(BBPOUT, OUTPUT);
  pinMode(BBPIN, INPUT);
  digitalWrite(BBPIN, HIGH);
    
  n.initNode();
  n.subscribe(sub);
  init_timer();
}

void loop(){
  //motor one is left
  //motor two is right
  //Testing motor with values of 1, 0, and -1.
  n.spinOnce();

  if(timer_reached1sec){  //stop robot if no messages received for 
    setLeftMotorSpeed(0.0);
    setRightMotorSpeed(0.0);
  }

  char beamState=digitalRead(BBPIN);
  // check if the sensor beam is broken
  // if it is, the sensorState is LOW:
  if (beamState != lastState) {
    robonaldo::beam_break msg;
    msg.beam_broken = beamState;
    beam_pub.publish(msg);
  } 
  lastState = beamState;
  
  /*Oscilloscope measurements:
   * 1 pulse every 4 ms -> frequency of 250
   * Value called in analogwrite: 
   * 255: oscilloscope displays two constant lines 2.5 V apart 
   * 64: oscilloscope displays low signal for 3/4s of the time, high signal 1/4. 3 ms, 1 ms alternating. ___-___-
   * 128: half the time low signal, other half high signal. alternates at 2 ms, 2 ms alternating. 
   * 
   * Period of 4 ms. This makes sense because it's set to 250 (see tccr2b line in setup()).
   * Multiply duty cycle decimal by 255 to get value to call in analogwrite?
   * 0.4902 *255 corresponds to 2 ms on, 2 ms off pulses.
   * 0.2451 *255 corresponds to 1 ms on, 3 ms off pulses.
   */
   
}
void pin_init(){
  TCCR2B = TCCR2B & B11111000 | B00000101; //pins 10 & 9
  //TCCR0B = TCCR0B & B11111000 | B00000100; //pins 13 & 4
}
void setMotorSpeed(int pin, float motorSpeed){ //pin: pin connected to motor, motorSpeed from -1 to 1.
  //Takes value from -1 to 1, calls appropriate analogwrite
  double freq;
  //if pin is 9 or 10, register is TCCR2B 
  //else if pin is 13 or 4, resgister is TCCR0B
  if(pin == 9 || pin == 10){  //register is TCCR2B
    freq = TCCR2B_FREQ;
  }
  else if(pin == 13 || pin == 4){  //register is TCCR0B
    freq = TCCR0B_FREQ;
  }
  double period = 1000.0f/freq;
  double adjusted = ((1.0025f*motorSpeed)/2.0f) + (1.5f);  //5%, 2%, 1.5% worked, 1% didn't
  double val = (255.0f*adjusted)/period;
  int rounded = round(val);
  analogWrite(pin, rounded);
  //analogWrite(4 or 13, .2441); (backward)
  //analogWrite(13 or 4, .4883);(forward)  //244.14 Hz
  //analogWrite(9, .2451); //(backward) 
  //analogWrite(10, .4902);  //(forward)
  //analogWrite(LMOTOR, 255*.4902); // 2 ms pulses
}

void setLeftMotorSpeed(float motorSpeed){
  setMotorSpeed(LMOTOR, motorSpeed);
}
void setRightMotorSpeed(float motorSpeed){
  setMotorSpeed(RMOTOR, -1.0f*motorSpeed);
}
void init_timer(){
  //set timer1 interrupt at 1Hz
  TCCR1A = 0;// set entire TCCR1A register to 0
  TCCR1B = 0;// same for TCCR1B
  TCNT1  = 0;//initialize counter value to 0
  // set compare match register for 1hz increments
  OCR1A = 3906;// = (16*10^6) / (0.25*1024) - 1 (must be <65536)
  // turn on CTC mode
  TCCR1B |= (1 << WGM12);
  // Set CS10 and CS12 bits for 1024 prescaler
  TCCR1B |= (1 << CS12) | (1 << CS10);  
  // enable timer compare interrupt
  TIMSK1 |= (1 << OCIE1A);
    //enable interrupts    
    sei();
}
void resetTimer(){ //resets timer every time a message is received
    TCNT1 = 0;
}
ISR(TIMER1_COMPA_vect){
  timer_reached1sec = 1;
}
