#include <ros.h>
#include <robonaldo/motor_speeds.h>
#include <robonaldo/beam_break.h>
#include <robonaldo/imu_values.h>
#include <robonaldo/encoder_values.h>
#include <avr/interrupt.h>
#include <avr/io.h>
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_LSM9DS1.h>
#include <Adafruit_Sensor.h>


#define LSM9DS1_SCK A5
#define LSM9DS1_MISO 12
#define LSM9DS1_MOSI A4
#define LSM9DS1_XGCS 6
#define LSM9DS1_MCS 5

/*imu*/
Adafruit_LSM9DS1 lsm = Adafruit_LSM9DS1();

/*motors*/
const int LMOTOR = 10;  //can be 10 or 13  Motor 1 = left, motor 2 = right
const int RMOTOR = 9;  //can be 9 or 4
const float TCCR2B_FREQ = 245.10f;
const float TCCR0B_FREQ = 244.14f;
/*beam breaker*/
const int BBPIN=50;
/*encoders*/
const int LEFT_ENCODER_A = 2;   //goes with ISR 4
const int LEFT_ENCODER_B = 3;   //goes with ISR 5
const int RIGHT_ENCODER_A = 18; //goes with ISR 3
const int RIGHT_ENCODER_B = 19; //goes with ISR 2
volatile unsigned char l_encoder_laststate, l_encoder_state;
volatile unsigned char r_encoder_laststate, r_encoder_state;
volatile int l_encoder_count=0;
volatile int r_encoder_count=0;
int l_last_count=0;
int r_last_count=0;

/*pid*/
float kp = 0.0f, kd = 0.0f, kf = 1.0f;  

// Converts -1, 1 motor speed to ticks/us
// Max motor rpm = 5330
// Max ticks/us = 1024
// Microseconds/min = 60 * 10^6 = 60,000,000
// Gear ratio of 12.75
// Max motor speed in revolutions per microsecond = 5330 * 1024 / 60,000,000 / 12.75 = 0.0071345359
const float conversionFactor = 0.00713f;

volatile unsigned char timer_reached1sec = 0;
unsigned long previousTime;
ros::NodeHandle n;

volatile float left_setpoint=0.0f;
volatile float right_setpoint=0.0f;

void messageCb(const robonaldo::motor_speeds& motor_speed_msg) {
  left_setpoint = motor_speed_msg.left_speed;
  right_setpoint = motor_speed_msg.right_speed;
  resetTimer();
  timer_reached1sec = 0;
}

void setupSensor()
{
  //Set the accelerometer range
  lsm.setupAccel(lsm.LSM9DS1_ACCELRANGE_2G); 
  // Set the magnetometer sensitivity
  lsm.setupMag(lsm.LSM9DS1_MAGGAIN_4GAUSS);
  // Setup the gyroscope
  lsm.setupGyro(lsm.LSM9DS1_GYROSCALE_245DPS);
}

ros::Subscriber<robonaldo::motor_speeds> sub("motor_control", &messageCb);

robonaldo::imu_values imu_msg;
robonaldo::encoder_values encoder_msg;
robonaldo::beam_break beam_msg;
ros::Publisher beam_pub("beam_state", &beam_msg);
ros::Publisher imu_pub("imu_values", &imu_msg);
ros::Publisher encoder_pub("encoder_values", &encoder_msg);

void setup() {
  // motors
  pinMode(LMOTOR, OUTPUT);
  pinMode(RMOTOR, OUTPUT);
  pin_init();
  
  //break beam code
  pinMode(BBPIN, INPUT);
  digitalWrite(BBPIN, HIGH); // turn on the pullup

  //encoder code
  pinMode(LEFT_ENCODER_A, INPUT);
  pinMode(LEFT_ENCODER_B, INPUT);
    
  n.initNode();
  n.subscribe(sub);
  n.advertise(beam_pub);
  n.advertise(imu_pub);
  n.advertise(encoder_pub);
  init_timer();
  sei();
	
  // imu code
  lsm.begin();
  setupSensor();

  l_encoder_initState();
  r_encoder_initState();
  previousTime = micros();
}

struct PDFController{
  
  float previous_error = 0.0f;

  float compute(float input, float setPoint, unsigned long deltaTime){     
  
    float error = setPoint - input;                                // determine error
    float rateError = (error - previous_error)/deltaTime;        // compute derivative
    
    float out = kp*error + kd*rateError + kf*setPoint;                //PID output               
    
    previous_error = error;                                //remember current error
  
    return out;                                        //have function return the PID output
  }
  
};

PDFController left_controller;
PDFController right_controller;

void loop(){
  //motor one is left
  //motor two is right
  //Testing motor with values of 1, 0, and -1.
  
  n.spinOnce();
  unsigned long currentTime = micros();
  
  if(timer_reached1sec){  //stop robot if no messages received recently
    setLeftMotorSpeed(0.0);
    setRightMotorSpeed(0.0);
  }

  char beamState = !digitalRead(BBPIN);
  beam_msg.beam_broken = beamState;
  beam_pub.publish(&beam_msg);
  
  unsigned long deltaTime = currentTime - previousTime;
  previousTime = currentTime;
  
  float l_velocity = (float)(l_encoder_count - l_last_count) / deltaTime;
  float r_velocity = (float)(r_encoder_count - r_last_count) / deltaTime; 

  //PID stuff
  float lPID = left_controller.compute(l_velocity, left_setpoint * conversionFactor, deltaTime);           //l_velocity is in encoder ticks/us, and everything else (lpid, L_motorSpeed) is in motorSpeak (-1 to 1), conversion needed
  setLeftMotorSpeed(lPID);

  float rPID = right_controller.compute(r_velocity, right_setpoint * conversionFactor, deltaTime);
  setRightMotorSpeed(rPID);

  l_last_count = l_encoder_count;
  r_last_count = r_encoder_count;
  
  encoder_msg.left_count = l_encoder_count;			
  encoder_msg.right_count = r_encoder_count;		
  encoder_msg.left_velocity = l_velocity;   //encoder ticks per microsecond
  encoder_msg.right_velocity = r_velocity;  //encoder ticks per microsecond

  encoder_pub.publish(&encoder_msg);					//encode_pub is the name of the 

  //imu stuff
  lsm.read();  /* ask it to read in the data */ 

  /* Get a new sensor event */ 
  sensors_event_t a, m, g, temp;

  lsm.getEvent(&a, &m, &g, &temp); 
  
  imu_msg.ax = a.acceleration.x;
  imu_msg.ay = a.acceleration.y;
  imu_msg.az = a.acceleration.z;
  
  imu_msg.mx = m.magnetic.x;
  imu_msg.my = m.magnetic.y;
  imu_msg.mz = m.magnetic.z;
  
  imu_msg.gx = g.gyro.x;
  imu_msg.gy = g.gyro.y;
  imu_msg.gz = g.gyro.z;
  
  imu_pub.publish(&imu_msg);
	
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
}
void resetTimer(){ //resets timer every time a message is received
    TCNT1 = 0;
}
ISR(TIMER1_COMPA_vect){
  timer_reached1sec = 1;
}
void l_encoder_initState(){
	unsigned int initA = digitalRead(LEFT_ENCODER_A);
	unsigned int initB = digitalRead(LEFT_ENCODER_B);
  // encoder_changed = 0;
  if (!initB && !initA){
    l_encoder_laststate = 0;
  }
  else if (!initB && initA){
    l_encoder_laststate = 1;
  }
  else if (initB && !initA){
    l_encoder_laststate = 2;
  }
  else{
    l_encoder_laststate = 3;
  }

  l_encoder_state = l_encoder_laststate;
}
void r_encoder_initState(){
	unsigned int initA = digitalRead(RIGHT_ENCODER_A);
	unsigned int initB = digitalRead(RIGHT_ENCODER_B);
  // encoder_changed = 0;
  if (!initB && !initA){
    r_encoder_laststate = 0;
  }
  else if (!initB && initA){
    r_encoder_laststate = 1;
  }
  else if (initB && !initA){
    r_encoder_laststate = 2;
  }
  else{
    r_encoder_laststate = 3;
  }

  r_encoder_state = r_encoder_laststate;
}
/*encoder interrupts*/
ISR(INT2_vect) {	//right encoder pin b
	unsigned char rightB = digitalRead(RIGHT_ENCODER_B);
	if (r_encoder_laststate == 0 && rightB) {
    r_encoder_state = 2;
    r_encoder_count--;
	}
	else if (r_encoder_laststate == 1 && rightB) {
    r_encoder_state = 3;
    r_encoder_count++;
	}
	else if (r_encoder_laststate == 2 && !rightB) {
    r_encoder_state = 0; 
    r_encoder_count++;
	}
	else if (!rightB) {   // encoder_laststate = 3
    r_encoder_state = 1;
    r_encoder_count--;			
	}
}
ISR(INT3_vect){   //right encoder pin a
	unsigned char rightA = digitalRead(RIGHT_ENCODER_A);
	if (r_encoder_laststate == 0 && rightA) {
    r_encoder_state = 1;
    r_encoder_count++;
	}
	else if (r_encoder_laststate == 1 && !rightA) {
    r_encoder_state = 0;
    r_encoder_count--;
	}
	else if (r_encoder_laststate == 2 && rightA) {
    r_encoder_state = 3;
    r_encoder_count--;
	}
	else if(!rightA){   // encoder_laststate = 3
    r_encoder_state = 2;
    r_encoder_count++;
	}
}

ISR(INT4_vect) {	//left encoder pin a
	unsigned char leftA = digitalRead(LEFT_ENCODER_A);
	if (l_encoder_laststate == 0 && leftA) {
    l_encoder_state = 1;
    l_encoder_count++;
	}
	else if (l_encoder_laststate == 1 && !leftA) {
    l_encoder_state = 0;
    l_encoder_count--;
	}
	else if (l_encoder_laststate == 2 && leftA) {
    l_encoder_state = 3;
    l_encoder_count--;
	}
	else if(!leftA){   // encoder_laststate = 3
    l_encoder_state = 2;
    l_encoder_count++;
	}
}
ISR(INT5_vect){   //left encoder pin b
	unsigned char leftB = digitalRead(LEFT_ENCODER_B);
	if (l_encoder_laststate == 0 && leftB) {
    l_encoder_state = 2;
    l_encoder_count--;
	}
	else if (l_encoder_laststate == 1 && leftB) {
    l_encoder_state = 3;
    l_encoder_count++;
	}
	else if (l_encoder_laststate == 2 && !leftB) {
    l_encoder_state = 0; 
    l_encoder_count++;
	}
	else if (!leftB) {   // encoder_laststate = 3
    l_encoder_state = 1;
    l_encoder_count--;			
	}
}
