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

/*imu*/
Adafruit_LSM9DS1 lsm = Adafruit_LSM9DS1();

#define LSM9DS1_SCK A5
#define LSM9DS1_MISO 12
#define LSM9DS1_MOSI A4
#define LSM9DS1_XGCS 6
#define LSM9DS1_MCS 5

/*motors*/
const int LMOTOR = 10;  //can be 10 or 13  Motor 1 = left, motor 2 = right
const int RMOTOR = 9;  //can be 9 or 4
const float TCCR2B_FREQ = 245.10f;
const float TCCR0B_FREQ = 244.14f;
/*beam breaker*/
const int BBPOUT=13; //might be unnecessary if 5V pin is open 
const int BBPIN=4;
/*encoders*/
const int LEFT_ENCODER_A = 1; 
const int LEFT_ENCODER_B = 5;
const int RIGHT_ENCODER_A = 2; //not sure about RIGHT ENCODER pins
const int RIGHT_ENCODER_B = 6; 
volatile unsigned char encoder_laststate, encoder_state;
volatile int encoder_count=0;

volatile unsigned char timer_reached1sec = 0;
unsigned char lastState = 0;

ros::NodeHandle n;

void messageCb(const robonaldo::motor_speeds& motor_speed_msg) {
  setLeftMotorSpeed(motor_speed_msg.left_speed);
  setRightMotorSpeed(motor_speed_msg.right_speed);
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
ros::Publisher beam_pub("beam_state", nullptr);
ros::Publisher imu_pub("imu_values", nullptr);
ros::Publisher encoder_pub("encoder_values", nullptr);

void setup() {
  // motors
  pinMode(LMOTOR, OUTPUT);
  pinMode(RMOTOR, OUTPUT);
  pin_init();
  
  //break beam code
  pinMode(BBPOUT, OUTPUT);
  pinMode(BBPIN, INPUT);
  digitalWrite(BBPIN, HIGH);

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
	
  //imu code
  lsm.begin();
  setupSensor();
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
  if (beamState != lastState) {
    robonaldo::beam_break beam_msg;
    beam_msg.beam_broken = beamState;
    beam_pub.publish(&beam_msg);
  } 
  lastState = beamState;   

  encoder_changeState();

  robonaldo::encoder_values encoder_msg;				//encoder_values is the name of the 
  encoder_msg.left_count = encoder_count;			//encoder_counts is new
  encoder_pub.publish(&encoder_msg);					//encode_pub is the name of the 

  //imu stuff
  lsm.read();  /* ask it to read in the data */ 

  /* Get a new sensor event */ 
  sensors_event_t a, m, g, temp;

  lsm.getEvent(&a, &m, &g, &temp); 

  robonaldo::imu_values imu_msg;
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
void encoder_changeState(){
	unsigned int initA = digitalRead(LEFT_ENCODER_A);
	unsigned int initB = digitalRead(LEFT_ENCODER_B);
  // encoder_changed = 0;
  if (!initB && !initA){
    encoder_laststate = 0;
  }
  else if (!initB && initA){
    encoder_laststate = 1;
  }
  else if (initB && !initA){
    encoder_laststate = 2;
  }
  else{
    encoder_laststate = 3;
  }

  encoder_state = encoder_laststate;
}
void resetTimer(){ //resets timer every time a message is received
    TCNT1 = 0;
}
ISR(TIMER1_COMPA_vect){
  timer_reached1sec = 1;
}
/*encoder interrupt*/
ISR(PCINT1_vect) {	//left encoder pin b
	unsigned char a = digitalRead(LEFT_ENCODER_A);
	unsigned char b = digitalRead(RIGHT_ENCODER_B);

	if (encoder_laststate == 0) {
		// Handle B input for state 0
		if(b){	//CCW
			encoder_state = 2;
			encoder_count--;
		}
	}
	else if (encoder_laststate == 1) {
		// Handle B input for state 1
    if(b){	//CW
			encoder_state = 3;
			encoder_count++;
		}
	}
	else if (encoder_laststate == 2) {
		// Handle B input for state 2
		if(!b){	//CW
			encoder_state = 0; 
			encoder_count++;
		}
	}
	else {   // encoder_laststate = 3
		// Handle B input for state 3
	  if(!b){	//CCW
			encoder_state = 1;
			encoder_count--;			
		}
	}

}
ISR(PCINT2_vect){   //for checking left encoder pin A
	unsigned char a = digitalRead(LEFT_ENCODER_A);
	unsigned char b = digitalRead(RIGHT_ENCODER_B);

	if (encoder_laststate == 0) {
		// Handle A input for state 0
		if(a){//CW
			encoder_state = 1;
			encoder_count++;
		}
	}
	else if (encoder_laststate == 1) {
		// Handle A input for state 1
		if(!a){	//CCW
			encoder_state = 0;
			encoder_count--;
		}

	}
	else if (encoder_laststate == 2) {
		// Handle A input for state 2
		if(a){	//CCW
			encoder_state = 3;
			encoder_count--;
		}
	}
	else {   // encoder_laststate = 3
		// Handle A input for state 3
		if(!a){	//CW
			encoder_state = 2;
			encoder_count++;
		}
	}
}
