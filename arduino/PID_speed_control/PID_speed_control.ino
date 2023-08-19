#include <util/atomic.h>

// For the Encoder for motor A
#define ENCA 2
#define ENCB 6
#define PWM 10

// For the Encoder for motor B
#define ENCA2 3
#define ENCB2 7
#define PWM2 11


// Define the Direction Pins
#define IN1 4 // Direction Pin for Motor 1
#define IN2 5 // Direction Pin for Motor 2

// Define the number of counts of encoder for a full revolution.
#define ENC_COUNT_REV 894 // For now we assume two motors are similar this may come to bite me in the future

// Global Variables
long prevT = 0;
int posPrev = 0;

// Global Variables for motor B
long prevT2 = 0;
int posPrev2 = 0;

// Use the volatile directive for Motor 1 
volatile int pos_i = 0;
volatile float velocity_i = 0;
volatile long prevT_i = 0;

// Use the volatile directive for Motor 2
volatile int pos_i2 = 0;
volatile float velocity_i2 = 0;
volatile long prevT_i2 = 0;


float v1Filt = 0; // Variables for the low pass filter for motor 1
float v1Prev = 0;
float v2Filt = 0;
float v2Prev = 0;

float v1Filt2 = 0; // Variables for the low pass filter for motor 2
float v1Prev2 = 0;
float v2Filt2 = 0;
float v2Prev2 = 0;

float eintegral = 0; // For motor 1
float eintegral2 = 0;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  // For motor 1
  pinMode(ENCA,INPUT);
  pinMode(ENCB,INPUT);
  pinMode(PWM,OUTPUT);

  // For motor 2
  pinMode(ENCA2,INPUT);
  pinMode(ENCB2,INPUT);
  pinMode(PWM2,OUTPUT);
  
  pinMode(IN1,OUTPUT);
  pinMode(IN2,OUTPUT);
  
  // Trigger an interrupt when encoder A rises
  attachInterrupt(digitalPinToInterrupt(ENCA2), readEncoder2, RISING);
  attachInterrupt(digitalPinToInterrupt(ENCA), readEncoder, RISING);
  
}

void loop() {
  //////////////////////////////////////////////////////
  //*************While Uninterrupted******************//
  //////////////////////////////////////////////////////
  // The Arduino will run the code below =
  // put your main code here, to run repeatedly:
  //int pwr = 100/3.0*micros()/1.0e6; // Increase Speed 
  //int dir = 1;
  //setMotor(dir, pwr, PWM, IN1, IN2);

  int pos = 0;// for motor2
  int pos2 = 0; // for motor 2
  
  float velocity2 = 0;// for motor 1
  float velocity22 = 0;// for motor 2
  
  // Atomic Block will block other code from running when executing. Therefore ensuring memeory integrity
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE){
    // Note here "pos_i" and "velocity_i" are both code chunk from the interrrupt callback.
    pos = pos_i;
    velocity2 = velocity_i;
    pos2 = pos_i2;
    velocity22 = velocity_i2;
  }

  // Compute velocity with method 1
  long currT = micros();
  float deltaT = ((float) (currT - prevT))/1.0e6; // Change in T
  // Velocity for Motor 1
  float velocity1 = (pos - posPrev)/deltaT; // Compute the difference between the current and previous encoder count.
  posPrev = pos;
  // Velocity for Motor 2
  float velocity12 = (pos2 - posPrev2)/deltaT; // Compute the difference between the current and previous encoder count.
  posPrev2 = pos2;
  
  prevT = currT;

  // Convert counts/s to RPM
  float v1 = (velocity1/ENC_COUNT_REV)*60;
  float v2 = (velocity2/ENC_COUNT_REV)*60;

  float v12 = (velocity12/ENC_COUNT_REV)*60;
  float v22 = (velocity22/ENC_COUNT_REV)*60;
  
  // Low-pass Filter (40 Hz Cutoff Pretty Good) <- Like Really need to change
  //v1Filt = 0.7767*v1Filt + 0.1116*v1 + 0.1116*v1Prev;
  //v1Prev = v1; 
  //v2Filt = 0.7767*v2Filt + 0.1116*v2 + 0.1116*v2Prev;
  //v2Prev = v2;

  v1Filt = 0.843*v1Filt + 0.0782*v1 + 0.0782*v1Prev;
  v1Prev = v1; 
  v2Filt = 0.843*v2Filt + 0.0792*v2 + 0.0782*v2Prev;
  v2Prev = v2;

  v1Filt2 = 0.843*v1Filt2 + 0.0782*v12 + 0.0782*v1Prev2;
  v1Prev2 = v12; 
  v2Filt2 = 0.843*v2Filt2 + 0.0792*v22 + 0.0782*v2Prev2;
  v2Prev2 = v22;

  
  float vt = 10;
  float vt2 = 100;

  // Compute the control signal u for motor 1
  float kp = 3;
  float ki = 1;
  float e = vt - v1Filt;
  eintegral = eintegral + e*deltaT;
  float u = kp*e + ki*eintegral; 
  
  // Compute the control signal u for motor 2
  float e2 = vt2 - v1Filt2;
  eintegral2 = eintegral2 + e2*deltaT;
  float u2 = kp*e2 + ki*eintegral2;
  
  int dir = 1;
  if (u<0){
    dir = -1;
  }
  int pwr = (int) fabs(u);
  if (pwr > 255){
    pwr = 255;
  }

  int dir2 = 1;
  if (u2<0){
    dir2 = -1;
  }
  int pwr2 = (int) fabs(u2);
  if (pwr2 > 255){
    pwr2 = 255;
  }
  
  setMotor(dir, pwr, PWM, IN1);
  setMotor(dir2, pwr2, PWM2, IN2);

  Serial.print(vt);
  Serial.print(" ");
  Serial.print(v2Filt);
  Serial.println();
  delay(1); // Ensure consistent sampling frequency
}


void setMotor(int dir, int pwmVal, int pwm, int in1){
  analogWrite(pwm, pwmVal);
  if (dir == 1){
    digitalWrite(in1, HIGH);  
  }
  else{
    digitalWrite(in1, LOW);
  }
}
// Read Encoder Module for Motor 1
void readEncoder(){
  int b = digitalRead(ENCB);
  int increment = 0;
  if(b>0){
    increment = 1;
  }
  else {
    increment = -1;
  }
  pos_i = pos_i + increment;
  long currT = micros(); // Micros returns the number of microseconds since the board has begin running the program
  float deltaT = ((float) (currT - prevT_i))/1.0e6; // Detects the time passed between the current signal being detected and the next signal convert the time into seconds. 
  velocity_i = increment/deltaT;
  prevT_i = currT;
}

// Read Encoder Module for Motor 2
void readEncoder2(){
  int b2 = digitalRead(ENCB2);
  int increment2 = 0;
  if(b2>0){
    increment2 = 1;
  }
  else {
    increment2 = -1;
  }
  pos_i2 = pos_i2 + increment2;
  long currT2 = micros(); // Micros returns the number of microseconds since the board has begin running the program
  float deltaT2 = ((float) (currT2 - prevT_i2))/1.0e6; // Detects the time passed between the current signal being detected and the next signal convert the time into seconds. 
  velocity_i2 = increment2/deltaT2;
  prevT_i2 = currT2;
}
