#include <Servo.h>

int Ch1, Ch2, Ch3, Ch4, Ch5, Ch6; // hold receiver signals
int R_wheel;
int L_wheel;

Servo L_Servo;
Servo R_Servo;

const int R_ServoPin = 0;
const int L_ServoPin = 1;
const int R_lightSensorPin = 2;
const int L_lightSensorPin = 3;
const int distSensorPin = 4;

const int Ch1Pin = 7;
const int Ch2Pin = 8;
const int Ch3Pin = 9;
const int Ch4Pin = 10;
const int Ch5Pin = 11;
const int Ch6Pin = 12;
const int onBoardLEDPin = 13;

const int R_ServoPin = 0;
const int L_ServoPin = 1;

// Servo control must fall between 1000uS and 2000uS
const int ServoLow = 1000;
const int ServoHigh = 2000;

const int transmitterZeroFreq = 1500; // fequency that indicates 0 position (high+low)/2
const int transmitterTimeout = 21000;

const int sharpStopValue = 400; // Value of sharp sensor indicating stopping distance

int L_lightSensor;    // hold photoresistor value
int R_lightSensor;    // hold photoresistor value
int distSensor;       // hold sharp sensor value
int lightSensorDiff;  // difference between L_lightSensor and R_lightSensor
int L_speed;          // speed changes for left wheel
int R_speed;          // speed changes for right wheel

// setup() runs once then loop() runs
void setup() {
  pinMode(Ch1Pin, INPUT); // channel 1 is right stick lateral
  pinMode(Ch2Pin, INPUT); // channel 2 is right stick vertical
  pinMode(Ch3Pin, INPUT); // channel 3 is left stick vertical
  pinMode(Ch4Pin, INPUT); // channel 4 is left stick lateral
  pinMode(Ch5Pin, INPUT); // channel 5 is right knob
  pinMode(Ch6Pin, INPUT); // channel 6 is left knob

  R_Servo.attach(R_ServoPin);
  L_Servo.attach(L_ServoPin);

  L_speed = transmitterZeroFreq;
  R_speed = transmitterZeroFreq;

  //Flash the onboard LED on and Off 10x 
  for (int i = 0; i < 10; i++) {
    digitalWrite(onBoardLEDPin, HIGH);
    delay(100);
    digitalWrite(onBoardLEDPin, LOW);
    delay(100);
  }

  Serial.begin(9600);
}

void stopDriving(int delayTime) {
  L_Servo.writeMilliseconds(transmitterZeroFreq);
  R_Servo.writeMilliseconds(transmitterZeroFreq);

  delay(delayTime);
}

void DriveServosRC() {
  if (Ch2 <= transmitterZeroFreq) {
    L_wheel = Ch1 + Ch2 - transmitterZeroFreq;
    R_wheel = Ch1 - Ch2 + transmitterZeroFreq;
  }
  else {
    int Ch1_mod = map(Ch1, ServoLow, ServoHigh, ServoLow, ServoHigh); // Invert CH1 axis to keep the math similar
    int Ch2_mod = map(Ch2, ServoLow, ServoHigh, ServoHigh, ServoLow); // Slow reaction time

    L_wheel = Ch1_mod + Ch2 - transmitterZeroFreq;
    R_wheel = Ch2_mod - Ch2 + transmitterZeroFreq;
  }

  constrain(L_wheel, ServoLow, ServoHigh);
  constrain(R_wheel, ServoLow, ServoHigh);

  L_Servo.writeMicroseconds(L_wheel);
  R_Servo.writeMicroseconds(R_wheel);
}

void PrintRC() {
  Serial.println(" RC Control Mode ");
  Serial.print("Value Ch1 = ");
  Serial.println(Ch1);
  Serial.print("Value Ch2 = ");
  Serial.println(Ch2);
  Serial.print("Value Ch3 = ");
  Serial.println(Ch3);
  Serial.print("Value Ch4 = ");
  Serial.println(Ch4);
  Serial.print("Value Ch5 = ");
  Serial.println(Ch5);
  Serial.print("Value Ch6 = ");
  Serial.println(Ch6);
  delay(1000);
}

void updateSensors() {
  L_lightSensor = analogRead(L_lightSensorPin);
  R_lightSensor = analogRead(R_lightSensorPin);

  lightSensorDiff = abs(L_lightSensor - R_lightSensor);

  for (int i = 0; i < 4; i++) {
    sharpVal += SharpVal + analogRead(sharpPin);
  }
  sharpVal /= 5;

  if (sharpVal >= sharpStopValue) {
    stopDriving(100);
  }
}

void loop() {
  Ch1 = pulseIn(Ch1Pin, HIGH, transmitterTimeout);
  Ch2 = pulseIn(Ch2Pin, HIGH, transmitterTimeout);
  Ch3 = pulseIn(Ch3Pin, HIGH, transmitterTimeout);
  Ch4 = pulseIn(Ch4Pin, HIGH, transmitterTimeout);
  Ch5 = pulseIn(Ch5Pin, HIGH, transmitterTimeout);
  Ch6 = pulseIn(Ch6Pin, HIGH, transmitterTimeout);

  
//  if (Ch5 > 1800)
//  {
//    DriveServosRC();
//  }
//  else
//  {
//    AutoM();
//  }

  DriveServosRC();
  //PrintRC();
}
