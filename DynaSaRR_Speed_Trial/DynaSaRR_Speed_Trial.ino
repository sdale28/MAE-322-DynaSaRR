#include <Servo.h>

int Ch1, Ch2, Ch3, Ch4, Ch5, Ch6; // hold receiver signals
int R_wheel;
int L_wheel;

Servo L_Servo;
Servo R_Servo;

const int R_ServoPin = 0;
const int L_ServoPin = 1;
const int R_lightSensorPin = A7;
const int L_lightSensorPin = A8;
const int distSensorPin = A6;

const int Ch1Pin = 7;   // channel 1 is right stick lateral
const int Ch2Pin = 8;   // channel 2 is right stick vertical
const int Ch3Pin = 9;   // channel 3 is left stick vertical
const int Ch4Pin = 10;  // channel 4 is left stick lateral
const int Ch5Pin = 11;  // channel 5 is right knob
const int Ch6Pin = 12;  // channel 6 is left knob
const int onBoardLEDPin = 13;

// Servo control must fall between 1000uS and 2000uS
const int ServoLow = 1000;//1000;
const int ServoHigh = 2000;// 2000;

const int transmitterZeroFreq = 1500; // fequency that indicates 0 position (high+low)/2
const int transmitterTimeout = 21000;

const int autonomousActivationFrequency = 1800; // knob turned completely clockwise

const int distSensorStopValue = 3000; // Value of sharp sensor indicating stopping distance
const int distSensorSlowValue = 2000;
const int lightThreshold = 200; // sensor value for detecting target light (vs. noise/reflection)

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

//   pinMode(R_lightSensorPin, INPUT); // channel 5 is right knob
//   pinMode(L_lightSensorPin, INPUT); // channel 6 is left knob
//   pinMode(distSensorPin, INPUT);

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


void driveServosRC() {
  //Serial.println("RC");
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

void printRC() {
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

  distSensor = analogRead(distSensorPin);

  for (int i = 0; i < 4; i++) {
    distSensor += distSensor + analogRead(distSensorPin);
  }
  distSensor /= 5;

  // Serial.print("Left Sensor = ");
  // Serial.println(L_lightSensor);
  // Serial.print("Right Sensor = ");
  // Serial.println(R_lightSensor);
  // Serial.print("Distance Sensor = ");
  // Serial.println(distSensor);
  
  delay(100);
}

void turnLeft(int delayTime) {
  R_Servo.writeMicroseconds(R_speed);
  L_Servo.writeMicroseconds(1620);
  delay(delayTime);
}

void turnRight(int delayTime) {
  R_Servo.writeMicroseconds(1450);
  L_Servo.writeMicroseconds(L_speed);
  delay(delayTime);
}

void driveForward(int delayTime) {
  int subtractMeDaddy = 100;
  if (distSensor >= distSensorSlowValue) {
    subtractMeDaddy = 300;
  }
  R_Servo.writeMicroseconds(ServoHigh - subtractMeDaddy);
  L_Servo.writeMicroseconds(ServoLow + subtractMeDaddy);
  delay(delayTime);
}

void driveBackward(int delayTime) {
  R_Servo.writeMicroseconds(1450); //ServoLow);
  L_Servo.writeMicroseconds(1550); //ServoHigh);
  delay(delayTime);
}

void stopDriving(int delayTime) {
  L_Servo.writeMicroseconds(transmitterZeroFreq);
  R_Servo.writeMicroseconds(transmitterZeroFreq);

  delay(delayTime);
  Serial.println("StopDriving");
}

void autonomousMode() {

  //Serial.println("Autonomous");
  updateSensors();
  Serial.print("distance");
  Serial.println(distSensor);

  if (distSensor < distSensorStopValue) {
    if (L_lightSensor <= lightThreshold) {
      if (lightSensorDiff > 70) {
        if (L_lightSensor > R_lightSensor) {
          R_speed += 5;
          constrain(R_speed, ServoLow, 1480);
          turnLeft(10);
        }
        else {
          L_speed -= 5;
          constrain(L_speed, 1540, ServoHigh);
          turnRight(10);
        }
      }
      else {
        driveForward(5);
      }
    } 
    else {
      R_speed = 1500;
      turnLeft(5);
      updateSensors();
    }
  }
  else {
    stopDriving(100);
    delay(100);
  }
  
}

void loop() {
  
  //Serial.println("Loop");
//   Ch5 = pulseIn(Ch5Pin, HIGH, transmitterTimeout);

//   if (Ch5 <= autonomousActivationFrequency) {
//     autonomousMode();
//   }
//   else {
    Ch1 = pulseIn(Ch1Pin, HIGH, transmitterTimeout);
    Ch2 = pulseIn(Ch2Pin, HIGH, transmitterTimeout);
    Ch3 = pulseIn(Ch3Pin, HIGH, transmitterTimeout);
    Ch4 = pulseIn(Ch4Pin, HIGH, transmitterTimeout);
    Ch5 = pulseIn(Ch5Pin, HIGH, transmitterTimeout);
    Ch6 = pulseIn(Ch6Pin, HIGH, transmitterTimeout);
    driveServosRC();
//   }

  /*
  Ch1 = pulseIn(Ch1Pin, HIGH, transmitterTimeout);
    Ch2 = pulseIn(Ch2Pin, HIGH, transmitterTimeout);
    Ch3 = pulseIn(Ch3Pin, HIGH, transmitterTimeout);
    Ch4 = pulseIn(Ch4Pin, HIGH, transmitterTimeout);
    Ch5 = pulseIn(Ch5Pin, HIGH, transmitterTimeout);
    Ch6 = pulseIn(Ch6Pin, HIGH, transmitterTimeout);
  printRC();
  */
}
