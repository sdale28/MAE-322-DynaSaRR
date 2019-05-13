#include <Servo.h>

bool atWall = false;
bool firstStep = false;
bool secondStep = false;
bool overTheWall = false;
bool inTheChute = false;
bool throughTheChute = false;
bool medkitPlaced = false;

int Ch1, Ch2, Ch3, Ch4, Ch5, Ch6; // hold receiver signals
int R_wheel;
int L_wheel;
int Medkit_arm;
int Lifting_arm;
 
Servo L_Servo;
Servo R_Servo;
Servo Lifting_Servo;
Servo Medkit_Servo;

const int R_lightSensorPin = A9;
const int L_lightSensorPin = A8;
const int F_distSensorPin = A7;
const int R_distSensorPin = A6;
const int L_distSensorPin = A5;

const int R_ServoPin = 0;
const int L_ServoPin = 1;
const int Lifting_ServoPin = 2;
const int Medkit_ServoPin = 3;
const int front_limitSwitchPin = 4;
const int back_limitSwitchPin = 5;

const int Ch1Pin = 7;   // channel 1 is right stick lateral
const int Ch2Pin = 8;   // channel 2 is right stick vertical
const int Ch3Pin = 9;   // channel 3 is left stick vertical
const int Ch4Pin = 10;  // channel 4 is left stick lateral
const int Ch5Pin = 11;  // channel 5 is right knob
const int Ch6Pin = 12;  // channel 6 is left knob
const int onBoardLEDPin = 13;

// Servo control must fall between 1000uS (full reverse) and 2000uS (full forward)
const int ServoLow = 1000;
const int ServoHigh = 2000;
const int ServoZero = 1500;
const int ServoHalfRange = 500;

const int transmitterTimeout = 21000;

const int autonomousActivationFrequency = 1800; // knob turned completely clockwise

const int distSensorStopValue = 450; // Value of prox sensor indicating stopping distance; 4ish inches
const int distSensorSlowValue = 2000; // 2000 = 1 foot ish
const int lightThreshold = 300;//550; // sensor value for detecting target light (vs. noise/reflection)
const int chuteDistThreshold = 150; // Minimum value of left and right prox sensors for robot to be in chute
const int distSensorMaxValue = 1000;

int L_lightSensor;    // hold photoresistor value
int R_lightSensor;    // hold photoresistor value
int F_distSensor;       // hold front sharp sensor value
int R_distSensor;            // hold right distance sensor value
int L_distSensor;            // hold left distance sensor value
double distDiff;      // difference between dist sensors. if positive, closer to right of chute
int lightSensorDiff;  // difference between L_lightSensor and R_lightSensor
//int L_speed;          // speed changes for left wheel
//int R_speed;          // speed changes for right wheel
int Lifting_speed;    // speed changes for lifting arm
int Medkit_speed;     // speed changes for Medkit arm

// note: limit switches read 1 when not pressed and 0 when pressed
bool front_limitSwitch;
bool back_limitSwitch;

// setup() runs once then loop() runs
void setup() {
  pinMode(Ch1Pin, INPUT); // channel 1 is right stick lateral
  pinMode(Ch2Pin, INPUT); // channel 2 is right stick vertical
  pinMode(Ch3Pin, INPUT); // channel 3 is left stick vertical
  pinMode(Ch4Pin, INPUT); // channel 4 is left stick lateral
  pinMode(Ch5Pin, INPUT); // channel 5 is right knob
  pinMode(Ch6Pin, INPUT); // channel 6 is left knob

  pinMode(R_lightSensorPin, INPUT); // channel 5 is right knob
  pinMode(L_lightSensorPin, INPUT); // channel 6 is left knob
  pinMode(F_distSensorPin, INPUT);
  pinMode(R_distSensorPin, INPUT);
  pinMode(L_distSensorPin, INPUT);

  pinMode(front_limitSwitchPin, INPUT);
  pinMode(back_limitSwitchPin, INPUT);

  R_Servo.attach(R_ServoPin);
  L_Servo.attach(L_ServoPin);
  Lifting_Servo.attach(Lifting_ServoPin);
  Medkit_Servo.attach(Medkit_ServoPin);

  //L_speed = ServoZero;
  //R_speed = ServoZero;
  Lifting_speed = ServoZero;
  Medkit_speed = ServoZero;

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
  
  if (Ch2 <= ServoZero) {
    L_wheel = Ch1 + Ch2 - ServoZero;
    R_wheel = Ch1 - Ch2 + ServoZero;
  }
  else {
    int Ch1_mod = map(Ch1, ServoLow, ServoHigh, ServoLow, ServoHigh); // Invert CH1 axis to keep the math similar
    int Ch2_mod = map(Ch2, ServoLow, ServoHigh, ServoHigh, ServoLow); // Slow reaction time

    L_wheel = Ch1_mod + Ch2 - ServoZero;
    R_wheel = Ch2_mod - Ch2 + ServoZero;
  }

  int Medkit_arm = Ch3; 
  int Lifting_arm = Ch4;

  front_limitSwitch = digitalRead(front_limitSwitchPin);
  back_limitSwitch = digitalRead(back_limitSwitchPin);

  if (!front_limitSwitch && (Medkit_arm < ServoZero)) {
   Medkit_arm = ServoZero;
  } 
  else if (!back_limitSwitch && (Medkit_arm > ServoZero)) {
   Medkit_arm = ServoZero;
  }

  constrain(L_wheel, ServoLow, ServoHigh);
  constrain(R_wheel, ServoLow, ServoHigh);
  constrain(Lifting_arm, ServoLow, ServoHigh);
  constrain(Medkit_arm, ServoLow, ServoHigh);

  L_Servo.writeMicroseconds(L_wheel);
  R_Servo.writeMicroseconds(R_wheel);
  Lifting_Servo.writeMicroseconds(Lifting_arm);
  Medkit_Servo.writeMicroseconds(Medkit_arm);
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

void printSensors() {
  Serial.print("Left Sensor = ");
  Serial.println(L_lightSensor);
  Serial.print("Right Sensor = ");
  Serial.println(R_lightSensor);


  Serial.print("Front Distance = ");
  Serial.println(F_distSensor);
  Serial.print("Left Distance = ");
  Serial.println(L_distSensor);
  Serial.print("Right Distance = ");
  Serial.println(R_distSensor);

  Serial.print("Front Limit Switch: ");
  Serial.println(front_limitSwitch);
  Serial.print("Back Limit Switch: ");
  Serial.println(back_limitSwitch);
}

void updateSensors() {
  L_lightSensor = analogRead(L_lightSensorPin);
  R_lightSensor = analogRead(R_lightSensorPin);

  lightSensorDiff = abs(L_lightSensor - R_lightSensor);

  F_distSensor = analogRead(F_distSensorPin);

  for (int i = 0; i < 4; i++) {
    F_distSensor += analogRead(F_distSensorPin);
    R_distSensor += analogRead(R_distSensorPin);
    L_distSensor += analogRead(L_distSensorPin);
  }
  F_distSensor /= 5;
  R_distSensor /= 5;
  L_distSensor /= 5;

  distDiff = R_distSensor - L_distSensor;

  front_limitSwitch = digitalRead(front_limitSwitchPin);
  back_limitSwitch = digitalRead(back_limitSwitchPin);
  
  delay(100);
}

void turnLeft(int runTime, double percent) {
  int r = ServoZero - ServoHalfRange*percent;
  int l = ServoZero - ServoHalfRange*percent - 25; // constant offset to account for difference in speeds (from direction of rotation)

  constrain(r, ServoLow, ServoHigh);
  constrain(l, ServoLow, ServoHigh);

  R_Servo.writeMicroseconds(r);
  L_Servo.writeMicroseconds(l);

  delay(runTime);
}

void turnRight(int runTime, double percent) {
  int r = ServoZero + ServoHalfRange*percent + 25;
  int l = ServoZero + ServoHalfRange*percent;

  constrain(r, ServoLow, ServoHigh);
  constrain(l, ServoLow, ServoHigh);

  R_Servo.writeMicroseconds(r);
  L_Servo.writeMicroseconds(l);

  delay(runTime);
}

void driveForward(int runTime, double percent) {
  int r = ServoZero - ServoHalfRange*percent;
  int l = ServoZero + ServoHalfRange*percent;

  constrain(r, ServoLow, ServoHigh);
  constrain(l, ServoLow, ServoHigh);

  R_Servo.writeMicroseconds(r); //1425;
  L_Servo.writeMicroseconds(l);//1575;

  delay(runTime);
}

void driveBackward(int runTime, double percent) {
  int r = ServoZero + ServoHalfRange*percent;
  int l = ServoZero - ServoHalfRange*percent;

  constrain(r, ServoLow, ServoHigh);
  constrain(l, ServoLow, ServoHigh);

  R_Servo.writeMicroseconds(r); //1550);
  L_Servo.writeMicroseconds(l);//1450);

  delay(runTime);
}

void stopDriving(int runTime) {
  L_Servo.writeMicroseconds(ServoZero);
  R_Servo.writeMicroseconds(ServoZero);

  delay(runTime);
  // Serial.println("StopDriving");
}

void medkitArmForward(int runTime, double percent) {
  if (front_limitSwitch) {
    int medkitSpeed = ServoZero - ServoHalfRange * percent;

    Medkit_Servo.writeMicroseconds(medkitSpeed);

    //Serial.println("Medkit Forward");
    delay(runTime);
  }
  else {
    medkitArmStop(runTime);
  }
}

void medkitArmBackward(int runTime, double percent) {
  if (back_limitSwitch) {
    int medkitSpeed = ServoZero + ServoHalfRange * percent;

    Medkit_Servo.writeMicroseconds(medkitSpeed);

    //Serial.println("Medkit Backward");
    delay(runTime);
  }
  else {
    medkitArmStop(runTime);
  }
}

void medkitArmStop(int runTime) {
  Medkit_Servo.writeMicroseconds(ServoZero);

  //Serial.println("Medkit Stop");
  delay(runTime);
}

void liftingArmForward(int runTime, double percent) {
  Lifting_arm = ServoZero - ServoHalfRange*percent;
  constrain(Lifting_arm, ServoLow, ServoHigh);
  Lifting_Servo.writeMicroseconds(Lifting_arm);
  delay(runTime);
}

void liftingArmStop(int runTime) {
  Lifting_arm = ServoZero;
  Lifting_Servo.writeMicroseconds(Lifting_arm);
  delay(runTime);
}

void steps(int runTime) {
  Lifting_arm = ServoZero - ServoHalfRange*(.75);
  int r = ServoZero - ServoHalfRange*(.25);
  int l = ServoZero + ServoHalfRange*(.25);
  constrain(Lifting_arm, ServoLow, ServoHigh);
  constrain(r, ServoLow, ServoHigh);
  constrain(l, ServoLow, ServoHigh);

  R_Servo.writeMicroseconds(r);
  L_Servo.writeMicroseconds(l);
  Lifting_Servo.writeMicroseconds(Lifting_arm);
  delay(runTime);

  
}

void overWall(int runTime) {
/*
 liftingArmStop(10);
 int r = ServoZero - ServoHalfRange*0.5;
 int l = ServoZero + ServoHalfRange*0.5;
 Lifting_arm = ServoZero - ServoHalfRange*.5;
 constrain(Lifting_arm, ServoLow, ServoHigh);
 constrain(r, ServoLow, ServoHigh);
 constrain(l, ServoLow, ServoHigh);

 R_Servo.writeMicroseconds(r);
 L_Servo.writeMicroseconds(l);
 Lifting_Servo.writeMicroseconds(Lifting_arm);
 delay(250);
*/

  int r = ServoZero - ServoHalfRange;
  int l = ServoZero + ServoHalfRange;
  Lifting_arm = ServoZero - ServoHalfRange;
  constrain(Lifting_arm, ServoLow, ServoHigh);
  constrain(r, ServoLow, ServoHigh);
  constrain(l, ServoLow, ServoHigh);
  Lifting_Servo.writeMicroseconds(Lifting_arm);
  R_Servo.writeMicroseconds(r);
  L_Servo.writeMicroseconds(l);
  
  
  delay(runTime);
  liftingArmStop(50);
  stopDriving(50);
}

void autonomousLightSeekingNew() {
  const int chuteEntranceThreshold = 100;
  const int distDiffThreshold = 70;
  const int chuteThreshold = 70;

  const int lightSensorMaxValue = 900;
  const int lightSensorDiffThreshold = 40;

  // Serial.print("Right: ");
  // Serial.println(R_distSensor);
  // Serial.print("Left: ");
  // Serial.println(L_distSensor);

  // Serial.print("distDiff: ");
  // Serial.println(distDiff);

  if ((L_lightSensor > R_lightSensor) && (-lightSensorDiff > lightSensorDiffThreshold)) {
    double r_speed = map(-lightSensorDiff, 0, distSensorMaxValue, 0.1, 1);   // map(value, fromLow, fromHigh, toLow, toHigh)
    turnRight(10, r_speed);
  }
  else if ((R_lightSensor > L_lightSensor) && (lightSensorDiff > lightSensorDiffThreshold)) {
    double l_speed = map(lightSensorDiff, 0, lightSensorMaxValue, 0.1, 1);
    turnLeft(10, l_speed);
  }
  else {
    driveForward(10, 0.2);
  }
}

void autonomousLightSeeking() {
  Serial.println("Autonomous light seeking");

/*
  if (F_distSensor < distSensorStopValue) {
    if (L_lightSensor <= lightThreshold) {
      if (lightSensorDiff > 70) {
        if (L_lightSensor > R_lightSensor) {
          Serial.println("Left");
          turnLeft(10, 0.2);
        }
        else {
          Serial.println("Right");
          turnRight(10, 0.2);
        }
      }
      else {
        Serial.println("Forward");
        driveForward(10, 0.2);
      }
    } 
    else {
      Serial.println("Lost light");
      turnRight(5, 0.2);
    }
  }
  else {
    Serial.println("Light reached");
    stopDriving(100);
    delay(100);
  }
*/
/*
  if (F_distSensor < distSensorStopValue) {
    Serial.println("1");
    if (L_lightSensor <= lightThreshold) {
      stopDriving(10);
      Serial.println(F_distSensor);
      if (lightSensorDiff > 30 ) {
        if (L_lightSensor > R_lightSensor) {
          delay(50);
          turnLeft(150, 0.2); // 0.1 is too low--doesn't drive
          driveForward(100, 0.2);
          Serial.println("turn left");
        }
        else {
          delay(50);
          turnRight(10, 0.2);
          driveForward(100, 0.2);
          Serial.println("turn right");
        }
      }
      else {
        driveForward(200, 0.2); // works on 2000 but takes too long to recognize distance
        Serial.println("forward");
      }
    } 
    else {
      turnRight(10, 0.2);
        Serial.println("lost");
      updateSensors();
    }
  }
  else {
    stopDriving(100);
    delay(500);
    placeMedkit();
  }
  */
}

void chuteTraverse() {
  const int chuteEntranceThreshold = 100;
  const int distDiffThreshold = 70;
  const int chuteThreshold = 70;

  // Serial.print("Right: ");
  // Serial.println(R_distSensor);
  // Serial.print("Left: ");
  // Serial.println(L_distSensor);

  // Serial.print("distDiff: ");
  // Serial.println(distDiff);

  if (!inTheChute) {
    driveForward(100, 0.2);
    if ((R_distSensor > chuteEntranceThreshold) && (L_distSensor > chuteEntranceThreshold)) {
      inTheChute = true;
    }
  }
  else if (inTheChute && ((R_distSensor > chuteThreshold) || (L_distSensor > chuteThreshold))) {
    if ((L_distSensor > R_distSensor) && (-distDiff > distDiffThreshold)) {
      double r_speed = map(-distDiff, 0, distSensorMaxValue, 0.1, 1);   // map(value, fromLow, fromHigh, toLow, toHigh)
      turnRight(10, r_speed);
    }
    else if ((R_distSensor > L_distSensor) && (distDiff > distDiffThreshold)) {
      double l_speed = map(distDiff, 0, distSensorMaxValue, 0.1, 1);
      turnLeft(10, l_speed);
    }
    else {
      driveForward(10, 0.2);
    }
  }
  else {
    Serial.println("out of chute");
    driveForward(300, 0.2);
    stopDriving(100);
    turnRight(150, 0.2);
    stopDriving(500);
    throughTheChute = true;
  }
}

void chuteTraverseOld() {
  int chuteEntranceThreshold = 100; //150 originally
  int distDiffThreshold = 75;
  int chuteThreshold = 70;

  if (inTheChute == false) {
    driveForward(100, 0.25);
    if ((R_distSensor > chuteEntranceThreshold) && (L_distSensor > chuteEntranceThreshold)) {
      inTheChute = true;
    }
    Serial.println("Not in Chute.... yet");
  }
  else if (inTheChute && ((R_distSensor > chuteThreshold) || (L_distSensor > chuteThreshold))) { //100 not 150
    Serial.println("Currently in the Chute.");
    if(distDiff > distDiffThreshold) { // around 2in difference
      turnLeft(10, 0.2);
      //Serial.println("turning left");
    }
    else if (distDiff < -distDiffThreshold) {
      turnRight(10, 0.2);
      //Serial.println("turning right");
    }
    else {
      //Serial.println("driving forward");
      driveForward(10, 0.25);
    }
  }
  else {
    Serial.println("Out of chute.");
    driveForward(300, 0.2);
    stopDriving(100);
    turnLeft(150, .2);
    stopDriving(500);
    throughTheChute = true;
  }
}

void wallTraverse() {
  if(F_distSensor < 350 && !atWall) {
    driveForward(100, 0.2);
  }
  else {
    atWall = true;
  }
  
  if(atWall && !firstStep) {
    steps(800); // first step
    firstStep = true;
    stopDriving(10);
    liftingArmStop(10);
    delay(500);
  }
  else if(atWall && !secondStep) {
    steps(500);
    secondStep = true;
    stopDriving(10);
    liftingArmStop(10);
    delay(500);
  }
  else if(atWall && !overTheWall) {
    //driveForward(200, 0.3); //THIS MIGHT NEED TO BE COMMENTED BACK IN
    overWall(2500);
    overTheWall = true;
    stopDriving(10);
    liftingArmStop(10);
    delay(250);
  }
  else if(overTheWall) {
    stopDriving(10);
    liftingArmStop(10);
  }
  
}

void placeMedkit() {
  medkitArmForward(500, 0.05); // tighten chain
  delay(100);
  medkitArmForward(250, 0.40);
  delay(100);
  medkitArmForward(250, 0.10);
  delay(250);
  
  medkitArmStop(500);

  driveBackward(100, 0.1);

  delay(500);
  
  medkitArmBackward(500, 0.05); // tighten chain
  delay(100);
  medkitArmBackward(200, 0.3);
  delay(100);
  medkitArmBackward(100, 0.10);
  delay(100);
  
  medkitArmStop(100);
  //moveMedkitArm(100, 1450);
  //delay(100);

  stopDriving(100);

  medkitPlaced = true;
}

void autonomousMode() {
  updateSensors();
  //Serial.println(distDiff);

  // wallTraverse();
  //chuteTraverse();
  //if (throughTheChute && !medkitPlaced) {
    //Serial.println("light");
    //printSensors();
    autonomousLightSeeking();
  //}
  //Serial.println("Autonomous");
}

void loop() {
  Ch5 = pulseIn(Ch5Pin, HIGH, transmitterTimeout); // ch 5 toggles autonomous mode
  Ch6 = pulseIn(Ch6Pin, HIGH, transmitterTimeout); // ch 6 fully clockwise resets medkitPlaced

  updateSensors();

  // do nothing if the controller is disconnected
  if (Ch5 < ServoLow) {
    stopDriving(10);
    medkitArmStop(10);
    liftingArmStop(10);
  } 
  else {
    if (Ch6 <= autonomousActivationFrequency) {
      medkitPlaced = false;
      inTheChute = false;
      throughTheChute = true; // change me daddie!!!!!!!!!!!!!!!!!!!!
      atWall = false;
      firstStep = false;
      secondStep = false;
      overTheWall = false;
    }
    
    if (Ch5 <= autonomousActivationFrequency) {
      autonomousMode();
    }
    else {
      Ch1 = pulseIn(Ch1Pin, HIGH, transmitterTimeout);
      Ch2 = pulseIn(Ch2Pin, HIGH, transmitterTimeout);
      Ch3 = pulseIn(Ch3Pin, HIGH, transmitterTimeout);
      Ch4 = pulseIn(Ch4Pin, HIGH, transmitterTimeout);

      driveServosRC();
    }
  }
}
