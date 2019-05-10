//#include <PID_v1.h>
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

const int distSensorPin = A7;
const int L_lightSensorPin = A8;
const int R_lightSensorPin = A9;
const int distRPin = A6;
const int distLPin = A5;
const int front_limitSwitchPin = A4;
const int back_limitSwitchPin = A3;

const int R_ServoPin = 0;
const int L_ServoPin = 1;
const int Lifting_ServoPin = 2;
const int Medkit_ServoPin = 3;

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
const int MedkitServoLow = 1000; // 1250;
const int MedkitServoHigh = 2000;// 1750;

const int transmitterTimeout = 21000;

const int autonomousActivationFrequency = 1800; // knob turned completely clockwise

const int distSensorStopValue = 450; // Value of prox sensor indicating stopping distance; 4ish inches
const int distSensorSlowValue = 2000; // 2000 = 1 foot ish
const int lightThreshold = 550; // sensor value for detecting target light (vs. noise/reflection)

int L_lightSensor;    // hold photoresistor value
int R_lightSensor;    // hold photoresistor value
int distSensor;       // hold front sharp sensor value
int distR;            // hold right distance sensor value
int distL;            // hold left distance sensor value
double distDiff;      // difference between dist sensors. if positive, closer to right of chute
int lightSensorDiff;  // difference between L_lightSensor and R_lightSensor
int L_speed;          // speed changes for left wheel
int R_speed;          // speed changes for right wheel
int Lifting_speed;    // speed changes for lifting arm
int Medkit_speed;     // speed changes for Medkit arm

bool front_limitSwitch;
bool back_limitSwitch;

////PID setup
//double Kp = 2;
//double Ki = 5;
//double Kd = 1;
//double distOut;
//PID chutePID(&distDiff, &distOut, 0, Kp, Ki, Kd, DIRECT);

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
  pinMode(distSensorPin, INPUT);
  pinMode(distRPin, INPUT);
  pinMode(distLPin, INPUT);

  pinMode(front_limitSwitchPin, INPUT);
  pinMode(back_limitSwitchPin, INPUT);

  R_Servo.attach(R_ServoPin);
  L_Servo.attach(L_ServoPin);
  Lifting_Servo.attach(Lifting_ServoPin);
  Medkit_Servo.attach(Medkit_ServoPin);

  L_speed = ServoZero;
  R_speed = ServoZero;
  Lifting_speed = ServoZero;
  Medkit_speed = ServoZero;

  //chutePID.SetMode(AUTOMATIC);
  //chutePID.SetOutputLimits(-1, 1); // 1 if closer to right, -1 if closer to left ?


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

  int Medkit_arm = map(Ch3, ServoLow, ServoHigh, MedkitServoLow, MedkitServoHigh); 
  int Lifting_arm = map(Ch4, ServoLow, ServoHigh, MedkitServoLow, MedkitServoHigh);
  

  constrain(L_wheel, ServoLow, ServoHigh);
  constrain(R_wheel, ServoLow, ServoHigh);
  constrain(Lifting_arm, ServoLow, ServoHigh);
  constrain(Medkit_arm, ServoLow, ServoHigh);

  //if (!front_limitSwitch && (Medkit_arm < ServoZero)) {
  //  Medkit_arm = ServoZero;
  //} 
  //else if (!back_limitSwitch && (Medkit_arm > ServoZero)) {
  //  Medkit_arm = ServoZero;
  //}

  L_Servo.writeMicroseconds(L_wheel);
  R_Servo.writeMicroseconds(R_wheel);
  Lifting_Servo.writeMicroseconds(Lifting_arm);
  Medkit_Servo.writeMicroseconds(Medkit_arm);
}

//not used; just for testing
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
  Serial.print("Distance Sensor = ");
  Serial.println(distSensor);
}

void updateSensors() {
  L_lightSensor = analogRead(L_lightSensorPin);
  R_lightSensor = analogRead(R_lightSensorPin);

  lightSensorDiff = abs(L_lightSensor - R_lightSensor);

  distSensor = analogRead(distSensorPin);

  for (int i = 0; i < 4; i++) {
    distSensor += analogRead(distSensorPin);
    distR += analogRead(distRPin);
    distL += analogRead(distLPin);
  }
  distSensor /= 5;
  distR /= 5;
  distL /= 5;

  distDiff = distR - distL;

  front_limitSwitch = digitalRead(front_limitSwitchPin);
  back_limitSwitch = digitalRead(back_limitSwitchPin);
  
  delay(100);

  //Serial.println(distSensor);
}

void turnLeft(int runTime, double percent) {
  int r = ServoZero - ServoHalfRange*percent;
  int l = ServoZero - ServoHalfRange*percent - 25; // constant offset to account for difference in speeds

  constrain(r, ServoLow, ServoHigh);
  constrain(l, ServoLow, ServoHigh);

  R_Servo.writeMicroseconds(r); //R_speed);
  L_Servo.writeMicroseconds(l);//1620);

  delay(runTime);
}

void turnRight(int runTime, double percent) {
  int r = ServoZero + ServoHalfRange*percent + 25;
  int l = ServoZero + ServoHalfRange*percent;

  constrain(r, ServoLow, ServoHigh);
  constrain(l, ServoLow, ServoHigh);

  R_Servo.writeMicroseconds(r); //1450);
  L_Servo.writeMicroseconds(l);//L_speed);

  delay(runTime);
}

void driveForward(int runTime, double percent) {
  // int subtractValue = 100;
  // if (distSensor >= distSensorSlowValue) {
  //   subtractValue = 300;
  // }
  // R_Servo.writeMicroseconds(ServoHigh - subtractValue);
  // L_Servo.writeMicroseconds(ServoLow + subtractValue);

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
  // if (!front_limitSwitch) {
    int medkitSpeed = ServoZero - ServoHalfRange * percent;

    Medkit_Servo.writeMicroseconds(medkitSpeed);

    //Serial.println("Medkit Forward");
    delay(runTime);
  // }
  // else {
  //   medkitArmStop(runTime);
  // }
}

void medkitArmBackward(int runTime, double percent) {
  // if (!back_limitSwitch) {
    int medkitSpeed = ServoZero + ServoHalfRange * percent;

    Medkit_Servo.writeMicroseconds(medkitSpeed);

    //Serial.println("Medkit Backward");
    delay(runTime);
  // }
  // else {
  //   medkitArmStop(runTime);
  // }
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

//  liftingArmStop(10);
//  int r = ServoZero - ServoHalfRange*0.5;
//  int l = ServoZero + ServoHalfRange*0.5;
//  Lifting_arm = ServoZero - ServoHalfRange*.5;
//  constrain(Lifting_arm, ServoLow, ServoHigh);
//  constrain(r, ServoLow, ServoHigh);
//  constrain(l, ServoLow, ServoHigh);
//
//  R_Servo.writeMicroseconds(r);
//  L_Servo.writeMicroseconds(l);
//  Lifting_Servo.writeMicroseconds(Lifting_arm);
//  delay(250);

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

void autonomousLightSeeking() {
  //Serial.println("Autonomous light seeking");
  //Serial.print("distance");
  //Serial.println(distSensor);

  if(medkitPlaced == false) {
    if (distSensor < distSensorStopValue) {
      Serial.println("1");
      if (L_lightSensor <= lightThreshold) {
        stopDriving(10);
        Serial.println(distSensor);
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
    
  }

}

void chuteTraverse() {
  //chutePID.Compute();
  //Serial.println(distR);
  //Serial.println(distL);
  //Serial.println(distDiff);
  if(inTheChute == false) {
    driveForward(100, 0.2);
    //Serial.println(distR);
    //Serial.println(distL);
    if(distR > 150) {
      if(distL > 150) {
        inTheChute = true;
        //Serial.println("in");
      }
    }
  }
   else if(distR > 100) {
    if(distL > 100) {
      if(distDiff > 75) { // around 2in difference
        turnLeft(10, 0.15);
        //Serial.println("turning left");
      }
      else if(distDiff < -75) {
        turnRight(10, 0.15);
        //Serial.println("turning right");
      }
      else {
        //Serial.println("driving forward");
        driveForward(10, 0.15);
      }
    }
   }
  else {
    //Serial.println("out of chute");
    driveForward(300, 0.2);
    stopDriving(100);
    turnLeft(150, .2);
    stopDriving(500);
    throughTheChute = true;
  }
}

void wallTraverse() {
  if(distSensor < 350 && !atWall) {
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
  //chuteTraverse();
//  if (throughTheChute && !medkitPlaced) {
//    Serial.println("light");
//    autonomousLightSeeking();
//  }
  //Serial.println("Autonomous");
  wallTraverse();
}

void loop() {
  Ch5 = pulseIn(Ch5Pin, HIGH, transmitterTimeout); // ch 5 toggles autonomous mode
  Ch6 = pulseIn(Ch6Pin, HIGH, transmitterTimeout); // ch 6 fully clockwise resets medkitPlaced

  updateSensors();

  if (Ch6 <= autonomousActivationFrequency) {
    medkitPlaced = false;
    inTheChute = false;
    throughTheChute = true; // MAKE SURE TO CHANGE THIS BACK TO FALSE TO TEST LIGHT
    atWall = false;
    firstStep = false;
    secondStep = false;
    overTheWall = false;
  }
  
  if (Ch5 <= autonomousActivationFrequency) {
    //medkitPlaced = false;
    //autonomous = true;
    autonomousMode();
    //placeMedkit();
  }
  else {
    Ch1 = pulseIn(Ch1Pin, HIGH, transmitterTimeout);
    Ch2 = pulseIn(Ch2Pin, HIGH, transmitterTimeout);
    //Ch3 = pulseIn(Ch3Pin, HIGH, transmitterTimeout); //disabled medkit arm
    Ch4 = pulseIn(Ch4Pin, HIGH, transmitterTimeout);
    //Ch5 = pulseIn(Ch5Pin, HIGH, transmitterTimeout);
    //Ch6 = pulseIn(Ch6Pin, HIGH, transmitterTimeout);
    //printRC();
    driveServosRC();
    //Serial.println(distSensor);
    //Serial.println(distR);
    //Serial.println(distL);
  }
}
