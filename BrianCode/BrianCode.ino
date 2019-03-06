/*
  Testing autonomous guidance with simple if statements and variable speeds
  3/14/17 by Brian Patton
  Feel free to do whatever you want with this code example
*/
#include <Servo.h>
Servo R_Servo;  // create servo object to control a servo
Servo L_Servo;  // create servo object to control a servo
const int lPhoto = A1;    // Left Photoresistor
const int rPhoto = A0;    // Right Photoresistor
const int sharpPin = A2;  // Sharp Sensor
const int LED = 13;       // Onboard LED location
int lPhotoVal;            // Variable to store L photoresistor value
int rPhotoVal;            // Variable to store R photoresistor value
int sharpVal;             // Variable to store Sharp Sensor value
int valDif;               // Variable to store difference between photo values
int rSpeed, lSpeed;       // Variables to hold speed changes for each wheel

void setup()
{
  Serial.begin(9600);     // OPen Serial Port
  pinMode(LED, OUTPUT);   // Set LED port direction to output
  R_Servo.attach(2);      // Attach R Servo
  L_Servo.attach(1);      // Attach L Servo
  rSpeed = 1450;          // Set Default right wheel forward speed
  lSpeed = 1620;          // Set Default left wheel forward speed
  for (int i = 4; i >= 0; i--) {  //Flash LED 4 times
    digitalWrite(LED, HIGH);
    delay(500);
    digitalWrite(LED, LOW);
    delay(500);
  }
}
//*****************  Forward(int Dlay)   ***********************
//              Move the robot Slowly Forward
//**************************************************************
void Forward(int Dlay)
{
  R_Servo.writeMicroseconds(1450);  // sets the servo position
  L_Servo.writeMicroseconds(1600);   // sets the servo position
  delay(Dlay);
}
//*****************  Reverse(int Dlay)   ***********************
//                   Reverse the robot
//**************************************************************
void Reverse(int Dlay)
{
  R_Servo.writeMicroseconds(2000);  // sets the servo position
  L_Servo.writeMicroseconds(1000);   // sets the servo position
  delay(Dlay);
}
//*****************  stopBot(int Dlay)   ***********************
//                    Stop the robot
//**************************************************************
void stopBot(int Dlay)
{
  R_Servo.writeMicroseconds(1500);  // sets the servo position
  L_Servo.writeMicroseconds(1500);   // sets the servo position
  delay(Dlay);
}
//************* TLeftSlow(int rVal,int Dlay) *******************
//        left turn with tapering speed and a duration
//**************************************************************
void TLeftSlow(int rVal, int Dlay)
{
  R_Servo.writeMicroseconds(rVal);  // sets the servo position
  L_Servo.writeMicroseconds(1620);   // sets the servo position
  delay(Dlay);
}
//************* TRightSlow(int lVal,int Dlay) *******************
//        Right turn with tapering speed and a duration
//**************************************************************
void TRightSlow(int lVal, int Dlay)
{
  R_Servo.writeMicroseconds(1450);  // sets the servo position
  L_Servo.writeMicroseconds(lVal);   // sets the servo position
  delay(Dlay);
}
//******************** checkSensors() **************************
// Check value of Sensors         Stop bot if object is close
//**************************************************************
void checkSensors()
{
  rPhotoVal = analogRead(rPhoto);
  lPhotoVal = analogRead(lPhoto);
  valDif = abs(rPhotoVal - lPhotoVal); // looking for threshold
  sharpVal = analogRead(sharpPin);
  for (int i = 0; i <= 3; i++) {
    sharpVal = sharpVal + analogRead(sharpPin);
  }
  sharpVal = sharpVal / 5;
  if (sharpVal >= 400) {//changed from 500
    stopBot(100);
  }
}
//******************** printSensors() **************************
// Print the sensor values  Slows robot when in use!!!
//**************************************************************
void printSensors() {
  Serial.println("Right Value = " + (String)rPhotoVal);
  Serial.println("Left Value = " + (String)lPhotoVal);
  Serial.println("Difference Value = " + (String)valDif);
  Serial.println("Sharp Value = " + (String)sharpVal);
  Serial.println(" ");
  delay(100);
}
//************************  loop()  ****************************
//**********************  Main Loop  ***************************
//**************************************************************
void loop() {
  checkSensors();
  while (lPhotoVal > 1000) {    //slowly turn until a light is seen
    TLeftSlow(1500, 5);
    checkSensors();             // While checking sensors
  }

  if (valDif > 70) {            // If the difference between senors is big enough.....
    if (lPhotoVal > rPhotoVal) {// If L photo is bigger than R photo
      rSpeed = rSpeed + 5;      // Each time the L photo is > than R..Change speed a bit
      if (rSpeed >= 1480) {     // but don't let it go too far....
        rSpeed = 1480;
      }
      TLeftSlow(rSpeed, 10);    // Call the function with changing speed
      //      Serial.println(rSpeed);
    }
    else {                      //Or if R photo is bigger than L photo...
      lSpeed = lSpeed - 5;      // Each time the R photo is > than L..Change speed a bit
      if (lSpeed <= 1540) {     // but don't let it go too far....
        lSpeed = 1540;
      }
      TRightSlow(lSpeed, 10);    // Call the function with changing speed
      //      Serial.println(lSpeed);
    }
  }
  else {
    rSpeed = 1450;              // Reset the slow forward values
    lSpeed = 1620;              // Reset the slow forward values
    Forward(20);                // Go forward a bit
    //    Serial.println("forward");
  }
//     printSensors();
}