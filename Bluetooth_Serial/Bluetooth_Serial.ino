/*
   Code by Brian Patton
   3/24/2017
   Feel free to do whatever you want with this code example

 Buetooth      Teensy
  Vcc----------5Volts
  GND----------GND
  RXD----------Pin1
  TXD----------Pin0
*/

#define blueSerial Serial1

String outStr =  "";  // finished string location
String tempStr = " "; //String to build on
char inChar; // Where to store the character read



void setup() {
  // initialize both serial ports:
  Serial.begin(9600);
  while(!Serial);
  blueSerial.begin(9600);
  Serial.println("ready");
  delay(500);
}

void loop() {
  getblueSerial1(); 
}

void getblueSerial1() {
  while (blueSerial.available() == 0);
  if (blueSerial.available() > 0) {
    inChar = blueSerial.read();
    if (inChar == '#') {
      outStr = tempStr;  // Send Data before appending the "#"
      tempStr += inChar;  // append #
      tempStr = "";        // Clear tempStr Buffer
     Serial.println("outStr = " + (String)outStr);
    }
    else {
      tempStr += inChar; // append characters until a 'Z' is found.
    }
  }
}