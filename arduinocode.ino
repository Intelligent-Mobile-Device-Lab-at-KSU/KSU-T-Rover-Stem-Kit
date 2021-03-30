#include <Servo.h>

//////////////////
//Global objects//
//////////////////
Servo myServo;
Servo myEsc;

////////////////////
//Global variables//
////////////////////

//pins
int escPin = 11; //Pin for esc signal
int servoPin = 3; //Pin for servo signal

//serial
const byte numChars = 32;
char receivedChars[numChars];
char tempChars[numChars]; // temporary array for use when parsing
boolean newData = false;
boolean configSet = false;
char messageType[numChars] = {0};
int attempts = 0;
String tmpstr;

//Recieved data
float initialTurn = 0;
float initialThrottle = 0;
float turnAngle = 90;
float throttle = 90;
int noDataCount = 0;
int noDataLimit = 20;
int nearStartingWaypoint=0;

void setup() {
    Serial.begin(9600);
    myServo.attach(servoPin);
    myEsc.attach(escPin);
     pinMode(LED_BUILTIN, OUTPUT);
}

//============

void loop() {
  if (!configSet) { //Send hello message to RPi until configs have been set
    Serial.println("hello?");
    delay(1000);
    readInConfigData();
  }

  if(configSet){// Receive control commands from RPi
    delay(100);
    if(!readInControlData()){
      noDataCount++;
    } else {
      noDataCount=0;
    }
  }

  if(nearStartingWaypoint==0){
    blinkLED();
  }

  if (noDataCount>noDataLimit){
    noDataCount = 0;
    configSet=false;
    Serial.println("Is the RPi alive? I'm stopping the vehicle!");
  }

}

boolean readInConfigData(){
  if (Serial.available() > 0) { //Recieved data
    Serial.readBytes(tempChars,numChars);
    serialBufferFlush(); //clear remaining data
    parseConfigData();
    return true;
  } else {
    return false;
  }
}

boolean readInControlData(){
  if (Serial.available() > 0) { //Recieved data
    Serial.readBytes(tempChars,numChars);
    serialBufferFlush(); //clear remaining data
    parseControlData();
    return true;
  } else {
    return false;
  }
}

void blinkLED(){
  digitalWrite(LED_BUILTIN, LOW);   // turn the LED off 
  delay(500);                       // wait for half a second
  digitalWrite(LED_BUILTIN, HIGH);    // turn the LED on
  delay(500);                       // wait for half a second
}

void parseConfigData() {      // split the data into its parts. Borrowed from https://forum.arduino.cc/index.php?topic=396450.0

  char * strtokIndx; // this is used by strtok() and strtof() as an index

  strtokIndx = strtok(tempChars, ":");
  strcpy(messageType, strtokIndx);

  if (*messageType == 'C') {//if configs are being set

    strtokIndx = strtok(NULL, ":");
    initialTurn = atof(strtokIndx);
    myServo.write(initialTurn);
    strtokIndx = strtok(NULL, ":");
    initialThrottle = atof(strtokIndx);
    myEsc.write(initialThrottle);
    Serial.println("thank you!");
    Serial.flush();
    configSet = true;
    //serialBufferFlush(); //Uncomment if Arduino is using old serial messages
  }

}

void parseControlData() {      // split the data into its parts. Borrowed from https://forum.arduino.cc/index.php?topic=396450.0

  char * strtokIndx; // this is used by strtok() as an index

  strtokIndx = strtok(tempChars, ":");
  strcpy(messageType, strtokIndx);

  if (*messageType == 'D') { //if controls are being set

    strtokIndx = strtok(NULL, ":");
    turnAngle = atof(strtokIndx);
    myServo.write(turnAngle);

    strtokIndx = strtok(NULL, ":");
    throttle = atof(strtokIndx);
    myEsc.write(throttle);

    strtokIndx = strtok(NULL, ":");
    nearStartingWaypoint = atof(strtokIndx);
    
    String str1 = String(turnAngle);
    Serial.println(String("ack "+str1));
    Serial.flush(); //This function does not clear the input buffer. Please refer to the second post in this thread: https://forum.arduino.cc/index.php?topic=396450.0
    serialBufferFlush(); //Uncomment if Arduino is using old serial messages
  }
}

void serialBufferFlush() {
  while (Serial.available() > 0) {
  Serial.read();
  }
}

//============

//void readInData2() { //Borrowed from https://forum.arduino.cc/index.php?topic=396450.0
//    static boolean recvInProgress = false;
//    static byte ndx = 0;
//    char startMarker = '<';
//    char endMarker = '>';
//    char rc;
//
//    while (Serial.available() > 0 && newData == false) {
//        rc = Serial.read();
//        if (recvInProgress == true) {
//            if (rc != endMarker) {
//                receivedChars[ndx] = rc;
//                ndx++;
//                if (ndx >= numChars) {
//                    ndx = numChars - 1;
//                }
//            }
//            else {
//                receivedChars[ndx] = '\0'; // terminate the string
//                recvInProgress = false;
//                ndx = 0;
//                newData = true;
//            }
//        }
//
//        else if (rc == startMarker) {
//            recvInProgress = true;
//        }
//    }
//}