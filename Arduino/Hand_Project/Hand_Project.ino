/******************************************************************************
  Gale Curry
  Based off example using the LSM6DS3 IMU with basic settings.  This sketch collects Gyro and
  Accelerometer data every second. It uses that data to detect the state of the arm and
  control servos which control the fingers. 

  Development environment specifics:
  Arduino IDE 1.6.11
  Teensy loader 1.30
  Teensyduino 3.2

  Hardware connections:
  Connect I2C SDA line to A4
  Connect I2C SCL line to A5
  Connect GND and 3.3v power to the IMU
******************************************************************************/

#include "SparkFunLSM6DS3.h"
#include <Servo.h>

Servo myservo_1;  // create servo object to control a servo
Servo myservo_2;
Servo myservo_3;
Servo myservo_4;
LSM6DS3 myIMU; // Default constructor is I2C, addr 0x6B

// Set this to the hardware serial port you wish to use
#define HWSERIAL Serial3

// Pins
const int ledPin =  13;         // the number of the LED pin
const int pointerFingerPin = 3;
const int middleFingerPin = 4;
const int ringFingerPin = 5;
const int pinkyFingerPin = 6;
const int flexpin = 15;         // flex sensor pin

int ledState = LOW;
int pos = 0;    // variable to store the servo position
int Bluetooth_flag;

// ARM states
int armState; 
const int armNeutral = 0; 
const int armLowered_palmUp = 1;
const int armLowered_palmDown = 2;
const int armLowered_palmHorizontal = 3;
const int armRaised = 4;
const int armDown = 5;

// THUMB states
int thumbState;
int relaxed = 0;
int bent = 1;

// GYRO states
int angularState;
const int xtwist = 0;
const int ytwist = 1;
const int ztwist = 2; 
 
// GESTURE states
enum gesture{
 neutral, 
 point,
 come_hither,
 wave,
 peace,
 number_one,
 rock_on,
 middle,
 gun,
 hang_loose,
};

gesture current_state;
gesture prev_state;

bool changed = false; //flag to determine if the state has changed

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  HWSERIAL.begin(9600);
  delay(1000); // wait 1 sec
  Serial.println("Processor came out of reset.\n");

  //Call .begin() to configure the IMU
  myIMU.begin();

  // set the digital pin as output:
  pinMode(ledPin, OUTPUT);

  //set flag
  Bluetooth_flag = 0;
  
  // attaches the servo to the servo object
  myservo_1.attach(pointerFingerPin);
  myservo_2.attach(middleFingerPin);
  myservo_3.attach(ringFingerPin);
  myservo_4.attach(pinkyFingerPin);
  
  // Start out in neutral state
  prev_state = neutral; 
  current_state = neutral;

  // Sets all servos to zero degrees
  goToNeutral();
  delay(1000);
  
}

void loop()
{
  int BLE_msg = readSerial();

  if (BLE_msg == 48){
    Bluetooth_flag = 1;
    }
  else if (BLE_msg == 56){
    Bluetooth_flag = 0;
    }  
    
  //Get all parameters
  float accel_x = myIMU.readFloatAccelX();
  float accel_y = myIMU.readFloatAccelY();
  float accel_z = myIMU.readFloatAccelZ();
  float gyro_x = myIMU.readFloatGyroX(); 
  float gyro_y = myIMU.readFloatGyroY();
  float gyro_z = myIMU.readFloatGyroZ();

  //printAccels(accel_x, accel_y, accel_z);
  //printGyros(gyro_x, gyro_y, gyro_z); 
   
// Determining the state from either IMU or Bluetooth
  if (Bluetooth_flag){
     Serial.print("\n----------------- Bluetooth Control ---------------------\n");
    determineStatesBluetooth(BLE_msg); 
  }
  else{
    determineStatesIMU(accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z);
    prev_state = current_state;
    determineGesturefromStates(armState, angularState, thumbState);
    }  

  ////// FLEX SENSOR ///////

//  int flexposition;    // Input value from the analog pin.
//  int servoposition;   // Output value to the servo.
//  
//  // Read the position of the flex sensor (0 to 1023):
//  flexposition = analogRead(flexpin);
//  servoposition = map(flexposition, 600, 900, 0, 180);
//  servoposition = constrain(servoposition, 0, 180);
//  //myservo_2.write(servoposition);
//
//  // Because every flex sensor has a slightly different resistance,
//  // the 600-900 range may not exactly cover the flex sensor's
//  // output. To help tune our program, we'll use the serial port to
//  // print out our values to the serial monitor window:
//  Serial.print("\nFlex:\n");
//  Serial.print("Sensor: ");
//  Serial.print(flexposition);
//  Serial.print("  Servo: ");
//  Serial.println(servoposition);

  delay(1000);
}

///////////////////////////////
////// HELPER FUNCTIONS ///////
///////////////////////////////

void openFinger(Servo myservo) {
  int pos = 0;
  for (pos = 0; pos <= 180; pos += 1) { // goes from 0 degrees to 180 degrees
    // in steps of 1 degree
    myservo.write(pos);                // tell servo to go to position in variable 'pos'
    delay(15);                       // waits 15ms for the servo to reach the position
  }
}

void openFingerHalfway(Servo myservo) {
  int pos = 90;
  for (pos = 90; pos <= 180; pos += 1) { // goes from 90 degrees to 180 degrees
    // in steps of 1 degree
    myservo.write(pos);                // tell servo to go to position in variable 'pos'
    delay(15);                       // waits 15ms for the servo to reach the position
  }
}

void closeFinger(Servo myservo) {
  int pos = 180;
  for (pos = 180; pos >= 0; pos -= 1) { // goes from 180 degrees to 0 degrees
    myservo.write(pos);              // tell servo to go to position in variable 'pos'
    delay(15);                       // waits 15ms for the servo to reach the position
  }
}

void closeFingerHalfway(Servo myservo) {
  int pos = 180;
  for (pos = 180; pos >= 90; pos -= 1) { // goes from 180 degrees to 90 degrees
    myservo.write(pos);              // tell servo to go to position in variable 'pos'
    delay(15);                       // waits 15ms for the servo to reach the position
  }
}

// Gesture functions:

void goToNeutral(){
  int i;
  int current1 = myservo_1.read();
  for (i=current1; i>=0; i-=1){
    myservo_1.write(i);             
    delay(15); 
    } 
  int current2 = myservo_2.read();
  for (i=current2; i>=0; i-=1){
    myservo_2.write(i);             
    delay(15); 
    }
  int current3 = myservo_3.read();
  for (i=current3; i>=0; i-=1){
    myservo_3.write(i);             
    delay(15); 
    }
  int current4 = myservo_4.read();
  for (i=current4; i>=0; i-=1){
    myservo_4.write(i);             
    delay(15); 
    }
  }
  
void comeHither(){
  openFinger(myservo_1);
  closeFingerHalfway(myservo_1);
  openFingerHalfway(myservo_1);
  closeFingerHalfway(myservo_1);
  openFingerHalfway(myservo_1);
  closeFinger(myservo_1);
  }
void pointing(){
  openFinger(myservo_1);
  }
void rockOn(){
  openFinger(myservo_1);
  openFinger(myservo_4);
  }
void allOpen(){
  openFinger(myservo_1);
  openFinger(myservo_2);
  openFinger(myservo_3);
  openFinger(myservo_4);
  } 
void gunHand(){
  openFinger(myservo_1);
  openFinger(myservo_2);
  }
void hangLoose(){
  openFinger(myservo_4);
  }
void flipTheBird(){
  openFinger(myservo_3);
  }
  
void determineStatesIMU(float accel_x, float accel_y, float accel_z, float gyro_x, float gyro_y, float gyro_z){
  // Determining the ARM State....
  if (accel_x > 0.75){    // if x is 1
    armState = armRaised; // two states are here!!
    }
  else if (accel_x < -0.75){
    armState = armDown;
    }  
  else if (accel_x <0.3){
    if (accel_y > 0.75){
      armState = armLowered_palmHorizontal; 
      }
    else if (accel_y < 0.75 && accel_z > 0.75){
      armState = armLowered_palmDown;
      }
     else if (accel_y <0.75 && accel_z < -0.75){
      armState = armLowered_palmUp;    
     }
    }
  else {
    armState = armNeutral; 
    }    
  // Determining the THUMB state:
  int flexposition= analogRead(flexpin);    // Input value from the analog pin. 
    if (flexposition > 800){
      thumbState = bent;
      }
    else{
      thumbState = relaxed;
      }  
  // Determining GYRO motion 
  if (gyro_x > 100 && gyro_y < 100 && gyro_z < 100){
    angularState = xtwist;
    }
  else if (gyro_y > 100 && gyro_x < 100 && gyro_z <100){
    angularState = ytwist;
    }
  else if(gyro_z > 100 && gyro_x < 100 && gyro_y < 100){
    angularState = ztwist;
    }
}

void determineGesturefromStates(int armState, int angularState, int thumbState){
  Serial.print("\nArm State: ");
  if (armState == 0){
    Serial.print("Neutral");
    Serial.print("\nGesture State: Neutral");
    goToNeutral();
    }
    
  else if (armState == 1){
    Serial.print("Arm Lowered, Palm Up");
    current_state = come_hither;
    Serial.print("\nGesture Sate: Come Hither "); 
    if (changed){
      // "Come hither"
      comeHither();
      changed = false;
      } 
    }
     
  else if (armState == 2){
    Serial.print("Arm Lowered, Palm Down");
    Serial.print("\nState: Pointing");
    current_state = point;
    if (changed){
      pointing();
      //allOpen();
      changed = false;
      }  
    }
    
  else if (armState == 3){
    Serial.print("Arm Lowered, Palm Horizontal");
    Serial.print("\nState: Gun Hand");
    current_state = gun;
     if (changed){
      gunHand();
      changed = false;
      }  
    } 
  else if (armState == 4){
    Serial.print("Arm Raised");
    if (angularState == 0){
      Serial.print("\nState: Hang Loose");
      current_state = hang_loose;
      if (changed){
        hangLoose();
        changed = false;
        } 
      }
    else if (angularState == 1){
      Serial.print("\nState: Rock On");
      current_state = rock_on;
      if (changed){
        rockOn();
        changed = false;
        } 
      }
    else if (angularState == 2){
      Serial.print("\nState: Waving");
      current_state = wave;
      if (changed){
        allOpen();
        changed = false;
        } 
      }
    }
    
  else if (armState == 5){
    Serial.print("Arm Down");
    Serial.print("\nState: At rest");
      current_state = neutral;
      if (changed){
        goToNeutral();
        changed = false;
        } 
    }  
    
  Serial.print("\nThumb State: ");  
  if (thumbState == 0){
    Serial.print("Relaxed");
    }
  else{
    Serial.print("Bent");
    }
  //Serial.print(" - ");  
  //Serial.println(flexposition);

  //Check if state has changed
  if (prev_state == current_state){
    changed = false; 
    }
  else{
    changed = true;
    ledState = HIGH;
    digitalWrite(ledPin, ledState);
    Serial.print("\n---------STATE HAS CHANGED---------\n");
    goToNeutral();
    ledState = LOW;
    digitalWrite(ledPin, ledState);
    } 
  }

void determineStatesBluetooth(int msg){
  if(msg == 55){
    current_state = come_hither;
    comeHither();
    }
  else if(msg == 53){
    current_state = wave;
    } 
  else if(msg == 49){
    current_state = point;
    pointing();
    } 
  else if(msg == 50){
    current_state = hang_loose;
    hangLoose();
    }
  else if (msg == 51){
    current_state = rock_on;
    rockOn();
  }
  else if (msg == 54){
    current_state = middle;
    flipTheBird();
  }
  else if (msg == 52){
    current_state = gun;
    gunHand();
    }  
  else if (msg == 57){
    current_state = neutral;
    goToNeutral();
    } 
  }
  
int readSerial(){
  int incomingByte;

  if (HWSERIAL.available() > 0) {
    incomingByte = HWSERIAL.read();
    Serial.print("\nUART received: ");
    Serial.println(incomingByte, DEC);
    HWSERIAL.print("UART received:");
    HWSERIAL.println(incomingByte, DEC);
  }
  else{
    incomingByte = 0;
    }
  return incomingByte;
  
// USB serial can be used for debugging  
//  if (Serial.available() > 0) {
//    incomingByte = Serial.read();
//    Serial.print("\nUSB received: ");
//    Serial.println(incomingByte, DEC);
//    HWSERIAL.write(incomingByte);
//    HWSERIAL.print("USB received:");
//    HWSERIAL.println(incomingByte, DEC);
//  }
}

void printAccels(float accel_x, float accel_y, float accel_z){
  Serial.print("\nAccelerometer:\n");
  Serial.print(" X = ");
  Serial.println(accel_x, 4);
  Serial.print(" Y = ");
  Serial.println(accel_y, 4);
  Serial.print(" Z = ");
  Serial.println(accel_z, 4);  
  }
void printGyros(float gyro_x, float gyro_y, float gyro_z){
  Serial.print("\nGyroscope:\n");
  Serial.print(" X = ");
  Serial.println(gyro_x, 4);
  Serial.print(" Y = ");
  Serial.println(gyro_y, 4);
  Serial.print(" Z = ");
  Serial.println(gyro_z, 4);
  }
