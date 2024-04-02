//LIRBARIES!!!
#include <Servo.h>
#include <Wire.h>
#include "Adafruit_APDS9960.h"
#include "DYPlayerArduino.h"


// DEFINITIONS
#define SOUND_INTERVAL 10000   // Interval between sound plays in milliseconds

#define AUDIO_TX_PIN 0
#define AUDIO_RX_PIN 1

#define COLOR_RIGHT_SDA_PIN 2
#define COLOR_RIGHT_SCL_PIN 3

#define COLOR_LEFT_SDA_PIN 4
#define COLOR_LEFT_SCL_PIN 5

#define HAND_RIGHT_SERVO_PIN 6   // Pin number for the hand right servo
#define HAND_LEFT_SERVO_PIN 7    // Pin number for the hand left servo

#define ELBOW_RIGHT_SERVO_PIN 8        // Pin number for the elbow right servo motor
#define ELBOW_LEFT_SERVO_PIN 9        // Pin number for the elbow left servo motor

#define RIGHT_LINEAR_ACTUATOR_PIN_UP 10  // Pin number for the right shoulder linear actuator
#define RIGHT_LINEAR_ACTUATOR_PIN_DOWN 11  // Pin number for the right shoulder linear actuator
#define LEFT_LINEAR_ACTUATOR_PIN_UP 12   // Pin number for the left shoulder linear actuator
#define LEFT_LINEAR_ACTUATOR_PIN_DOWN 13   // Pin number for the left shoulder linear actuator

#define ELCTRO_MAG_RIGHT_PIN 14    // pin number for right elctro magnet 
#define ELCTRO_MAG_LEFT_PIN 15    // pin number for left elctro magnet 

#define LED_PIN_BLUE 16 //pin number for blue pin led
#define  LED_PIN_GREEN 17 //pin number for green pin led
#define LED_PIN_RED 18     // pin number for red pin led

int minServo = 500;
int maxServo = 2500;


int red = 0;
int blue = 0;
int yellow = 0;

const int purple[] = {128,0,128};
const int green[] = {0,255,0};
const int orange[] = {255,50,0};

const int RED_THRESHOLD = 1000;
const int BLUE_THRESHOLD = 30000;
const int YELLOW_THRESHOLD = 45000;

// time keepers
unsigned long previousMoveTime = 0;
unsigned long previousSoundTime = 0;


enum RobotState {
  IDLE,
  MOVEMENT
};


// VARIABLES
DY::Player player(&Serial1);

RobotState currentState = MOVEMENT;

Servo handLeftServo;
Servo handRightServo;

Servo leftElbowServo;
Servo rightElbowServo;
 
Adafruit_APDS9960 colorSensor1;
Adafruit_APDS9960 colorSensor2;

bool colorDetected = false;

void setup() {
  pinMode(RIGHT_LINEAR_ACTUATOR_PIN_UP, OUTPUT);
  pinMode(LEFT_LINEAR_ACTUATOR_PIN_UP, OUTPUT);
  pinMode(RIGHT_LINEAR_ACTUATOR_PIN_DOWN, OUTPUT);
  pinMode(LEFT_LINEAR_ACTUATOR_PIN_DOWN, OUTPUT);

  pinMode(LED_PIN_RED, OUTPUT);
  pinMode(LED_PIN_GREEN, OUTPUT);
  pinMode(LED_PIN_BLUE, OUTPUT);
  
  handLeftServo.attach(HAND_LEFT_SERVO_PIN, minServo, maxServo);
  handRightServo.attach(HAND_RIGHT_SERVO_PIN, minServo, maxServo);

  leftElbowServo.attach(ELBOW_LEFT_SERVO_PIN, minServo, maxServo);
  rightElbowServo.attach(ELBOW_RIGHT_SERVO_PIN, minServo, maxServo);

  //set pins. Optional for Wire, not for Wire1
  Wire.setSDA(2);
  Wire.setSCL(3);
  Wire1.setSDA(4);
  Wire1.setSCL(5);

  //initialize sensor 1 and check initialization simultaneously
  if (colorSensor1.begin()) {
    Serial.println("Found sensor");
  } else {
    Serial.println("No colorSensor1 found ... check your connections");
    while (1);
  }

  colorSensor1.enableColor(true);
  //0x39, APDS9960_GGAIN_1, &Wire1
  //initialize sensor 2 and check initialization simultaneously
  if (colorSensor2.begin()) {
    Serial.println("Found sensor");
  } else {
    Serial.println("No colorSensor2 found ... check your connections");
    while (1);
  }

  colorSensor2.enableColor(true);
  
  //audio player
  player.begin();
}

void loop() {
  if (currentState == IDLE) {
    idleState();
  } else if (currentState == MOVEMENT) {
    movementState();
  }
}


void idleState() {
  unsigned long currentTime = millis();
  /*
  
  // Check if the trigger pin is triggered
  if (digitalRead(TRIGGER_PIN) == LOW) {
    currentState = MOVEMENT;  // Transition to movement state
    initializeMovementState();
  }
  */
  
  // Move servo every MOVEMENT_INTERVAL milliseconds
  if (currentTime - previousMoveTime >= 15000) {
    previousMoveTime = currentTime;
    Serial.println("in idle state move!"); 
    playSound();
    slightMovement(); // Call function to move the servo
  }

}

void slightMovement(){
  // Add slight movement
  handRightServo.write(45); //CHECK?????
  delay(3000); // Adjust delay based on your servo speed 
  handRightServo.write(90);
  delay(3000); // Adjust delay based on your servo speed 
}


// MOVEMENT STATE****************************************************************************************************


void movementState() {

  Serial.println("entering the movement state");
  engageElectromagnet();
  closeHand();
  colorSensing();
  liftArmShoulder();
  extendArmElbow();
  turnOnLED();
  setColorOff();
  returnArmElbow();
  returnArmShoulder();
  openHand();
  releaseElectromagnet();
  
  currentState = MOVEMENT;  // Transition back to idle state
}

void initializeMovementState() {
  // Additional code to initialize the movement state
}

void engageElectromagnet() {
  // Code to engage the electromagnet
  Serial.println("electro engaged");
  delay(1000);
}

void closeHand() {
  Serial.println("close hand!!!!!");
  handLeftServo.write(90);  // closed hand is 180     
  handRightServo.write(90);  // closed hand is 180
  delay(3000);                    
}

void colorSensing(){

  Serial.println("starting color sensing");
  //redundant copies of variables are included to process both sets of variables independently
  uint16_t r, g, b, c, colorTemp, lux;
  
  uint16_t r1, g1, b1, c1, colorTemp1, lux1;

  //wait for color data to be ready
  while(!colorSensor1.colorDataReady() || !colorSensor2.colorDataReady()){
    delay(5);
  }

  colorSensor1.getColorData(&r, &g, &b, &c);
  // colorTemp = colorSensor1.calculateColorTemperature(r, g, b);
  //colorTemp = colorSensor1.calculateColorTemperature_dn40(r, g, b, c);
  //lux = colorSensor1.calculateLux(r, g, b);

  colorSensor2.getColorData(&r1, &g1, &b1, &c1);
  // colorTemp = colorSensor2.calculateColorTemperature(r, g, b);
  //colorTemp1 = colorSensor2.calculateColorTemperature_dn40(r1, g1, b1, c1);
  //lux1 = colorSensor2.calculateLux(r1, g1, b1);

  setBeakerColors(r, g, b, c);
  setBeakerColors(r1, g1, b1, c1);

}

void setBeakerColors(uint16_t r, uint16_t g, uint16_t b, uint16_t c) {
  Serial.println("setting color!");
  Serial.println(r);
  Serial.println(g);
  Serial.println(b);
  Serial.println(c);

  // Red detected
  if (r > g && r > b) {
    red = 1;
    Serial.println("is red");
    colorDetected = true; 
    delay(1000);
  }

  // Blue detected
  else if (b > r && b > g) {
    blue = 1;
    Serial.println("is blue");
    colorDetected = true;
    delay(1000);
  }

  // Yellow detected
  else if (r > b && g > b) {
    yellow = 1;
    Serial.println("is yellow");
    delay(1000);
  }
}


void liftArmShoulder() {
  Serial.println("Arm shoulder opens");
  digitalWrite(RIGHT_LINEAR_ACTUATOR_PIN_UP, HIGH);
  digitalWrite(LEFT_LINEAR_ACTUATOR_PIN_UP, HIGH);
  delay(5000); // adjust the delay time as needed to reach position
  digitalWrite(RIGHT_LINEAR_ACTUATOR_PIN_UP, LOW);
  digitalWrite(LEFT_LINEAR_ACTUATOR_PIN_UP, LOW);

}

void extendArmElbow() {

// *********************** angle 90 - 180

  //for(pos = 0; pos<=90; pos+=1);

  //Serial.println("extending elbow!!!!!");
  leftElbowServo.write(180);  // left elbow
  rightElbowServo.write(180); // right elbow
  Serial.println("done extending elbow!!!!!");
  delay(1000); 
}


void turnOnLED() {
  Serial.println("LED!!!!!");
  if (red == HIGH && yellow == HIGH) {
    setColor(orange);
  }
  else if(red == HIGH && blue == HIGH) {
    setColor(purple);
  }
  else if(yellow == HIGH && blue == HIGH) {
    setColor(green);
  }
  else { // invalid combination
    setColorOff();
  }

  delay(3000);
}

void setColor(const int color[]) {
  
  analogWrite(LED_PIN_RED, color[0]); // set red pin
  analogWrite(LED_PIN_GREEN, color[1]); // set green pin color
  analogWrite(LED_PIN_BLUE, color[2]); // set blue pin color
  
}

void setColorOff() {
  
  analogWrite(LED_PIN_RED, 255); // set red pin off 
  analogWrite(LED_PIN_GREEN, 255); // set green pin off 
  analogWrite(LED_PIN_BLUE, 255); // set blue pin off 
  
}

void returnArmElbow() {

// *********************** WHAT IS THIS ANGLE??? ELBOW
  Serial.println("return elbow!!!!!");
  leftElbowServo.write(90);  // left elbow
  rightElbowServo.write(90); // right elbow
  delay(3000); 

}

void returnArmShoulder() {
  Serial.println("Arm shoulder returns");
  digitalWrite(RIGHT_LINEAR_ACTUATOR_PIN_DOWN, HIGH);
  digitalWrite(LEFT_LINEAR_ACTUATOR_PIN_DOWN, HIGH);
  delay(5000); // adjust the delay time as needed to reach position
  digitalWrite(RIGHT_LINEAR_ACTUATOR_PIN_DOWN, LOW);
  digitalWrite(LEFT_LINEAR_ACTUATOR_PIN_DOWN, LOW);
}


void openHand() {
  Serial.println("open hand!!!!!");
  handLeftServo.write(0);  // open hand is     
  handRightServo.write(0);  // open hand is ?????WHAT ANGLE
  delay(3000);  

}

void releaseElectromagnet() {
  // Code to release the electromagnet
}

void playSound() {
  // Code to play the sound
  //playSpecifiedDevicePath(..)
  player.playSpecified(1);
}
