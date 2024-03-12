//LIRBARIES!!!
#include <Servo.h>
#include <Wire.h>
#include "Adafruit_TCS34725.h"
#include "DYPlayerArduino.h"


// DEFINITIONS
#define SOUND_INTERVAL 10000   // Interval between sound plays in milliseconds

#define AUDIO_TX_PIN 0
#define AUDIO_RX_PIN 1

#define COLOR_RIGHT_SDA_PIN 2
#define COLOR_RIGHT_SCL_PIN 3

#define COLOR_LEFT_SDA_PIN 4
#define COLOR_LEFT_SCL_PIN 5

#define LEFT_LINEAR_ACTUATOR_PIN 6   // Pin number for the left shoulder linear actuator
#define RIGHT_LINEAR_ACTUATOR_PIN 7  // Pin number for the right shoulder linear actuator

#define ELBOW_LEFT_SERVO_PIN 8        // Pin number for the elbow left servo motor
#define ELBOW_RIGHT_SERVO_PIN 9        // Pin number for the elbow right servo motor

#define LED_PIN_RED 10      // pin number for red pin led
#define  LED_PIN_GREEN 11 //pin number for green pin led
#define LED_PIN_BLUE 12 //pin number for blue pin led

#define HAND_LEFT_SERVO_PIN 13    // Pin number for the hand left servo
#define HAND_RIGHT_SERVO_PIN 14    // Pin number for the hand right servo
int minServo = 500;
int maxServo = 2500;


int red = LOW;
int blue = LOW;
int yellow = LOW;

const int purple[] = {128,0,128};
const int green[] = {0,255,0};
const int orange[] = {255,50,0};

const int RED_THRESHOLD = 50000;
const int BLUE_THRESHOLD = 30000;
const int YELLOW_THRESHOLD = 45000;


enum RobotState {
  IDLE,
  MOVEMENT
};


// VARIABLES
DY::Player player;

RobotState currentState = IDLE;
unsigned long previousSoundTime = 0;

Servo handLeftServo;
Servo handRightServo;

Servo leftElbowServo;
Servo rightElbowServo;
 
Adafruit_TCS34725 colorSensor1 = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_614MS, TCS34725_GAIN_1X);
Adafruit_TCS34725 colorSensor2 = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_614MS, TCS34725_GAIN_1X);

void setup() {

  pinMode(LEFT_LINEAR_ACTUATOR_PIN, OUTPUT);
  pinMode(RIGHT_LINEAR_ACTUATOR_PIN, OUTPUT);

  pinMode(LED_PIN_RED, OUTPUT);
  pinMode(LED_PIN_GREEN, OUTPUT);
  pinMode(LED_PIN_BLUE, OUTPUT);
  
  handLeftServo.attach(HAND_LEFT_SERVO_PIN, minServo, maxServo);
  handRightServo.attach(HAND_RIGHT_SERVO_PIN, minServo, maxServo);

  leftElbowServo.attach(ELBOW_LEFT_SERVO_PIN, minServo, maxServo);
  rightElbowServo.attach(ELBOW_RIGHT_SERVO_PIN, minServo, maxServo);

  //set pins. Optional for Wire, not for Wire1
  Wire1.setSDA(2);
  Wire1.setSCL(3);
  Wire.setSDA(4);
  Wire.setSCL(5);

  //initialize sensor 1 and check initialization simultaneously
  if (colorSensor1.begin(TCS34725_ADDRESS, &Wire)) {
    Serial.println("Found sensor");
  } else {
    Serial.println("No TCS34725 found ... check your connections");
    while (1);
  }

  //initialize sensor 2 and check initialization simultaneously
  if (colorSensor2.begin(TCS34725_ADDRESS, &Wire1)) {
    Serial.println("Found sensor");
  } else {
    Serial.println("No TCS34725 found ... check your connections");
    while (1);
  }
  
  // Additional setup code for other motors or sensors
  //audio player
  player.begin();
}

// void loop() {
//   switch (currentState) {
//     case IDLE:
//       idleState();
//       break;
      
//     case MOVEMENT:
//       movementState();
//       break;
//   }
// }

void loop() {
  if (currentState == IDLE) {
    idleState();
  } else if (currentState == MOVEMENT) {
    movementState();
  }
}


void idleState() {
  unsigned long currentTime = millis();
  Serial.println("in idle state"); 
  /*
  
  // Play sound every SOUND_INTERVAL milliseconds
  if (currentTime - previousSoundTime >= SOUND_INTERVAL) {
    previousSoundTime = currentTime;
    playSound();
  }
  
  // Check if the trigger pin is triggered
  if (digitalRead(TRIGGER_PIN) == LOW) {
    currentState = MOVEMENT;  // Transition to movement state
    initializeMovementState();
  }
  */

  // Add slight movement
  handRightServo.write(45); //CHECK?????
  delay(1000); // Adjust delay based on your servo speed 
 
}

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
  
  currentState = IDLE;  // Transition back to idle state
}

void initializeMovementState() {
  // Additional code to initialize the movement state
}

void engageElectromagnet() {
  // Code to engage the electromagnet
}

void closeHand() {
  handLeftServo.write(180);  // closed hand is 180     
  handRightServo.write(180);  // closed hand is 180
  delay(1000);                    
}

void colorSensing(){

  Serial.println("starting color sensing");
  //redundant copies of variables are included to process both sets of variables independently
  uint16_t r, g, b, c, colorTemp, lux;
  
  uint16_t r1, g1, b1, c1, colorTemp1, lux1;

  colorSensor1.getRawData(&r, &g, &b, &c);
  // colorTemp = colorSensor1.calculateColorTemperature(r, g, b);
  colorTemp = colorSensor1.calculateColorTemperature_dn40(r, g, b, c);
  lux = colorSensor1.calculateLux(r, g, b);

  colorSensor2.getRawData(&r1, &g1, &b1, &c1);
  // colorTemp = colorSensor2.calculateColorTemperature(r, g, b);
  colorTemp1 = colorSensor2.calculateColorTemperature_dn40(r1, g1, b1, c1);
  lux1 = colorSensor2.calculateLux(r1, g1, b1);

  setBeakerColors(r, g, b);
  setBeakerColors(r1, g1, b1);

}

void setBeakerColors(uint16_t r, uint16_t g, uint16_t b) {
  Serial.println("setting color!");
  
  // Red detected
  if (r > RED_THRESHOLD && g < YELLOW_THRESHOLD && b < YELLOW_THRESHOLD) {
    red = HIGH;
  }

  // Blue detected
  else if (r < BLUE_THRESHOLD && g < BLUE_THRESHOLD && b > BLUE_THRESHOLD) {
    blue = HIGH;
  }

  // Yellow detected
  else if (r > YELLOW_THRESHOLD && g > YELLOW_THRESHOLD && b < BLUE_THRESHOLD) {
    yellow = HIGH;
  }
}

// ************************* This way of doing the arm shoulders has got to change 
// because both shoulders should move at once 

void liftArmShoulder() {
  //liftArmShoulderLeft();
  //liftArmShoulderRight();
}
/*
void liftArmShoulderLeft() {
  // Code to move the left shoulder linear actuator to lift the arm elbow to a 90-degree angle
  digitalWrite(LEFT_LINEAR_ACTUATOR_PIN, HIGH);
  delay(2000); // adjust the delay time as needed to reach position
  digitalWrite(LEFT_LINEAR_ACTUATOR_PIN, LOW);
}

void liftArmShoulderRight() {
  // Code to move the right shoulder linear actuator to lift the arm elbow to a 90-degree angle
  digitalWrite(RIGHT_LINEAR_ACTUATOR_PIN, HIGH);
  delay(2000); // adjust the delay time as needed to reach position
  digitalWrite(RIGHT_LINEAR_ACTUATOR_PIN, LOW);
}
*/

void extendArmElbow() {

// *********************** WHAT IS THIS ANGLE??? ELBOW

  leftElbowServo.write(180);  // left elbow
  rightElbowServo.write(180); // right elbow
  delay(1000); 
}


void turnOnLED() {
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

  delay(100);
}

void setColor(const int color[]) {
  /*
  analogWrite(LED_PIN_RED, color[0]); // set red pin
  analogWrite(LED_PIN_GREEN, color[1]); // set green pin color
  analogWrite(LED_PIN_BLUE, color[2]); // set blue pin color
  */
}

void setColorOff() {
  /*
  digitalWrite(LED_PIN_RED, LOW); // set red pin off 
  digitalWrite(LED_PIN_GREEN, LOW); // set green pin off 
  digitalWrite(LED_PIN_BLUE, LOW); // set blue pin off 
  */
}

void returnArmElbow() {

// *********************** WHAT IS THIS ANGLE??? ELBOW

  leftElbowServo.write(180);  // left elbow
  rightElbowServo.write(180); // right elbow
  delay(1000); 

}

// ************************* This way of doing the arm shoulders has got to change 
// because both shoulders should move at once 

void returnArmShoulder() {
  //returnArmShoulderLeft();
  //returnArmShoulderRight();
}

void returnArmShoulderLeft() {
  // Code to move the left shoulder linear actuator to return the arm elbow to a 0 degree
  digitalWrite(LEFT_LINEAR_ACTUATOR_PIN, HIGH);
  delay(2000); // adjust the delay time as needed to reach position
  digitalWrite(LEFT_LINEAR_ACTUATOR_PIN, LOW);
}

void returnArmShoulderRight() {
  // Code to move the right shoulder linear actuator to return the arm elbow to a 0 degree
  digitalWrite(RIGHT_LINEAR_ACTUATOR_PIN, HIGH);
  delay(2000); // adjust the delay time as needed to reach position
  digitalWrite(RIGHT_LINEAR_ACTUATOR_PIN, LOW);
}


void openHand() {
  handLeftServo.write(0);  // open hand is     
  handRightServo.write(0);  // open hand is ?????WHAT ANGLE
  delay(1000);  

}

void releaseElectromagnet() {
  // Code to release the electromagnet
}



void playSound() {
  // Code to play the sound
  //playSpecifiedDevicePath(..)
  
  player.playSpecified(1);
}
