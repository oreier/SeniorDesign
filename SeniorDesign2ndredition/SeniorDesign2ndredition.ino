//LIRBARIES!!!
#include <Servo.h>
#include <Wire.h>
#include "Adafruit_APDS9960.h"
#include "DYPlayerArduino.h"
#include "Adafruit_NeoPixel.h"


// DEFINITIONS
#define SOUND_INTERVAL 10000   // Interval between sound plays in milliseconds

#define AUDIO_TX_PIN 0
#define AUDIO_RX_PIN 1

#define COLOR_RIGHT_SDA_PIN 2
#define COLOR_RIGHT_SCL_PIN 3

#define COLOR_LEFT_SDA_PIN 4
#define COLOR_LEFT_SCL_PIN 5

#define HAND_RIGHT_SERVO_PIN 7   // Pin number for the hand right servo
#define HAND_LEFT_SERVO_PIN 6    // Pin number for the hand left servo

#define ELBOW_RIGHT_SERVO_PIN 8        // Pin number for the elbow right servo motor
#define ELBOW_LEFT_SERVO_PIN 9        // Pin number for the elbow left servo motor

#define RIGHT_LINEAR_ACTUATOR_PIN_UP 10  // Pin number for the right shoulder linear actuator
#define RIGHT_LINEAR_ACTUATOR_PIN_DOWN 11  // Pin number for the right shoulder linear actuator
#define LEFT_LINEAR_ACTUATOR_PIN_UP 12   // Pin number for the left shoulder linear actuator
#define LEFT_LINEAR_ACTUATOR_PIN_DOWN 13   // Pin number for the left shoulder linear actuator

#define ELECTRO_MAG_RIGHT_PIN 14    // pin number for right elctro magnet 
#define ELECTRO_MAG_LEFT_PIN 15    // pin number for left elctro magnet 

#define LED_PIN     16
#define LED_COUNT  44 // how many led's are the the led strip
// NeoPixel brightness, 0 (min) to 255 (max)
#define BRIGHTNESS 200 // Set BRIGHTNESS to about 1/5 (max = 255)

#define CLOSED 0
#define OPEN 100

int minServo = 500;
int maxServo = 2500;

int red = 0;
int blue = 0;
int yellow = 0;

const int purple[] = {128,0,128};
const int green[] = {0,255,0};
const int orange[] = {255, 69, 0};

const int RED_THRESHOLD = 1000;
const int BLUE_THRESHOLD = 30000;
const int YELLOW_THRESHOLD = 45000;

const int PROXIMITY_THRESHOLD = 175;

// time keepers
unsigned long previousMoveTime = 0;
unsigned long previousSoundTime = 0;
unsigned long proximityRight = 0;
unsigned long proximityLeft = 0;

char soundYawn[] = "/00001*MP3";
char soundBloop[] = "/00002*MP3";
char soundBloop2[] = "/00003*MP3";
char greenBeaker[] = "/00004*MP3";
char purpleBeaker[] = "/00005*MP3";
char orangeBeaker[] = "/00006*MP3";


// // Array of sound file names
// char* soundPaths[] = {
//   "/00002*MP3",
//   "/00003*MP3"
// };

int numberOfSounds = 2;

enum Sides{
  LEFT,
  RIGHT,
  BOTH,
  UP,
  DOWN
};

enum RobotState {
  IDLE,
  MOVEMENT
};


// VARIABLES
DY::Player player(&Serial1);

RobotState currentState = IDLE;

Servo handLeftServo;
Servo handRightServo;

Servo leftElbowServo;
Servo rightElbowServo;
 
Adafruit_APDS9960 colorSensorR;
Adafruit_APDS9960 colorSensorL;

Adafruit_NeoPixel strip = Adafruit_NeoPixel(LED_COUNT, LED_PIN, NEO_GRB + NEO_KHZ800);

bool colorDetected = false;

int LHandClosed = 35;
int LHandOpen = 70;
int RHandClosed = 80;
int RHandOpen = 30;

int RBowBent = 10;
int RBowStraight = 85;
int LBowBent = 90;
int LBowStraight = 15;

void setup() {
  
  pinMode(RIGHT_LINEAR_ACTUATOR_PIN_UP, OUTPUT);
  pinMode(LEFT_LINEAR_ACTUATOR_PIN_UP, OUTPUT);
  pinMode(RIGHT_LINEAR_ACTUATOR_PIN_DOWN, OUTPUT);
  pinMode(LEFT_LINEAR_ACTUATOR_PIN_DOWN, OUTPUT);
  
  handLeftServo.attach(HAND_LEFT_SERVO_PIN, minServo, maxServo);
  handRightServo.attach(HAND_RIGHT_SERVO_PIN, minServo, maxServo);

  leftElbowServo.attach(ELBOW_LEFT_SERVO_PIN, minServo, maxServo);
  rightElbowServo.attach(ELBOW_RIGHT_SERVO_PIN, minServo, maxServo);

  handRightServo.write(RHandOpen);
  handLeftServo.write(LHandOpen);
  leftElbowServo.write(LBowStraight);
  rightElbowServo.write(RBowStraight);
  

  // Set the electromagnet control pins as outputs
  pinMode(ELECTRO_MAG_RIGHT_PIN, OUTPUT);
  pinMode(ELECTRO_MAG_LEFT_PIN, OUTPUT);

  // Initially set both electromagnets to LOW
  digitalWrite(ELECTRO_MAG_RIGHT_PIN, LOW);
  digitalWrite(ELECTRO_MAG_LEFT_PIN, LOW);
  
  //set pins. Optional for Wire, not for Wire1
  //Wire is right, Wire1 is left
  Wire1.setSDA(2);
  Wire1.setSCL(3);
  Wire.setSDA(4);
  Wire.setSCL(5);
  
  //initialize sensor 1 and check initialization simultaneously
  if(!colorSensorR.begin()){
    Serial.println("failed to initialize sensor device R! Please check your wiring.");
  }
  else Serial.println("Sensor R Device initialized!");
  
  if(!colorSensorL.begin(10, APDS9960_AGAIN_4X, APDS9960_ADDRESS, &Wire1)){

    Serial.println("failed to initialize sensor device L! Please check your wiring.");
  }
  else Serial.println("Sensor L Device initialized!");
  
  colorSensorL.enableColor(true);
  colorSensorR.enableColor(true);
  colorSensorL.enableProximity(true);
  colorSensorR.enableProximity(true);
  
  //audio player
  player.begin();
  //player.setPlayingDevice(1);

  //led strip
  strip.begin();           
  strip.show();            // Turn OFF all led's ASAP
  strip.setBrightness(BRIGHTNESS);


  //calibrate shoulders
  digitalWrite(RIGHT_LINEAR_ACTUATOR_PIN_UP, HIGH);
  digitalWrite(LEFT_LINEAR_ACTUATOR_PIN_UP, HIGH);
  delay(12000);
  digitalWrite(RIGHT_LINEAR_ACTUATOR_PIN_UP, LOW);
  digitalWrite(LEFT_LINEAR_ACTUATOR_PIN_UP, LOW);

  
  player.playSpecifiedDevicePath(DY::Device::Sd, soundYawn);
  
  digitalWrite(RIGHT_LINEAR_ACTUATOR_PIN_DOWN, HIGH);
  digitalWrite(LEFT_LINEAR_ACTUATOR_PIN_DOWN, HIGH);
  delay(6000);  //set to find preferred starting position
  digitalWrite(RIGHT_LINEAR_ACTUATOR_PIN_DOWN, LOW);
  digitalWrite(LEFT_LINEAR_ACTUATOR_PIN_DOWN, LOW);
  
}

void loop() {
  if (currentState == IDLE) {
    idleState();
  } else if (currentState == MOVEMENT) {
    movementState();
  }
  //Serial.println("Running!");
  //delay(100);
}


void idleState() {
  unsigned long currentTime = millis();
  
  proximityRight = colorSensorR.readProximity(); 
  // check if proximity trigger than there must be a beaker in the hand
  if (proximityRight > PROXIMITY_THRESHOLD) {
    Serial.println("beaker in hand, moving to movement state");
    digitalWrite(ELECTRO_MAG_RIGHT_PIN, HIGH);
    handRightServo.write(RHandClosed);  // closed hand is 180
    currentState = MOVEMENT; // transition to movement state
  }

  else if (proximityLeft > PROXIMITY_THRESHOLD) {
    Serial.println("beaker in hand, moving to movement state");
    digitalWrite(ELECTRO_MAG_LEFT_PIN, HIGH);
    handLeftServo.write(LHandClosed);
    currentState = MOVEMENT; // transition to movement state
  }
  
  // Move servo every MOVEMENT_INTERVAL milliseconds
  if (currentTime - previousMoveTime >= 5000) {
    previousMoveTime = currentTime;
    Serial.println("in idle state move!"); 
    player.playSpecifiedDevicePath(DY::Device::Sd, soundBloop);
    //playRandomSound();
    slightMovement(); // Call function to move the servo
  }

}

void slightMovement(){
  // Add slight movement
  handLeftServo.write(LHandClosed); //CHECK?????
  delay(3000); // Adjust delay based on your servo speed 
  handLeftServo.write(LHandOpen);
  delay(3000); // Adjust delay based on your servo speed 
}

// void playRandomSound() {
//   // Generate a random index to select a sound
//   int randomIndex = random(0, numberOfSounds);

//   // Get the selected sound path
//   char selectedSound = soundPaths[randomIndex];

//   // Print the selected sound for debugging
//   Serial.print("Playing sound: ");
//   Serial.println(selectedSound);

//   // Play the selected sound
//   player.playSpecifiedDevicePath(DY::Device::Sd, selectedSound);
// }
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
  delay(3000);
  currentState = IDLE;  // Transition back to idle state
}


void engageElectromagnet() {

  bool beakersDetected = false;

  // Code to engage the electromagnet
  Serial.println("electro magnets engaged");
  digitalWrite(ELECTRO_MAG_LEFT_PIN, HIGH);
  delay(1000);

  while (!beakersDetected) {
    proximityRight = colorSensorR.readProximity(); 
    proximityLeft = colorSensorL.readProximity(); 
    // check if proximity trigger than there must be a beaker in BOTH hands
    if (proximityRight > PROXIMITY_THRESHOLD && proximityLeft > PROXIMITY_THRESHOLD) {
      // Set flag to exit the while loop
      beakersDetected = true;
    }
    else {
      Serial.println("Waiting for beakers in both hands...");
      delay(500);
    }
  }

}

void closeHand() {
  Serial.println("close hand!!!!!");
  handLeftServo.write(LHandClosed);  // closed hand is 180     
  handRightServo.write(RHandClosed);  // closed hand is 180
  delay(3000);                    
}

void colorSensing(){

  Serial.println("starting color sensing");
  //redundant copies of variables are included to process both sets of variables independently
  uint16_t r, g, b, c, colorTemp, lux;
  
  uint16_t r1, g1, b1, c1, colorTemp1, lux1;

  //wait for color data to be ready
  while(!colorSensorR.colorDataReady() || !colorSensorL.colorDataReady()){
    delay(100);
  }

  colorSensorR.getColorData(&r, &g, &b, &c);
  colorSensorL.getColorData(&r1, &g1, &b1, &c1);

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
  // leftElbowServo.write(LBowBent);  // left elbow
  // rightElbowServo.write(RBowBent); // right elbow
  // Serial.println("done extending elbow!!!!!");
  // delay(1000); 

  int increment = 1;  // Increment value for gradual movement
  int delayTime = 20; // Delay time in milliseconds between each increment

  int currentPosLeft = 90;   // Read current position of left elbow servo
  int currentPosRight = rightElbowServo.read(); // Read current position of right elbow servo

  // Extend left elbow servo gradually to LBowBent position
  if (currentPosLeft < LBowBent) {
    for (int pos = currentPosLeft; pos <= LBowBent; pos -= increment) {
      leftElbowServo.write(pos);
      delay(delayTime);
    }
  }
  if (currentPosRight < RBowBent) {
    for (int pos = currentPosRight; pos <= RBowBent; pos += increment) {
      rightElbowServo.write(pos);
      delay(delayTime);
    }
  }
}


void turnOnLED() {
  Serial.println("LED!!!!!");
  if (red == 1 && yellow == 1) {
    player.playSpecifiedDevicePath(DY::Device::Sd, orangeBeaker);
    setColor(orange);
  }
  else if(red == 1 && blue == 1) {
    Serial.println("Purple!!!");
    player.playSpecifiedDevicePath(DY::Device::Sd, purpleBeaker);
    setColor(purple);
  }
  else if(yellow == 1 && blue == 1) {
    player.playSpecifiedDevicePath(DY::Device::Sd, greenBeaker);
    setColor(green);
  }
  else { // invalid combination
    setColorOff();
    //setColor(orange);
  }

  delay(8000);
}

void setColor(const int color[]) {

  uint8_t r = color[0];  // Extract red value from array
  uint8_t g = color[1];  // Extract green value from array
  uint8_t b = color[2];  // Extract blue value from array

  colorWipe(strip.Color(r, g, b), 50);
  
}

void setColorOff() {
  colorWipe(strip.Color(0, 0, 0), 50);
  red = 0;
  yellow = 0;
  blue = 0;
}

void returnArmElbow() {

// *********************** WHAT IS THIS ANGLE??? ELBOW
  Serial.println("return elbow!!!!!");
  leftElbowServo.write(LBowStraight);  // left elbow
  rightElbowServo.write(RBowStraight); // right elbow
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
  handLeftServo.write(LHandOpen);  // open hand is     
  handRightServo.write(RHandOpen);  // open hand is ?????WHAT ANGLE
  delay(3000);  

}

void releaseElectromagnet() {
  digitalWrite(ELECTRO_MAG_RIGHT_PIN, LOW);
  digitalWrite(ELECTRO_MAG_LEFT_PIN, LOW);
}

void colorWipe(uint32_t color, int wait) {
  for(int i=0; i<LED_COUNT; i++) {
    strip.setPixelColor(i, color);
    strip.show();
    delay(wait);
  }
}
