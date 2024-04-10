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

#define HAND_RIGHT_SERVO_PIN 6   // Pin number for the hand right servo
#define HAND_LEFT_SERVO_PIN 7    // Pin number for the hand left servo

#define ELBOW_RIGHT_SERVO_PIN 8        // Pin number for the elbow right servo motor
#define ELBOW_LEFT_SERVO_PIN 9        // Pin number for the elbow left servo motor

#define RIGHT_LINEAR_ACTUATOR_PIN_UP 10  // Pin number for the right shoulder linear actuator
#define RIGHT_LINEAR_ACTUATOR_PIN_DOWN 11  // Pin number for the right shoulder linear actuator
#define LEFT_LINEAR_ACTUATOR_PIN_UP 12   // Pin number for the left shoulder linear actuator
#define LEFT_LINEAR_ACTUATOR_PIN_DOWN 13   // Pin number for the left shoulder linear actuator

#define ELECTRO_MAG_RIGHT_PIN 14    // pin number for right elctro magnet 
#define ELECTRO_MAG_LEFT_PIN 15    // pin number for left elctro magnet 

#define LED_PIN_BLUE 16 //pin number for blue pin led
#define  LED_PIN_GREEN 17 //pin number for green pin led
#define LED_PIN_RED 18     // pin number for red pin led

#define LED_PIN     16
#define LED_COUNT  44 // how many led's are the the led strip
// NeoPixel brightness, 0 (min) to 255 (max)
#define BRIGHTNESS 200 // Set BRIGHTNESS to about 1/5 (max = 255)

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

const int PROXIMITY_THRESHOLD = 175;

// time keepers
unsigned long previousMoveTime = 0;
unsigned long previousSoundTime = 0;
unsigned long proximityRight = 0;
unsigned long proximityLeft = 0;


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

Adafruit_NeoPixel strip(LED_COUNT, LED_PIN, NEO_GRBW + NEO_KHZ800);

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

  // Set the electromagnet control pins as outputs
  pinMode(ELECTRO_MAG_RIGHT_PIN, OUTPUT);
  pinMode(ELECTRO_MAG_LEFT_PIN, OUTPUT);

  // Initially set both electromagnets to LOW
  digitalWrite(ELECTRO_MAG_RIGHT_PIN, LOW);
  digitalWrite(ELECTRO_MAG_LEFT_PIN, LOW);

  //set pins. Optional for Wire, not for Wire1
  Wire1.setSDA(2);
  Wire1.setSCL(3);
  Wire.setSDA(4);
  Wire.setSCL(5);

  //initialize sensor 1 and check initialization simultaneously
  if(!colorSensor1.begin(10, APDS9960_AGAIN_4X, APDS9960_ADDRESS, &Wire)){
    Serial.println("Found sensor");
  } else {
    Serial.println("No colorSensor1 found ... check your connections");
    while (1);
  }
  
  if(!colorSensor2.begin(10, APDS9960_AGAIN_4X, APDS9960_ADDRESS, &Wire1)){
    Serial.println("Found sensor");
  } else {
    Serial.println("No colorSensor2 found ... check your connections");
    while (1);
  }

  colorSensor1.enableColor(true);
  colorSensor2.enableColor(true);
  colorSensor1.enableProximity(true);
  
  //audio player
  player.begin();

  //led strip
  strip.begin();           
  strip.show();            // Turn OFF all led's ASAP
  strip.setBrightness(BRIGHTNESS);
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
  proximityRight = colorSensor1.readProximity(); 
  // check if proximity trigger than there must be a beaker in the hand
  if (proximityRight > PROXIMITY_THRESHOLD) {
    Serial.println("beaker in hand, moving to movement state");
    currentState = MOVEMENT; // transition to movement state
  }
  
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
  handLeftServo.write(45); //CHECK?????
  delay(3000); // Adjust delay based on your servo speed 
  handLeftServo.write(90);
  delay(3000); // Adjust delay based on your servo speed 
}
void playSound() {
  // Code to play the sound
  //playSpecifiedDevicePath(..)
  player.playSpecified(1);
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


void engageElectromagnet() {

  bool beakersDetected = false;

  // Code to engage the electromagnet
  Serial.println("electro magnets engaged");
  digitalWrite(ELECTRO_MAG_RIGHT_PIN, HIGH);
  digitalWrite(ELECTRO_MAG_LEFT_PIN, HIGH);
  delay(1000);

  while (!beakersDetected) {
    proximityRight = colorSensor1.readProximity(); 
    proximityLeft = colorSensor2.readProximity(); 
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
    delay(100);
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
  if (red == 1 && yellow == 1) {
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

  uint8_t r = color[0];  // Extract red value from array
  uint8_t g = color[1];  // Extract green value from array
  uint8_t b = color[2];  // Extract blue value from array
  
  for(int i=0; i<LED_COUNT; i++) { // For each pixel...

    // pixels.Color() takes RGB values, from 0,0,0 up to 255,255,255
    strip.setPixelColor(i, strip.Color(r, g, b));

    strip.show();   // Send the updated pixel colors to the hardware.

    delay(5); // Pause before next pass through loop
  }
  
}

void setColorOff() {
  strip.clear();
  rainbow(10);
  
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
  digitalWrite(ELECTRO_MAG_RIGHT_PIN, LOW);
  digitalWrite(ELECTRO_MAG_LEFT_PIN, LOW);
}

// Rainbow cycle along whole strip. Pass delay time (in ms) between frames.
void rainbow(int wait) {
  // Hue of first pixel runs 5 complete loops through the color wheel.
  // Color wheel has a range of 65536 but it's OK if we roll over, so
  // just count from 0 to 5*65536. Adding 256 to firstPixelHue each time
  // means we'll make 5*65536/256 = 1280 passes through this loop:
  for(long firstPixelHue = 0; firstPixelHue < 5*65536; firstPixelHue += 256) {
    // strip.rainbow() can take a single argument (first pixel hue) or
    // optionally a few extras: number of rainbow repetitions (default 1),
    // saturation and value (brightness) (both 0-255, similar to the
    // ColorHSV() function, default 255), and a true/false flag for whether
    // to apply gamma correction to provide 'truer' colors (default true).
    strip.rainbow(firstPixelHue);
    // Above line is equivalent to:
    // strip.rainbow(firstPixelHue, 1, 255, 255, true);
    strip.show(); // Update strip with new contents
    delay(wait);  // Pause for a moment
  }
}
