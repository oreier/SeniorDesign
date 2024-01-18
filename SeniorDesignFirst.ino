#include <Servo.h>

#define SOUND_INTERVAL 10000   // Interval between sound plays in milliseconds
#define TRIGGER_PIN 2          // Pin number for the trigger input
#define ARM_SERVO_PIN 9        // Pin number for the arm servo motor

#define HAND_LEFT_SERVO_PIN 10    // Pin number for the hand left servo
#define HAND_RIGHT_SERVO_PIN 11    // Pin number for the hand right servo

#define LEFT_LINEAR_ACTUATOR_PIN 5   // Pin number for the left shoulder linear actuator
#define RIGHT_LINEAR_ACTUATOR_PIN 6  // Pin number for the right shoulder linear actuator

#define LED_PIN_RED 12      // pin number for red pin led
#define  LED_PIN_GREEN 13 //pin number for green pin led
#define LED_PIN_BLUE 13 //pin number for green pin led

#define const int purple[] = {128,0,128}
#define const int green[] = {0,255,0}
#define const int orange[] = {255,50,0}

#define const int red = LOW
#define const int blue = LOW
#define const int yellow = LOW

enum RobotState {
  IDLE,
  MOVEMENT
};



RobotState currentState = IDLE;
unsigned long previousSoundTime = 0;

Servo handLeftServo;
Servo handRightServo;

//Servo leftElbow;
//Servo rightELbow;

void setup() {
  pinMode(TRIGGER_PIN, INPUT_PULLUP);
  pinMode(LEFT_LINEAR_ACTUATOR_PIN, OUTPUT);
  pinMode(RIGHT_LINEAR_ACTUATOR_PIN, OUTPUT);
  pinMode(LED_PIN_RED, OUTPUT);
  pinMode(LED_PIN_GREEN, OUTPUT);
  pinMode(LED_PIN_BLUE, OUTPUT);
  
  handLeftServo.attach(HAND_LEFT_SERVO_PIN);
  handRightServo.attach(HAND_RIGHT_SERVO_PIN);
  
  // Additional setup code for other motors or sensors
}

void loop() {
  switch (currentState) {
    case IDLE:
      idleState();
      break;
      
    case MOVEMENT:
      movementState();
      break;
  }
}

void idleState() {
  unsigned long currentTime = millis();
  
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
}

void movementState() {
  engageElectromagnet();
  closeHand();
  colorSensor();
  liftArmElbow();
  turnOnLED();
  setColorOff();
  returnArmsToSide();
  releaseServoAndElectromagnet();
  
  currentState = IDLE;  // Transition back to idle state
}

void initializeMovementState() {
  // Additional code to initialize the movement state
}

void engageElectromagnet() {
  // Code to engage the electromagnet
}

void closeHand() {
  fingerServo.write(90);  // Move the finger servo to 90 degrees
  delay(200);             // Adjust the delay as needed
}

colorSensor(){
  // code to set 2 colors
}

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
  analogWrite(LED_PIN_RED, color[0]); // set red pin
  analogWrite(LED_PIN_GREEN, color[1]); // set green pin color
  analogWrite(LED_PIN_BLUE, color[2]); // set blue pin color
}

void setColorOff() {
  digitalWrite(LED_PIN_RED, LOW); // set red pin off 
  digitalWrite(LED_PIN_GREEN, LOW); // set green pin off 
  digitalWrite(LED_PIN_BLUE, LOW); // set blue pin off 
}


void returnArmsToSide() {
  // Code to return the arms back to the side (0-degree position)
}

void releaseServoAndElectromagnet() {
  // Code to release the servo and electromagnet
}

void playSound() {
  // Code to play the sound
}
