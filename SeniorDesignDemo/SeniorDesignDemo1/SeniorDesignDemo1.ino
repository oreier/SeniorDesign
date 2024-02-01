#include <Arduino.h>

enum RobotState {
  IDLE,
  MOVEMENT
};

RobotState currentState = IDLE;
unsigned long previousSoundTime = 0;

void simulateHardwareSetup() {
  // Simulate any hardware setup or initialization here
  Serial.begin(9600);  // Initialize serial communication
  Serial.println("Simulating hardware setup...");
  delay(1000);  // Simulate delay for 1 second
  Serial.println("Hardware setup complete.");
}

void simulateDigitalRead() {
  // Simulate the trigger pin being triggered
  Serial.println("Simulating trigger pin being triggered...");
  currentState = MOVEMENT;  // Transition to movement state
}

void simulateHardwareAction(const char* action) {
  // Simulate any hardware action here
  Serial.print("Simulating ");
  Serial.print(action);
  Serial.println("...");
  delay(1000);  // Simulate delay for 1 second
  Serial.print(action);
  Serial.println(" complete.");
}

void simulateMovementState() {
  simulateHardwareAction("Engaging electromagnet");
  simulateHardwareAction("Closing hand");
  simulateHardwareAction("Performing color sensing");
  simulateHardwareAction("Lifting arm elbow");
  simulateHardwareAction("Turning on LED");
  simulateHardwareAction("Setting LED color");
  simulateHardwareAction("Returning arms to side");
  simulateHardwareAction("Releasing servo and electromagnet");
  currentState = IDLE;  // Transition back to idle state
}

void setup() {
  // Code that runs once
  simulateHardwareSetup();
}

void loop() {
  // Code that runs repeatedly

  bool TRIGGER_PIN = false;

  switch (currentState) {
    case IDLE:
      // Simulate idle state actions
      Serial.println("Robot is in IDLE state.");
      delay(1000);  // Simulate delay for 1 second
      simulateHardwareAction("Playing sound");
      //Add any additional conditions for transitioning to MOVEMENT state
      if (TRIGGER_PIN) {
        currentState = MOVEMENT;  // Transition to movement state
      }
      else {
        currentState = IDLE;
      }
      break;

    case MOVEMENT:
      // Simulate movement state actions
      Serial.println("Robot is in MOVEMENT state.");
      simulateMovementState();
      break;
  }

  // Simulate trigger pin being checked
  simulateDigitalRead();
}
