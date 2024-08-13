#include <AccelStepper.h> //accelstepper library

// Stepper motor setup
AccelStepper stepperX(1, 4, 5); // DIR:5, STEP: 4
AccelStepper stepperY(1, 9, 10); // DIR:10, STEP: 9
AccelStepper stepperZ(1, 7, 8); // DIR:8, STEP: 7
AccelStepper stepperA(1, 6, 45); // DIR:45, STEP: 6
AccelStepper stepperB(1, 11, 12); // DIR:12, STEP: 11
AccelStepper stepperC(1, 43, 44); // DIR:44, STEP: 43

// Pin arrays for lasers and detectors
int lasers[] = {36, 14, 51, 20, 50, 47, 49, 33, 34, 35}; // Laser pins
int detectors[] = {40, 39, 38, 31, 32, 30, 29, 28, 21, 16}; // Detector pins

const int NUM_LASERS = sizeof(lasers) / sizeof(lasers[0]); // Number of lasers/detectors
const int BAUD_RATE = 9600; // Baud rate for serial communication

float previousInterruptX = 0;
float previousInterruptY = 0;
float previousInterruptZ = 0;
float previousInterruptA = 0;
float previousInterruptB = 0;
float previousInterruptC = 0;

const int LimitSwitch_Xa = 23; //Input for the limit switch
const int LimitSwitch_Xb = 22; //Input for the limit switch
const int LimitSwitch_Ya = 24; //Input for the limit switch
const int LimitSwitch_Yb = 25; //Input for the limit switch
const int LimitSwitch_Za = 27; //Input for the limit switch
const int LimitSwitch_Zb = 26; //Input for the limit switch
const int LimitSwitch_Aa = 53; //Input for the limit switch
const int LimitSwitch_Ab = 53; //Input for the limit switch
const int LimitSwitch_Ba = 3; //Input for the limit switch
const int LimitSwitch_Bb = 2; //Input for the limit switch
const int LimitSwitch_Ca = 17; //Input for the limit switch
const int LimitSwitch_Cb = 15; //Input for the limit switch

volatile float travelDistanceX = 10; // cm
float travelStepsX; //steps
volatile float travelSpeedX = 10;    // mm/s
float travelVelocityX; //steps/s
volatile float travelTimeX = 0.17;     // minutes
float microSteppingX = 3200; //make sure you set it up correctly on the stepper motor controller too!

volatile float travelDistanceY = 10; // cm
float travelStepsY; //steps
volatile float travelSpeedY = 10;    // mm/s
float travelVelocityY; //steps/s
volatile float travelTimeY = 0.17;     // minutes
float microSteppingY = 3200; //make sure you set it up correctly on the stepper motor controller too!

volatile float travelDistanceZ = 10; // cm
float travelStepsZ; //steps
volatile float travelSpeedZ = 10;    // mm/s
float travelVelocityZ; //steps/s
volatile float travelTimeZ = 0.17;     // minutes
float microSteppingZ = 3200; //make sure you set it up correctly on the stepper motor controller too!

volatile float travelDistanceA = 10; // cm
float travelStepsA; //steps
volatile float travelSpeedA = 10;    // mm/s
float travelVelocityA; //steps/s
volatile float travelTimeA = 0.17;     // minutes
float microSteppingA = 3200; //make sure you set it up correctly on the stepper motor controller too!


volatile float travelDistanceB = 10; // cm
float travelStepsB; //steps
volatile float travelSpeedB = 10;    // mm/s
float travelVelocityB; //steps/s
volatile float travelTimeB = 0.17;     // minutes
float microSteppingB = 3200; //make sure you set it up correctly on the stepper motor controller too!

volatile float travelDistanceC = 10; // cm
float travelStepsC; //steps
volatile float travelSpeedC = 10;    // mm/s
float travelVelocityC; //steps/s
volatile float travelTimeC = 0.17;     // minutes
float microSteppingC = 3200; //make sure you set it up correctly on the stepper motor controller too!


bool stepperHoming_SelectedX = false;
bool stepperHoming_CompletedX = false;
int homingPositionX = 1; //1: motor side, 2: tensioner side
float limitSwitchTempPositionX; //position where the limit switch was hit

bool stepperHoming_SelectedY = false;
bool stepperHoming_CompletedY = false;
int homingPositionY = 1; //1: motor side, 2: tensioner side
float limitSwitchTempPositionY; //position where the limit switch was hit

bool stepperHoming_SelectedZ = false;
bool stepperHoming_CompletedZ = false;
int homingPositionZ = 1; //1: motor side, 2: tensioner side
float limitSwitchTempPositionZ; //position where the limit switch was hit

bool stepperHoming_SelectedA = false;
bool stepperHoming_CompletedA = false;
int homingPositionA = 1; //1: motor side, 2: tensioner side
float limitSwitchTempPositionA; //position where the limit switch was hit

bool stepperHoming_SelectedB = false;
bool stepperHoming_CompletedB = false;
int homingPositionB = 1; //1: motor side, 2: tensioner side
float limitSwitchTempPositionB; //position where the limit switch was hit

bool stepperHoming_SelectedC = false;
bool stepperHoming_CompletedC = false;
int homingPositionC = 1; //1: motor side, 2: tensioner side
float limitSwitchTempPositionC; //position where the limit switch was hit


long receivedStepsX = 0; //Number of steps
long receivedSpeedX = 0; //Steps / second
long receivedMaxSpeedX = 0; //Steps / second
long receivedAccelerationX = 0; //Steps / second^2
long CurrentPositionX = 0;
char receivedCommandX; //a letter sent from the terminal
long StartTime = 0;
long PreviousTimeX = 0;
//-------------------------------------------------------------------------------
bool newDataX, runallowedX = false; // booleans for new data from serial, and runallowed flag
bool lastStepPositionX = false; //follows the steps to see if the last step was preformed


long receivedStepsY = 0; //Number of steps
long receivedSpeedY = 0; //Steps / second
long receivedMaxSpeedY = 0; //Steps / second
long receivedAccelerationY = 0; //Steps / second^2
long CurrentPositionY = 0;
char receivedCommandY; //a letter sent from the terminal
long StartTimeY = 0;
long PreviousTimeY = 0;
//-------------------------------------------------------------------------------
bool newDataY, runallowedY = false; // booleans for new data from serial, and runallowed flag
bool lastStepPositionY = false; //follows the steps to see if the last step was preformed


long receivedStepsZ = 0; //Number of steps
long receivedSpeedZ = 0; //Steps / second
long receivedMaxSpeedZ = 0; //Steps / second
long receivedAccelerationZ = 0; //Steps / second^2
long CurrentPositionZ = 0;
char receivedCommandZ; //a letter sent from the terminal
long StartTimeZ = 0;
long PreviousTimeZ = 0;
//-------------------------------------------------------------------------------
bool newDataZ, runallowedZ = false; // booleans for new data from serial, and runallowed flag
bool lastStepPositionZ = false; //follows the steps to see if the last step was preformed


long receivedStepsA = 0; //Number of steps
long receivedSpeedA = 0; //Steps / second
long receivedMaxSpeedA = 0; //Steps / second
long receivedAccelerationA = 0; //Steps / second^2
long CurrentPositionA = 0;
char receivedCommandA; //a letter sent from the terminal
long StartTimeA = 0;
long PreviousTimeA = 0;
//-------------------------------------------------------------------------------
bool newDataA, runallowedA = false; // booleans for new data from serial, and runallowed flag
bool lastStepPositionA = false; //follows the steps to see if the last step was preformed


long receivedStepsB = 0; //Number of steps
long receivedSpeedB = 0; //Steps / second
long receivedMaxSpeedB = 0; //Steps / second
long receivedAccelerationB = 0; //Steps / second^2
long CurrentPositionB = 0;
char receivedCommandB; //a letter sent from the terminal
long StartTimeB = 0;
long PreviousTimeB = 0;
//-------------------------------------------------------------------------------
bool newDataB, runallowedB = false; // booleans for new data from serial, and runallowed flag
bool lastStepPositionB = false; //follows the steps to see if the last step was preformed


long receivedStepsC = 0; //Number of steps
long receivedSpeedC = 0; //Steps / second
long receivedMaxSpeedC = 0; //Steps / second
long receivedAccelerationC = 0; //Steps / second^2
long CurrentPositionC = 0;
char receivedCommandC; //a letter sent from the terminal
long StartTimeC = 0;
long PreviousTimeC = 0;
//-------------------------------------------------------------------------------
bool newDataC, runallowedC = false; // booleans for new data from serial, and runallowed flag
bool lastStepPositionC = false; //follows the steps to see if the last step was preformed

struct MotorState {
    long targetPosition;
    float speed;
    bool moving;
};

MotorState motorStateX, motorStateY, motorStateZ, motorStateA, motorStateB, motorStateC;

// Add a global variable to store the last time lasers were checked
unsigned long lastLaserCheckTime = 0;
bool isPaused = false;
int consecutiveLaserErrors = 0;

void setup()
{
  Serial.begin(BAUD_RATE);

  pinMode(LimitSwitch_Xa, INPUT_PULLUP);
  pinMode(LimitSwitch_Xb, INPUT_PULLUP);
  pinMode(LimitSwitch_Ya, INPUT_PULLUP);
  pinMode(LimitSwitch_Yb, INPUT_PULLUP);
  pinMode(LimitSwitch_Za, INPUT_PULLUP);
  pinMode(LimitSwitch_Zb, INPUT_PULLUP);
  pinMode(LimitSwitch_Aa, INPUT_PULLUP);
  pinMode(LimitSwitch_Ab, INPUT_PULLUP);
  pinMode(LimitSwitch_Ba, INPUT_PULLUP);
  pinMode(LimitSwitch_Bb, INPUT_PULLUP);
  pinMode(LimitSwitch_Ca, INPUT_PULLUP);
  pinMode(LimitSwitch_Cb, INPUT_PULLUP);

  stepperX.setMaxSpeed(400);
  stepperX.setAcceleration(1000);

  stepperY.setMaxSpeed(400);
  stepperY.setAcceleration(1000);

  stepperZ.setMaxSpeed(400);
  stepperZ.setAcceleration(1000);

  stepperA.setMaxSpeed(50);
  stepperA.setAcceleration(1000);

  stepperB.setMaxSpeed(400);
  stepperB.setAcceleration(1000);

  stepperC.setMaxSpeed(400);
  stepperC.setAcceleration(1000);

  // Initialize all laser pins as outputs and set to HIGH (lasers ON)
  for (int i = 0; i < NUM_LASERS; i++) {
    pinMode(lasers[i], OUTPUT);
    digitalWrite(lasers[i], HIGH); // Turn on all lasers
  }

  // Initialize all detector pins as inputs
  for (int i = 0; i < NUM_LASERS; i++) {
    pinMode(detectors[i], INPUT);
  }

  delay(200);
}

void loop() {
    checkSerial(); // Check serial port for new commands
    requestDataFromArduinoB();  // Handle data request independently
    RunTheMotorX(); // Function to handle the motor
    SendPositionX();
    RunTheMotorY(); // Function to handle the motor
    SendPositionY();
    RunTheMotorZ(); // Function to handle the motor
    SendPositionZ();
    RunTheMotorA(); // Function to handle the motor
    SendPositionA();
    RunTheMotorB(); // Function to handle the motor
    SendPositionB();
    RunTheMotorC(); // Function to handle the motor
    SendPositionC();

    // Resume motors if paused and all lasers are detected
    if (isPaused && checkAllLasers()) {
        Serial.println("All lasers DETECTED. Resuming movement.");
        isPaused = false;

        // Resume the state of all motors
        if (motorStateX.moving) {
            stepperX.move(motorStateX.targetPosition);
            stepperX.setSpeed(motorStateX.speed);
            stepperX.enableOutputs();
        }
        if (motorStateY.moving) {
            stepperY.move(motorStateY.targetPosition);
            stepperY.setSpeed(motorStateY.speed);
            stepperY.enableOutputs();
        }
        if (motorStateZ.moving) {
            stepperZ.move(motorStateZ.targetPosition);
            stepperZ.setSpeed(motorStateZ.speed);
            stepperZ.enableOutputs();
        }
        if (motorStateA.moving) {
            stepperA.move(motorStateA.targetPosition);
            stepperA.setSpeed(motorStateA.speed);
            stepperA.enableOutputs();
        }
        if (motorStateB.moving) {
            stepperB.move(motorStateB.targetPosition);
            stepperB.setSpeed(motorStateB.speed);
            stepperB.enableOutputs();
        }
        if (motorStateC.moving) {
            stepperC.move(motorStateC.targetPosition);
            stepperC.setSpeed(motorStateC.speed);
            stepperC.enableOutputs();
        }
    }
}

void checkSerial() //function for receiving the commands
{ 
  if (Serial.available() > 0) //if something comes from the computer
  {
    receivedCommandX = Serial.read(); // pass the value to the receivedCommad variable
    newDataX = true; //indicate that there is a new data by setting this bool to true

    if (newDataX == true) //we only enter this long switch-case statement if there is a new command from the computer
    {
      switch (receivedCommandX) //we check what is the command
      {
        case 'X':
            receivedStepsX = Serial.parseFloat();
            receivedSpeedX = Serial.parseFloat();
            Serial.println("X, going Positive direction.");
            RotateRelativeX();
            break;

        case 'G':
            handleGCommand(); // Handle 'G' command
            break;

        case 'Y':
            receivedStepsY = Serial.parseFloat();
            receivedSpeedY = Serial.parseFloat();
            Serial.println("Y, going Positive direction.");
            RotateRelativeY();
            break;

        case 'Z':
            receivedStepsZ = Serial.parseFloat();
            receivedSpeedZ = Serial.parseFloat();
            Serial.println("Z, going Positive direction.");
            RotateRelativeZ();
            break;

        case 'A':
            receivedStepsA = Serial.parseFloat();
            receivedSpeedA = Serial.parseFloat();
            Serial.println("A, going Positive direction.");
            RotateRelativeA();
            break;

        case 'B':
            receivedStepsB = Serial.parseFloat();
            receivedSpeedB = Serial.parseFloat();
            Serial.println("B, going Positive direction.");
            RotateRelativeB();
            break;

        case 'C':
            receivedStepsC = Serial.parseFloat();
            receivedSpeedC = Serial.parseFloat();
            Serial.println("C, going Positive direction.");
            RotateRelativeC();
            break;

        case 'x':
            receivedStepsX = Serial.parseFloat();
            receivedSpeedX = Serial.parseFloat();
            Serial.println("Absolute position (+).");
            RotateAbsoluteX();
            break;

        case 'y':
            receivedStepsY = Serial.parseFloat();
            receivedSpeedY = Serial.parseFloat();
            Serial.println("Absolute position (+).");
            RotateAbsoluteY();
            break;

        case 'z':
            receivedStepsZ = Serial.parseFloat();
            receivedSpeedZ = Serial.parseFloat();
            Serial.println("Absolute position (+).");
            RotateAbsoluteZ();
            break;

        case 'a':
            receivedStepsA = Serial.parseFloat();
            receivedSpeedA = Serial.parseFloat();
            Serial.println("Absolute position (+).");
            RotateAbsoluteA();
            break;

        case 'b':
            receivedStepsB = Serial.parseFloat();
            receivedSpeedB = Serial.parseFloat();
            Serial.println("Absolute position (+).");
            RotateAbsoluteB();
            break;

        case 'c':
            receivedStepsC = Serial.parseFloat();
            receivedSpeedC = Serial.parseFloat();
            Serial.println("Absolute position (+).");
            RotateAbsoluteC();
            break;

        case 'S':
            stopAllMotors();
            Serial.println("Stopped.");
            break;

        case 'Q':
            receivedAccelerationX = Serial.parseFloat(); 
            stepperX.setAcceleration(receivedAccelerationX); 
            break;

        case 'V':
            receivedSpeedX = Serial.parseFloat(); 
            stepperX.setSpeed(receivedSpeedX); 
            break;

        case 'v':
            receivedMaxSpeedX = Serial.parseFloat();
            stepperX.setMaxSpeed(receivedMaxSpeedX);
            break;

        case 'L':
            runallowedX = false;
            stepperX.disableOutputs();
            Serial.print("L");
            Serial.println(stepperX.currentPosition());
            break;

        case 'U':
            runallowedX = false; 
            stepperX.disableOutputs(); 
            stepperX.setCurrentPosition(0);      
            stepperX.setSpeed(receivedSpeedX); 
            Serial.print("L"); 
            Serial.println(stepperX.currentPosition());
            break;

        case 'W': {
            long setPositionX = Serial.parseInt(); 
            stepperX.setCurrentPosition(setPositionX);
            Serial.print("Set X position to: ");
            Serial.println(setPositionX);
            break;
        }

        case 'w': {
            long currentPositionX = stepperX.currentPosition(); 
            Serial.print("Current X position is: ");
            Serial.println(currentPositionX);
            break;
        }

        case 'E': {
            long setPositionY = Serial.parseInt(); 
            stepperY.setCurrentPosition(setPositionY);
            Serial.print("Set Y position to: ");
            Serial.println(setPositionY);
            break;
        }

        case 'e': {
            long currentPositionY = stepperY.currentPosition(); 
            Serial.print("Current Y position is: ");
            Serial.println(currentPositionY);
            break;
        }

        case 'R': {
            long setPositionZ = Serial.parseInt();
            stepperZ.setCurrentPosition(setPositionZ);
            Serial.print("Set Z position to: ");
            Serial.println(setPositionZ);
            break;
        }
        case 'r': {
            long currentPositionZ = stepperZ.currentPosition();
            Serial.print("Current Z position is: ");
            Serial.println(currentPositionZ);
            break;
        }

        case 'T': {
            long setPositionA = Serial.parseInt();
            stepperA.setCurrentPosition(setPositionA);
            Serial.print("Set A position to: ");
            Serial.println(setPositionA);
            break;
        }
        case 't': {
            long currentPositionA = stepperA.currentPosition();
            Serial.print("Current A position is: ");
            Serial.println(currentPositionA);
            break;
        }

        case 'O': {
            long setPositionB = Serial.parseInt();
            stepperB.setCurrentPosition(setPositionB);
            Serial.print("Set B position to: ");
            Serial.println(setPositionB);
            break;
        }
        case 'o': {
            long currentPositionB = stepperB.currentPosition();
            Serial.print("Current B position is: ");
            Serial.println(currentPositionB);
            break;
        }

        case 'P': {
            long setPositionC = Serial.parseInt();
            stepperC.setCurrentPosition(setPositionC);
            Serial.print("Set C position to: ");
            Serial.println(setPositionC);
            break;
        }
        case 'p': {
            long currentPositionC = stepperC.currentPosition();
            Serial.print("Current C position is: ");
            Serial.println(currentPositionC);
            break;
        }

        case 'H':
            Serial.println("homing, aka finding X minimum");
            stepperHomingX();
            break;

        case 'h':
            Serial.println("finding X maximum");
            MaximumX();
            break;

        case 'I':
            Serial.println("homing, aka finding Y minimum");
            stepperHomingY();
            break;

        case 'i':
            Serial.println("finding Y maximum");
            MaximumY();
            break;

        case 'J':
            Serial.println("homing, aka finding Z minimum");
            stepperHomingZ();
            break;

        case 'j':
            Serial.println("finding Z maximum");
            MaximumZ();
            break;

        case 'K':
            Serial.println("homing, aka finding A minimum");
            stepperHomingA();
            break;

        case 'k':
            Serial.println("finding A maximum");
            MaximumA();
            break;

        case 'M':
            Serial.println("homing, aka finding B minimum");
            stepperHomingB();
            break;

        case 'm':
            Serial.println("finding B maximum");
            MaximumB();
            break;

        case 'N':
            Serial.println("homing, aka finding C minimum");
            stepperHomingC();
            break;

        case 'n':
            Serial.println("finding C maximum");
            MaximumC();
            break;

        case 'D':
            Serial.println("detecting with lasers");
            checkLasersAndDetectors();
            break;

        default:
            break;
      }
    }
    newDataX = false;
    newDataY = false;    
    newDataZ = false;    
    newDataA = false; 
    newDataB = false;    
    newDataC = false;    
  }
}

void initializeMotorPositions(long posX, long posY, long posZ, long posA, long posB, long posC) {
    stepperX.setCurrentPosition(posX);
    stepperY.setCurrentPosition(posY);
    stepperZ.setCurrentPosition(posZ);
    stepperA.setCurrentPosition(posA);
    stepperB.setCurrentPosition(posB);
    stepperC.setCurrentPosition(posC);

    // Log positions to serial for debugging
    Serial.print("Initialized X to: "); Serial.println(posX);
    Serial.print("Initialized Y to: "); Serial.println(posY);
    Serial.print("Initialized Z to: "); Serial.println(posZ);
    Serial.print("Initialized A to: "); Serial.println(posA);
    Serial.print("Initialized B to: "); Serial.println(posB);
    Serial.print("Initialized C to: "); Serial.println(posC);
}

void handleGCommand() {
  Serial.println("G command processed.");
  Serial1.println("GET VALUES"); // Request data from ArduinoB
}

void requestDataFromArduinoB() {
  if (Serial1.available()) {
    String messageFromB = Serial1.readStringUntil('\n');
    Serial.print("Received from ArduinoB: ");
    Serial.println(messageFromB);
  }
}

void checkLasersAndDetectors() {
  Serial.println("Quick check of all laser-detector pairs:");
  for (int i = 0; i < NUM_LASERS; i++) {
    digitalWrite(lasers[i], HIGH);
    delay(10);
    bool detectorState = digitalRead(detectors[i]);

    if (i == 0 || (i >= 5 && i <= 9)) {
      detectorState = !detectorState;
    }

    digitalWrite(lasers[i], LOW);

    Serial.print("Laser ");
    Serial.print(i + 1);
    Serial.print(" Detector ");
    Serial.print(i + 1);
    Serial.print(": ");
    Serial.println(detectorState ? "DETECTED" : "NOT DETECTED");
  }
}

bool checkAllLasers() {
    for (int i = 0; i < NUM_LASERS; i++) {
        digitalWrite(lasers[i], HIGH);
        delay(10);
        bool detectorState = digitalRead(detectors[i]);

        if (i == 0 || (i >= 5 && i <= 9)) {
            detectorState = !detectorState;
        }

        digitalWrite(lasers[i], LOW);

        if (!detectorState) {
            return false;
        }
    }
    return true;
}

void SendPositionX() {
  if (stepperX.distanceToGo() != 0) {
    if ((millis() - StartTime) >= 400) {
      StartTime = millis();
      Serial.print("L");
      Serial.println(stepperX.currentPosition());
    }
  }
}

void SendPositionY() {
  if (stepperY.distanceToGo() != 0) {
    if ((millis() - StartTime) >= 400) {
      StartTime = millis();
      Serial.print("L");
      Serial.println(stepperY.currentPosition());
    }
  }
}

void SendPositionZ() {
  if (stepperZ.distanceToGo() != 0) {
    if ((millis() - StartTime) >= 400) {
      StartTime = millis();
      Serial.print("L");
      Serial.println(stepperZ.currentPosition());
    }
  }
}

void SendPositionA() {
  if (stepperA.distanceToGo() != 0) {
    if ((millis() - StartTime) >= 400) {
      StartTime = millis();
      Serial.print("L");
      Serial.println(stepperA.currentPosition());
    }
  }
}

void SendPositionB() {
  if (stepperB.distanceToGo() != 0) {
    if ((millis() - StartTime) >= 400) {
      StartTime = millis();
      Serial.print("L");
      Serial.println(stepperB.currentPosition());
    }
  }
}

void SendPositionC() {
  if (stepperC.distanceToGo() != 0) {
    if ((millis() - StartTime) >= 400) {
      StartTime = millis();
      Serial.print("L");
      Serial.println(stepperC.currentPosition());
    }
  }
}

void RotateRelativeX() { 
  runallowedX = true; 
  stepperX.setMaxSpeed(receivedSpeedX);
  stepperX.move(receivedStepsX); 
}

void RotateRelativeY() { 
  runallowedY = true;
  stepperY.setMaxSpeed(receivedSpeedY);
  stepperY.move(receivedStepsY);
}

void RotateRelativeZ() { 
  runallowedZ = true; 
  stepperZ.setMaxSpeed(receivedSpeedZ);
  stepperZ.move(receivedStepsZ);
}

void RotateRelativeA() { 
  runallowedA = true;
  stepperA.setMaxSpeed(receivedSpeedA);
  stepperA.move(receivedStepsA);
}

void RotateRelativeB() { 
  runallowedB = true;
  stepperB.setMaxSpeed(receivedSpeedB);
  stepperB.move(receivedStepsB);
}

void RotateRelativeC() { 
  runallowedC = true;
  stepperC.setMaxSpeed(receivedSpeedC);
  stepperC.move(receivedStepsC);
}

void RotateAbsoluteX() {
  runallowedX = true;
  stepperX.setMaxSpeed(receivedSpeedX);
  stepperX.moveTo(receivedStepsX);
}

void RotateAbsoluteY() {
  runallowedY = true;
  stepperY.setMaxSpeed(receivedSpeedY);
  stepperY.moveTo(receivedStepsY);
}

void RotateAbsoluteZ() {
  runallowedZ = true;
  stepperZ.setMaxSpeed(receivedSpeedZ);
  stepperZ.moveTo(receivedStepsZ);
}

void RotateAbsoluteA() {
  runallowedA = true;
  stepperA.setMaxSpeed(receivedSpeedA);
  stepperA.moveTo(receivedStepsA);
}

void RotateAbsoluteB() {
  runallowedB = true;
  stepperB.setMaxSpeed(receivedSpeedB);
  stepperB.moveTo(receivedStepsB);
}

void RotateAbsoluteC() {
  runallowedC = true;
  stepperC.setMaxSpeed(receivedSpeedC);
  stepperC.moveTo(receivedStepsC);
}

void RunTheMotorX() {
  if (stepperX.distanceToGo() != 0) {
    stepperX.enableOutputs();
    stepperX.run();
    lastStepPositionX = true;
  } else {
    stepperX.disableOutputs();
    if (lastStepPositionX == true) {
      Serial.print("L");
      Serial.println(stepperX.currentPosition());
      Serial.println("X movement done");
      lastStepPositionX = false;
    }
  }
}

void RunTheMotorY() {
  if (stepperY.distanceToGo() != 0) {
    stepperY.enableOutputs();
    stepperY.run();
    lastStepPositionY = true;
  } else {
    stepperY.disableOutputs();
    if (lastStepPositionY == true) {
      Serial.print("L");
      Serial.println(stepperY.currentPosition());
      Serial.println("Y movement done");
      lastStepPositionY = false;
    }
  }
}

void RunTheMotorZ() {
  if (stepperZ.distanceToGo() != 0) {
    stepperZ.enableOutputs();
    stepperZ.run();
    lastStepPositionZ = true;
  } else {
    stepperZ.disableOutputs();
    if (lastStepPositionZ == true) {
      Serial.print("L");
      Serial.println(stepperZ.currentPosition());
      Serial.println("Z movement done");
      lastStepPositionZ = false;
    }
  }
}

void RunTheMotorA() {
  if (stepperA.distanceToGo() != 0) {
    stepperA.enableOutputs();
    stepperA.run();
    lastStepPositionA = true;
  } else {
    stepperA.disableOutputs();
    if (lastStepPositionA == true) {
      Serial.print("L");
      Serial.println(stepperA.currentPosition());
      Serial.println("A movement done");
      lastStepPositionA = false;
    }
  }
}

void RunTheMotorB() {
  if (stepperB.distanceToGo() != 0) {
    stepperB.enableOutputs();
    stepperB.run();
    lastStepPositionB = true;
  } else {
    stepperB.disableOutputs();
    if (lastStepPositionB == true) {
      Serial.print("L");
      Serial.println(stepperB.currentPosition());
      Serial.println("B movement done");
      lastStepPositionB = false;
    }
  }
}

void RunTheMotorC() {
  if (stepperC.distanceToGo() != 0) {
    stepperC.enableOutputs();
    stepperC.run();
    lastStepPositionC = true;

    if (millis() - lastLaserCheckTime >= 1000) {
        lastLaserCheckTime = millis();
        if (!checkAllLasers()) {
            consecutiveLaserErrors++;
            if (consecutiveLaserErrors >= 2) {
                stopAllMotors();
                Serial.println("Laser NOT DETECTED twice consecutively. Stopping all motors.");
                isPaused = true;
            }
        } else {
            consecutiveLaserErrors = 0;
        }
    }
  } else {
    stepperC.disableOutputs();
    if (lastStepPositionC == true) {
      Serial.print("L");
      Serial.println(stepperC.currentPosition());
      Serial.println("C movement done");
      lastStepPositionC = false;
    }
  }
}

void stopAllMotors() {
  stepperX.stop();
  stepperX.disableOutputs();
  stepperY.stop();
  stepperY.disableOutputs();
  stepperZ.stop();
  stepperZ.disableOutputs();
  stepperA.stop();
  stepperA.disableOutputs();
  stepperB.stop();
  stepperB.disableOutputs();
  stepperC.stop();
  stepperC.disableOutputs();
}

void stepperHomingX() {
  if (homingPositionX == 1) {
    while (digitalRead(LimitSwitch_Xa) == 1) {
      stepperX.setSpeed(-600);
      stepperX.runSpeed();
    }
    while (digitalRead(LimitSwitch_Xa) == 0) {
      stepperX.setSpeed(200);
      stepperX.runSpeed();
    }
    stepperX.setCurrentPosition(0);
  } else {
    while (digitalRead(LimitSwitch_Xb) == 1) {
      stepperX.setSpeed(600);
      stepperX.runSpeed();
    }
    while (digitalRead(LimitSwitch_Xb) == 0) {
      stepperX.setSpeed(-200);
      stepperX.runSpeed();
    }
  }
}

void stepperHomingY() {
  if (homingPositionY == 1) {
    while (digitalRead(LimitSwitch_Ya) == 1) {
      stepperY.setSpeed(-600);
      stepperY.runSpeed();
    }
    while (digitalRead(LimitSwitch_Ya) == 0) {
      stepperY.setSpeed(200);
      stepperY.runSpeed();
    }
    stepperY.setCurrentPosition(0);
  } else {
    while (digitalRead(LimitSwitch_Yb) == 1) {
      stepperY.setSpeed(600);
      stepperY.runSpeed();
    }
    while (digitalRead(LimitSwitch_Yb) == 0) {
      stepperY.setSpeed(-200);
      stepperY.runSpeed();
    }
  }
}

void stepperHomingZ() {
  if (homingPositionZ == 1) {
    while (digitalRead(LimitSwitch_Za) == 1) {
      stepperZ.setSpeed(-600);
      stepperZ.runSpeed();
    }
    while (digitalRead(LimitSwitch_Za) == 0) {
      stepperZ.setSpeed(200);
      stepperZ.runSpeed();
    }
    stepperZ.setCurrentPosition(0);
  } else {
    while (digitalRead(LimitSwitch_Zb) == 1) {
      stepperZ.setSpeed(600);
      stepperZ.runSpeed();
    }
    while (digitalRead(LimitSwitch_Zb) == 0) {
      stepperZ.setSpeed(-200);
      stepperZ.runSpeed();
    }
  }
}

void stepperHomingA() {
  if (homingPositionA == 1) {
    while (digitalRead(LimitSwitch_Aa) == 1) {
      stepperA.setSpeed(-600);
      stepperA.runSpeed();
    }
    while (digitalRead(LimitSwitch_Aa) == 0) {
      stepperA.setSpeed(200);
      stepperA.runSpeed();
    }
    stepperA.setCurrentPosition(0);
  } else {
    while (digitalRead(LimitSwitch_Ab) == 1) {
      stepperA.setSpeed(600);
      stepperA.runSpeed();
    }
    while (digitalRead(LimitSwitch_Ab) == 0) {
      stepperA.setSpeed(-200);
      stepperA.runSpeed();
    }
  }
}

void stepperHomingB() {
  if (homingPositionB == 1) {
    while (digitalRead(LimitSwitch_Ba) == 1) {
      stepperB.setSpeed(-600);
      stepperB.runSpeed();
    }
    while (digitalRead(LimitSwitch_Ba) == 0) {
      stepperB.setSpeed(200);
      stepperB.runSpeed();
    }
    stepperB.setCurrentPosition(0);
  } else {
    while (digitalRead(LimitSwitch_Bb) == 1) {
      stepperB.setSpeed(600);
      stepperB.runSpeed();
    }
    while (digitalRead(LimitSwitch_Bb) == 0) {
      stepperB.setSpeed(-200);
      stepperB.runSpeed();
    }
  }
}

void stepperHomingC() {
  if (homingPositionC == 1) {
    while (digitalRead(LimitSwitch_Ca) == 1) {
      stepperC.setSpeed(-600);
      stepperC.runSpeed();
    }
    while (digitalRead(LimitSwitch_Ca) == 0) {
      stepperC.setSpeed(200);
      stepperC.runSpeed();
    }
    stepperC.setCurrentPosition(0);
  } else {
    while (digitalRead(LimitSwitch_Cb) == 1) {
      stepperC.setSpeed(600);
      stepperC.runSpeed();
    }
    while (digitalRead(LimitSwitch_Cb) == 0) {
      stepperC.setSpeed(-200);
      stepperC.runSpeed();
    }
  }
}

void MaximumX() {
    Serial.println("going to x max");
    while (digitalRead(LimitSwitch_Xb) == 1) {
      stepperX.setSpeed(600);
      stepperX.runSpeed();
    }
    while (digitalRead(LimitSwitch_Xb) == 0) {
      stepperX.setSpeed(-200);
      stepperX.runSpeed();
    }
}

void MaximumY() {
    Serial.println("going to y max");
    while (digitalRead(LimitSwitch_Yb) == 1) {
      stepperY.setSpeed(600);
      stepperY.runSpeed();
    }
    while (digitalRead(LimitSwitch_Yb) == 0) {
      stepperY.setSpeed(-200);
      stepperY.runSpeed();
    }
}

void MaximumZ() {
    Serial.println("going to z max");
    while (digitalRead(LimitSwitch_Zb) == 1) {
      stepperZ.setSpeed(600);
      stepperZ.runSpeed();
    }
    while (digitalRead(LimitSwitch_Zb) == 0) {
      stepperZ.setSpeed(-200);
      stepperZ.runSpeed();
    }
}

void MaximumA() {
    Serial.println("going to A max");
    while (digitalRead(LimitSwitch_Ab) == 1) {
      stepperA.setSpeed(600);
      stepperA.runSpeed();
    }
    while (digitalRead(LimitSwitch_Ab) == 0) {
      stepperA.setSpeed(-200);
      stepperA.runSpeed();
    }
}

void MaximumB() {
    Serial.println("going to B max");
    while (digitalRead(LimitSwitch_Bb) == 1) {
      stepperB.setSpeed(600);
      stepperB.runSpeed();
    }
    while (digitalRead(LimitSwitch_Bb) == 0) {
      stepperB.setSpeed(-200);
      stepperB.runSpeed();
    }
}

void MaximumC() {
    Serial.println("going to C max");
    while (digitalRead(LimitSwitch_Cb) == 1) {
      stepperC.setSpeed(600);
      stepperC.runSpeed();
    }
    while (digitalRead(LimitSwitch_Cb) == 0) {
      stepperC.setSpeed(-200);
      stepperC.runSpeed();
    }
}
