#include <Arduino.h>
#include <TimerInterrupt.h>
#include <Wire.h>
#include <byte-sized-encoder-decoder.h>
#include <Derivs_Limiter.h>
#include <TimerOne.h>


// Orient Code

#include <Wire.h>
#include <Adafruit_VL53L0X.h>

#define TCAADDR 0x70

/* Assign a unique ID to this sensor at the same time */
Adafruit_VL53L0X lox1 = Adafruit_VL53L0X();
Adafruit_VL53L0X lox2 = Adafruit_VL53L0X();

const int trigPin = 9;
const int echoPin = 10;

float duration, distance;

uint8_t orienting = true;

void tcaselect(uint8_t i) {
  if (i > 7) return;
  Wire.beginTransmission(TCAADDR);
  Wire.write(1 << i);
  Wire.endTransmission();  
}

void orient(void) {
  float sensor1;
  float sensor2;
  float sensor3;
  
  while (orienting) {
    VL53L0X_RangingMeasurementData_t measure;

    tcaselect(2);
    lox1.rangingTest(&measure, false);  // pass in 'true' to get debug data printout!

    if (measure.RangeStatus != 4) {  // phase failures have incorrect data
      sensor1 = measure.RangeMilliMeter;
      Serial.print("Sensor1 = ");
      Serial.print(sensor1);
    } else {
      sensor1 = 10000;
      //Serial.println(" out of range ");
    }

    VL53L0X_RangingMeasurementData_t measure2;

    tcaselect(7);
    lox2.rangingTest(&measure2, false);  // pass in 'true' to get debug data printout!

    if (measure2.RangeStatus != 4) {  // phase failures have incorrect data
      sensor2 = measure2.RangeMilliMeter;
      Serial.print(" Sensor2 = ");
      Serial.print(sensor2);
    } else {
      sensor2 = 11000;
      //Serial.println(" out of range ");
    }
  
    digitalWrite(trigPin, LOW);
    delayMicroseconds(2);
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin, LOW);

    duration = pulseIn(echoPin, HIGH);
    distance = (duration*.343)/2;
    sensor3 = distance;
    Serial.print(" US Sensor3 =  ");
    Serial.println(distance);

    if((sensor1 > (sensor2 - 10)) && (sensor1 < (sensor2 + 10)) && (sensor3 < 100) && (sensor1 > 500)) {
      Serial.println("Oriented!");
      orienting = false;
    }
    //delay(100);

  }
}

// ==============================
// Encoder Control Setup
// ==============================
ByteSizedEncoderDecoder bsed = ByteSizedEncoderDecoder(&Wire, 0x0F);

Derivs_Limiter YLimiter = Derivs_Limiter(15000, 400, 500); // velocity, increasing acceleration, decreasing acceleration
Derivs_Limiter XLimiter = Derivs_Limiter(35000, 2000, 900); // velocity, increasing acceleration, decreasing acceleration

int16_t xTarget = 0;
int16_t yTarget = 0;

// ==============================
// Pin Definitions for 3-Wheel Drive
// ==============================
#define FrontMotorPWM 5    // ~D5
#define FrontMotorDIR 4    // D4
#define BackMotorPWM 6     // ~D6
#define BackMotorDIR 7     // D7
#define SideMotorsPWM 11   // ~D11 (Motor 3,4 speed)
#define SideMotorsDIR 12   // D12 (Motor 3,4 dir)


// Pin Definitions from the provided sheet
#define FAN_PIN         8     // Celebration fan / LED
#define IGNITER_PIN     10    // Servo 2 igniter
#define BALL_DROP_PIN   9     // Servo 1 ball drop
#define LAUNCHER_MOTOR  3     // Launcher Motor
#define LAUNCHER_SENSOR 2     // Launcher Sensor
#define LAUNCHER_LED    A2    // Launcher Indicator Light
#define START_BUTTON    A0    // Start Button

// ==============================
// Motor Control Parameters
// ==============================
#define MIN_Power 7     // Power required to get motor to start moving
#define MAX_Power 255   // Maximum power

// ==============================
// Control Loop Parameters
// ==============================
#define Kp 7

// ==============================
// Encoder Position Variables
// ==============================
int16_t RPos = 0;
int16_t LPos = 0;
int16_t FPos = 0;
int16_t BPos = 0;

// ==============================
// State Definitions
// ==============================
enum State {
  INIT,
  ORIENT_NORTH,
  DRIVE_DOWN,
  DRIVE_LEFT_INIT,
  MOVE_UP,
  DRIVE_RIGHT,
  DRIVE_FORWARD,
  PUSH_POT,
  DRIVE_DOWN_IGNITE,
  MOVE_TO_IGNITER,
  PRESS_IGNITER,
  MOVE_RIGHT_BURNER,
  MOVE_UP_BURNER,
  DROP_BALL,
  MOVE_BACK,
  MOVE_RIGHT_WALL,
  MOVE_DOWN_WALL,
  START_LAUNCH,
  LAUNCHING,
  STOP_LAUNCH,
  MOVE_OUT_PANTRY,
  MOVE_LEFT_WALL,
  POSITION_UNDER_BURNER,
  MOVE_TO_BURNER_WALL,
  MOVE_POT_TO_CUSTOMER,
  CELEBRATE
};

State state = INIT;

// ==============================
// Global Timing Variables (ms)
// ==============================
volatile unsigned long global_timer1 = 0;  // Main match timer (2 min 10 sec)
volatile unsigned long global_timer2 = 0;  // Launching timer (~1 min 30 sec)

void timerISR() {
    global_timer1++;
    global_timer2++;
}


// ==============================
// Position Constants (in encoder ticks)
// ==============================
// 31 ticks per inch according to the encoder code

const int16_t POS_DRIVE_DOWN_X = 0;         // ~0 inches in X
const int16_t POS_DRIVE_DOWN_Y = -20 * 31;  // ~20 inches down in Y

const int16_t POS_DRIVE_LEFT_INIT_X = -11 * 31; // ~11 inches left in X
const int16_t POS_DRIVE_LEFT_INIT_Y = 0;        // ~0 inches in Y

const int16_t POS_MOVE_UP_X = 0;           // ~0 inches in X
const int16_t POS_MOVE_UP_Y = 15 * 31;     // ~15 inches up in Y

const int16_t POS_DRIVE_RIGHT_X = 25 * 31; // ~25 inches right in X
const int16_t POS_DRIVE_RIGHT_Y = 0;       // ~0 inches in Y

const int16_t POS_DRIVE_FORWARD_X = 0;     // ~0 inches in X
const int16_t POS_DRIVE_FORWARD_Y = 18 * 31; // ~18 inches forward in Y

const int16_t POS_PUSH_POT_X = -22 * 31;   // ~22 inches left in X
const int16_t POS_PUSH_POT_Y = 0;          // ~0 inches in Y

const int16_t POS_DRIVE_DOWN_IGNITE_X = 0; // ~0 inches in X
const int16_t POS_DRIVE_DOWN_IGNITE_Y = -8 * 31; // ~8 inches down in Y

const int16_t POS_MOVE_LEFT_X = -13 * 31;  // ~13 inches left in X
const int16_t POS_MOVE_LEFT_Y = 0;         // ~0 inches in Y

const int16_t POS_MOVE_RIGHT_BURNER_X = 7 * 31; // ~7 inches right in X
const int16_t POS_MOVE_RIGHT_BURNER_Y = 0;      // ~0 inches in Y

const int16_t POS_MOVE_UP_BURNER_X = 0;    // ~0 inches in X
const int16_t POS_MOVE_UP_BURNER_Y = 12 * 31; // ~12 inches up in Y

const int16_t POS_MOVE_BACK_X = 0;         // ~0 inches in X
const int16_t POS_MOVE_BACK_Y = -10 * 31;  // ~10 inches back in Y

const int16_t POS_MOVE_RIGHT_WALL_X = 15 * 31; // ~15 inches right in X
const int16_t POS_MOVE_RIGHT_WALL_Y = 0;       // ~0 inches in Y

const int16_t POS_MOVE_DOWN_WALL_X = 0;    // ~0 inches in X
const int16_t POS_MOVE_DOWN_WALL_Y = -12 * 31; // ~12 inches down in Y

const int16_t POS_MOVE_OUT_PANTRY_X = 0;   // ~0 inches in X
const int16_t POS_MOVE_OUT_PANTRY_Y = 15 * 31; // ~15 inches forward in Y

const int16_t POS_MOVE_TO_CUSTOMER_X = 25 * 31; // ~25 inches right in X
const int16_t POS_MOVE_TO_CUSTOMER_Y = 0;       // ~0 inches in Y

// Position tolerance (how close is "close enough")
const int16_t POSITION_TOLERANCE = 31 / 2;     // 0.5 inch tolerance



//MOVE FUNCTIONS

void moveForward(int16_t x, int16_t y) {
  xTarget = x;
  yTarget = y;
}

void moveBackward(int16_t x, int16_t y) {
  xTarget = x;
  yTarget = y;
}

void moveLeft(int16_t x, int16_t y) {
  xTarget = x;
  yTarget = y;
}

void moveRight(int16_t x, int16_t y) {
  xTarget = x;
  yTarget = y;
}

// ==============================
// Action Functions
// ==============================
void activateFan() {
    digitalWrite(FAN_PIN, HIGH);
}

void deactivateFan() {
    digitalWrite(FAN_PIN, LOW);
}

void pressIgniter() {
    digitalWrite(IGNITER_PIN, HIGH);
    delay(500);
    digitalWrite(IGNITER_PIN, LOW);
}

void dropBall() {
    digitalWrite(BALL_DROP_PIN, HIGH);
    delay(500);
    digitalWrite(BALL_DROP_PIN, LOW);
}

void startLauncher() {
    digitalWrite(LAUNCHER_MOTOR, HIGH);
    digitalWrite(LAUNCHER_LED, HIGH);
}

void stopLauncher() {
    digitalWrite(LAUNCHER_MOTOR, LOW);
    digitalWrite(LAUNCHER_LED, LOW);
}

// Function to detect orientation using ultrasonic sensors
void orientToNorth() {
    // Placeholder for orientation logic
    // Could use ultrasonic sensors to equalize distances to walls
    orient();
}


// ==============================
// Motor Control Functions
// ==============================


void ZeroEncoders() {
  bsed.resetEncoderPositions(); 
}

void XDrivePower(int16_t power) {
  if (power == 0) { // don't move
    analogWrite(FrontMotorPWM, 0);
    analogWrite(BackMotorPWM, 0);
  } else if (power < 0) { // go left (-X < )
    digitalWrite(FrontMotorDIR, HIGH);
    digitalWrite(BackMotorDIR, HIGH);
    uint8_t pwmValue = power2PWM(abs(power));
    analogWrite(FrontMotorPWM, pwmValue);
    analogWrite(BackMotorPWM, pwmValue);
  } else {  // go right (+X > )
    digitalWrite(FrontMotorDIR, LOW);
    digitalWrite(BackMotorDIR, LOW);
    uint8_t pwmValue = power2PWM(abs(power));
    analogWrite(FrontMotorPWM, pwmValue);
    analogWrite(BackMotorPWM, pwmValue);
  }
}

void FDrivePower(int16_t power) {
  if (power == 0) { // don't move
    analogWrite(FrontMotorPWM, 0);
  } else if (power < 0) { // go left (-X < )
    digitalWrite(FrontMotorDIR, HIGH);
    uint8_t pwmValue = power2PWM(abs(power));
    analogWrite(FrontMotorPWM, pwmValue);
  } else {  // go right (+X > )
    digitalWrite(FrontMotorDIR, LOW);
    uint8_t pwmValue = power2PWM(abs(power));
    analogWrite(FrontMotorPWM, pwmValue);
  }
}

void BDrivePower(int16_t power) {
  if (power == 0) { // don't move
    analogWrite(BackMotorPWM, 0);
  } else if (power < 0) { // go left (-X < )
    digitalWrite(BackMotorDIR, HIGH);
    uint8_t pwmValue = power2PWM(abs(power));
    analogWrite(BackMotorPWM, pwmValue);
  } else {  // go right (+X > )
    digitalWrite(BackMotorDIR, LOW);
    uint8_t pwmValue = power2PWM(abs(power));
    analogWrite(BackMotorPWM, pwmValue);
  }
}

void YDrivePower(int16_t power) {
  if (power == 0) { // don't move
    analogWrite(SideMotorsPWM, 0);
  } else if (power < 0) { // go down (-Y ⌄)
    digitalWrite(SideMotorsDIR, HIGH);
    uint8_t pwmValue = power2PWM(abs(power));
    analogWrite(SideMotorsPWM, pwmValue);
  } else {  // go up (+Y ^)
    digitalWrite(SideMotorsDIR, LOW);
    uint8_t pwmValue = power2PWM(abs(power));
    analogWrite(SideMotorsPWM, pwmValue);
  }
}

void SpinDrivePower(int16_t power) {
  if (power == 0) { // don't move
    analogWrite(FrontMotorPWM, 0);
    analogWrite(BackMotorPWM, 0);
  } else if (power < 0) { // spin CW
    analogWrite(SideMotorsPWM, 0);
    digitalWrite(FrontMotorDIR, LOW);
    digitalWrite(BackMotorDIR, HIGH);
    uint8_t pwmValue = power2PWM(abs(power));
    analogWrite(FrontMotorPWM, pwmValue);
    analogWrite(BackMotorPWM, pwmValue);
  } else {  // spin CCW
    analogWrite(SideMotorsPWM, 0);
    digitalWrite(FrontMotorDIR, HIGH);
    digitalWrite(BackMotorDIR, LOW);
    uint8_t pwmValue = power2PWM(abs(power));
    analogWrite(FrontMotorPWM, pwmValue);
    analogWrite(BackMotorPWM, pwmValue);
  }
}

void StopDrivePower(void) { // don't move
  analogWrite(FrontMotorPWM, 0);
  analogWrite(BackMotorPWM, 0);
  analogWrite(SideMotorsPWM, 0);
}

uint8_t power2PWM(uint8_t power) { // scaling function
  if (power > MAX_Power) power = MAX_Power;
  uint8_t pwm = map(power, 0, MAX_Power, MIN_Power, MAX_Power);
  return pwm;
}

// ==============================
// Acceleration Logic
// ==============================
bool AccelPosition(int16_t Xtarget, int16_t Ytarget) {  
  LPos = -bsed.getEncoderPositionWithoutOverflows(1);
  BPos = bsed.getEncoderPositionWithoutOverflows(2);
  RPos = bsed.getEncoderPositionWithoutOverflows(3);
  FPos = -bsed.getEncoderPositionWithoutOverflows(4);
  XDrivePower(constrain(Kp * ((int16_t)XLimiter.calc(Xtarget) - (FPos)), -255, 255)); 
  YDrivePower(constrain(Kp * ((int16_t)YLimiter.calc(Ytarget) - (LPos)), -255, 255)); 
  return (abs(FPos - Xtarget) < POSITION_TOLERANCE && abs(LPos - Ytarget) < POSITION_TOLERANCE);
}
// ==============================
// State Machine 
// ==============================
void runStateMachine() {
  static unsigned long state_start_time = 0;

  // Update encoder positions
  LPos = -bsed.getEncoderPositionWithoutOverflows(1);
  BPos = bsed.getEncoderPositionWithoutOverflows(2);
  RPos = bsed.getEncoderPositionWithoutOverflows(3);
  FPos = -bsed.getEncoderPositionWithoutOverflows(4);

  // Update position control for X and Y axes
  bool positionReached = AccelPosition(xTarget, yTarget);

  switch (state) {
    case INIT:
      ZeroEncoders();
      state_start_time = millis();
      state = ORIENT_NORTH;
      break;

    case ORIENT_NORTH:
      // Placeholder for orientation logic
      delay(1000); // Simulate orientation
      ZeroEncoders(); // FIRST ZERO ENCODER
      state = DRIVE_DOWN;
      break;

    case DRIVE_DOWN:
      if (state_start_time == 0) { // First time in this state
        moveForward(POS_DRIVE_DOWN_X, POS_DRIVE_DOWN_Y);
        state_start_time = millis();
      }
      if (positionReached) {
        StopDrivePower();
        state_start_time = 0;
        state = DRIVE_LEFT_INIT;
      }
      break;

    case DRIVE_LEFT_INIT:
      if (state_start_time == 0) { // First time in this state
        moveLeft(POS_DRIVE_LEFT_INIT_X, POS_DRIVE_LEFT_INIT_Y);
        state_start_time = millis();
      }
      if (positionReached) {
        StopDrivePower();
        state_start_time = 0;
        state = MOVE_UP;
      }
      ZeroEncoders(); // SECOND ZERO ENCODER(OFFICIAL ZERO) 
      break;

    case MOVE_UP:
      if (state_start_time == 0) { // First time in this state
        moveForward(POS_MOVE_UP_X, POS_MOVE_UP_Y);
        state_start_time = millis();
      }
      if (positionReached) {
        StopDrivePower();
        state_start_time = 0;
        state = DRIVE_RIGHT;
      }
      break;

    case DRIVE_RIGHT:
      if (state_start_time == 0) { // First time in this state
        moveRight(POS_DRIVE_RIGHT_X, POS_DRIVE_RIGHT_Y);
        state_start_time = millis();
      }
      if (positionReached) {
        StopDrivePower();
        state_start_time = 0;
        state = DRIVE_FORWARD;
      }
      break;

    case DRIVE_FORWARD:
      if (state_start_time == 0) { // First time in this state
        moveForward(POS_DRIVE_FORWARD_X, POS_DRIVE_FORWARD_Y);
        state_start_time = millis();
      }
      if (positionReached) {
        StopDrivePower();
        state_start_time = 0;
        state = PUSH_POT;
      }
      break;

    case PUSH_POT:
      if (state_start_time == 0) { // First time in this state
        moveLeft(POS_PUSH_POT_X, POS_PUSH_POT_Y);
        state_start_time = millis();
      }
      if (positionReached) {
        StopDrivePower();
        state_start_time = 0;
        state = DRIVE_DOWN_IGNITE;
      }
      break;

    case DRIVE_DOWN_IGNITE:
      if (state_start_time == 0) { // First time in this state
        moveBackward(POS_DRIVE_DOWN_IGNITE_X, POS_DRIVE_DOWN_IGNITE_Y);
        state_start_time = millis();
      }
      if (positionReached) {
        StopDrivePower();
        state_start_time = 0;
        state = MOVE_TO_IGNITER;
      }
      break;

    case MOVE_TO_IGNITER:
      if (state_start_time == 0) { // First time in this state
        moveLeft(POS_MOVE_LEFT_X, POS_MOVE_LEFT_Y);
        state_start_time = millis();
      }
      if (positionReached) {
        StopDrivePower();
        state_start_time = 0;
        state = PRESS_IGNITER;
      }
      break;

    case PRESS_IGNITER:
      // Simulate pressing the igniter
      delay(500); // Simulate igniter press
      state_start_time = 0;
      state = MOVE_RIGHT_BURNER;
      break;

    case MOVE_RIGHT_BURNER:
      if (state_start_time == 0) { // First time in this state
        moveRight(POS_MOVE_RIGHT_BURNER_X, POS_MOVE_RIGHT_BURNER_Y);
        state_start_time = millis();
      }
      if (positionReached) {
        StopDrivePower();
        state_start_time = 0;
        state = MOVE_UP_BURNER;
      }
      break;

    case MOVE_UP_BURNER:
      if (state_start_time == 0) { // First time in this state
        moveForward(POS_MOVE_UP_BURNER_X, POS_MOVE_UP_BURNER_Y);
        state_start_time = millis();
      }
      if (positionReached) {
        StopDrivePower();
        state_start_time = 0;
        state = DROP_BALL;
      }
      break;

    case DROP_BALL:
      // Simulate dropping the ball
      delay(500); // Simulate ball drop
      state_start_time = 0;
      state = MOVE_BACK;
      break;

    case MOVE_BACK:
      if (state_start_time == 0) { // First time in this state
        moveBackward(POS_MOVE_BACK_X, POS_MOVE_BACK_Y);
        state_start_time = millis();
      }
      if (positionReached) {
        StopDrivePower();
        state_start_time = 0;
        state = MOVE_RIGHT_WALL;
      }
      break;

    case MOVE_RIGHT_WALL:
      if (state_start_time == 0) { // First time in this state
        moveRight(POS_MOVE_RIGHT_WALL_X, POS_MOVE_RIGHT_WALL_Y);
        state_start_time = millis();
      }
      if (positionReached) {
        StopDrivePower();
        state_start_time = 0;
        state = MOVE_DOWN_WALL;
      }
      break;

    case MOVE_DOWN_WALL:
      if (state_start_time == 0) { // First time in this state
        moveBackward(POS_MOVE_DOWN_WALL_X, POS_MOVE_DOWN_WALL_Y);
        state_start_time = millis();
      }
      if (positionReached) {
        StopDrivePower();
        state_start_time = 0;
        state = START_LAUNCH;
      }
      break;

    case START_LAUNCH:
      // Simulate starting the launcher
      delay(500); // Simulate launcher start
      state_start_time = 0;
      state = LAUNCHING;
      break;

    case LAUNCHING:
      // Continue launching until timer expires
      if (global_timer2 >= 90) { // 90 seconds (1 min 30 sec)
        state_start_time = 0;
        state = STOP_LAUNCH;
      }
      break;

    case STOP_LAUNCH:
      // Simulate stopping the launcher
      delay(500); // Simulate launcher stop
      state_start_time = 0;
      state = MOVE_OUT_PANTRY;
      break;

    case MOVE_OUT_PANTRY:
      if (state_start_time == 0) { // First time in this state
        moveForward(POS_MOVE_OUT_PANTRY_X, POS_MOVE_OUT_PANTRY_Y);
        state_start_time = millis();
      }
      if (positionReached) {
        StopDrivePower();
        state_start_time = 0;
        state = MOVE_LEFT_WALL;
      }
      break;

    case MOVE_LEFT_WALL:
      if (state_start_time == 0) { // First time in this state
        moveLeft(POS_MOVE_LEFT_X, POS_MOVE_LEFT_Y);
        state_start_time = millis();
      }
      if (positionReached) {
        StopDrivePower();
        state_start_time = 0;
        state = POSITION_UNDER_BURNER;
      }
      break;

    case POSITION_UNDER_BURNER:
      if (state_start_time == 0) { // First time in this state
        moveRight(POS_MOVE_RIGHT_BURNER_X, POS_MOVE_RIGHT_BURNER_Y);
        state_start_time = millis();
      }
      if (positionReached) {
        StopDrivePower();
        state_start_time = 0;
        state = MOVE_TO_BURNER_WALL;
      }
      break;

    case MOVE_TO_BURNER_WALL:
      if (state_start_time == 0) { // First time in this state
        moveForward(POS_MOVE_UP_BURNER_X, POS_MOVE_UP_BURNER_Y);
        state_start_time = millis();
      }
      if (positionReached) {
        StopDrivePower();
        state_start_time = 0;
        state = MOVE_POT_TO_CUSTOMER;
      }
      break;

    case MOVE_POT_TO_CUSTOMER:
      if (state_start_time == 0) { // First time in this state
        moveRight(POS_MOVE_TO_CUSTOMER_X, POS_MOVE_TO_CUSTOMER_Y);
        state_start_time = millis();
      }
      if (positionReached) {
        StopDrivePower();
        state_start_time = 0;
        state = CELEBRATE;
      }
      break;

    case CELEBRATE:
      StopDrivePower();
      // Activate celebration fan or other actions
      break;
  }
}

// ==============================
// Setup and Loop
// ==============================
void setup() {
    // Orient Setup
    Wire.begin();
    Serial.begin(9600);
    Serial.println("\nTCAScanner ready!");

    Serial.println("VL53L0X ToF Test"); Serial.println("");
    
    /* Initialise the 1st sensor */
    tcaselect(2);
    if(!lox1.begin()) {
      /* There was a problem detecting the HMC5883 ... check your connections */
      Serial.println("Ooops, no VL53L0X detected ... Check your wiring!");
      while(1);
    }
    
    /* Initialise the 2nd sensor */
    tcaselect(7);
    if(!lox2.begin()){
      /* There was a problem detecting the HMC5883 ... check your connections */
      Serial.println("Ooops, no VL53L0X detected ... Check your wiring!");
      while(1);
    }

    pinMode(trigPin, OUTPUT);
    pinMode(echoPin, INPUT);
    // Motor pins for 3-wheel drive
    pinMode(FrontMotorPWM, OUTPUT);
    pinMode(FrontMotorDIR, OUTPUT);
    pinMode(BackMotorPWM, OUTPUT);
    pinMode(BackMotorDIR, OUTPUT);
    pinMode(SideMotorsPWM, OUTPUT);
    pinMode(SideMotorsDIR, OUTPUT);
    
    // Other pins
    pinMode(LAUNCHER_MOTOR, OUTPUT);
    pinMode(LAUNCHER_SENSOR, INPUT);
    pinMode(LAUNCHER_LED, OUTPUT);
    pinMode(IGNITER_PIN, OUTPUT);
    pinMode(BALL_DROP_PIN, OUTPUT);
    pinMode(FAN_PIN, OUTPUT);
    pinMode(START_BUTTON, INPUT_PULLUP);
    
    // Initialize all outputs to LOW
    StopDrivePower();
    digitalWrite(LAUNCHER_MOTOR, LOW);
    digitalWrite(LAUNCHER_LED, LOW);
    digitalWrite(IGNITER_PIN, LOW);
    digitalWrite(BALL_DROP_PIN, LOW);
    digitalWrite(FAN_PIN, LOW);
    
    // Initialize encoder control
    Wire.begin();  // Use default SDA and SCL pins
    Wire.setClock(400000UL);
    bsed.begin();
    bsed.setWhichEncoders(0b11110000); // encoder channels: 1,2,3,4
    
    // Zero the encoders at start
    ZeroEncoders();
    
    // Optional: Wait for start button press
    while (digitalRead(START_BUTTON) == HIGH) {
        // Wait for button press
        delay(10);
    }
    delay(1000); // Debounce delay

    Timer1.initialize(1000);
    Timer1.attachInterrupt(timerISR);
}

void loop() {
    // Run encoder interface
    bsed.run();
    
    // Run state machine
    runStateMachine();
    
    // Small delay to prevent CPU hogging
    delay(5);
}
