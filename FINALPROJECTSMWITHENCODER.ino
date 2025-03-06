#include <Arduino.h>
#include <TimerInterrupt.h>
#include <Wire.h>
#include <byte-sized-encoder-decoder.h>
#include <Derivs_Limiter.h>
#include <Servo.h> // Added Servo library


// Timer Init

unsigned long startTime; 

// Orient Code

#include <Wire.h>
#include <VL53L0X_mod.h>
#define TCAADDR 0x70
VL53L0X_mod sensor1;
VL53L0X_mod sensor2;
VL53L0X_mod sensor3;
bool oriented = false;
void tcaselect(uint8_t i) {
  if (i > 7) return;
  Wire.beginTransmission(TCAADDR);
  Wire.write(1 << i);
  Wire.endTransmission();
}

void orient(void) {
  uint16_t distance1;
  uint16_t distance2;
  uint16_t distance3;
  while (!oriented) {
    tcaselect(2);
    distance1 = sensor1.readRangeSingleMillimeters();
    Serial.print("Sensor1 = ");
    Serial.print(distance1);

    tcaselect(7);
    distance2 = sensor2.readRangeSingleMillimeters();
    Serial.print(" Sensor2 = ");
    Serial.print(distance2);
    tcaselect(5);
    distance3 = sensor3.readRangeSingleMillimeters();
    Serial.print(" Sensor3 = ");
    Serial.println(distance3);

    if((distance1 > (distance2 - 25)) && (distance1 < (distance2 + 25)) && (distance3 < 300) && (distance1 > 500)) {
      StopDrivePower();
      Serial.print(distance1);
      Serial.println("Oriented!");
      oriented = true;
      break;
    }
    SpinDrivePower(7);
  }
}
// ==============================
// Encoder Control Setup
// ==============================
ByteSizedEncoderDecoder bsed = ByteSizedEncoderDecoder(&Wire, 0x0F);

Derivs_Limiter YLimiter = Derivs_Limiter(30000, 1000, 900); // velocity, increasing acceleration, decreasing acceleration
Derivs_Limiter XLimiter = Derivs_Limiter(30000, 1000, 900); // velocity, increasing acceleration, decreasing acceleration

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
#define EYE_PIN         8     // Servo 3 EYE
#define IGNITER_PIN     10    // Servo 2 igniter
#define BALL_DROP_PIN   9     // Servo 1 ball drop
#define LAUNCHER_MOTOR  3     // Launcher Motor
#define LAUNCHER_SENSOR 2     // Launcher Sensor
#define LAUNCHER_LED    A2    // Launcher Indicator Light
#define START_BUTTON    A0    // Start Button

// ==============================
// Servo Definitions and Constants
// ==============================
Servo igniterServo;    // Create servo object for igniter
Servo ballDropServo;   // Create servo object for ball drop

// Define servo positions
#define IGNITER_PRESSED_POS   20   // Servo angle for pressed position (adjust as needed)
#define IGNITER_RELEASED_POS  120  // Servo angle for released position (adjust as needed)
#define BALL_DROP_OPEN_POS    0   // Servo angle for open position (adjust as needed)
#define BALL_DROP_CLOSED_POS  140  // Servo angle for closed position (adjust as needed)

// ==============================
// Motor Control Parameters
// ==============================
#define MIN_Power 7     // Power required to get motor to start moving
#define MAX_Power 255   // Maximum power

// ==============================
// Control Loop Parameters
// ==============================
#define Kp 5

// ==============================
// Encoder Position Variables
// ==============================
int16_t RPos = 0;
int16_t LPos = 0;
int16_t FPos = 0;
int16_t BPos = 0;

bool pressed = false;
bool dropped = false;
unsigned long servostarttime;

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
  MOVE_RIGHT_FROM_POT,
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
// Action Functions
// ==============================

void activateFan() {
    digitalWrite(EYE_PIN, HIGH);
}

void deactivateFan() {
    digitalWrite(EYE_PIN, LOW);
}

// Updated servo control functions
bool pressIgniter() {
  if (!pressed){
    igniterServo.write(IGNITER_PRESSED_POS);
    pressed=true;
    servostarttime = millis();
    return false;
  }
  unsigned long current = millis();
  //Serial.print("current time for igniter: ");
  //Serial.println(current);
  //Serial.print("start time: ");
  //Serial.println(servostarttime);
  if (current > servostarttime + 1000) {
    igniterServo.write(IGNITER_RELEASED_POS);
    return true;
  } 
  return false;
}

bool dropBall() {
    if (!dropped){
      ballDropServo.write(BALL_DROP_OPEN_POS);
      dropped=true;
      servostarttime = millis();
      return false;
    }
    unsigned long current = millis();
    //Serial.print("current time for igniter: ");
    //Serial.println(current);
    //Serial.print("start time: ");
    //Serial.println(servostarttime);
    if (current > servostarttime + 1000) {
      ballDropServo.write(BALL_DROP_CLOSED_POS);
      return true;
    } 
    return false;
}


void startLauncher() {
    analogWrite(LAUNCHER_MOTOR, 30);
    digitalWrite(LAUNCHER_LED, HIGH);
}

void stopLauncher() {
    digitalWrite(LAUNCHER_MOTOR, LOW);
    digitalWrite(LAUNCHER_LED, LOW);
}

// Function to detect orientation using ultrasonic sensors
// void orientToNorth() {
//     // Placeholder for orientation logic
//     // Could use ultrasonic sensors to equalize distances to walls
//     orient();
// }


// ==============================
// Motor Control Functions
// ==============================

bool positionReached = false;
void ZeroEncoders() {
  bsed.resetEncoderPositions(); 
  XLimiter.setPosition(0);
  YLimiter.setPosition(0);
  XLimiter.resetTime();
  YLimiter.resetTime();
  
}

// void XDrivePower(int16_t power) {
//   if (power == 0) { // don't move
//     analogWrite(FrontMotorPWM, 0);
//     analogWrite(BackMotorPWM, 0);
//   } else if (power < 0) { // go left (-X < )
//     digitalWrite(FrontMotorDIR, HIGH);
//     digitalWrite(BackMotorDIR, HIGH);
//     uint8_t pwmValue = power2PWM(abs(power));
//     analogWrite(FrontMotorPWM, pwmValue);
//     analogWrite(BackMotorPWM, pwmValue);
//   } else {  // go right (+X > )
//     digitalWrite(FrontMotorDIR, LOW);
//     digitalWrite(BackMotorDIR, LOW);
//     uint8_t pwmValue = power2PWM(abs(power));
//     analogWrite(FrontMotorPWM, pwmValue);
//     analogWrite(BackMotorPWM, pwmValue);
//   }
// }

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
    digitalWrite(FrontMotorDIR, LOW);
    digitalWrite(BackMotorDIR, HIGH);
//    digitalWrite(SideMotorsDIR, LOW);  //REMOVE SOMEDAY
    uint8_t pwmValue = power2PWM(abs(power));
    analogWrite(FrontMotorPWM, pwmValue);
    analogWrite(BackMotorPWM, pwmValue);
//    analogWrite(SideMotorsPWM, pwmValue); //REMOVE SOMEDAY
  } else {  // spin CCW
    digitalWrite(FrontMotorDIR, HIGH);
    digitalWrite(BackMotorDIR, LOW);
//    digitalWrite(SideMotorsDIR, HIGH); //REMOVE SOMEDAY
    uint8_t pwmValue = power2PWM(abs(power));
    analogWrite(FrontMotorPWM, pwmValue);
    analogWrite(BackMotorPWM, pwmValue);
//    analogWrite(SideMotorsPWM, pwmValue); //REMOVE SOMEDAY

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
bool AccelPosition(int16_t Xtarget,int16_t Ytarget) {  
  BPos = bsed.getEncoderPositionWithoutOverflows(1);
  RPos = bsed.getEncoderPositionWithoutOverflows(2);
  FPos = -bsed.getEncoderPositionWithoutOverflows(3);
  LPos = -bsed.getEncoderPositionWithoutOverflows(4);
  FDrivePower(constrain(Kp*((int16_t)XLimiter.calc(Xtarget)-(FPos)),-255, 255)); 
  BDrivePower(constrain(Kp*((int16_t)XLimiter.calc(Xtarget)-(BPos)),-255, 255));
  YDrivePower(constrain(Kp*((int16_t)YLimiter.calc(Ytarget)-((LPos+RPos)/2)),-255, 255)); //add RPos average later
  return (abs(FPos - Xtarget) < 2 && abs(BPos - Xtarget) < 2 && abs(LPos-Ytarget) < 2 && abs(RPos-Ytarget) < 2); //added Rpos
}

bool WallAccelPosition(int16_t Xtarget,int16_t Ytarget) {  
  BPos = bsed.getEncoderPositionWithoutOverflows(1);
  RPos = bsed.getEncoderPositionWithoutOverflows(2);
  FPos = -bsed.getEncoderPositionWithoutOverflows(3);
  LPos = -bsed.getEncoderPositionWithoutOverflows(4);
  FDrivePower(constrain(Kp*((int16_t)XLimiter.calc(Xtarget)-(FPos)),-255, 255)); 
  BDrivePower(constrain(Kp*((int16_t)XLimiter.calc(Xtarget)-(BPos)),-255, 255)); 
  YDrivePower(constrain(Kp*((int16_t)YLimiter.calc(Ytarget)-((LPos+RPos)/2)),-255, 255)); //added RPos average
  return ((int16_t)XLimiter.calc(Xtarget) == Xtarget && (int16_t)YLimiter.calc(Ytarget) == Ytarget) && bsed.getEncoderVelocity(1) == 0 && bsed.getEncoderVelocity(2) == 0 && bsed.getEncoderVelocity(3) == 0 && bsed.getEncoderVelocity(4) == 0;
}
// ==============================
// State Machine 
// ==============================
void runStateMachine() {
  // static unsigned long state_start_time = 0;
  // // Update encoder positions
  // LPos = -bsed.getEncoderPositionWithoutOverflows(1);
  // BPos = bsed.getEncoderPositionWithoutOverflows(2);
  // RPos = bsed.getEncoderPositionWithoutOverflows(3);
  // FPos = -bsed.getEncoderPositionWithoutOverflows(4);

  unsigned long currentTime = millis();

  // Calculate the elapsed time
  unsigned long elapsedTime = currentTime - startTime;

  Serial.print("time: ");
  Serial.println(elapsedTime);

  if(elapsedTime > 130000){ // Celebrate forced 
    Serial.println("celebrating");
    StopDrivePower();
    activateFan(); // Activate celebration fan
    state = CELEBRATE;
  }

  switch (state) {
    case INIT:
      Serial.println("Starting init");
      state = ORIENT_NORTH;
      break;

    case ORIENT_NORTH:
      Serial.println("orient");
      orient();
      if (oriented){
        ZeroEncoders(); // FIRST ZERO ENCODER
        state = DRIVE_DOWN;
      }
      break;

    case DRIVE_DOWN:
      Serial.println("drive down");
      if (WallAccelPosition(0,-10*31)) {
        ZeroEncoders(); // FIRST ZERO ENCODER
        state = DRIVE_LEFT_INIT;
      }
      break;

    case DRIVE_LEFT_INIT:
      Serial.println("drive left");
      if (WallAccelPosition(-10*31,0)) {
        state = MOVE_UP;
      ZeroEncoders(); 
      }
      break;

    case MOVE_UP:
      Serial.println("move up");
      if (AccelPosition(0,16*31)) {
        state = DRIVE_RIGHT;
      }
      break;

    case DRIVE_RIGHT:
       Serial.println("drive right");
      if (WallAccelPosition(86*31,15*31)) {
        state = DRIVE_FORWARD;
      }
      break;

    case DRIVE_FORWARD:
       Serial.println("drive forward");
      if (WallAccelPosition(86*31,35*31)) {
      ZeroEncoders();  
        state = PUSH_POT;
      }
      break;

    case PUSH_POT:
       Serial.println("push pot");
    
      if (WallAccelPosition(-77*31, 0)) {
        ZeroEncoders();
        state = MOVE_RIGHT_FROM_POT;
      }
      break;
    

    case MOVE_RIGHT_FROM_POT:
      Serial.println("Move right from pot");
     
     if (AccelPosition(3*31, 0)) {
       state = DRIVE_DOWN_IGNITE;
     }
     break;  

    case DRIVE_DOWN_IGNITE:
      Serial.println("down ignite");
      if (AccelPosition(3*31,-15*31)) {
        state = MOVE_TO_IGNITER;
      }
      break;

    case MOVE_TO_IGNITER:
       Serial.println("to igniter");
      if (WallAccelPosition(-20*31,-15*31)) {
        ZeroEncoders();
        StopDrivePower();
        state = PRESS_IGNITER;
      }
      break;

    case PRESS_IGNITER:
      Serial.println("press");
      if (pressIgniter()){ // Use the servo to press the igniter
        XLimiter.resetTime();
        YLimiter.resetTime();
        state = MOVE_RIGHT_BURNER;

      };
      break;

    case MOVE_RIGHT_BURNER:
      Serial.println("right burner");
      if (AccelPosition(5*31,0)) {
        state = MOVE_UP_BURNER;
      }
      break;

    case MOVE_UP_BURNER:
      Serial.println("up burner");
      if (WallAccelPosition(5*31,18*31)) {
        ZeroEncoders();
        StopDrivePower();
        state = DROP_BALL;
      }
      break;

    case DROP_BALL:
      Serial.println("dropping");
      if (dropBall()){ // Use the servo to drop the ball
        XLimiter.resetTime();
        YLimiter.resetTime();
        state = MOVE_BACK;
      };
      break;

    case MOVE_BACK:
     Serial.println("MB");
      if (AccelPosition(0,-17*31)) {    
        state = MOVE_RIGHT_WALL;
      }
      break;

    case MOVE_RIGHT_WALL:
    Serial.println("MRW");
      if (WallAccelPosition(82*31,-17*31)) {
        ZeroEncoders();
        state = MOVE_DOWN_WALL;
      }
      break;

    case MOVE_DOWN_WALL:
     Serial.println("MDW");
      if (WallAccelPosition(10,-17*31)) {
        StopDrivePower();
        state = START_LAUNCH;
      }
      break;

    case START_LAUNCH:
    Serial.println("SL");
      // Simulate starting the launcher
      startLauncher();
   
      state = LAUNCHING;
      break;

    case LAUNCHING:
      if (elapsedTime >= 90000) { // 90 seconds (1 min 30 sec)
        state = STOP_LAUNCH;
      }
      break;

    case STOP_LAUNCH:
      // Stop the launcher
      stopLauncher();
      ZeroEncoders();
      state = MOVE_OUT_PANTRY;
      break;

    case MOVE_OUT_PANTRY:
     Serial.println("MOP");
      if (AccelPosition(0,14*31)) {
        state = MOVE_LEFT_WALL;
      }
      break;

    case MOVE_LEFT_WALL:
     Serial.println("MLW");
      if (WallAccelPosition(-86*31,14*31)) {
        ZeroEncoders();
        state = POSITION_UNDER_BURNER;
      }
      break;

    case POSITION_UNDER_BURNER:
    Serial.println("PUB");
      if (AccelPosition(5*31,0)) {
        state = MOVE_TO_BURNER_WALL;
      }
      break;

    case MOVE_TO_BURNER_WALL:
    Serial.println("MBW");
      if (WallAccelPosition(5*31,13*31)) {
        ZeroEncoders();
        state = MOVE_POT_TO_CUSTOMER;
      }
      break;

    case MOVE_POT_TO_CUSTOMER:
     Serial.println("MPC");
      // moveRight(POS_MOVE_TO_CUSTOMER_X, POS_MOVE_TO_CUSTOMER_Y);
      // positionReached = WallAccelPosition(xTarget, yTarget); 
      if (WallAccelPosition(83*31,0)) {
        StopDrivePower();
        state = CELEBRATE;
      }
      break;

    case CELEBRATE:
    Serial.println("celebrating");
      StopDrivePower();
      activateFan(); // Activate celebration fan
      break;
  }
}

// ==============================
// Setup and Loop
// ==============================
void setup() {
    // Orient Setup
    // Wire.begin();
    Serial.begin(115200);
//    delay(5000);

    // ORIENTATION SETUP 

    Wire.begin();
    Serial.println("starting");
    //sensor.setTimeout(500);
     /* Initialise the 1st sensor */
    tcaselect(2);
    if (!sensor1.init())
    {
      Serial.println("Failed to detect and initialize sensor1!");
      while (1) {}
    }
   // Initialise the 2nd sensor
    tcaselect(7);
    if (!sensor2.init())
    {
      Serial.println("Failed to detect and initialize sensor2!");
      while (1) {}
    }
   // Initialise the 3rd sensor
    tcaselect(5);
    if (!sensor3.init())
    {
      Serial.println("Failed to detect and initialize sensor3!");
      while (1) {}
    }

    // PIN SET UP 
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
    pinMode(EYE_PIN, OUTPUT);
    pinMode(START_BUTTON, INPUT_PULLUP);

    ballDropServo.attach(BALL_DROP_PIN);
    igniterServo.attach(IGNITER_PIN);    
    // Initialize all outputs to LOW
    StopDrivePower();
    digitalWrite(LAUNCHER_MOTOR, LOW);
    digitalWrite(LAUNCHER_LED, LOW);
    digitalWrite(IGNITER_PIN, LOW);
    digitalWrite(BALL_DROP_PIN, LOW);
    digitalWrite(EYE_PIN, LOW);
    
    // Initialize encoder control
    Wire.begin();  // Use default SDA and SCL pins
    Wire.setClock(400000UL);
    bsed.begin();
    bsed.setWhichEncoders(0b11110000); // encoder channels: 1,2,3,4
    
    // Zero the encoders at start
    ZeroEncoders();
    
    //Servo setup
    igniterServo.write(IGNITER_RELEASED_POS);
    ballDropServo.write(BALL_DROP_CLOSED_POS);
    // Optional: Wait for start button press
    while (digitalRead(START_BUTTON) == HIGH) {
        // Wait for button press
        delay(10);
    }
    Serial.println("setup complete");

    Serial.println("starting timer");
    startTime = millis();
}

void loop() {
    // Run encoder interface
    bsed.run();
    
    // Run state machine
    runStateMachine();
    
    // Small delay to prevent CPU hogging
    delay(1);
}
