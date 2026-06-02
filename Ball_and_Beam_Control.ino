#include <Arduino.h>
#include <math.h>
#include <AVR_PWM.h>

const uint8_t POT_PIN = A5;
const uint8_t US_TRIG_PIN = 5;
const uint8_t US_ECHO_PIN = 6;
const uint8_t MOTOR_PWM_PIN = 9;
const uint8_t MOTOR_IN1_PIN = 10;
const uint8_t MOTOR_IN2_PIN = 11;
const uint8_t MOTOR_CCW_IN1_STATE = HIGH;
const uint8_t MOTOR_CCW_IN2_STATE = LOW;
const uint8_t MOTOR_CW_IN1_STATE = LOW;
const uint8_t MOTOR_CW_IN2_STATE = HIGH;

const float MOTOR_PWM_FREQUENCY = 250.5f;
const float POT_MIN = 110.0f;
const float POT_MAX = 180.0f;
const float US_MIN = 4.0f;
const float US_MAX = 41.0f;
const float MOTOR_MIN = -50.0f;
const float MOTOR_MAX = 50.0f;
const float MOTOR_DEADBAND = 0.5f; 

const float POT_ERROR_DEADBAND = 1.8f; 

const uint8_t MOTOR_MIN_DUTY_CYCLE = 32;   
const uint8_t MOTOR_MAX_DUTY_CYCLE = 45;  

const float TARGET_DISTANCE = 20.0f; 
const float POT_FLAT_ANGLE = 145.0f; 

float Kp_inner = 0.35f;  
float Ki_inner = 0.03f;  
float Kd_inner = 0.05f;  

float lastDistanceError = 0.0f;
unsigned long lastTime = 0;

struct FuzzyTerm { float a; float b; float c; float d; };
struct FuzzyRule { uint8_t errTerm; uint8_t spdTerm; uint8_t angleTerm; };

enum ErrorTerm { ERR_NEG, ERR_ZRO, ERR_POS };
enum SpeedTerm { SPD_NEG, SPD_ZRO, SPD_POS };
enum AngleTerm { ANG_LOW, ANG_FLAT, ANG_HIGH };

const FuzzyTerm ERROR_TERMS[] = {
  {-40.0f, -40.0f, -4.0f, -1.2f},  
  {-2.5f,  -0.8f,   0.8f,  2.5f},  
  { 1.2f,   4.0f,  40.0f, 40.0f}   
};

const FuzzyTerm SPEED_TERMS[] = {
  {-60.0f, -60.0f, -8.0f, -0.8f}, 
  {-2.0f,   0.0f,   0.0f,   2.0f}, 
  { 0.8f,   8.0f,  60.0f,  60.0f}  
};

const FuzzyTerm ANGLE_TERMS[] = {
  {110.0f, 132.0f, 132.0f, 142.0f}, 
  {137.0f, 143.0f, 147.0f, 153.0f}, 
  {148.0f, 158.0f, 158.0f, 180.0f}  
};

const FuzzyRule RULES[] = {
  {ERR_NEG, SPD_NEG, ANG_LOW},  
  {ERR_NEG, SPD_ZRO, ANG_LOW},  
  {ERR_NEG, SPD_POS, ANG_HIGH}, 
  {ERR_ZRO, SPD_NEG, ANG_HIGH}, 
  {ERR_ZRO, SPD_ZRO, ANG_FLAT}, 
  {ERR_ZRO, SPD_POS, ANG_LOW},  
  {ERR_POS, SPD_NEG, ANG_LOW},  
  {ERR_POS, SPD_ZRO, ANG_HIGH}, 
  {ERR_POS, SPD_POS, ANG_HIGH}  
};
const uint8_t RULE_COUNT = sizeof(RULES) / sizeof(RULES[0]);

AVR_PWM* PWM_Instance;

float calculateMembership(const FuzzyTerm& term, float input){
  if(input <= term.a || input >= term.d) return 0.0f;
  if(input >= term.b && input <= term.c) return 1.0f;
  if(input > term.a && input < term.b) return (input - term.a) / (term.b - term.a);
  if(input > term.c && input < term.d) return (term.d - input) / (term.d - term.c);
  return 0.0f;
}

float calculateOutputTermCentroid(const FuzzyTerm& term){
  float total = 0.0f;
  for(int value = static_cast<int>(term.a); value <= term.d; value++){
    total += value;
  }
  return total / (term.d - term.a);
}

float calculateTargetAngle(float error, float speed){
  float centroid = 0.0f;
  float total = 0.0f;
  for(uint8_t i = 0; i < RULE_COUNT; i++){
    float em = calculateMembership(ERROR_TERMS[RULES[i].errTerm], error);
    float sm = calculateMembership(SPEED_TERMS[RULES[i].spdTerm], speed);
    float act = min(em, sm);
    if(act > 0.0f){
      const FuzzyTerm& out = ANGLE_TERMS[RULES[i].angleTerm];
      centroid += calculateOutputTermCentroid(out) * act;
      total += act;
    }
  }
  if(total == 0.0f) return POT_FLAT_ANGLE;
  return centroid / total;
}

float readPotAngle(){
  int rawValue = analogRead(POT_PIN);
  float currentAngle = constrain(static_cast<float>(rawValue), POT_MIN, POT_MAX);
  static float filteredPot = POT_FLAT_ANGLE;

  filteredPot = (currentAngle * 0.1f) + (filteredPot * 0.9f);
  return filteredPot;
}

float readUltrasonicDistance(){
  digitalWrite(US_TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(US_TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(US_TRIG_PIN, LOW);
  unsigned long duration = pulseIn(US_ECHO_PIN, HIGH, 20000UL);
  float rawDistance;
  if(duration == 0) {
    rawDistance = US_MAX;
  } else {
    rawDistance = (duration * 0.0343f) / 2.0f;
  }
  rawDistance = constrain(rawDistance, US_MIN, US_MAX);
  static float filteredDist = TARGET_DISTANCE;
  filteredDist = (rawDistance * 0.2f) + (filteredDist * 0.8f); 
  return filteredDist;
}

void setMotorOutput(float motorSpeed){
  float clampedSpeed = constrain(motorSpeed, MOTOR_MIN, MOTOR_MAX);
  float speedAbs = fabsf(clampedSpeed);
  int dutyCycle = 0;
  if(speedAbs > MOTOR_DEADBAND){
    dutyCycle = map(static_cast<int>(roundf(speedAbs)), static_cast<int>(MOTOR_DEADBAND), static_cast<int>(MOTOR_MAX), MOTOR_MIN_DUTY_CYCLE, MOTOR_MAX_DUTY_CYCLE);
    dutyCycle = constrain(dutyCycle, MOTOR_MIN_DUTY_CYCLE, MOTOR_MAX_DUTY_CYCLE);
    if(clampedSpeed > 0){
      digitalWrite(MOTOR_IN1_PIN, MOTOR_CW_IN1_STATE);
      digitalWrite(MOTOR_IN2_PIN, MOTOR_CW_IN2_STATE);
    } else {
      digitalWrite(MOTOR_IN1_PIN, MOTOR_CCW_IN1_STATE);
      digitalWrite(MOTOR_IN2_PIN, MOTOR_CCW_IN2_STATE);
    }
  } else {
    digitalWrite(MOTOR_IN1_PIN, LOW);
    digitalWrite(MOTOR_IN2_PIN, LOW);
    dutyCycle = 0;
  }
  if(PWM_Instance) {
    PWM_Instance->setPWM(MOTOR_PWM_PIN, MOTOR_PWM_FREQUENCY, dutyCycle);
  } else {
    int pwmValue = map(dutyCycle, 0, 100, 0, 255);
    analogWrite(MOTOR_PWM_PIN, pwmValue);
  }
}

void setup(){
  Serial.begin(115200);
  pinMode(POT_PIN, INPUT);
  pinMode(US_TRIG_PIN, OUTPUT);
  pinMode(US_ECHO_PIN, INPUT);
  pinMode(MOTOR_PWM_PIN, OUTPUT);
  pinMode(MOTOR_IN1_PIN, OUTPUT);
  pinMode(MOTOR_IN2_PIN, OUTPUT);
  digitalWrite(US_TRIG_PIN, LOW);
  PWM_Instance = new AVR_PWM(MOTOR_PWM_PIN, MOTOR_PWM_FREQUENCY, 0.0f);
  if(PWM_Instance) PWM_Instance->setPWM();
  lastTime = millis();
}

void loop(){
  unsigned long currentTime = millis();
  float dt = (currentTime - lastTime) / 1000.0f;
  if (dt <= 0.0f || dt > 0.1f) { 
    lastTime = currentTime;
    return;
  }
  float distanceCM = readUltrasonicDistance();
  float actualPotAngle = readPotAngle();
  float distError = TARGET_DISTANCE - distanceCM;
  float distSpeed = (distError - lastDistanceError) / dt;
  static float filteredDistSpeed = 0.0f;
  filteredDistSpeed = (distSpeed * 0.2f) + (filteredDistSpeed * 0.8f);
  
  float targetPotAngle = calculateTargetAngle(distError, filteredDistSpeed);
  targetPotAngle = constrain(targetPotAngle, 132.0f, 158.0f);

  static float smoothedTargetAngle = POT_FLAT_ANGLE;
  smoothedTargetAngle = (targetPotAngle * 0.15f) + (smoothedTargetAngle * 0.85f);
  
  float potError = smoothedTargetAngle - actualPotAngle;
  float motorOutput = 0.0f;

  static float potIntegral = 0.0f; 

  if (fabsf(potError) > POT_ERROR_DEADBAND) {
    static float lastPotError = 0.0f;
    
    potIntegral += potError * dt;
    potIntegral = constrain(potIntegral, -10.0f, 10.0f); 

    float potDerivative = (potError - lastPotError) / dt;
    
    motorOutput = (Kp_inner * potError) + (Ki_inner * potIntegral) + (Kd_inner * potDerivative);
    
    lastPotError = potError;
  } else {
    motorOutput = 0.0f; 
    potIntegral = 0.0f; 
  }

  setMotorOutput(motorOutput);
  lastDistanceError = distError;
  lastTime = currentTime;
  
  Serial.print("Dist:"); Serial.print(distanceCM);
  Serial.print(" TarAng:"); Serial.print(smoothedTargetAngle);
  Serial.print(" ActAng:"); Serial.print(actualPotAngle);
  Serial.print(" MotorOutput:"); Serial.println(motorOutput);
  delay(20); 
}
