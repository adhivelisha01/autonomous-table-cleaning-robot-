// ============================================
// Cleaning Robot - Boustrophedon FINAL
// v6 with dual pivot times
// Ultrasonic distance + PID drift + math pivot turns
// ============================================

#include <Wire.h>
#include <MPU6050_light.h>

MPU6050 mpu(Wire);

// ===== Pins =====
#define LEFT_IN1   26
#define LEFT_IN2   27
#define RIGHT_IN3  14
#define RIGHT_IN4  12
#define LEFT_ENA   25
#define RIGHT_ENB  33
#define BRUSH_LEFT   32
#define BRUSH_RIGHT  23
#define TRIG_PIN   18
#define ECHO_PIN   19

// ===== Speeds (0-255) =====
int wheelSpeed = 135;
int pivotSpeed = 140;
int brushSpeed = 200;

// ===== TRIM (drift compensation) =====
int LEFT_TRIM = 0;
int RIGHT_TRIM = -6;

// ===== Behavior =====
const int OBSTACLE_DISTANCE = 50;       // stops 50cm from object
const int NUM_PASSES = 5;
const bool USE_BRUSHES = true;

// ===== DUAL Pivot Times =====
// First pivot: RIGHT wheel stationary, LEFT wheel (faster) moves
const unsigned long PIVOT_TIME_RIGHT_WHEEL = 1225;

// Second pivot: LEFT wheel stationary, RIGHT wheel (slower) moves
const unsigned long PIVOT_TIME_LEFT_WHEEL  = 1500;

// ===== PID =====
float Kp = 4.0;
float Kd = 0.3;
float prevError = 0.0;
unsigned long lastPIDTime = 0;
const int PID_INTERVAL = 20;
float headingTarget = 0.0;

bool mpuAvailable = false;

void setup() {
  Serial.begin(115200);
  Wire.begin();
  Serial.println("=== Boustrophedon FINAL ===");
  Serial.print("Pivot RIGHT wheel time: "); Serial.print(PIVOT_TIME_RIGHT_WHEEL); Serial.println(" ms");
  Serial.print("Pivot LEFT wheel time: "); Serial.print(PIVOT_TIME_LEFT_WHEEL); Serial.println(" ms");
  Serial.print("Stop distance: "); Serial.print(OBSTACLE_DISTANCE); Serial.println(" cm");
  Serial.print("Right trim: "); Serial.println(RIGHT_TRIM);
  Serial.print("Brush speed: "); Serial.println(brushSpeed);

  byte status = mpu.begin();
  if (status == 0) {
    mpuAvailable = true;
    Serial.println("Calibrating MPU6050...");
    delay(2000);
    mpu.calcOffsets();
    Serial.println("MPU6050 ready");
  } else {
    Serial.println("MPU6050 not found - PID disabled");
  }

  pinMode(LEFT_IN1, OUTPUT); pinMode(LEFT_IN2, OUTPUT);
  pinMode(RIGHT_IN3, OUTPUT); pinMode(RIGHT_IN4, OUTPUT);
  pinMode(LEFT_ENA, OUTPUT); pinMode(RIGHT_ENB, OUTPUT);
  pinMode(BRUSH_LEFT, OUTPUT); pinMode(BRUSH_RIGHT, OUTPUT);
  pinMode(TRIG_PIN, OUTPUT); pinMode(ECHO_PIN, INPUT);

  stopMotors();

  Serial.println("Starting in 3 seconds...");
  delay(3000);

  if (USE_BRUSHES) {
    Serial.println("Brushes ON");
    analogWrite(BRUSH_LEFT, brushSpeed);
    analogWrite(BRUSH_RIGHT, brushSpeed);
  }

  if (mpuAvailable) {
    mpu.update();
    headingTarget = mpu.getAngleZ();
  }

  // ============ Boustrophedon Pattern ============
  for (int pass = 0; pass < NUM_PASSES; pass++) {
    Serial.print(">>> Pass "); Serial.print(pass + 1);
    Serial.print(" of "); Serial.println(NUM_PASSES);

    forwardUntilObstacle();

    if (pass == NUM_PASSES - 1) break;

    if (pass % 2 == 0) {
      Serial.println(">>> Pivot RIGHT wheel (LEFT moves)");
      pivotAroundRightWheel();
    } else {
      Serial.println(">>> Pivot LEFT wheel (RIGHT moves)");
      pivotAroundLeftWheel();
    }

    if (mpuAvailable) {
      mpu.update();
      headingTarget = mpu.getAngleZ();
    }
  }

  Serial.println(">>> CLEANING COMPLETE <<<");
  stopAll();
  while (true) delay(1000);
}

void loop() {}

// ============ Movement Functions ============

void forwardUntilObstacle() {
  prevError = 0;

  digitalWrite(LEFT_IN1, LOW); digitalWrite(LEFT_IN2, HIGH);
  digitalWrite(RIGHT_IN3, LOW); digitalWrite(RIGHT_IN4, HIGH);

  unsigned long maxRunTime = millis() + 20000;
  int consecutiveDetections = 0;

  while (true) {
    long distance = readUltrasonicFiltered();

    if (distance > 0 && distance <= OBSTACLE_DISTANCE) {
      consecutiveDetections++;
      if (consecutiveDetections >= 2) {
        Serial.print("Obstacle at "); Serial.print(distance); Serial.println(" cm - STOP");
        stopMotors();
        delay(300);
        return;
      }
    } else {
      consecutiveDetections = 0;
    }

    if (millis() > maxRunTime) {
      Serial.println("Timeout - no obstacle");
      stopMotors();
      delay(300);
      return;
    }

    if (mpuAvailable) mpu.update();

    if (millis() - lastPIDTime >= PID_INTERVAL) {
      lastPIDTime = millis();
      runPID();
    }
  }
}

void pivotAroundRightWheel() {
  // RIGHT wheel stays still, LEFT wheel moves
  digitalWrite(LEFT_IN1, LOW); digitalWrite(LEFT_IN2, HIGH);
  digitalWrite(RIGHT_IN3, LOW); digitalWrite(RIGHT_IN4, LOW);
  analogWrite(LEFT_ENA, pivotSpeed);
  analogWrite(RIGHT_ENB, 0);
  delay(PIVOT_TIME_RIGHT_WHEEL);
  stopMotors();
  delay(400);
}

void pivotAroundLeftWheel() {
  // LEFT wheel stays still, RIGHT wheel moves
  digitalWrite(LEFT_IN1, LOW); digitalWrite(LEFT_IN2, LOW);
  digitalWrite(RIGHT_IN3, LOW); digitalWrite(RIGHT_IN4, HIGH);
  analogWrite(LEFT_ENA, 0);
  analogWrite(RIGHT_ENB, pivotSpeed);
  delay(PIVOT_TIME_LEFT_WHEEL);
  stopMotors();
  delay(400);
}

void runPID() {
  if (!mpuAvailable) {
    analogWrite(LEFT_ENA, constrain(wheelSpeed + LEFT_TRIM, 0, 255));
    analogWrite(RIGHT_ENB, constrain(wheelSpeed + RIGHT_TRIM, 0, 255));
    return;
  }

  float current = mpu.getAngleZ();
  float error = headingTarget - current;
  float derivative = (error - prevError) / (PID_INTERVAL / 1000.0);
  float correction = (Kp * error) + (Kd * derivative);
  prevError = error;

  correction = constrain(correction, -40, 40);
  int leftPWM = constrain(wheelSpeed - correction + LEFT_TRIM, 70, 255);
  int rightPWM = constrain(wheelSpeed + correction + RIGHT_TRIM, 70, 255);

  analogWrite(LEFT_ENA, leftPWM);
  analogWrite(RIGHT_ENB, rightPWM);
}

long readUltrasonicFiltered() {
  long readings[3];
  for (int i = 0; i < 3; i++) {
    readings[i] = readUltrasonicRaw();
    delay(15);
  }
  if (readings[0] > readings[1]) { long t = readings[0]; readings[0] = readings[1]; readings[1] = t; }
  if (readings[1] > readings[2]) { long t = readings[1]; readings[1] = readings[2]; readings[2] = t; }
  if (readings[0] > readings[1]) { long t = readings[0]; readings[0] = readings[1]; readings[1] = t; }
  return readings[1];
}

long readUltrasonicRaw() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  long duration = pulseIn(ECHO_PIN, HIGH, 30000);
  if (duration == 0) return -1;
  return duration * 0.034 / 2;
}

void stopMotors() {
  digitalWrite(LEFT_IN1, LOW); digitalWrite(LEFT_IN2, LOW);
  digitalWrite(RIGHT_IN3, LOW); digitalWrite(RIGHT_IN4, LOW);
  analogWrite(LEFT_ENA, 0); analogWrite(RIGHT_ENB, 0);
}

void stopAll() {
  stopMotors();
  analogWrite(BRUSH_LEFT, 0);
  analogWrite(BRUSH_RIGHT, 0);
}
