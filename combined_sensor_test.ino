#include <Wire.h>
#include <MPU6050.h>

// Motor A direction pins
const int AIN1 = 25;
const int AIN2 = 26;

// Motor B direction pins
const int BIN1 = 32;
const int BIN2 = 33;

// TB6612 PWM pins
const int PWMA = 23;   // change if needed
const int PWMB = 13;   // change if needed


// Ultrasonic sensor pins
const int trigPin = 5;
const int echoPin = 18;

// PWM settings
const int pwmFreq = 1000;
const int pwmResolution = 8;     // 0-255
const int maxPWM = 70;           // 30% of 255 = 76.5
const int rampStep = 3;          // smaller = smoother
const int rampDelay = 15;        // ms between speed changes

int currentSpeedA = 0;
int currentSpeedB = 0;
int targetSpeedA = 0;
int targetSpeedB = 0;

// IMU object
MPU6050 mpu;

// Ultrasonic variables
long duration;
int distance;

enum DriveMode {
  STOPPED,
  FORWARD,
  TURN_RIGHT
};

DriveMode currentMode = STOPPED;

void setup() {
  Serial.begin(115200);

  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);



  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);

  // ESP32 PWM setup
  ledcAttach(PWMA, pwmFreq, pwmResolution);
  ledcAttach(PWMB, pwmFreq, pwmResolution);

  Wire.begin(21, 22);

  Serial.println("Initializing MPU6050...");
  mpu.initialize();

  if (mpu.testConnection()) {
    Serial.println("MPU6050 connection successful!");
  } else {
    Serial.println("MPU6050 connection failed!");
  }

  Serial.println("\n=== System Ready ===\n");
  delay(1000);
}

void loop() {
  distance = getDistance();

  int16_t ax, ay, az;
  int16_t gx, gy, gz;
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

  Serial.print("Distance: ");
  Serial.print(distance);
  Serial.print(" cm | ");

  Serial.print("Accel X: ");
  Serial.print(ax);
  Serial.print(" Y: ");
  Serial.print(ay);
  Serial.print(" Z: ");
  Serial.print(az);

  Serial.print(" | Gyro X: ");
  Serial.print(gx);
  Serial.print(" Y: ");
  Serial.print(gy);
  Serial.print(" Z: ");
  Serial.println(gz);

  if (distance > 20) {
    setForward();
  } 
  else if (distance > 10 && distance <= 20) {
    setTurnRight();
  } 
  else {
    setStop();
  }

  updateMotorSpeed();

  delay(100);
}

void setForward() {
  if (currentMode != FORWARD) {
    Serial.println("BOTH MOTORS: FORWARD");

    digitalWrite(AIN1, HIGH);
    digitalWrite(AIN2, LOW);
    digitalWrite(BIN1, HIGH);
    digitalWrite(BIN2, LOW);

    currentMode = FORWARD;
  }

  targetSpeedA = maxPWM;
  targetSpeedB = maxPWM;
}

void setTurnRight() {
  if (currentMode != TURN_RIGHT) {
    Serial.println("TURNING RIGHT");

    digitalWrite(AIN1, HIGH);
    digitalWrite(AIN2, LOW);
    digitalWrite(BIN1, LOW);
    digitalWrite(BIN2, HIGH);

    currentMode = TURN_RIGHT;
  }

  targetSpeedA = maxPWM;
  targetSpeedB = maxPWM;
}

void setStop() {
  if (currentMode != STOPPED) {
    Serial.println("BOTH MOTORS: STOP");
    currentMode = STOPPED;
  }

  targetSpeedA = 0;
  targetSpeedB = 0;
}

void updateMotorSpeed() {
  currentSpeedA = rampTowards(currentSpeedA, targetSpeedA);
  currentSpeedB = rampTowards(currentSpeedB, targetSpeedB);

  ledcWrite(PWMA, currentSpeedA);
  ledcWrite(PWMB, currentSpeedB);

  delay(rampDelay);
}

int rampTowards(int current, int target) {
  if (current < target) {
    current += rampStep;
    if (current > target) current = target;
  } 
  else if (current > target) {
    current -= rampStep;
    if (current < target) current = target;
  }

  return current;
}

int getDistance() {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);

  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  duration = pulseIn(echoPin, HIGH, 30000);

  if (duration == 0) {
    return 999; // no echo detected
  }

  int measuredDistance = duration * 0.034 / 2;
  return measuredDistance;
}