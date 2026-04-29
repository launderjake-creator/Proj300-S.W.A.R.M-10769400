// RECEIVER ESP32 - Robot with HC-SR04 + MPU6050 + PCA9685 Servos
// Reads sensors, executes missions, transmits data to Central ESP

#include <esp_now.h>
#include <WiFi.h>
#include <Wire.h>
#include <MPU6050.h>
#include <Adafruit_PWMServoDriver.h>

// ===== HARDWARE PINS =====
const int TRIG_PIN = 22;
const int ECHO_PIN = 23;

// ===== MPU6050 =====
MPU6050 mpu;
float gyroBiasZ = 0;
float accelBiasX = 0, accelBiasY = 0;
float heading = 0;
unsigned long lastIMURead = 0;

// ===== PCA9685 Servo Driver =====
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();
#define SERVOMIN  150
#define SERVOMAX  600
const int SERVO_DOCKED = 40;
const int SERVO_UNDOCKED = 110;

// ===== CENTRAL ESP MAC ADDRESS =====
uint8_t centralESP_MAC[] = {0x8C, 0x94, 0xDF, 0x6E, 0x10, 0x50};

// ===== MISSION STATE =====
volatile int currentMission = 0;
volatile int pendingMission = -1;
unsigned long lastSensorTransmit = 0;
const unsigned long SENSOR_TRANSMIT_INTERVAL = 33;// sampling rate of sensors 

// ===== DATA PACKET STRUCTURE =====
typedef struct {
  uint8_t robot_id;
  float heading;
  float accel_x;
  float accel_y;
  float gyro_z;
  float ultrasonic_cm;
  unsigned long timestamp;
} SensorPacket;

SensorPacket sensorData;

// ===== COMMAND STRUCTURE (incoming from Central ESP) =====
typedef struct {
  char command[32];
  int missionID;
} CommandPacket;

CommandPacket incomingCommand;

// ========== MPU6050 CALIBRATION ==========
void calibrateIMU() {
  Serial.println("\n========================================");
  Serial.println("    MPU6050 CALIBRATION - 10 SECONDS    ");
  Serial.println("  Keep robot STILL on FLAT surface!    ");
  Serial.println("========================================\n");
  
  const int CALIB_SAMPLES = 2000;
  long sumGZ = 0, sumAX = 0, sumAY = 0;
  int16_t ax, ay, az, gx, gy, gz;
  
  for (int i = 0; i < CALIB_SAMPLES; i++) {
    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    sumGZ += gz;
    sumAX += ax;
    sumAY += ay;
    
    if (i % 200 == 0) {
      Serial.print(".");
    }
    delay(5);
  }
  
  gyroBiasZ = (float)sumGZ / CALIB_SAMPLES;
  accelBiasX = (float)sumAX / CALIB_SAMPLES;
  accelBiasY = (float)sumAY / CALIB_SAMPLES;
  
  Serial.println("\n\nCalibration complete!");
  Serial.print("Gyro Z bias: "); Serial.println(gyroBiasZ, 2);
  Serial.print("Accel X bias: "); Serial.println(accelBiasX, 2);
  Serial.print("Accel Y bias: "); Serial.println(accelBiasY, 2);
  Serial.println("========================================\n");
}

// ========== HC-SR04 ULTRASONIC ==========
float measureDistanceCm() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  
  unsigned long duration = pulseIn(ECHO_PIN, HIGH, 30000);
  
  if (duration == 0) return -1.0;
  
  float distance = (duration * 0.0343) / 2.0;
  if (distance < 2.0 || distance > 400.0) return -1.0;
  
  return distance;
}

// ========== SERVO CONTROL ==========
void setAllServos(int angle) {
  Serial.print(" Moving servos: ");
  Serial.print(angle);
  Serial.print("° ... ");
  
  angle = constrain(angle, 0, 180);
  int pulse = map(angle, 0, 180, SERVOMIN, SERVOMAX);
  
  for (int i = 0; i < 16; i++) {
    pwm.setPWM(i, 0, pulse);
  }
  
  Serial.println("DONE");
  delay(500);
}

// ========== READ SENSORS & INTEGRATE HEADING ==========
void readSensors() {
  unsigned long now = micros();
  float dt = (now - lastIMURead) / 1000000.0;
  lastIMURead = now;
  if (dt > 0.5) dt = 0.05;
  
  int16_t ax, ay, az, gx, gy, gz;
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  
  float gyro_z_corrected = ((float)gz - gyroBiasZ) / 131.0 * PI / 180.0;
  float accel_x_corrected = ((float)ax - accelBiasX) / 16384.0 * 9.81;
  float accel_y_corrected = ((float)ay - accelBiasY) / 16384.0 * 9.81;
  
  heading += gyro_z_corrected * dt;
  while (heading > PI) heading -= 2*PI;
  while (heading < -PI) heading += 2*PI;
  
  float distance = measureDistanceCm();
  
  sensorData.robot_id = 0;
  sensorData.heading = heading;
  sensorData.accel_x = accel_x_corrected;
  sensorData.accel_y = accel_y_corrected;
  sensorData.gyro_z = gyro_z_corrected;
  sensorData.ultrasonic_cm = distance;
  sensorData.timestamp = millis();
}

// ========== TRANSMIT SENSOR DATA ==========
void transmitSensorData() {
  esp_err_t result = esp_now_send(centralESP_MAC, (uint8_t*)&sensorData, sizeof(sensorData));
  
  if (result == ESP_OK) {
    Serial.print(" H:");
    Serial.print(sensorData.heading, 2);
    Serial.print(" | US:");
    Serial.print(sensorData.ultrasonic_cm, 1);
    Serial.println("cm");
  } else {
    Serial.println("  TX failed");
  }
}

// ========== EXECUTE MISSION CHANGE ==========
void executeMissionChange(int newMission) {
  Serial.println("\n╔════════════════════════════════╗");
  Serial.print("║  EXECUTING MISSION ");
  Serial.print(newMission);
  Serial.println("          ║");
  Serial.println("╚════════════════════════════════╝");
  
  switch(newMission) {
    case 1:
      Serial.println(" MISSION 1: Sensor readings active");
      break;
    case 2:
      Serial.println(" MISSION 2: DOCKING");
      setAllServos(SERVO_DOCKED);
      break;
    case 3:
      Serial.println(" MISSION 3: UNDOCKING");
      setAllServos(SERVO_UNDOCKED);
      break;
    case 0:
      Serial.println("  MISSION STOPPED");
      break;
  }
  currentMission = newMission;
  Serial.println();
}

// ========== ESP-NOW RECEIVE CALLBACK ==========
void OnDataRecv(const esp_now_recv_info *recv_info, const uint8_t *data, int len) {
  Serial.println("\n>>> CALLBACK TRIGGERED <<<");
  Serial.print("Data length: "); Serial.println(len);
  
  memcpy(&incomingCommand, data, sizeof(incomingCommand));
  
  Serial.print("Command: ");
  Serial.println(incomingCommand.command);
  Serial.print("Mission ID: ");
  Serial.println(incomingCommand.missionID);
  
  pendingMission = incomingCommand.missionID;
  Serial.print("Pending mission set to: ");
  Serial.println(pendingMission);
}

// ========== SETUP ==========
void setup() {
  Serial.begin(115200);
  delay(1000);
  
  Serial.println("\n🚀 RECEIVER ESP STARTING...\n");
  
  WiFi.mode(WIFI_STA);
  delay(500);
  
  Serial.println("=================================");
  Serial.print(" Receiver MAC: ");
  Serial.println(WiFi.macAddress());
  Serial.println("=================================\n");
  
  if (esp_now_init() != ESP_OK) {
    Serial.println(" ESP-NOW init failed");
    while(true) delay(1000);
  }
  Serial.println(" ESP-NOW initialized");
  
  esp_now_register_recv_cb(OnDataRecv);
  
  esp_now_peer_info_t peerInfo = {};
  memcpy(peerInfo.peer_addr, centralESP_MAC, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;
  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println(" Failed to add Central ESP peer");
  } else {
    Serial.println(" Central ESP added as peer");
  }
  
  Serial.println();
  
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  digitalWrite(TRIG_PIN, LOW);
  Serial.println(" HC-SR04 initialized");
  
  Wire.begin(21, 22);
  Serial.println(" I2C initialized");
  
  Serial.println("\nInitializing MPU6050...");
  mpu.initialize();
  if (!mpu.testConnection()) {
    Serial.println(" MPU6050 connection failed!");
    while (true) delay(1000);
  }
  Serial.println(" MPU6050 connected");
  
  calibrateIMU();
  lastIMURead = micros();
  
  Serial.println("Initializing PCA9685...");
  pwm.begin();
  pwm.setPWMFreq(50);
  delay(100);
  setAllServos(90);
  Serial.println(" Servos initialized (90° neutral)");
  
  Serial.println("\n ALL SYSTEMS READY");
  Serial.println("Waiting for mission commands from Central ESP...\n");
}

// ========== LOOP ==========
void loop() {
  // Check for pending mission changes
  if (pendingMission != -1) {
    Serial.println(">>> PENDING MISSION DETECTED IN LOOP <<<");
    executeMissionChange(pendingMission);
    pendingMission = -1;
  }
  
  // MISSION 1: Continuous sensor reading and transmission
  if (currentMission == 1) {
    if (millis() - lastSensorTransmit >= SENSOR_TRANSMIT_INTERVAL) {
      lastSensorTransmit = millis();
      readSensors();
      transmitSensorData();
    }
  }
  
  yield();
}