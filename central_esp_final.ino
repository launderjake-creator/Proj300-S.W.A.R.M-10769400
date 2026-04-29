// CENTRAL ESP32 - Keyboard-Controlled Gateway
// Type commands in Serial Monitor to control Receiver ESP
// Displays received sensor data

#include <esp_now.h>
#include <WiFi.h>

// ===== RECEIVER ESP MAC ADDRESS =====
uint8_t receiverESP_MAC[] = {0xC4, 0xDE, 0xE2, 0x9C, 0x7E, 0xE0};

// ===== DATA STRUCTURES =====
typedef struct {
  uint8_t robot_id;
  float heading;
  float accel_x;
  float accel_y;
  float gyro_z;
  float ultrasonic_cm;
  unsigned long timestamp;
} SensorPacket;

typedef struct {
  char command[32];
  int missionID;
} CommandPacket;

SensorPacket receivedData;
CommandPacket commandToSend;
esp_now_peer_info_t peerInfo;

// ===== ESP-NOW RECEIVE CALLBACK =====
void OnDataRecv(const esp_now_recv_info *recv_info, const uint8_t *data, int len) {
  if (len != sizeof(SensorPacket)) {
    Serial.println(" Invalid packet size");
    return;
  }
  
  memcpy(&receivedData, data, sizeof(receivedData));
  
  // Display received sensor data
  Serial.println("┌─────────────────────────────────────┐");
  Serial.print("│ Heading:    "); Serial.print(receivedData.heading, 2); Serial.println(" rad");
  Serial.print("│ Accel X:    "); Serial.print(receivedData.accel_x, 2); Serial.println(" m/s²");
  Serial.print("│ Accel Y:    "); Serial.print(receivedData.accel_y, 2); Serial.println(" m/s²");
  Serial.print("│ Gyro Z:     "); Serial.print(receivedData.gyro_z, 4); Serial.println(" rad/s");
  Serial.print("│ Ultrasonic: "); Serial.print(receivedData.ultrasonic_cm, 1); Serial.println(" cm");
  Serial.print("│ Timestamp:  "); Serial.print(receivedData.timestamp); Serial.println(" ms");
  Serial.println("└─────────────────────────────────────┘");
}

// ===== ESP-NOW SEND CALLBACK =====
void OnDataSent(const uint8_t *mac, esp_now_send_status_t status) {
  Serial.print(" Send status: ");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? " SUCCESS" : " FAILED");
}

// ===== SEND COMMAND TO RECEIVER =====
void sendCommand(const char* cmd, int missionID) {
  memset(&commandToSend, 0, sizeof(commandToSend));
  strncpy(commandToSend.command, cmd, 31);
  commandToSend.missionID = missionID;
  
  Serial.println("\n╔════════════════════════════════════╗");
  Serial.print("║  SENDING: Mission ");
  Serial.print(missionID);
  Serial.print(" - ");
  Serial.print(cmd);
  for (int i = strlen(cmd); i < 15; i++) Serial.print(" ");
  Serial.println("║");
  Serial.println("╚════════════════════════════════════╝");
  
  esp_err_t result = esp_now_send(receiverESP_MAC, (uint8_t*)&commandToSend, sizeof(commandToSend));
}

// ===== SETUP =====
void setup() {
  Serial.begin(115200);
  delay(1000);
  
  Serial.println("\n╔════════════════════════════════════╗");
  Serial.println("║   CENTRAL ESP - MISSION CONTROL    ║");
  Serial.println("╚════════════════════════════════════╝\n");
  
  WiFi.mode(WIFI_STA);
  delay(500);
  
  Serial.println("=================================");
  Serial.print(" Central MAC: ");
  Serial.println(WiFi.macAddress());
  Serial.println("=================================\n");
  
  if (esp_now_init() != ESP_OK) {
    Serial.println(" ESP-NOW init failed");
    return;
  }
  Serial.println(" ESP-NOW initialized");
  
  esp_now_register_recv_cb(OnDataRecv);
  esp_now_register_send_cb(reinterpret_cast<esp_now_send_cb_t>(OnDataSent));
  
  memset(&peerInfo, 0, sizeof(peerInfo));
  memcpy(peerInfo.peer_addr, receiverESP_MAC, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;
  
  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println(" Failed to add Receiver peer");
  } else {
    Serial.println("Receiver peer added");
  }
  
  Serial.println("\n⌨️  KEYBOARD CONTROLS:");
  Serial.println("  Type '1' + ENTER → Mission 1 (Sensors)");
  Serial.println("  Type '2' + ENTER → Mission 2 (Dock)");
  Serial.println("  Type '3' + ENTER → Mission 3 (Undock)");
  Serial.println("  Type '0' + ENTER → Mission 0 (Stop)");
  Serial.println("\n READY - Type commands in Serial Monitor\n");
}

// ===== LOOP =====
void loop() {
  // Check for keyboard input from Serial Monitor
  if (Serial.available() > 0) {
    char key = Serial.read();
    
    switch(key) {
      case '1':
        sendCommand("MISSION_SENSORS", 1);
        break;
      case '2':
        sendCommand("MISSION_DOCK", 2);
        break;
      case '3':
        sendCommand("MISSION_UNDOCK", 3);
        break;
      case '0':
        sendCommand("MISSION_STOP", 0);
        break;
      case '\n':
      case '\r':
      case ' ':
        // Ignore newlines, carriage returns, spaces
        break;
      default:
        Serial.print(" Unknown command: '");
        Serial.print(key);
        Serial.println("' - Use 0, 1, 2, or 3");
        break;
    }
  }
}