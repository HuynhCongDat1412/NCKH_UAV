#include <esp_now.h>
#include <WiFi.h>
#include <ArduinoJson.h>

// Hardcoded GCS MAC
uint8_t gcsMac[6] = {0x10, 0x51, 0xDB, 0x85, 0x50, 0xB0};
bool gcsPaired = false;

typedef struct {
  uint32_t timestamp;
  float roll, pitch, yaw;
  float battery;
  bool armed;
} TelemetryPacket;

TelemetryPacket telemetry;

bool isArmed = false;
float simRoll = 0, simPitch = 0, simYaw = 0;
float simBattery = 12.6;
unsigned long lastUpdate = 0;
unsigned long lastSend = 0;

bool testMode = false;
int testFreq = 50;
unsigned long testInterval = 1000 / 50;
float testPhase = 0;

// --- SỬA OnDataRecv ---
void OnDataRecv(const esp_now_recv_info_t *info, const uint8_t *data, int len) {
  if (!gcsPaired || memcmp(info->src_addr, gcsMac, 6) != 0) return;

  // Xử lý lệnh...
  if (len > 0) {
    String cmd = String((char*)data);
    cmd.trim();
    Serial.println("[Drone_ESP32] CMD: " + cmd);

    StaticJsonDocument<256> doc;
    if (deserializeJson(doc, cmd)) return;

    const char* type = doc["type"] | "";
    if (strcmp(type, "CMD") == 0) {
      const char* c = doc["cmd"] | "";
      if (strcmp(c, "ARM") == 0) { isArmed = true; sendAck("ARMED"); }
      else if (strcmp(c, "DISARM") == 0) { isArmed = false; sendAck("DISARMED"); }
    } else if (strcmp(type, "PID") == 0) {
      sendAck("PID_UPDATED");
    }
  }
}

void sendAck(const char* status) {
  StaticJsonDocument<128> doc;
  doc["ack"] = status;
  doc["timestamp"] = millis();
  char buf[128];
  size_t n = serializeJson(doc, buf);
  esp_now_send(gcsMac, (uint8_t*)buf, n);
}

void sendData(uint8_t packetType, const void* data, size_t len) {
  uint8_t buffer[len + 1];
  buffer[0] = packetType;
  memcpy(buffer + 1, data, len);
  esp_now_send(gcsMac, buffer, sizeof(buffer));
}

void setup() {
  Serial.begin(115200);
  Serial.println("[Drone_ESP32] Booting...");
  WiFi.mode(WIFI_STA);
  delay(500);
  Serial.println("[Drone_ESP32] WiFi MAC: " + WiFi.macAddress());

  if (esp_now_init() != ESP_OK) {
    Serial.println("[Drone_ESP32] ESP-NOW Init Failed");
    while (1);
  }

  esp_now_register_recv_cb(OnDataRecv);

  esp_now_peer_info_t peer = {};
  memcpy(peer.peer_addr, gcsMac, 6);
  peer.channel = 1;
  peer.encrypt = false;
  if (esp_now_add_peer(&peer) == ESP_OK) {
    gcsPaired = true;
    Serial.println("[Drone_ESP32] GCS paired: " + macToString(gcsMac));
  } else {
    Serial.println("[Drone_ESP32] Pair GCS failed!");
    while (1);
  }

  Serial.println("[Drone_ESP32] Ready");
}

void loop() {
  unsigned long now = millis();
  if (now - lastSend >= testInterval) {
    lastSend = now;
    testPhase += 0.1;

    float sine = sin(testPhase) * 10.0;
    float sine1 = sin(testPhase + 2) * 10.0;
    float sine2 = sin(testPhase - 2) * 10.0;

    StaticJsonDocument<256> doc;
    doc["timestamp"] = now;
    doc["roll"] = roundf(sine * 10) / 10.0;
    doc["pitch"] = roundf(sine1 * 10) / 10.0;
    doc["yaw"] = roundf(sine2 * 10) / 10.0;
    doc["battery"] = 12.6;
    doc["armed"] = isArmed;

    char buf[256];
    serializeJson(doc, buf);
    sendData(0x02, buf, strlen(buf));
  }
}

String macToString(uint8_t* mac) {
  char buf[18];
  snprintf(buf, sizeof(buf), "%02X:%02X:%02X:%02X:%02X:%02X",
           mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
  return String(buf);
}