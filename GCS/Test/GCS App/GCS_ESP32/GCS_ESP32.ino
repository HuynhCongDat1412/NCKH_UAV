#include <esp_now.h>
#include <WiFi.h>
#include <ArduinoJson.h>

// Hardcoded Drone MAC
uint8_t droneMac[6] = {0x10, 0x51, 0xDB, 0x85, 0xCD, 0xE8};  // THAY BẰNG MAC CỦA DRONE_ESP32
bool dronePaired = false;

typedef struct {
  uint32_t timestamp;
  float roll, pitch, yaw;
  float battery;
  bool armed;
  int8_t rssi;
  float loss;
} TelemetryPacket;

TelemetryPacket telemetry;

esp_now_peer_info_t peerInfo;

// Callback khi send thành công/thất bại
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  if (status != ESP_NOW_SEND_SUCCESS) {
    Serial.println("[GCS_ESP32] Send failed");
  }
}

// Hàm nhận dữ liệu với format rõ ràng (dễ scale)
void OnDataRecv(const esp_now_recv_info_t *info, const uint8_t *data, int len) {
  if (!dronePaired || memcmp(info->src_addr, droneMac, 6) != 0) return;

  // Kiểm tra loại packet (dùng byte đầu tiên làm type, ví dụ)
  if (len < 1) return;
  uint8_t packetType = data[0];
  
  switch (packetType) {
    case 0x01:  // Telemetry type
      if (len == sizeof(TelemetryPacket) + 1) {  // +1 for type
        memcpy(&telemetry, data + 1, sizeof(TelemetryPacket));
        
        // Serialize to JSON
        StaticJsonDocument<256> doc;
        doc["timestamp"] = telemetry.timestamp;
        doc["roll"] = roundf(telemetry.roll * 10) / 10.0;
        doc["pitch"] = roundf(telemetry.pitch * 10) / 10.0;
        doc["yaw"] = roundf(telemetry.yaw * 10) / 10.0;
        doc["battery"] = roundf(telemetry.battery * 10) / 10.0;
        doc["armed"] = telemetry.armed;
        doc["rssi"] = (int8_t)telemetry.rssi;
        doc["loss"] = telemetry.loss;

        char output[256];
        serializeJson(doc, output);
        Serial.println(output);
      }
      break;
      
    case 0x02:  // Test data
      if (len > 1) {
        // Gửi nguyên JSON string (không thêm type byte)
        Serial.write(data + 1, len - 1);
        Serial.println();  // Thêm newline
      }
      break;
      
    default:
      Serial.println("[GCS_ESP32] Unknown packet type");
      break;
  }
}

void setup() {
  Serial.begin(115200);
  Serial.println("[GCS_ESP32] Booting...");
  WiFi.mode(WIFI_STA);
  delay(500);
  Serial.println("[GCS_ESP32] WiFi MAC: " + WiFi.macAddress());

  if (esp_now_init() != ESP_OK) {
    Serial.println("[GCS_ESP32] ESP-NOW Init Failed");
    while (1);
  }

  Serial.println("[GCS_ESP32] ESP-NOW Initialized");

  esp_now_register_recv_cb(OnDataRecv);
  esp_now_register_send_cb(OnDataSent);

  // Pair drone hardcoded
  memset(&peerInfo, 0, sizeof(peerInfo));
  memcpy(peerInfo.peer_addr, droneMac, 6);
  peerInfo.channel = 1;
  peerInfo.encrypt = false;
  if (esp_now_add_peer(&peerInfo) == ESP_OK) {
    dronePaired = true;
    Serial.println("[GCS_ESP32] Drone paired: " + macToString(droneMac));
  } else {
    Serial.println("[GCS_ESP32] Pair Drone failed!");
  }

  Serial.println("[GCS_ESP32] Ready");
  Serial.println("[GCS_ESP32] Waiting for CONNECT...");
}

void loop() {
  if (!Serial.available()) return;

  String input = Serial.readStringUntil('\n');
  input.trim();

  if (input == "CONNECT") {
    Serial.println("CONNECT_OK");
    Serial.flush();
    return;
  }

  if (input.startsWith("{") && dronePaired) {
    // Send JSON command qua ESP-NOW
    uint8_t sendStatus = esp_now_send(droneMac, (uint8_t*)input.c_str(), input.length());
    if (sendStatus != ESP_OK) {
      Serial.println("[GCS_ESP32] esp_now_send failed");
    }
  }
}

String macToString(uint8_t* mac) {
  char buf[18];
  snprintf(buf, sizeof(buf), "%02X:%02X:%02X:%02X:%02X:%02X",
           mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
  return String(buf);
}