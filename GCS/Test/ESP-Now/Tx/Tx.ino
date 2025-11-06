#include <esp_now.h>
#include <WiFi.h>

// MAC của Node B (GOOUUU)
uint8_t macB[] = {0x94, 0xA9, 0x90, 0x30, 0x51, 0xB0};

unsigned long lastTime = 0;
const long interval = 1000;  // 1 giây

// Gửi xong (thành công hay thất bại)
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  if (status == ESP_NOW_SEND_SUCCESS) {
    Serial.println("A send: Hello B");
  } else {
    Serial.println("A send failed");
  }
  // Không cần cờ messageSent nữa
}

void OnDataRecv(const esp_now_recv_info_t *recv_info, const uint8_t *data, int len) {
  char msg[32];
  memcpy(msg, data, len);
  msg[len] = '\0';
  Serial.printf("A receive: %s\n", msg);
}

void setup() {
  Serial.begin(115200);
  WiFi.mode(WIFI_STA);

  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW init failed");
    while (1);
  }

  esp_now_register_send_cb(OnDataSent);
  esp_now_register_recv_cb(OnDataRecv);

  esp_now_peer_info_t peerInfo{};
  memcpy(peerInfo.peer_addr, macB, 6);
  peerInfo.channel = 1;
  peerInfo.encrypt = false;

  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("Failed to add peer B");
    while (1);
  }

  Serial.println("Node A (YoloUno) ready!");
  Serial.println("MAC: " + WiFi.macAddress());
}

void loop() {
  unsigned long now = millis();

  // Gửi mỗi 1 giây, không cần cờ
  if (now - lastTime >= interval) {
    const char* message = "Hello B";
    esp_err_t result = esp_now_send(macB, (uint8_t*)message, strlen(message));
    
    lastTime = now;  // Cập nhật thời gian ngay lập tức
  }
}