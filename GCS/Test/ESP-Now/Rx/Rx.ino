#include <esp_now.h>
#include <WiFi.h>

// MAC của Node A (YoloUno)
uint8_t macA[] = {0xDC, 0xDA, 0x0C, 0x18, 0xE8, 0x08};

bool canReply = true;

// Nhận dữ liệu
void OnDataRecv(const esp_now_recv_info_t *recv_info, const uint8_t *data, int len) {
  char msg[32];
  memcpy(msg, data, len);
  msg[len] = '\0';
  Serial.printf("B receive: %s\n", msg);

  if (strcmp(msg, "Hello B") == 0 && canReply) {
    canReply = false;
    delay(5);  // Tránh xung đột
    const char* reply = "Hello A";
    esp_now_send(macA, (uint8_t*)reply, strlen(reply));
    Serial.println("B send: Hello A");
  }
}

// Gửi xong → cho phép nhận lại
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  canReply = true;
}

void setup() {
  Serial.begin(115200);
  WiFi.mode(WIFI_STA);

  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW init failed");
    while (1);
  }

  esp_now_register_recv_cb(OnDataRecv);
  esp_now_register_send_cb(OnDataSent);

  esp_now_peer_info_t peerInfo{};
  memcpy(peerInfo.peer_addr, macA, 6);
  peerInfo.channel = 1;
  peerInfo.encrypt = false;

  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("Failed to add peer A");
    while (1);
  }

  Serial.println("Node B (GOOUUU) ready!");
  Serial.println("MAC: " + WiFi.macAddress());
}

void loop() {
  // Chỉ phản hồi khi nhận tin
}