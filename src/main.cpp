#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

#define CE_PIN 4
#define CSN_PIN 5

RF24 radio(CE_PIN, CSN_PIN);
const byte txAddr[6] = "CTRL1"; // 发送给接收端
const byte rxAddr[6] = "BASE1"; // 接收ACK

struct ControlPacket {
  int16_t throttle;
  int16_t steering;
  uint8_t flags;
};

unsigned long lastAck = 0;
bool isConnected = false;
const int ACK_TIMEOUT = 1000; // 毫秒

void setup() {
  Serial.begin(115200);
  SPI.begin(18, 19, 23);

  if (!radio.begin()) {
    Serial.println("NRF24 初始化失败");
    while (1);
  }

  radio.setPALevel(RF24_PA_LOW);
  radio.setDataRate(RF24_1MBPS);
  radio.setChannel(90);
  radio.openWritingPipe(txAddr);
  radio.openReadingPipe(1, rxAddr);
  radio.stopListening();

  Serial.println("🎮 遥控端启动完成");
}

void loop() {
  ControlPacket packet;

  // 模拟控制数据（可以用摇杆或 ADC 替换）
  packet.throttle = 400;  // 你可以从摇杆读取真实值
  packet.steering = 100;
  packet.flags = 0b00000001; // 比如打开灯光

  // 发送数据
  radio.stopListening();
  bool sent = radio.write(&packet, sizeof(packet));

  if (sent) {
    Serial.println("📤 控制信号已发送");
  } else {
    Serial.println("⚠️ 发送失败");
  }

  // 等待回应（ACK）
  radio.startListening();
  unsigned long waitStart = millis();
  bool gotAck = false;
  while (millis() - waitStart < ACK_TIMEOUT) {
    if (radio.available()) {
      char ack[32];
      radio.read(&ack, sizeof(ack));
      if (strcmp(ack, "ACK") == 0) {
        gotAck = true;
        break;
      }
    }
  }

  if (gotAck) {
    if (!isConnected) {
      Serial.println("✅ 已连接接收端！");
      isConnected = true;
    }
    lastAck = millis();
  } else {
    Serial.println("❌ 无回应，接收端可能掉线");
    if (isConnected && millis() - lastAck > ACK_TIMEOUT * 5) {
      isConnected = false;
      Serial.println("⛔ 连接断开");
    }
  }

  delay(100); // 控制帧发送频率
}
