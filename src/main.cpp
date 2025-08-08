#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

#define CE_PIN 4
#define CSN_PIN 5

RF24 radio(CE_PIN, CSN_PIN);

const byte txAddr[6] = "NODE1"; // 写给接收端
const byte rxAddr[6] = "NODE2"; // 等待接收端回应

const int MAX_FAILS = 5;
int failCount = 0;
bool isConnected = false;

void setup() {
  Serial.begin(115200);
  SPI.begin(18, 19, 23);

  if (!radio.begin()) {
    Serial.println("❌ NRF24 初始化失败！");
    while (1);
  }

  radio.setPALevel(RF24_PA_LOW);
  radio.setDataRate(RF24_1MBPS);
  radio.setChannel(76);
  radio.openWritingPipe(txAddr);    // 发送地址
  radio.openReadingPipe(1, rxAddr); // 接收回应
  radio.stopListening();            // 初始为发送模式

  Serial.println("📡 发送端准备就绪，开始握手");
}

void loop() {
  const char ping[] = "PING";
  radio.stopListening();
  Serial.print("🔼 发送握手请求：");
  Serial.println(ping);

  bool writeSuccess = radio.write(&ping, sizeof(ping));
  if (!writeSuccess) {
    Serial.println("❌ 无法写入，模块可能未准备好");
    failCount++;
  }

  radio.startListening();
  unsigned long startTime = millis();
  bool pongReceived = false;

  while (millis() - startTime < 500) {
    if (radio.available()) {
      char buffer[32] = "";
      radio.read(&buffer, sizeof(buffer));
      if (strcmp(buffer, "PONG") == 0) {
        Serial.println("✅ 握手成功，接收端在线！");
        pongReceived = true;
        break;
      }
    }
  }

  if (pongReceived) {
    if (!isConnected) {
      Serial.println("🎉 通信连接已建立！");
      isConnected = true;
    }
    failCount = 0;
  } else {
    Serial.println("⚠️ 没有回应，接收端可能掉线");
    failCount++;

    if (failCount >= MAX_FAILS && isConnected) {
      Serial.println("⛔ 连接断开，等待重连...");
      isConnected = false;
    }
  }

  delay(2000); // 每2秒握手一次
}