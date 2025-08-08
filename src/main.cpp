#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

#define CE_PIN 4
#define CSN_PIN 5
#define SCK_PIN 18
#define MISO_PIN 19
#define MOSI_PIN 23

// ====== 调速参数（按需改） ======
static const uint16_t SEND_INTERVAL_MS = 50;   // 每帧间隔 50ms
static const uint16_t TIMEOUT_MS       = 150;  // 150ms 未ACK 判定掉线（≈3帧丢失）

RF24 radio(CE_PIN, CSN_PIN);

// 地址配对：发给 CTRL1，接收端监听 CTRL1；接收端回ACK（硬件自动）
const byte txAddr[6] = "CTRL1"; // 发送地址（接收端 openReadingPipe(…, "CTRL1")）
const byte rxAddr[6] = "BASE1"; // 仅用于保留对称（此方案不读回payload）

// 控制数据包（rctestflight风格：简单、实用）
struct ControlPacket {
  int16_t throttle;  // -512..+512
  int16_t steering;  // -512..+512
  uint8_t flags;     // 位标志：如灯、模式等
} __attribute__((packed));

unsigned long lastSend = 0;
unsigned long lastAck  = 0;
bool isConnected = false;

void setup() {
  Serial.begin(115200);
  SPI.begin(SCK_PIN, MISO_PIN, MOSI_PIN);

  if (!radio.begin()) {
    Serial.println("❌ NRF24 初始化失败（检查接线/电源/电容）");
    while (1) {}
  }

  // —— 通信调参：稳优先 ——
  radio.setPALevel(RF24_PA_LOW);     // LOW 稳定省电；远距可试 HIGH（需电容更稳）
  radio.setDataRate(RF24_1MBPS);     // 1Mbps 抗干扰/延迟兼顾
  radio.setChannel(90);              // 避开Wi-Fi，可按环境微调
  radio.setRetries(5, 15);           // 自动重发（硬件ARQ）
  radio.setCRCLength(RF24_CRC_16);
  radio.enableDynamicPayloads();     // 可变负载（不是必须）
  radio.setAutoAck(true);            // **关键：开启硬件ACK**

  radio.openWritingPipe(txAddr);
  radio.openReadingPipe(1, rxAddr);  // 本方案不拉取ACK payload，但保留对称配置
  radio.stopListening();              // 纯发送端：常驻发送模式

  lastSend = millis();
  lastAck  = millis();

  Serial.println("🎮 遥控端就绪（50ms/帧，150ms 掉线判定）");
}

static inline int16_t readThrottleMock() {
  // TODO: 用真实摇杆/ADC替换
  return 300;
}
static inline int16_t readSteeringMock() {
  // TODO: 用真实摇杆/ADC替换
  return -100;
}
static inline uint8_t readFlagsMock() {
  // TODO: 位0=灯、位1=蜂鸣器... 自己定义
  return 0b00000001;
}

void loop() {
  const unsigned long now = millis();

  // 到了发帧时间就发
  if (now - lastSend >= SEND_INTERVAL_MS) {
    lastSend = now;

    ControlPacket pkt;
    pkt.throttle = readThrottleMock();
    pkt.steering = readSteeringMock();
    pkt.flags    = readFlagsMock();

    // 发送 + 硬件ACK 判断在线
    bool acked = radio.write(&pkt, sizeof(pkt));
    if (acked) {
      // 任一帧ACK即认为对端在线
      if (!isConnected) {
        Serial.println("✅ 接收端上线（ACK ok）");
        isConnected = true;
      }
      lastAck = now;
    } else {
      Serial.println("⚠️ 本帧未ACK（可能瞬时丢包/远距）");
    }
  }

  // 低延迟掉线判定（150ms ≈ 丢 3 帧）
  if (isConnected && (millis() - lastAck > TIMEOUT_MS)) {
    isConnected = false;
    Serial.println("⛔ 连接丢失（>150ms 无ACK）—— 等待自动重连…");
    // 继续按 50ms 周期发帧，收到任意ACK会自动恢复状态
  }
}
