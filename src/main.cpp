// ===== TX (ESP32): Joystick -> nRF24 (x,y,seq) =====
#include <Arduino.h>
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

#define CE_PIN  4
#define CSN_PIN 5
#define SCK_PIN 18
#define MISO_PIN 19
#define MOSI_PIN 23

RF24 radio(CE_PIN, CSN_PIN);

const byte txAddr[6] = "CTRL1";
const byte rxAddr[6] = "BASE1";

// Joystick
static const int PIN_X = 36;   // VRx
static const int PIN_Y = 39;   // VRy

// 无线参数（避免与 RF24 的 RF_CH 宏冲突，改名 RF_CHANNEL）
static const uint8_t RF_CHANNEL = 76;

// 映射/滤波参数
static const uint32_t CALIB_MS   = 600;
static const float    EMA_ALPHA  = 0.25f;
static const int      DEADZONE   = 10;      // 原始 ADC ±10
static const bool     INVERT_Y   = true;
static const int      MIN_SPAN   = 400;

struct __attribute__((packed)) JoyPacket { uint8_t x, y, seq; };

static float emaX = 2048.0f, emaY = 2048.0f;
static int centerX = 2048, centerY = 2048;
static int minX, maxX, minY, maxY;

unsigned long lastSend = 0;
uint8_t seq = 1;
uint8_t failCount = 0;

static inline int map1to255_fromMinMax(int raw, int mn, int mx) {
  int span = mx - mn; if (span < 1) return 128;
  float t = (float)(raw - mn) / (float)span;
  float out = 1.0f + t * 254.0f;
  if (out < 1.0f) out = 1.0f; if (out > 255.0f) out = 255.0f;
  return (int)(out + 0.5f);
}

void radioReinitTX() {
  radio.stopListening();
  radio.setPALevel(RF24_PA_MIN);
  radio.setDataRate(RF24_250KBPS);
  radio.setChannel(RF_CHANNEL);
  radio.setRetries(15, 15);
  radio.setCRCLength(RF24_CRC_16);
  radio.setAutoAck(true);
  radio.disableDynamicPayloads();
  radio.setPayloadSize(sizeof(JoyPacket));
  radio.openWritingPipe(txAddr);
  radio.openReadingPipe(1, rxAddr); // 保留
  radio.stopListening();
}

void setup() {
  Serial.begin(115200); delay(200);
  analogSetPinAttenuation(PIN_X, ADC_11db);
  analogSetPinAttenuation(PIN_Y, ADC_11db);
  analogReadResolution(12);

  // 启动均值中心
  uint32_t t0 = millis(); uint32_t cnt=0; uint64_t sx=0, sy=0;
  while (millis()-t0 < CALIB_MS) { sx += analogRead(PIN_X); sy += analogRead(PIN_Y); cnt++; delay(5); }
  if (cnt){ centerX = (int)(sx/cnt); centerY = (int)(sy/cnt); }
  emaX=centerX; emaY=centerY;

  minX=max(0,centerX-MIN_SPAN/2); maxX=min(4095,centerX+MIN_SPAN/2);
  minY=max(0,centerY-MIN_SPAN/2); maxY=min(4095,centerY+MIN_SPAN/2);

  SPI.begin(SCK_PIN, MISO_PIN, MOSI_PIN);
  if (!radio.begin()) { Serial.println("❌ NRF24 init fail"); while(1){} }
  radioReinitTX();

  Serial.print("Center X="); Serial.print(centerX);
  Serial.print(" Y="); Serial.println(centerY);
  Serial.println("TX ready: 50ms/frame");
}

void loop() {
  // 采样 + EMA
  int rx = analogRead(PIN_X), ry = analogRead(PIN_Y);
  emaX += EMA_ALPHA * ((float)rx - emaX);
  emaY += EMA_ALPHA * ((float)ry - emaY);
  int x=(int)emaX, y=(int)emaY;

  // 学习极值
  if (x<minX) minX=x; if (x>maxX) maxX=x;
  if (y<minY) minY=y; if (y>maxY) maxY=y;
  if (maxX-minX<MIN_SPAN){ int m=(maxX+minX)/2; minX=max(0,m-MIN_SPAN/2); maxX=min(4095,m+MIN_SPAN/2); }
  if (maxY-minY<MIN_SPAN){ int m=(maxY+minY)/2; minY=max(0,m-MIN_SPAN/2); maxY=min(4095,m+MIN_SPAN/2); }

  // 死区 + 映射
  int outX = (abs(x-centerX)<=DEADZONE)?128:map1to255_fromMinMax(x,minX,maxX);
  int outY = (abs(y-centerY)<=DEADZONE)?128:map1to255_fromMinMax(y,minY,maxY);
  if (INVERT_Y) outY = 256 - outY;

  // 发送
  if (millis()-lastSend >= 50) {
    lastSend = millis();
    JoyPacket pkt{ (uint8_t)outX, (uint8_t)outY, seq };
    bool ok = radio.write(&pkt, sizeof(pkt));
    if (!ok) {
      if (++failCount >= 8) {
        Serial.println("🔁 TX reinit");
        radio.powerDown(); delay(5); radio.powerUp(); delay(5); radioReinitTX();
        failCount = 0;
      }
    } else failCount = 0;
    seq = (seq>=255)?1:(uint8_t)(seq+1);
  }

  delay(5);
}
