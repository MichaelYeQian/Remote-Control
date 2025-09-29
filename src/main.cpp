// // ===== TX (ESP32): Joystick -> nRF24 (x,y,seq) =====
// #include <Arduino.h>
// #include <SPI.h>
// #include <nRF24L01.h>
// #include <RF24.h>

// #define CE_PIN  4
// #define CSN_PIN 5
// #define SCK_PIN 18
// #define MISO_PIN 19
// #define MOSI_PIN 23

// RF24 radio(CE_PIN, CSN_PIN);

// const byte txAddr[6] = "CTRL1";
// const byte rxAddr[6] = "BASE1";

// // Joystick
// static const int PIN_X = 36;   // VRx
// static const int PIN_Y = 39;   // VRy

// // 无线参数（避免与 RF24 的 RF_CH 宏冲突，改名 RF_CHANNEL）
// static const uint8_t RF_CHANNEL = 76;

// // 映射/滤波参数
// static const uint32_t CALIB_MS   = 600;
// static const float    EMA_ALPHA  = 0.25f;
// static const int      DEADZONE   = 10;      // 原始 ADC ±10
// static const bool     INVERT_Y   = true;
// static const int      MIN_SPAN   = 400;

// struct __attribute__((packed)) JoyPacket { uint8_t x, y, seq; };

// static float emaX = 2048.0f, emaY = 2048.0f;
// static int centerX = 2048, centerY = 2048;
// static int minX, maxX, minY, maxY;

// unsigned long lastSend = 0;
// uint8_t seq = 1;
// uint8_t failCount = 0;

// static inline int map1to255_fromMinMax(int raw, int mn, int mx) {
//   int span = mx - mn; if (span < 1) return 128;
//   float t = (float)(raw - mn) / (float)span;
//   float out = 1.0f + t * 254.0f;
//   if (out < 1.0f) out = 1.0f; if (out > 255.0f) out = 255.0f;
//   return (int)(out + 0.5f);
// }

// void radioReinitTX() {
//   radio.stopListening();
//   radio.setPALevel(RF24_PA_MIN);
//   radio.setDataRate(RF24_250KBPS);
//   radio.setChannel(RF_CHANNEL);
//   radio.setRetries(15, 15);
//   radio.setCRCLength(RF24_CRC_16);
//   radio.setAutoAck(true);
//   radio.disableDynamicPayloads();
//   radio.setPayloadSize(sizeof(JoyPacket));
//   radio.openWritingPipe(txAddr);
//   radio.openReadingPipe(1, rxAddr); // 保留
//   radio.stopListening();
// }

// void setup() {
//   Serial.begin(115200); delay(200);
//   analogSetPinAttenuation(PIN_X, ADC_11db);
//   analogSetPinAttenuation(PIN_Y, ADC_11db);
//   analogReadResolution(12);

//   // 启动均值中心
//   uint32_t t0 = millis(); uint32_t cnt=0; uint64_t sx=0, sy=0;
//   while (millis()-t0 < CALIB_MS) { sx += analogRead(PIN_X); sy += analogRead(PIN_Y); cnt++; delay(5); }
//   if (cnt){ centerX = (int)(sx/cnt); centerY = (int)(sy/cnt); }
//   emaX=centerX; emaY=centerY;

//   minX=max(0,centerX-MIN_SPAN/2); maxX=min(4095,centerX+MIN_SPAN/2);
//   minY=max(0,centerY-MIN_SPAN/2); maxY=min(4095,centerY+MIN_SPAN/2);

//   SPI.begin(SCK_PIN, MISO_PIN, MOSI_PIN);
//   if (!radio.begin()) { Serial.println("❌ NRF24 init fail"); while(1){} }
//   radioReinitTX();

//   Serial.print("Center X="); Serial.print(centerX);
//   Serial.print(" Y="); Serial.println(centerY);
//   Serial.println("TX ready: 50ms/frame");
// }

// void loop() {
//   // 采样 + EMA
//   int rx = analogRead(PIN_X), ry = analogRead(PIN_Y);
//   emaX += EMA_ALPHA * ((float)rx - emaX);
//   emaY += EMA_ALPHA * ((float)ry - emaY);
//   int x=(int)emaX, y=(int)emaY;

//   // 学习极值
//   if (x<minX) minX=x; if (x>maxX) maxX=x;
//   if (y<minY) minY=y; if (y>maxY) maxY=y;
//   if (maxX-minX<MIN_SPAN){ int m=(maxX+minX)/2; minX=max(0,m-MIN_SPAN/2); maxX=min(4095,m+MIN_SPAN/2); }
//   if (maxY-minY<MIN_SPAN){ int m=(maxY+minY)/2; minY=max(0,m-MIN_SPAN/2); maxY=min(4095,m+MIN_SPAN/2); }

//   // 死区 + 映射
//   int outX = (abs(x-centerX)<=DEADZONE)?128:map1to255_fromMinMax(x,minX,maxX);
//   int outY = (abs(y-centerY)<=DEADZONE)?128:map1to255_fromMinMax(y,minY,maxY);
//   if (INVERT_Y) outY = 256 - outY;

//   // 发送
//   if (millis()-lastSend >= 50) {
//     lastSend = millis();
//     JoyPacket pkt{ (uint8_t)outX, (uint8_t)outY, seq };
//     bool ok = radio.write(&pkt, sizeof(pkt));
//     if (!ok) {
//       if (++failCount >= 8) {
//         Serial.println("🔁 TX reinit");
//         radio.powerDown(); delay(5); radio.powerUp(); delay(5); radioReinitTX();
//         failCount = 0;
//       }
//     } else failCount = 0;
//     seq = (seq>=255)?1:(uint8_t)(seq+1);
//   }

//   delay(5);
// }


// ESP32 + OLED (SSD1306 I2C 128x64) + Dual Joystick
// Mapping: Left joystick X(GPIO36) -> THR (inverted)
//          Right joystick Y(GPIO33) -> STR (inverted)
// Output range: 1..255

// ESP32 + OLED (SSD1306 I2C 128x64) + Dual Joystick
// Mapping: Left joystick X(GPIO36) -> THR (INVERTED)
//          Right joystick Y(GPIO33) -> STR (INVERTED)
// Output range: 1..255

// ESP32 + OLED (SSD1306 I2C 128x64) + Dual Joystick
// Mapping: Left joystick X(GPIO36) -> THR (normal)
//          Right joystick Y(GPIO33) -> STR (normal)
// Output range: 1..255






























// #include <Arduino.h>
// #include <Wire.h>
// #include <Adafruit_GFX.h>
// #include <Adafruit_SSD1306.h>

// // ==== OLED (128x64) ====
// #define SCREEN_WIDTH 128
// #define SCREEN_HEIGHT 64
// const uint8_t PIN_SDA = 21, PIN_SCL = 22;   // I2C pins on ESP32
// const uint8_t OLED_ADDR = 0x3C;             // 0x3C or 0x3D
// Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

// // ==== Joystick pins ====
// // Left joystick X -> THR; Right joystick Y -> STR
// const uint8_t JOY_LEFT_X  = 36; // THR (ADC1_CH0, input-only)
// const uint8_t JOY_RIGHT_Y = 33; // STR (ADC1_CH5)

// // ==== Mapping parameters ====
// static const int DEADZONE = 15;     // deadzone near center
// static const int OUT_MIN  = 1;      // final output range 1..255
// static const int OUT_MAX  = 255;
// static const int OUT_MID  = (OUT_MIN + OUT_MAX) / 2; // 128

// // Now both are normal (not inverted)
// bool INVERT_THR = false;  // GPIO36 normal
// bool INVERT_STR = false;  // GPIO33 normal

// // Map ADC(0..4095) -> -512..512 -> deadzone -> 1..255
// int mapAxisToParam(uint8_t pin, bool invert) {
//   int raw = analogRead(pin);                 // 0..4095
//   int v   = map(raw, 0, 4095, -512, 512);    // scale to -512..512
//   if (invert) v = -v;                        // invert if needed
//   if (abs(v) < DEADZONE) v = 0;              // apply deadzone
//   int p = map(v, -512, 512, OUT_MIN, OUT_MAX);
//   return constrain(p, OUT_MIN, OUT_MAX);
// }

// // Draw a horizontal bar centered at mid (128)
// void drawHBar(int16_t y, int value1to255) {
//   int len = map(value1to255, OUT_MIN, OUT_MAX, -60, 60);
//   int16_t x0 = 4, cx = x0 + 60; // center x
//   display.drawFastHLine(x0, y, 120, SSD1306_WHITE);
//   display.drawFastVLine(cx, y-3, 7, SSD1306_WHITE);
//   if (len > 0)      display.fillRect(cx+1,   y-2,  len,   5, SSD1306_WHITE);
//   else if (len < 0) display.fillRect(cx+len, y-2, -len,   5, SSD1306_WHITE);
// }

// void setup() {
//   Serial.begin(115200);

//   // I2C + OLED
//   Wire.begin(PIN_SDA, PIN_SCL);
//   if (!display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR)) {
//     Serial.println("[OLED] Not found. Check wiring/address.");
//   }
//   display.clearDisplay();
//   display.setTextSize(1);
//   display.setTextColor(SSD1306_WHITE);
//   display.setCursor(0,0);
//   display.println("ESP32 Joystick Test (1..255)");
//   display.display();

//   // ADC configuration
//   analogReadResolution(12); // 0..4095
//   analogSetPinAttenuation(JOY_LEFT_X,  ADC_11db);
//   analogSetPinAttenuation(JOY_RIGHT_Y, ADC_11db);
// }

// void loop() {
//   // Read mapped values (normal direction)
//   int thr = mapAxisToParam(JOY_LEFT_X,  INVERT_THR);
//   int str = mapAxisToParam(JOY_RIGHT_Y, INVERT_STR);

//   // Serial debug
//   static uint32_t t0 = 0;
//   if (millis() - t0 > 150) {
//     t0 = millis();
//     Serial.printf("THR=%3d  STR=%3d\n", thr, str);
//   }

//   // OLED output
//   display.clearDisplay();
//   display.setCursor(0, 0);  display.print("THR: "); display.println(thr);
//   drawHBar(14, thr);
//   display.setCursor(0, 30); display.print("STR: "); display.println(str);
//   drawHBar(44, str);
//   display.setCursor(0, 56); display.print("Range 1-255  Mid=128");
//   display.display();

//   delay(30);
// }























// #include <Bounce2.h>

// #define BUTTON1 25
// #define BUTTON2 26

// Bounce debouncer1 = Bounce();
// Bounce debouncer2 = Bounce();

// void setup() {
//   Serial.begin(115200);

//   pinMode(BUTTON1, INPUT_PULLUP);
//   pinMode(BUTTON2, INPUT_PULLUP);

//   debouncer1.attach(BUTTON1);
//   debouncer1.interval(25); // 25ms 防抖
//   debouncer2.attach(BUTTON2);
//   debouncer2.interval(25);
// }

// void loop() {
//   debouncer1.update();
//   debouncer2.update();

//   if (debouncer1.fell()) {  // 按下瞬间
//     Serial.println("hello Alex");
//   }
//   if (debouncer2.fell()) {
//     Serial.println("hello Michael");
//   }
// }



















#include <Arduino.h>

const int BUZZ_PIN = 27;  // 你的蜂鸣器负极接在 GPIO27

void setup() {
  pinMode(BUZZ_PIN, OUTPUT);
  digitalWrite(BUZZ_PIN, LOW);  // 上电保持静音
}

void loop() {
  // 响 200ms
  digitalWrite(BUZZ_PIN, HIGH);
  delay(200);

  // 停 800ms
  digitalWrite(BUZZ_PIN, LOW);
  delay(800);
}

