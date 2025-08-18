#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

#define CE_PIN 4
#define CSN_PIN 5
#define SCK_PIN 18
#define MISO_PIN 19
#define MOSI_PIN 23

// ====== Speed control parameters (adjust as needed) ======
static const uint16_t SEND_INTERVAL_MS = 50;   // Interval between frames: 50ms
static const uint16_t TIMEOUT_MS       = 150;  // 150ms without ACK → considered disconnected (≈ 3 lost frames)

RF24 radio(CE_PIN, CSN_PIN);

// Address pairing: send to "CTRL1", receiver listens on "CTRL1"; receiver replies with ACK (hardware automatic)
const byte txAddr[6] = "CTRL1"; // Transmit address (receiver must openReadingPipe(…, "CTRL1"))
const byte rxAddr[6] = "BASE1"; // Only for symmetry (ACK payload is not used in this setup)

// Control packet (rctestflight style: simple and practical)
struct ControlPacket {
  int16_t throttle;  // -512..+512
  int16_t steering;  // -512..+512
  uint8_t flags;     // Bit flags: e.g. light, mode, etc.
} __attribute__((packed));

unsigned long lastSend = 0;
unsigned long lastAck  = 0;
bool isConnected = false;

void setup() {
  Serial.begin(115200);
  SPI.begin(SCK_PIN, MISO_PIN, MOSI_PIN);

  if (!radio.begin()) {
    Serial.println("❌ NRF24 initialization failed (check wiring/power/capacitor)");
    while (1) {}
  }

  // —— Communication tuning: prioritize stability ——
  radio.setPALevel(RF24_PA_LOW);     // LOW: stable and power-saving; use HIGH for longer range (requires stable capacitor)
  radio.setDataRate(RF24_1MBPS);     // 1Mbps: good balance of interference resistance and latency
  radio.setChannel(90);              // Avoid Wi-Fi bands; adjust based on environment
  radio.setRetries(5, 15);           // Hardware auto-retry (ARQ)
  radio.setCRCLength(RF24_CRC_16);
  radio.enableDynamicPayloads();     // Variable payloads (not mandatory)
  radio.setAutoAck(true);            // **Important: enable hardware ACK**

  radio.openWritingPipe(txAddr);
  radio.openReadingPipe(1, rxAddr);  // ACK payload is not pulled in this setup, but symmetry config is kept
  radio.stopListening();             // Transmitter mode: always sending

  lastSend = millis();
  lastAck  = millis();

  Serial.println("🎮 Transmitter ready (50ms/frame, 150ms timeout)");
}

// Mock throttle input — replace with actual joystick/ADC
static inline int16_t readThrottleMock() {
  return 300;
}

// Mock steering input — replace with actual joystick/ADC
static inline int16_t readSteeringMock() {
  return -100;
}

// Mock flag input — define your own bit layout
// Example: bit 0 = light, bit 1 = buzzer, etc.
static inline uint8_t readFlagsMock() {
  return 0b00000001;
}

void loop() {
  const unsigned long now = millis();

  // Send a frame if it's time
  if (now - lastSend >= SEND_INTERVAL_MS) {
    lastSend = now;

    ControlPacket pkt;
    pkt.throttle = readThrottleMock();
    pkt.steering = readSteeringMock();
    pkt.flags    = readFlagsMock();

    // Send packet + check if hardware ACK was received
    bool acked = radio.write(&pkt, sizeof(pkt));
    if (acked) {
      // Any successful ACK means the receiver is online
      if (!isConnected) {
        Serial.println("✅ Receiver online (ACK received)");
        isConnected = true;
      }
      lastAck = now;
    } else {
      Serial.println("⚠️ No ACK for this frame (possible packet loss / long range)");
    }
  }

  // Low-latency disconnect detection (150ms ≈ 3 missed frames)
  if (isConnected && (millis() - lastAck > TIMEOUT_MS)) {
    isConnected = false;
    Serial.println("⛔ Connection lost (>150ms without ACK) — waiting for auto-reconnect…");
    // Keep sending frames every 50ms; will auto-recover on any ACK
  }
}

