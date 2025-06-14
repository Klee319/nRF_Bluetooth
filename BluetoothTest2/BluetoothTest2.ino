/*****************************************************************
 *  Auto-Link + RSSI ブリンク
 *  -------------------------------------------------------------
 *  ● 電源 ON 直後に自動リンク（1 対 1）
 *  ● 接続が確立すると両方の D13 LED を点滅
 *  ● -40 dBm（近）→ 100 ms、-90 dBm（遠）→ 1200 ms へ線形マップ
 *  ● LED は XIAO の仕様に合わせ LOW＝点灯
 *****************************************************************/
#include <ArduinoBLE.h>

/* ====== ここでロールを選択 ====== */
#define IS_CENTRAL   false      // false にすると Peripheral
/* ============================== */

constexpr char   SERVICE_UUID[] = "19B10000-E8F2-537E-4F6C-D104768A1214";
constexpr uint8_t LED_PIN      = LED_BUILTIN;
constexpr uint8_t LED_ON_LVL   = LOW;   // XIAO BLE: LOW = 点灯
constexpr uint8_t LED_OFF_LVL  = HIGH;

/* ----- Peripheral 用オブジェクト ----- */
BLEService ledService(SERVICE_UUID);
BLEByteCharacteristic dummyChar("19B10001-E8F2-537E-4F6C-D104768A1214",
                                BLERead | BLEWriteWithoutResponse);

/* ----- Central 用 ----- */
BLEDevice peer;

/* ----- ブリンクタイミング ----- */
uint32_t lastToggle = 0;
uint16_t blinkMs    = 1000;           // デフォルト 1 s
bool     ledState   = false;
int8_t   lastRSSI   = -100;

/* ----- ユーティリティ ----- */
inline void ledOn()  { digitalWrite(LED_PIN, LED_ON_LVL);  }
inline void ledOff() { digitalWrite(LED_PIN, LED_OFF_LVL); }
inline void ledToggle() { ledState = !ledState; digitalWrite(LED_PIN, ledState ? LED_ON_LVL : LED_OFF_LVL); }

void startScan() {
  BLE.scanForUuid(SERVICE_UUID);
}

void setup() {
  pinMode(LED_PIN, OUTPUT);
  ledOff();

  // Serial.begin(115200);
  // while (!Serial);          // デバッグ不要なら削除可

  if (!BLE.begin()) {
    Serial.println("BLE init failed"); while (1);
  }

  if (IS_CENTRAL) {
    Serial.println("CENTRAL: scanning…");
    startScan();
  } else {
    Serial.println("PERIPHERAL: advertising…");
    ledService.addCharacteristic(dummyChar);
    BLE.setAdvertisedService(ledService);   // UUID を広告へ
    BLE.addService(ledService);
    BLE.setLocalName("RSSI_TAG");           // 任意
    BLE.advertise();
  }
}

void loop() {
  bool connected = false;

  /* --------- Central 動作 --------- */
  if (IS_CENTRAL) {
    if (!peer) {                            // まだ相手が無ければスキャン
      BLEDevice dev = BLE.available();
      if (dev && dev.connect()) {
        peer = dev;
        BLE.stopScan();
        Serial.println("Connected!");
      }
    }
    if (peer && peer.connected()) {
      connected = true;
      lastRSSI  = peer.rssi();              // -40 … -90
    } else if (peer) {                      // 切断
      Serial.println("Disconnected");
      peer = BLEDevice();
      startScan();
    }
  }

  /* --------- Peripheral 動作 --------- */
  else {
    if (BLE.connected()) {
      connected = true;
      lastRSSI  = BLE.rssi();
    }
  }

  /* --------- RSSI → ブリンク周期 --------- */
  if (connected && lastRSSI != 127) {       // 127 は取得失敗値
    blinkMs = constrain(map(lastRSSI, -90, -40, 1200, 100), 100, 1200);
  } else {
    blinkMs = 1000;                         // 未接続: 1 s
  }

  /* --------- LED ブリンク --------- */
  uint32_t now = millis();
  if (now - lastToggle >= blinkMs) {
    ledToggle();
    lastToggle = now;
  }

  BLE.poll();
}
