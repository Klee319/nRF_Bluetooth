/*****************************************************************
 *  Auto-Link + RSSI Enhanced - BluetoothTest3 (パッシブブザー最適化版)
 *  -------------------------------------------------------------
 *  ● 電源 ON 直後に自動リンク（1 対 1）
 *  ● 接続が確立すると両方の D13 LED を点滅
 *  ● 距離に応じた多段階出力（滑らかな変化）
 *  ● パッシブブザー最適化：tone()関数による周波数制御
 *  ● RSSI値の平滑化フィルタ搭載
 *  ● LED は XIAO の仕様に合わせ LOW＝点灯
 *****************************************************************/
#include <ArduinoBLE.h>

/* ====== ここでロールを選択 ====== */
#define IS_CENTRAL   true      // false にすると Peripheral
/* ============================== */

// ピン定義
constexpr char   SERVICE_UUID[] = "19B10000-E8F2-537E-4F6C-D104768A1214";
constexpr uint8_t LED_PIN      = LED_BUILTIN;
constexpr uint8_t BUZZER_PIN   = 2;           // D2ピンにパッシブブザーを接続
constexpr uint8_t LED_ON_LVL   = LOW;         // XIAO BLE: LOW = 点灯
constexpr uint8_t LED_OFF_LVL  = HIGH;

/* ----- Peripheral 用オブジェクト ----- */
BLEService ledService(SERVICE_UUID);
BLEByteCharacteristic dummyChar("19B10001-E8F2-537E-4F6C-D104768A1214",
                                BLERead | BLEWriteWithoutResponse);

/* ----- Central 用 ----- */
BLEDevice peer;

/* ----- 出力制御用変数 ----- */
uint32_t lastToggle = 0;
uint32_t lastBuzzer = 0;
uint16_t blinkMs    = 1000;           // LEDブリンク間隔
uint16_t buzzerMs   = 0;              // ブザー鳴動間隔（0=無音）
uint16_t buzzerFreq = 0;              // ブザー周波数（Hz）
uint8_t  buzzerVolume = 0;            // ブザー音量（0-255、PWM値）
bool     ledState   = false;
bool     buzzerState = false;

/* ----- RSSI処理 ----- */
int8_t   lastRSSI   = -100;
int8_t   smoothedRSSI = -100;         // 平滑化されたRSSI値
constexpr float RSSI_ALPHA = 0.2f;    // 平滑化係数（0-1, 小さいほど滑らか）

/* ----- 距離段階定義 ----- */
struct DistanceLevel {
  int8_t rssiThreshold;
  uint16_t ledInterval;
  uint16_t buzzerInterval;
  uint16_t buzzerFrequency;           // 周波数（Hz）
  uint8_t buzzerVolume;               // PWM音量値（0-255）
  const char* description;
};

// 距離レベル定義（近い順）- パッシブブザー最適化
constexpr DistanceLevel DISTANCE_LEVELS[] = {
  {-35, 50,   100,  2000, 255, "極近距離"},    // 極めて近い（高音・最大音量）
  {-45, 100,  200,  1500, 200, "近距離"},      // 非常に近い（中高音・大音量）
  {-55, 200,  400,  1200, 150, "中近距離"},    // 近い（中音・中音量）
  {-65, 400,  800,  1000, 100, "中距離"},      // やや近い（中低音・小音量）
  {-75, 800,  1600, 800,  50,  "中遠距離"},    // やや遠い（低音・微音量）
  {-85, 1200, 0,    0,    0,   "遠距離"},      // 遠い（無音）
};
constexpr uint8_t NUM_LEVELS = sizeof(DISTANCE_LEVELS) / sizeof(DISTANCE_LEVELS[0]);

/* ----- ユーティリティ関数 ----- */
inline void ledOn()    { digitalWrite(LED_PIN, LED_ON_LVL);    }
inline void ledOff()   { digitalWrite(LED_PIN, LED_OFF_LVL);   }

// パッシブブザー専用制御関数
inline void buzzerOn(uint16_t frequency, uint8_t volume = 255) {
  if (frequency > 0) {
    // tone()で基本周波数を設定
    tone(BUZZER_PIN, frequency);
    
    // 必要に応じてPWMで音量調整（一部環境では効果あり）
    // analogWrite(BUZZER_PIN, volume);
  }
}

inline void buzzerOff() { 
  noTone(BUZZER_PIN);                 // tone()停止
  digitalWrite(BUZZER_PIN, LOW);      // 確実に停止
}

inline void ledToggle() { 
  ledState = !ledState; 
  digitalWrite(LED_PIN, ledState ? LED_ON_LVL : LED_OFF_LVL); 
}

inline void buzzerToggle() {
  buzzerState = !buzzerState;
  if (buzzerState && buzzerFreq > 0) {
    buzzerOn(buzzerFreq, buzzerVolume); // 周波数指定でON
  } else {
    buzzerOff();                        // OFF
  }
}

// RSSI値の平滑化フィルタ
int8_t smoothRSSI(int8_t newRSSI) {
  if (smoothedRSSI == -100) {  // 初回
    smoothedRSSI = newRSSI;
  } else {
    smoothedRSSI = (int8_t)(RSSI_ALPHA * newRSSI + (1.0f - RSSI_ALPHA) * smoothedRSSI);
  }
  return smoothedRSSI;
}

// 距離レベルの判定
uint8_t getDistanceLevel(int8_t rssi) {
  for (uint8_t i = 0; i < NUM_LEVELS; i++) {
    if (rssi >= DISTANCE_LEVELS[i].rssiThreshold) {
      return i;
    }
  }
  return NUM_LEVELS - 1;  // 最も遠い距離
}

// 出力パラメータの更新
void updateOutputParameters(int8_t rssi) {
  uint8_t level = getDistanceLevel(rssi);
  blinkMs = DISTANCE_LEVELS[level].ledInterval;
  buzzerMs = DISTANCE_LEVELS[level].buzzerInterval;
  buzzerFreq = DISTANCE_LEVELS[level].buzzerFrequency;    // 周波数も更新
  buzzerVolume = DISTANCE_LEVELS[level].buzzerVolume;     // 音量も更新
  
  // デバッグ出力
  Serial.print("RSSI: ");
  Serial.print(rssi);
  Serial.print(" dBm, Level: ");
  Serial.print(DISTANCE_LEVELS[level].description);
  Serial.print(", LED: ");
  Serial.print(blinkMs);
  Serial.print(" ms, Buzzer: ");
  Serial.print(buzzerMs);
  Serial.print(" ms, Freq: ");
  Serial.print(buzzerFreq);
  Serial.print(" Hz, Volume: ");
  Serial.print(buzzerVolume);
  Serial.println("/255");
}

void startScan() {
  BLE.scanForUuid(SERVICE_UUID);
}

void setup() {
  // ピン初期化
  pinMode(LED_PIN, OUTPUT);
  pinMode(BUZZER_PIN, OUTPUT);
  ledOff();
  buzzerOff();

  Serial.begin(115200);
  while (!Serial);          // デバッグ用（本番では削除可）

  if (!BLE.begin()) {
    Serial.println("BLE init failed"); 
    while (1);
  }

  // Bluetooth設定の最適化
  BLE.setConnectionInterval(16, 32);      // 接続間隔の最適化（20-40ms）
  BLE.setTimeout(5000);                   // タイムアウト設定

  if (IS_CENTRAL) {
    Serial.println("CENTRAL: Enhanced scanning...");
    startScan();
  } else {
    Serial.println("PERIPHERAL: Enhanced advertising...");
    ledService.addCharacteristic(dummyChar);
    BLE.setAdvertisedService(ledService);
    BLE.addService(ledService);
    BLE.setLocalName("RSSI_TAG_V3_PASSIVE");  // パッシブ版識別
    BLE.advertise();
  }
  
  Serial.println("BluetoothTest3 - Passive Buzzer Optimized Version Started");
  
  // 起動時テスト音（パッシブブザーの動作確認）
  Serial.println("Buzzer test: 1000Hz for 200ms");
  buzzerOn(1000, 255);
  delay(200);
  buzzerOff();
}

void loop() {
  bool connected = false;
  int8_t currentRSSI = -100;

  /* --------- Central 動作 --------- */
  if (IS_CENTRAL) {
    if (!peer) {                            
      BLEDevice dev = BLE.available();
      if (dev && dev.connect()) {
        peer = dev;
        BLE.stopScan();
        Serial.println("Connected to peripheral!");
      }
    }
    if (peer && peer.connected()) {
      connected = true;
      currentRSSI = peer.rssi();
    } else if (peer) {                      
      Serial.println("Disconnected - restarting scan");
      peer = BLEDevice();
      smoothedRSSI = -100;  // リセット
      startScan();
    }
  }

  /* --------- Peripheral 動作 --------- */
  else {
    if (BLE.connected()) {
      connected = true;
      currentRSSI = BLE.rssi();
    }
  }

  /* --------- RSSI処理と出力制御 --------- */
  if (connected && currentRSSI != 127) {     // 127 は取得失敗値
    lastRSSI = smoothRSSI(currentRSSI);      // 平滑化
    updateOutputParameters(lastRSSI);        // 出力パラメータ更新
  } else {
    blinkMs = 1000;                          // 未接続時のデフォルト
    buzzerMs = 0;                            // ブザー停止
    buzzerFreq = 0;                          // 周波数0
    buzzerVolume = 0;                        // 音量0
  }

  /* --------- LED制御 --------- */
  uint32_t now = millis();
  if (now - lastToggle >= blinkMs) {
    ledToggle();
    lastToggle = now;
  }

  /* --------- ブザー制御（パッシブ対応） --------- */
  if (buzzerMs > 0 && buzzerFreq > 0) {      // ブザー・周波数有効時のみ
    if (now - lastBuzzer >= buzzerMs) {
      buzzerToggle();
      lastBuzzer = now;
    }
  } else {
    buzzerOff();                             // ブザー無効時は確実に停止
    buzzerState = false;
  }

  BLE.poll();
} 