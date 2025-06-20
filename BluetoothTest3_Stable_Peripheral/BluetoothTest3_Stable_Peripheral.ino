/*****************************************************************
 *  Auto-Link + RSSI Enhanced - BluetoothTest3 (接続修正版 - Peripheral)
 *  -------------------------------------------------------------
 *  ● 電源 ON 直後に自動リンク（1 対 1）
 *  ● シンプルで確実な接続方式
 *  ● 中央値フィルタによる強化されたRSSI平滑化
 *  ● 電波干渉に対する耐性向上（10サンプル中央値）
 *****************************************************************/
#include <ArduinoBLE.h>

/* ====== ここでロールを選択 ====== */
#define IS_CENTRAL   false      // Peripheral版
/* ============================== */

// ピン定義
constexpr char   SERVICE_UUID[] = "19B10000-E8F2-537E-4F6C-D104768A1214";
constexpr uint8_t LED_PIN      = LED_BUILTIN;
constexpr uint8_t BUZZER_PIN   = 2;           
constexpr uint8_t LED_ON_LVL   = LOW;         
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
uint32_t lastRSSISample = 0;                  // RSSIサンプリング用
uint32_t lastOutputUpdate = 0;                // 出力更新用
uint16_t blinkMs    = 1000;           
uint16_t buzzerMs   = 0;              
uint16_t buzzerFreq = 0;              
uint8_t  buzzerVolume = 0;            
bool     ledState   = false;
bool     buzzerState = false;

/* ----- RSSI処理（中央値フィルタ版） ----- */
int8_t   lastRSSI   = -100;
int8_t   filteredRSSI = -100;         
int8_t   rssiSamples[10] = {-100, -100, -100, -100, -100, -100, -100, -100, -100, -100};  // 10サンプル
uint8_t  sampleIndex = 0;
uint8_t  validSampleCount = 0;
constexpr uint32_t RSSI_SAMPLE_INTERVAL = 50;    // RSSIサンプリング間隔(ms)
constexpr uint32_t OUTPUT_UPDATE_INTERVAL = 500;  // 出力更新間隔(ms) 

/* ----- 距離段階定義 ----- */
struct DistanceLevel {
  int8_t rssiThreshold;
  uint16_t ledInterval;
  uint16_t buzzerInterval;
  uint16_t buzzerFrequency;           
  uint8_t buzzerVolume;               
  const char* description;
};

constexpr DistanceLevel DISTANCE_LEVELS[] = {
  {-35, 50,   100,  2000, 255, "極近距離"},    
  {-45, 100,  200,  1500, 200, "近距離"},      
  {-55, 200,  400,  1200, 150, "中近距離"},    
  {-65, 400,  800,  1000, 100, "中距離"},      
  {-75, 800,  1600, 800,  50,  "中遠距離"},    
  {-85, 1200, 0,    0,    0,   "遠距離"},      
};
constexpr uint8_t NUM_LEVELS = sizeof(DISTANCE_LEVELS) / sizeof(DISTANCE_LEVELS[0]);

/* ----- ユーティリティ関数 ----- */
inline void ledOn()    { digitalWrite(LED_PIN, LED_ON_LVL);    }
inline void ledOff()   { digitalWrite(LED_PIN, LED_OFF_LVL);   }

inline void buzzerOn(uint16_t frequency, uint8_t volume = 255) {
  if (frequency > 0) {
    tone(BUZZER_PIN, frequency);
  }
}

inline void buzzerOff() { 
  noTone(BUZZER_PIN);                 
  digitalWrite(BUZZER_PIN, LOW);      
}

inline void ledToggle() { 
  ledState = !ledState; 
  digitalWrite(LED_PIN, ledState ? LED_ON_LVL : LED_OFF_LVL); 
}

inline void buzzerToggle() {
  buzzerState = !buzzerState;
  if (buzzerState && buzzerFreq > 0) {
    buzzerOn(buzzerFreq, buzzerVolume); 
  } else {
    buzzerOff();                        
  }
}

// 中央値計算用ソート関数（バブルソート）
void sortRSSISamples(int8_t* arr, uint8_t size) {
  for (uint8_t i = 0; i < size - 1; i++) {
    for (uint8_t j = 0; j < size - i - 1; j++) {
      if (arr[j] > arr[j + 1]) {
        int8_t temp = arr[j];
        arr[j] = arr[j + 1];
        arr[j + 1] = temp;
      }
    }
  }
}

// 中央値フィルタによるRSSI処理
int8_t processRSSIWithMedianFilter() {
  if (validSampleCount == 0) return -100;
  
  // 有効なサンプルを一時配列にコピー
  int8_t tempSamples[10];
  uint8_t validCount = 0;
  
  for (uint8_t i = 0; i < 10; i++) {
    if (rssiSamples[i] > -100) {
      tempSamples[validCount++] = rssiSamples[i];
    }
  }
  
  if (validCount == 0) return -100;
  
  // ソートして中央値を取得
  sortRSSISamples(tempSamples, validCount);
  
  if (validCount % 2 == 1) {
    // 奇数個の場合：中央の値
    return tempSamples[validCount / 2];
  } else {
    // 偶数個の場合：中央2つの平均
    uint8_t mid1 = validCount / 2 - 1;
    uint8_t mid2 = validCount / 2;
    return (tempSamples[mid1] + tempSamples[mid2]) / 2;
  }
}

// RSSIサンプル追加
void addRSSISample(int8_t newRSSI) {
  // 異常値フィルタリング
  if (newRSSI == 127 || newRSSI < -100 || newRSSI > -20) {
    return;  // 異常値は無視
  }
  
  // サンプル追加
  rssiSamples[sampleIndex] = newRSSI;
  sampleIndex = (sampleIndex + 1) % 10;
  
  if (validSampleCount < 10) {
    validSampleCount++;
  }
}

// 距離レベルの判定
uint8_t getDistanceLevel(int8_t rssi) {
  for (uint8_t i = 0; i < NUM_LEVELS; i++) {
    if (rssi >= DISTANCE_LEVELS[i].rssiThreshold) {
      return i;
    }
  }
  return NUM_LEVELS - 1;  
}

// 出力パラメータの更新
void updateOutputParameters(int8_t rssi) {
  uint8_t level = getDistanceLevel(rssi);
  blinkMs = DISTANCE_LEVELS[level].ledInterval;
  buzzerMs = DISTANCE_LEVELS[level].buzzerInterval;
  buzzerFreq = DISTANCE_LEVELS[level].buzzerFrequency;    
  buzzerVolume = DISTANCE_LEVELS[level].buzzerVolume;     
  
  // デバッグ出力（頻度制限）
  static uint32_t lastDebugTime = 0;
  if (millis() - lastDebugTime > 2000) {  // 2秒間隔
    Serial.print("RSSI: ");
    Serial.print(rssi);
    Serial.print(" dBm (Median), Level: ");
    Serial.print(DISTANCE_LEVELS[level].description);
    Serial.print(", Samples: ");
    Serial.print(validSampleCount);
    Serial.println("/10 [PERIPHERAL]");
    lastDebugTime = millis();
  }
}

void startScan() {
  BLE.scanForUuid(SERVICE_UUID);
}

void setup() {
  pinMode(LED_PIN, OUTPUT);
  pinMode(BUZZER_PIN, OUTPUT);
  ledOff();
  buzzerOff();

  Serial.begin(115200);
  // Serial待機を削除してすぐに開始

  if (!BLE.begin()) {
    Serial.println("BLE init failed"); 
    while (1);
  }

  if (IS_CENTRAL) {
    Serial.println("CENTRAL: Median Filter Enhanced Version");
    startScan();
  } else {
    Serial.println("PERIPHERAL: Median Filter Enhanced Version");
    ledService.addCharacteristic(dummyChar);
    BLE.setAdvertisedService(ledService);
    BLE.addService(ledService);
    BLE.setLocalName("RSSI_TAG");  // シンプルな名前に戻す
    BLE.advertise();
  }
  
  Serial.println("BluetoothTest3 - Connection Fixed Version Started (Peripheral)");
  
  // 起動時テスト
  Serial.println("System test...");
  buzzerOn(1000, 255);
  delay(100);
  buzzerOff();
}

void loop() {
  bool connected = false;
  int8_t currentRSSI = -100;
  uint32_t now = millis();

  /* --------- Central 動作（簡素化版） --------- */
  if (IS_CENTRAL) {
    // Central版の処理は上記と同じ
  }

  /* --------- Peripheral 動作（簡素化版） --------- */
  else {
    if (BLE.connected()) {
      connected = true;
      
      // RSSIサンプリング（50ms間隔）
      if (now - lastRSSISample >= RSSI_SAMPLE_INTERVAL) {
        currentRSSI = BLE.rssi();
        addRSSISample(currentRSSI);
        lastRSSISample = now;
      }
    }
  }

  /* --------- RSSI処理と出力制御（500ms間隔で更新） --------- */
  if (connected && (now - lastOutputUpdate >= OUTPUT_UPDATE_INTERVAL)) {
    filteredRSSI = processRSSIWithMedianFilter();
    if (filteredRSSI > -100) {
      updateOutputParameters(filteredRSSI);
    }
    lastOutputUpdate = now;
  } else if (!connected) {
    blinkMs = 1000;                          
    buzzerMs = 0;                            
    buzzerFreq = 0;                          
    buzzerVolume = 0;                        
  }

  /* --------- LED制御 --------- */
  if (now - lastToggle >= blinkMs) {
    ledToggle();
    lastToggle = now;
  }

  /* --------- ブザー制御 --------- */
  if (buzzerMs > 0 && buzzerFreq > 0) {      
    if (now - lastBuzzer >= buzzerMs) {
      buzzerToggle();
      lastBuzzer = now;
    }
  } else {
    buzzerOff();                             
    buzzerState = false;
  }

  BLE.poll();
} 