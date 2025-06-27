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

/* ----- RSSI処理（EMA + 標準偏差50近似フィルタ版） ----- */
int8_t   lastRSSI   = -100;
int8_t   filteredRSSI = -100;         
float    emaRSSI = -100.0f;           // EMA用の浮動小数点値
int8_t   rssiSamples[40] = {-100, -100, -100, -100, -100, -100, -100, -100, -100, -100, -100, -100, -100, -100, -100, -100, -100, -100, -100, -100, -100, -100, -100, -100, -100, -100, -100, -100, -100, -100, -100, -100, -100, -100, -100, -100, -100, -100, -100};  // 40サンプル
uint8_t  sampleIndex = 0;
uint8_t  validSampleCount = 0;
constexpr uint32_t RSSI_SAMPLE_INTERVAL = 25;    // RSSIサンプリング間隔(ms) - 25msに変更
constexpr uint32_t OUTPUT_UPDATE_INTERVAL = 800;  // 出力更新間隔(ms)
constexpr float EMA_ALPHA = 0.3f;                 // EMA平滑化係数

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

// 標準偏差50に最も近い個別値10個を選択
float calculateStdDev50BasedAverage(int8_t* sortedSamples, uint8_t count) {
  if (count < 10) {
    // サンプル数が10未満の場合は通常の平均
    int16_t sum = 0;
    for (uint8_t i = 0; i < count; i++) {
      sum += sortedSamples[i];
    }
    return (float)sum / count;
  }
  
  float targetStdDev = 50.0f;
  
  // 各値について「その値を含む仮想的な標準偏差50グループ」との近さを評価
  float valueScores[40];
  for (uint8_t i = 0; i < count; i++) {
    // この値が標準偏差50のグループにどれだけ適しているかをスコア化
    // 方法1: 値そのものの範囲から推定（簡易版）
    // RSSI値-50～-90の範囲で、標準偏差50に適した分布を想定
    float value = (float)sortedSamples[i];
    
    // 理想的な標準偏差50の分布における適合度を計算
    // 平均を-70dBmと仮定し、標準偏差50での正規分布適合度を評価
    float expectedMean = -70.0f;
    float deviation = abs(value - expectedMean);
    
    // 標準偏差50の正規分布において、この偏差がどれだけ「適切」かを評価
    // 標準偏差50なら、約68%が±50以内、95%が±100以内
    float normalizedDeviation = deviation / targetStdDev;
    
    // スコア計算：偏差が50に近いほど良いスコア（小さい値ほど良い）
    if (normalizedDeviation <= 1.0f) {
      // ±1σ以内なら良いスコア
      valueScores[i] = normalizedDeviation;
    } else if (normalizedDeviation <= 2.0f) {
      // ±2σ以内なら中程度のスコア
      valueScores[i] = 1.0f + (normalizedDeviation - 1.0f) * 2.0f;
    } else {
      // ±2σ超なら悪いスコア
      valueScores[i] = 3.0f + (normalizedDeviation - 2.0f) * 5.0f;
    }
  }
  
  // より精密な方法：実際に各値を中心とした近傍での標準偏差を計算
  for (uint8_t i = 0; i < count; i++) {
    float centerValue = (float)sortedSamples[i];
    
    // この値を中心とした近傍値群の標準偏差を計算
    // 近傍の定義：この値から±一定範囲内の値
    float nearbySum = 0;
    uint8_t nearbyCount = 0;
    
    for (uint8_t j = 0; j < count; j++) {
      float otherValue = (float)sortedSamples[j];
      if (abs(otherValue - centerValue) <= targetStdDev) {
        nearbySum += otherValue;
        nearbyCount++;
      }
    }
    
    if (nearbyCount >= 3) {
      float nearbyMean = nearbySum / nearbyCount;
      
      // 近傍群の標準偏差を計算
      float variance = 0;
      for (uint8_t j = 0; j < count; j++) {
        float otherValue = (float)sortedSamples[j];
        if (abs(otherValue - centerValue) <= targetStdDev) {
          float diff = otherValue - nearbyMean;
          variance += diff * diff;
        }
      }
      float actualStdDev = sqrt(variance / nearbyCount);
      
      // 目標標準偏差との差をスコアとして使用
      float stdDevScore = abs(actualStdDev - targetStdDev);
      
      // より良いスコアなら更新
      if (stdDevScore < valueScores[i]) {
        valueScores[i] = stdDevScore;
      }
    }
  }
  
  // 最良スコアの10個を選択（挿入ソートで上位10個を特定）
  uint8_t bestIndices[10];
  float bestScores[10];
  
  // 初期化
  for (uint8_t i = 0; i < 10; i++) {
    bestIndices[i] = 0;
    bestScores[i] = 999999.0f;
  }
  
  // 全値について評価し、上位10個を保持
  for (uint8_t i = 0; i < count; i++) {
    float currentScore = valueScores[i];
    
    // 現在のスコアが上位10個に入るかチェック
    for (uint8_t j = 0; j < 10; j++) {
      if (currentScore < bestScores[j]) {
        // 挿入位置が見つかった場合、後ろにシフト
        for (uint8_t k = 9; k > j; k--) {
          bestScores[k] = bestScores[k-1];
          bestIndices[k] = bestIndices[k-1];
        }
        bestScores[j] = currentScore;
        bestIndices[j] = i;
        break;
      }
    }
  }
  
  // 選択された10個の平均を計算
  float sum = 0;
  for (uint8_t i = 0; i < 10; i++) {
    sum += sortedSamples[bestIndices[i]];
  }
  
  return sum / 10.0f;
}

// EMA + 標準偏差50近似フィルタによるRSSI処理
int8_t processRSSIWithEMAAndStdDev50() {
  if (validSampleCount == 0) return -100;
  
  // 有効なサンプルを一時配列にコピー
  int8_t tempSamples[40];
  uint8_t validCount = 0;
  
  for (uint8_t i = 0; i < 40; i++) {
    if (rssiSamples[i] > -100) {
      tempSamples[validCount++] = rssiSamples[i];
    }
  }
  
  if (validCount == 0) return -100;
  
  // ソート
  sortRSSISamples(tempSamples, validCount);
  
  // 標準偏差50に近い10個の平均を計算
  float rawAverage = calculateStdDev50BasedAverage(tempSamples, validCount);
  
  // EMA適用
  if (emaRSSI == -100.0f) {
    // 初回はそのまま設定
    emaRSSI = rawAverage;
  } else {
    // EMA計算: new_value = α * current + (1-α) * previous
    emaRSSI = EMA_ALPHA * rawAverage + (1.0f - EMA_ALPHA) * emaRSSI;
  }
  
  return (int8_t)round(emaRSSI);
}

// RSSIサンプル追加
void addRSSISample(int8_t newRSSI) {
  // 異常値フィルタリング
  if (newRSSI == 127 || newRSSI < -100 || newRSSI > -20) {
    return;  // 異常値は無視
  }
  
  // サンプル追加
  rssiSamples[sampleIndex] = newRSSI;
  sampleIndex = (sampleIndex + 1) % 40;
  
  if (validSampleCount < 40) {
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
    Serial.print(" dBm (EMA+StdDev50), Level: ");
    Serial.print(DISTANCE_LEVELS[level].description);
    Serial.print(", Samples: ");
    Serial.print(validSampleCount);
    Serial.print("/40, EMA: ");
    Serial.print(emaRSSI, 1);
    Serial.println(" [PERIPHERAL]");
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
    Serial.println("CENTRAL: EMA + StdDev50 Filter Enhanced Version");
    startScan();
  } else {
    Serial.println("PERIPHERAL: EMA + StdDev50 Filter Enhanced Version");
    ledService.addCharacteristic(dummyChar);
    BLE.setAdvertisedService(ledService);
    BLE.addService(ledService);
    BLE.setLocalName("RSSI_TAG");  // シンプルな名前に戻す
    BLE.advertise();
  }
  
  Serial.println("BluetoothTest3 - Connection Fixed Version Started (Peripheral)");
  Serial.println("RSSI: 25ms interval, 40 samples (1s), EMA(α=0.3) + StdDev≈50 10-value average");
  
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
    filteredRSSI = processRSSIWithEMAAndStdDev50();
    if (filteredRSSI > -100) {
      updateOutputParameters(filteredRSSI);
    }
    lastOutputUpdate = now;
  } else if (!connected) {
    blinkMs = 1000;                          
    buzzerMs = 0;                            
    buzzerFreq = 0;                          
    buzzerVolume = 0;                        
    emaRSSI = -100.0f;  // 接続断時にEMAもリセット
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