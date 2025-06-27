/*****************************************************************
 *  Auto-Link + RSSI Enhanced - BluetoothTest3 (接続修正版)
 *  -------------------------------------------------------------
 *  ● 電源 ON 直後に自動リンク（1 対 1）
 *  ● シンプルで確実な接続方式
 *  ● 中央値フィルタによる強化されたRSSI平滑化
 *  ● 電波干渉に対する耐性向上（10サンプル中央値）
 *****************************************************************/
#include <ArduinoBLE.h>

/* ====== ここでロールを選択 ====== */
#define IS_CENTRAL   true      // false にすると Peripheral
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
double   outputIntensity = 0.0;               // 連続的な出力強度 (0.0-100.0)            
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

/* ----- 連続的出力強度計算用定数 ----- */
constexpr int8_t  RSSI_MIN = -85;                 // 最弱信号 (遠距離)
constexpr int8_t  RSSI_MAX = -35;                 // 最強信号 (極近距離)
constexpr double  INTENSITY_MIN = 1.0;            // 最弱出力強度
constexpr double  INTENSITY_MAX = 100.0;          // 最強出力強度



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

/* ===== 連続的出力強度計算関数 ===== */
/**
 * RSSI値を0.0～100.0の連続的な出力強度に変換
 * @param rssi RSSI値 (dBm)
 * @return 出力強度 (0.0: 無効値, 1.0-100.0: 有効範囲)
 */
double getOutputIntensity(int8_t rssi) {
  // 無効なRSSI値の場合
  if (rssi <= -100 || rssi > -20) {
    return 0.0;
  }
  
  // RSSI範囲外をクランプ
  if (rssi < RSSI_MIN) rssi = RSSI_MIN;
  if (rssi > RSSI_MAX) rssi = RSSI_MAX;
  
  // 線形マッピング: RSSI_MIN(-85) → 1.0, RSSI_MAX(-35) → 100.0
  double normalizedRSSI = (double)(rssi - RSSI_MIN) / (double)(RSSI_MAX - RSSI_MIN);
  double intensity = INTENSITY_MIN + normalizedRSSI * (INTENSITY_MAX - INTENSITY_MIN);
  
  return intensity;
}

/* ===== 連続的LED制御関数 ===== */
/**
 * 出力強度に基づいてLEDの点滅間隔を計算
 * @param intensity 出力強度 (1.0-100.0)
 * @return 点滅間隔 (ms)
 */
uint16_t calculateLEDInterval(double intensity) {
  if (intensity <= 0.0) return 1000;  // 未接続時の標準点滅
  
  // 強度1.0 → 1200ms, 強度100.0 → 50ms の指数関数的変化
  // interval = 1200 * exp(-0.035 * (intensity - 1))
  double exponent = -0.035 * (intensity - 1.0);
  double interval = 1200.0 * exp(exponent);
  
  // 最小・最大値制限
  if (interval < 50.0) interval = 50.0;
  if (interval > 1200.0) interval = 1200.0;
  
  return (uint16_t)interval;
}

/* ===== 連続的ブザー制御関数 ===== */
/**
 * 出力強度に基づいてブザーの周波数を計算
 * @param intensity 出力強度 (1.0-100.0)
 * @return ブザー周波数 (Hz), 0なら無音
 */
uint16_t calculateBuzzerFrequency(double intensity) {
  if (intensity <= 1.0) return 0;  // 遠距離では無音
  
  // 強度1.0 → 800Hz, 強度100.0 → 2000Hz の線形変化
  double frequency = 800.0 + (intensity - 1.0) * (2000.0 - 800.0) / (100.0 - 1.0);
  
  return (uint16_t)frequency;
}

/**
 * 出力強度に基づいてブザーの点滅間隔を計算
 * @param intensity 出力強度 (1.0-100.0)
 * @return 点滅間隔 (ms), 0なら連続音
 */
uint16_t calculateBuzzerInterval(double intensity) {
  if (intensity <= 1.0) return 0;  // 遠距離では無音
  
  // 強度1.0 → 1600ms, 強度100.0 → 100ms の指数関数的変化
  double exponent = -0.025 * (intensity - 1.0);
  double interval = 1600.0 * exp(exponent);
  
  // 最小・最大値制限
  if (interval < 100.0) interval = 100.0;
  if (interval > 1600.0) interval = 1600.0;
  
  return (uint16_t)interval;
}

/**
 * 出力強度に基づいてブザーの音量を計算
 * @param intensity 出力強度 (1.0-100.0)
 * @return 音量 (0-255)
 */
uint8_t calculateBuzzerVolume(double intensity) {
  if (intensity <= 1.0) return 0;  // 遠距離では無音
  
  // 強度1.0 → 50, 強度100.0 → 255 の線形変化
  double volume = 50.0 + (intensity - 1.0) * (255.0 - 50.0) / (100.0 - 1.0);
  
  // 範囲制限
  if (volume < 0.0) volume = 0.0;
  if (volume > 255.0) volume = 255.0;
  
  return (uint8_t)volume;
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

// 出力パラメータの更新（連続的制御版）
void updateOutputParameters(int8_t rssi) {
  // 連続的な出力強度を計算
  outputIntensity = getOutputIntensity(rssi);
  
  if (outputIntensity > 0.0) {
    // 連続的制御パラメータを計算
    blinkMs = calculateLEDInterval(outputIntensity);
    buzzerMs = calculateBuzzerInterval(outputIntensity);
    buzzerFreq = calculateBuzzerFrequency(outputIntensity);    
    buzzerVolume = calculateBuzzerVolume(outputIntensity);
  } else {
    // 無効値の場合は無効状態
    blinkMs = 1000;
    buzzerMs = 0;
    buzzerFreq = 0;
    buzzerVolume = 0;
  }
  
  // デバッグ出力（頻度制限）
  static uint32_t lastDebugTime = 0;
  if (millis() - lastDebugTime > 2000) {  // 2秒間隔
    Serial.print("RSSI: ");
    Serial.print(rssi);
    Serial.print(" dBm (EMA+StdDev50), Intensity: ");
    Serial.print(outputIntensity, 1);
    Serial.print(", LED: ");
    Serial.print(blinkMs);
    Serial.print("ms, Buzz: ");
    Serial.print(buzzerFreq);
    Serial.print("Hz/");
    Serial.print(buzzerMs);
    Serial.print("ms/Vol");
    Serial.print(buzzerVolume);
    Serial.print(", Samples: ");
    Serial.print(validSampleCount);
    Serial.print("/40, EMA: ");
    Serial.println(emaRSSI, 1);
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
    Serial.println("CENTRAL: EMA + Min Variance Filter Enhanced Version");
    startScan();
  } else {
    Serial.println("PERIPHERAL: EMA + Min Variance Filter Enhanced Version");
    ledService.addCharacteristic(dummyChar);
    BLE.setAdvertisedService(ledService);
    BLE.addService(ledService);
    BLE.setLocalName("RSSI_TAG");  // シンプルな名前に戻す
    BLE.advertise();
  }
  
  Serial.println("BluetoothTest3 - Connection Fixed Version Started");
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
    if (!peer) {                            
      BLEDevice dev = BLE.available();
      if (dev && dev.connect()) {
        peer = dev;
        BLE.stopScan();
        Serial.println("Connected!");
        
        // サンプルバッファをリセット
        for (uint8_t i = 0; i < 40; i++) {
          rssiSamples[i] = -100;
        }
        sampleIndex = 0;
        validSampleCount = 0;
        emaRSSI = -100.0f;  // EMAもリセット
      }
    }
    
    if (peer && peer.connected()) {
      connected = true;
      
      // RSSIサンプリング（50ms間隔）
      if (now - lastRSSISample >= RSSI_SAMPLE_INTERVAL) {
        currentRSSI = peer.rssi();
        addRSSISample(currentRSSI);
        lastRSSISample = now;
      }
    } else if (peer) {                      // 切断
      Serial.println("Disconnected");
      peer = BLEDevice();
      filteredRSSI = -100;  
      emaRSSI = -100.0f;  // EMAもリセット
      // サンプルバッファリセット
      for (uint8_t i = 0; i < 40; i++) {
        rssiSamples[i] = -100;
      }
      sampleIndex = 0;
      validSampleCount = 0;
      startScan();
    }
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
    outputIntensity = 0.0;                   // 未接続時は強度0
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