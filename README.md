# XIAO nRF52840 Bluetooth距離測定システム

## 📋 プロジェクト概要

XIAO nRF52840を使用したBluetooth Low Energy（BLE）による距離測定・音響フィードバックシステムです。RSSI値を基に2デバイス間の距離を推定し、LEDとブザーによる段階的な出力制御を行います。

## 🎯 主要機能

### ✨ 距離測定
- **Bluetooth Low Energy 5.0**による無線距離測定
- **RSSI値**（-35dBm〜-85dBm）から距離推定
- **6段階**の細かい距離レベル判定
- **平滑化フィルタ**によるノイズ軽減

### 🔊 音響フィードバック
- **PWM制御**による段階的音量調整（0-255レベル）
- **距離連動**：近距離で大音量・高頻度、遠距離で小音量・低頻度
- **トランジスタアンプ回路**対応（音量2-3倍向上）
- **最大10倍**音量向上（全手法組み合わせ時）

### 💡 視覚フィードバック
- **内蔵LED**による距離表示
- **段階的点滅間隔**：50ms（極近）〜1200ms（遠距離）
- **接続状態**の視覚確認

## 📊 バージョン比較

| バージョン | 距離段階 | 音量制御 | フィルタ | 主な用途 |
|------------|----------|----------|----------|----------|
| **BluetoothTest1** | 2段階 | ON/OFF | なし | 基本動作確認 |
| **BluetoothTest2** | 2段階 | ON/OFF | なし | Peripheral版 |
| **BluetoothTest3** | 6段階 | PWM制御 | RSSI平滑化 | 実用版 |

## 🏗️ システム構成

### 基本構成
```
Central Device (スキャナー)  ←──BLE通信──→  Peripheral Device (アドバタイザー)
├── XIAO nRF52840                          ├── XIAO nRF52840
├── 内蔵LED                                ├── 内蔵LED  
├── ブザー（D2ピン）                       ├── ブザー（D2ピン）
└── 電源（USB/バッテリー）                 └── 電源（USB/バッテリー）
```

### 音量向上構成（推奨）
```
XIAO nRF52840
├── D2 ──→ 1kΩ抵抗 ──→ トランジスタBase
├── 5V ──→ ブザー(+) ──→ トランジスタCollector  
└── GND ──→ トランジスタEmitter & ブザー(-)
```

## 📏 距離レベル詳細

| レベル | RSSI閾値 | 推定距離 | LED間隔 | ブザー間隔 | 音量 | 説明 |
|--------|----------|----------|---------|------------|------|------|
| 1 | -35dBm以上 | 〜1m | 50ms | 100ms | 最大(255) | 極近距離 |
| 2 | -45dBm以上 | 1-2m | 100ms | 200ms | 大(200) | 近距離 |
| 3 | -55dBm以上 | 2-4m | 200ms | 400ms | 中(150) | 中近距離 |
| 4 | -65dBm以上 | 4-6m | 400ms | 800ms | 小(100) | 中距離 |
| 5 | -75dBm以上 | 6-8m | 800ms | 1600ms | 微(50) | 中遠距離 |
| 6 | -85dBm未満 | 8m〜 | 1200ms | 無音 | 0 | 遠距離 |

## 🚀 クイックスタート

### 1. 必要な機材
- **XIAO nRF52840** × 2台
- **ブザー** × 2個（3-5V対応）
- **ジャンパーワイヤー**
- **Arduino IDE**（ArduinoBLEライブラリ）

### 2. 基本セットアップ
```bash
# 1. Arduino IDEでArduinoBLEライブラリをインストール
# 2. XIAO nRF52840ボードパッケージを追加
# 3. 以下のコードをそれぞれのデバイスにアップロード
```

**Central Device**: `BluetoothTest3/BluetoothTest3.ino`
```cpp
#define IS_CENTRAL   true    // Central版
```

**Peripheral Device**: `BluetoothTest3_Peripheral/BluetoothTest3_Peripheral.ino`
```cpp
#define IS_CENTRAL   false   // Peripheral版
```

### 3. 配線
```
両デバイス共通:
XIAO D2 ──→ ブザー（+極）
XIAO GND ──→ ブザー（-極）
```

### 4. 動作確認
1. 両デバイスの電源ON
2. 自動接続開始（シリアルモニタで確認）
3. 距離を変えてLED・ブザーの動作確認

## 🔧 音量向上オプション

### 方法1: PWM制御（実装済み）
- **効果**: 1.5-2倍音量向上
- **コスト**: 無料
- **実装**: `analogWrite()`によるPWM制御

### 方法2: トランジスタアンプ回路
- **効果**: 2-3倍音量向上  
- **部品**: NPNトランジスタ（2SC1815）+ 1kΩ抵抗
- **詳細**: `BuzzerAmplifier_Circuit.md`参照

### 方法3: 高音量ブザー
- **効果**: ブザー仕様による
- **推奨**: 80dB以上の高音量ブザー
- **詳細**: `BuzzerUpgrade_Options.md`参照

## ⚙️ カスタマイズ

### 距離レベル調整
```cpp
constexpr DistanceLevel DISTANCE_LEVELS[] = {
  {RSSI閾値, LED間隔, ブザー間隔, 音量, "説明"},
  // 値を変更して感度調整
};
```

### 平滑化調整
```cpp
constexpr float RSSI_ALPHA = 0.2f;  // 0.1-0.5で調整
// 小さい値 = より滑らか、大きい値 = より応答性
```

### 音量制限
```cpp
constexpr uint8_t MAX_VOLUME = 200;  // 最大音量制限
```

## 📁 ファイル構成

```
BluetoothTest/
├── README.md                          # このファイル
├── BluetoothTest1/                    # 基本版（Central）
│   └── BluetoothTest1.ino
├── BluetoothTest2/                    # 基本版（Peripheral）
│   └── BluetoothTest2.ino  
├── BluetoothTest3/                    # 拡張版（Central）
│   └── BluetoothTest3.ino
├── BluetoothTest3_Peripheral/         # 拡張版（Peripheral）
│   └── BluetoothTest3_Peripheral.ino
├── BluetoothTest3_Instructions.md     # 詳細使用説明書
├── BuzzerAmplifier_Circuit.md         # アンプ回路図
├── BuzzerUpgrade_Options.md           # 音量向上オプション
└── image.png                          # XIAO nRF52840ピン配置図
```

## 🔍 トラブルシューティング

### 接続できない
- シリアルモニタでデバッグ情報確認
- 両デバイスの電源リセット
- Bluetooth設定の確認

### 距離が不安定
- `RSSI_ALPHA`を小さく調整（0.1程度）
- 環境ノイズ（Wi-Fi等）から離す
- 金属物体から離す

### ブザーが鳴らない
- 配線確認（極性注意）
- 電源供給確認
- ブザー仕様確認（電圧・タイプ）

### 音量が小さい
- PWM音量値確認（デバッグ出力）
- トランジスタアンプ回路追加
- 高音量ブザーに交換

## 📊 技術仕様

| 項目 | 仕様 |
|------|------|
| **通信方式** | Bluetooth Low Energy 5.0 |
| **測定範囲** | 約1-10m（環境依存） |
| **距離分解能** | 6段階 |
| **更新間隔** | 20-40ms |
| **電力消費** | 10-20mA（3.3V）/ 15-30mA（5V音量向上） |
| **音量制御** | 0-255段階PWM |
| **対応電圧** | 3.3V（基本）/ 5V（音量向上） |

## 🛠️ 開発環境

- **マイコン**: XIAO nRF52840 (nRF52840チップ)
- **開発環境**: Arduino IDE
- **ライブラリ**: ArduinoBLE
- **通信**: Bluetooth Low Energy 5.0
- **言語**: C++（Arduino）

## 📈 性能向上のポイント

### Bluetooth最適化
```cpp
BLE.setConnectionInterval(16, 32);  // 接続間隔最適化
BLE.setTimeout(5000);               // タイムアウト設定
```

### RSSI安定化
- 平滑化フィルタによるノイズ除去
- 環境要因の考慮（障害物、電波干渉）
- 複数回測定による信頼性向上

### 音響性能
- PWM制御による段階的音量調整
- ハードウェアアンプによる音量増幅
- 高性能ブザーとの組み合わせ

## 🎯 用途例

- **物品探知システム**：紛失物の発見
- **距離測定ツール**：概算距離の把握
- **プロキシミティアラート**：接近・離脱の検知
- **学習・実験用途**：BLE通信の理解
- **プロトタイプ開発**：IoTシステムの基礎

## 📝 ライセンス

このプロジェクトはオープンソースです。個人・商用利用ともに自由にご利用ください。

## 🤝 貢献

改良や機能追加のご提案・プルリクエストを歓迎します。

---

**最終更新**: 2024年12月
**バージョン**: BluetoothTest3 (拡張版)
**対応デバイス**: XIAO nRF52840
