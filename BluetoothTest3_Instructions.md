# BluetoothTest3 - 機能拡張版使用説明書

## 概要
XIAO nRF52840を使用したBluetoothLE距離測定システムの拡張版です。従来の2段階出力から6段階の細かい距離判定に改良し、ブザー機能を追加しました。

## 新機能
### 1. 多段階距離判定（6レベル）
- **極近距離** (-35dBm以上): LED 50ms間隔、ブザー 100ms間隔
- **近距離** (-45dBm以上): LED 100ms間隔、ブザー 200ms間隔  
- **中近距離** (-55dBm以上): LED 200ms間隔、ブザー 400ms間隔
- **中距離** (-65dBm以上): LED 400ms間隔、ブザー 800ms間隔
- **中遠距離** (-75dBm以上): LED 800ms間隔、ブザー 1600ms間隔
- **遠距離** (-85dBm未満): LED 1200ms間隔、ブザー無音

### 2. ブザー機能
- 距離に応じて鳴動間隔が変化
- 近くなるほど連続的に鳴動
- 遠距離では無音

### 3. RSSI平滑化フィルタ
- ノイズの影響を軽減
- 安定した距離判定を実現
- 平滑化係数は調整可能（RSSI_ALPHA = 0.2f）

## 配線図
```
XIAO nRF52840 配線:
├── LED_BUILTIN (内蔵LED) - 自動制御
├── D2 - ブザー（+極）
├── GND - ブザー（-極）
└── 3V3 - 電源供給用（必要に応じて）
```

## ファイル構成
- `BluetoothTest3.ino` - Central版（スキャナー）
- `BluetoothTest3_Peripheral.ino` - Peripheral版（アドバタイザー）

## 使用方法
1. **2台のXIAO nRF52840を準備**
2. **Central機（1台目）**:
   - `BluetoothTest3.ino`をアップロード
   - D2ピンにブザーを接続
3. **Peripheral機（2台目）**:
   - `BluetoothTest3_Peripheral.ino`をアップロード  
   - D2ピンにブザーを接続
4. **動作確認**:
   - 両方の電源をON
   - 自動接続後、距離に応じてLED・ブザーが動作

## Bluetooth性能最適化設定
```cpp
BLE.setConnectionInterval(16, 32);  // 20-40ms間隔
BLE.setTimeout(5000);               // 5秒タイムアウト
```

## カスタマイズ設定
### 距離レベルの調整
```cpp
constexpr DistanceLevel DISTANCE_LEVELS[] = {
  {閾値, LED間隔, ブザー間隔, "説明"}
};
```

### 平滑化係数の調整
```cpp
constexpr float RSSI_ALPHA = 0.2f;  // 0.1-0.5推奨
```

## トラブルシューティング
- **接続しない**: シリアルモニタでデバッグ情報を確認
- **距離が不安定**: RSSI_ALPHAを小さく調整（0.1程度）
- **ブザーが鳴らない**: 配線と電源供給を確認

## 技術仕様
- **通信方式**: Bluetooth Low Energy 5.0
- **測定範囲**: 約1m-10m（環境により変動）
- **更新間隔**: 約20-40ms
- **電力消費**: 約10-20mA（動作時） 