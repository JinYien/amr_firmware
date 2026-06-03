# AMR-firmware

自律移動ロボット用の Teensy 4.0 ファームウェア
- 左右2つのDCモータを制御
- CyberGearモータをCAN経由で制御
- 3軸力センサを MCP3208（ADC）で読み取り

## 要件

デバイス: Teensy 4.0  
フレームワーク: Arduino

## ビルド / 書き込み

1. レポジトリをクローンする
    ```
    git clone --recursive https://github.com/JinYien/amr_firmware.git
    ```

2. PlatformIO を用いて Teensy に書き込む
    ```
    ＃ 本番ビルド
    pio run -e release -t upload

    ＃ デバッグビルド
    pio run -e debug -t upload
    ```
    各ペリフェラルの単体テスト用ビルド
    - `adc`
    - `cybergear`
    - `encoder`
    - `pwm`
    - `serial`