# ラズパイのプログラム

[![README in English](https://img.shields.io/badge/English-d9d9d9)](./README.md)
[![日本語版 README](https://img.shields.io/badge/日本語-d9d9d9)](./README_JA.md)

![GitHub contributors](https://img.shields.io/github/contributors/SpaceData-Inc/temp_rep)
![GitHub issues](https://img.shields.io/github/issues/SpaceData-Inc/temp_rep)
![GitHub fork](https://img.shields.io/github/forks/SpaceData-Inc/temp_rep)
![GitHub stars](https://img.shields.io/github/stars/SpaceData-Inc/temp_rep)




## 目次
1. [**概要**](#概要)
2. [**セットアップ**](#セットアップ)
    1. [環境](#環境)
    2. [pipのインストール](#pipのインストール)
    3. [ROS2のセットアップ](#ROS2のセットアップ)
    4. [音声関係のセットアップ](#音声関係のセットアップ)
    5. [I2Cのセットアップ](#I2Cのセットアップ)
    6. [IC制御用ライブラリのインストール](#IC制御用ライブラリのインストール)

3. [**クイックユーザガイド**](#クイックユーザガイド)
    1. [各機能のテスト](#各機能のテスト)
    2. [デモ](#デモ)

4. [**各モジュールの説明**](#各モジュールの説明)
    1. [air_sensor.py](#air_sensor.py)
    2. [ROS 2 (Humble)のセットアップ](#ROS 2 (Humble)のセットアップ)
    3. [display.py](#display.py)
    4. [microphone.py](#microphone.py)
    5. [speaker.py](#speaker.py)
    6. [keyboard_commander.py](#keyboard_commander.py)
5. [**トピック一覧**](#トピック一覧)
6. [**サービス一覧**](#サービス一覧)



## 概要

ロボットに搭載されるラズパイ上で動くROS2コードのリポジトリ。

ラズパイはJetsonの指揮下で、ディスプレイやセンサ等の各種デバイスを制御する。



## セットアップ

### 環境

ハードウェア：ラズパイ4, 8GB RAM, 64GB ストレージ

OS: Ubunutu Desktop 22.04

ROS2: Humble



### pipのインストール

``` sh
sudo apt install pip
```



### ROS2のセットアップ

参考: [公式ドキュメント](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html)

ロケールの設定

``` sh
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8
```



aptリポジトリにROS 2を追加

``` sh
sudo apt install software-properties-common
sudo add-apt-repository universe
sudo apt update
sudo apt upgrade
```



GPGキーの追加とリポジトリの追加。

``` sh
sudo apt install curl
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
```



ROS 2のインストール。

``` sh
sudo apt install ros-humble-desktop ros-dev-tools
```



ROS 2のセットアップ+ターミナル実行時に毎回実行するよう設定。

``` sh
source /opt/ros/humble/setup.bash
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
```



### 音声関係のセットアップ

スピーカー関係のパッケージをインストール
``` sh
sudo apt install portaudio19-dev
pip install sounddevice
pip install SpeechRecognition
pip install vosk
```



alsa-base.confを編集して、サウンドデバイスの優先度を変更

``` sh 
sudo nano /etc/modprobe.d/alsa-base.conf
```



末尾に下記を追記し、再起動

``` sh 
options snd_usb_audio index=0
options snd_bcm2835 index=1
```



優先度の確認

``` sh 
cat /proc/asound/modules
 0 snd_usb_audio  # usbの優先度が一番上ならOK
 1 snd_bcm2835
 2 vc4
 3 vc4
```



音声合成用のパッケージをインストール

``` sh
sudo apt install espeak      # 英語
sudo apt install open-jtalk  # 日本語
sudo apt install open-jtalk-mecab-naist-jdic # open_jtalkが使う辞書
```



名古屋工業大が開発したメイちゃんの音声モデルをダウンロードして、/usr/local/share/hts-voiceに配置

``` sh
cd Download
wget https://sourceforge.net/projects/mmdagent/files/MMDAgent_Example/MMDAgent_Example-1.8/MMDAgent_Example-1.8.zip
sudo mkdir /usr/local/share/hts-voice
sudo cp MMDAgent_Example-1.8/Voice/mei/* /usr/local/share/hts-voice/
```



音声合成モデルのダウンロードと配置

``` sh
mkdir ~/material && cd ~/material
wget https://alphacephei.com/vosk/models/vosk-model-small-ja-0.22.zip
wget https://alphacephei.com/vosk/models/vosk-model-small-en-us-0.15.zip
unzip vosk-model-small-ja-0.22.zip 
unzip vosk-model-small-en-us-0.15.zip
```



### I2Cのセットアップ

raspi-configを起動し、 3.Interface Options. -> 5.I2C -> Yes を選択し、I2Cを有効化。

``` sh
sudo apt install raspi-config
sudo raspi-config
```



ユーザをdialoutグループ(シリアル通信等へのアクセス権限をもつグループ)に追加して、再ログイン。

``` sh
sudo usermod -aG dialout $USER
sudo reboot
```



i2cdetectで接続してるi2cデバイスのアドレスを確認できる。※root権限を要求されたら、ユーザのグループ追加に問題がある。

``` sh
i2cdetect -y 1
```



### IC制御用ライブラリのインストール

``` sh
pip install smbus2  #　バッテリ監視IC
pip install scd30_i2c  # co2・温湿度センサ
```



## クイックユーザガイド

### 各機能のテスト

1. テストするノードを起動(コマンド：air_sensor/battery_monitor/display/microphone/speaker)

``` sh
ros2 run device_control display
```



2. 別ターミナルでキーボード入力ノードを起動し、コマンドを入力

**コマンド：**

* `display [smile/neutral/air_quality]` (ディスプレイの変更)
* `speak ja こんにちは` (発話)
* `trs ja` (録音＆文字起こし)

``` sh
ros2 run device_control keyboard_commander
input command: display smile
```



### デモ

話しかけてディスプレイの表示を変更するデモ。

1. それぞれ別のターミナルでair_sensor, microphone, speaker, displayを起動

``` sh
ros2 run device_control air_sensor
ros2 run device_control microphone
ros2 run device_control speaker
ros2 run device_control display  # ディスプレイを専有するので最後に起動すると良い
```



2. 「ねえねえイケボ」と呼びかけたあと命令「笑って/笑わないで/空気品質」



## 各モジュールの説明

### air_sensor.py
* **概説**
  空気品質(CO2濃度, 温度, 湿度)を計測し、値を配信する。

* **ノード** 
  
  * 名前: air_sensor
  * 役割: publisher (topic: [air_quality](#トピック一覧))
  * 定義クラス: AirSensor
  
* **動作**

  * `AirSensor.UPDATE_INTERVAL` (デフォルト:5)秒に1回、センサから値を取得し、[air_quality](#トピック一覧)にC02 (ppm), 温度 (°C), 湿度 (%)の順番でリストとして配信。
  
  * センサ値取得に失敗すると0.2秒後にもう一度取得を試みる。この試行は一回の配信タイミング毎に`AirSensor.TRIAL`(デフォルト:10)回繰り返す。
  
    
  

### battery_monitor.py

* **機能**
  バッテリの電源と電圧を計測し、battery_VIにpublishする。
* **ノード**
  - 名前: battery_monitor
  - 役割: publisher (topic:  [battery_VI](#トピック一覧))



### display.py

* **概説**
  ディスプレイに表示するGUIを制御するサービスを提供する。
  
* **ノード**
  - 名前: display_controller
  - 役割: server (service: [control_display](#サービス一覧)), subscriber(topic: [air_quality](#トピック一覧))
  - 定義クラス: DisplayController
  
* **動作**
  * [control_display](#トピック一覧)にディスプレイ制御用コマンドがリクエストされると、コマンドに応じてディスプレイを変更し、終了ステータス(0: 正常, 0以外: 異常)をかえす。
  
  * ディスプレイ制御用コマンドは`DisplayController.SUPPORTED_COMMAND`により定義
    * smile: 笑顔のGIFを再生
    
    * neutral: 真顔 (デフォルト)のGIFを再生
    
    * air_quality: 空気品質を[air_quality](#トピック一覧)から取得し、テキストでディスプレイに表示
    
      


### microphone.py

* **概説**
  話しかけ機能と音声認識(文字起こし)サービスを提供する。
  
* **ノード**
  - 名前: transcriber
  - 役割: server (service: [transcribe_speech](#サービス一覧)), client (service:  [speaker](#サービス一覧),  [control_display](#サービス一覧))
  - 定義クラス: Transcriber
  
* **動作**
  * ノードが起動されると録音が開始され、ウェイクワード待ち状態となる。
  
  * ウェイクワードが検出されると一定時間録音を行い、録音した音声に対して文字起こしをする。その後検出した文字に対応したアクションをリクエストする(下記表を参照)。
  
    | 認識した文字 | 対応するアクション                                           |
    | :----------- | :----------------------------------------------------------- |
    | 笑って       | ディスプレイを笑顔に変更                                     |
    | 笑わないで   | ディスプレイを真顔に変更                                     |
    | 空気品質     | 「空気品質を表示します」と発話し、ディスプレイに空気品質を表示 |
    
    (現在のバージョンでは本ノードから他のノードへ直接リクエストを投げているが、将来的には認識した文字を全てjetsonのコミュニケーションを管理するノードに投げ、そこから各機能を担当するノードにリクエストを投げるべきである。)
    
  * [transcribe_speech](#サービス一覧)に言語["en", "ja"]がリクエストされると、一定時間録音を行い、その後文字起こしをしてレスポンスとして文字列で返す。
  
    (現在のバージョンではリクエストによらず日本語として文字起こしを行う)
  
  * 文字起こし用の録音中は、ウェイクワード検出用の録音は停止される。
  
  * ウェイクワードは`Transcriber.WAKE_WORD`(デフォルト: "ねえねえイケボ")によって定義される。
  
  * 録音時間は`Transcriber.REC_DURATION`(単位: 秒, デフォルト: 4)によって定義される。
  
    


### speaker.py

* **概説**
  発話サービスを提供する。
  
* **ノード**
  - 名前: speaker
  - 役割: server (service: [speak_speech](#サービス一覧))
  - 定義クラス: Speaker
  
* **動作**
  *  [speak_speech](#サービス一覧)に言語と発話内容がリクエストされると音声合成を行って発話する。



### keyboard_commander.py

* **概説**
  キーボードから各機能を操作する。デバッグ用

* **ノード**

  - 名前: keyboard_commander
  - 役割: client (service: [speak_speech](#サービス一覧), [transcribe_speech](#サービス一覧), [control_display](#サービス一覧))
  - 定義クラス: KeyboardCommander

* **動作**

  * "speak <言語> <発話内容>" と入力すると、 [speak_speech](#サービス一覧)にリクエストを投げて発話させる。

  * "trs <言語>"と入力すると、 [transcribe_speech](#サービス一覧)にリクエストを投げて録音、文字起こしをさせる。

  * "display <コマンド>"と入力すると、 [control_display](#サービス一覧)にリクエストを投げてディスプレイの表示を変更させる。(コマンドの種類: smile, neutral, air_quality)

    





## トピック一覧



| トピック名  | publisher | メッセージ型 |  説明  |
| :---------- | :---------- | :---------- | :---------- |
| air_quality | [air_sensor](#air_sensor.py) | std_msgs/Float32MultiArray |  [\<C02 (ppm)>, \<温度 (°C)>, \<湿度 (%)>]  |
| battery_VI | [battery_monitor](#battery_monitor.py) | std_msgs/Float32MultiArray | [<電圧 (V)>, <電流 (A)>] |



## サービス一覧



| サービス名        | server                            | サービス型                | 説明                                                         |
| :---------------- | :-------------------------------- | :------------------------ | :----------------------------------------------------------- |
| control_display   | [display_controller](#display.py) | rpi_interfaces/Display    | string command #制御コマンド("smile"等）<br />---<br />int32 status #終了ステータス |
| transcribe_speech | [transcriber](#microphone.py)     | rpi_interfaces/Transcribe | string lang #言語<br/>---<br/>string speech #認識した内容<br/>int32 status #終了ステータス |
| speak_speech      | [speaker](#speaker.py)            | rpi_interfaces/Speak      | string speech #発話内容<br/>string lang #言語<br/>---<br/>int32 status #終了ステータス |


---

[トップに戻る](#ラズパイのプログラム)
