# MobilityDynamicsSystem

## 📌 プロジェクト概要 (Overview)
**PC連動型 1軸姿勢制御（ジンバル）システム**

本プロジェクトは、STM32マイコンによるリアルタイム姿勢制御と、C#で構築したPC側の可視化・チューニングツールを一気通貫で連携させたフルスクラッチのシステムです。

組み込み領域において「ブラックボックス化しやすい物理現象と制御状態」を、自作のPCツールを用いて波形として可視化することで、理論的かつ効率的なシステム開発・デバッグ環境を構築しました。

*(※ここにシステムの動作風景、およびPCツールのリアルタイム波形描画のGIF動画や画像を配置)*

## 🎯 開発の背景と目的 (Motivation)
業務系ソフトウェア（C#等）の開発経験を活かし、モビリティ・重工分野の組み込みシステム開発へアプローチするためのポートフォリオとして開発しました。

「マイコン内で完結させる」のではなく、「PC側で計測・制御UIを自作し、システム全体を俯瞰して課題を解決する」というシステムズエンジニアリングのアプローチを実践しています。

## 🛠 技術スタック (Tech Stack)

### ハードウェア・組み込み側 (MCU & Hardware)
* **Microcontroller:** STM32 Nucleo (C言語)
* **RTOS:** FreeRTOS (タスク分割、Queueによるデータ伝達)
* **Sensor:** MPU6050 (I2C通信、相補フィルターによるノイズ除去・角度計算)
* **Actuator:** サーボモーター (TIM/PWM制御)
* **Communication:** UART (NVIC受信割り込みを用いたコマンド解析)

### PC・アプリケーション側 (PC Software)
* **Language/Framework:** C# / Windows Forms
* **Library:** ScottPlot (100Hzの多次元データ・リアルタイム描画)
* **Function:** 双方向シリアル通信による動的PIDゲイン（Kp, Ki, Kd）送信・チューニングGUI

### 開発環境・その他 (Tools)
* **Version Control:** Git / TortoiseGit

## ⚙️ システム構成 (System Architecture)

[ PC (C# WinForms / ScottPlot) ]
   ▲  │ (100Hz リアルタイム波形データ)
   │  ▼ (PIDゲイン 動的送信)
 ＝＝ UART (115200bps) ＝＝
   ▲  │
   │  ▼
[ STM32 Nucleo + FreeRTOS ]
   │  ├─ Task 1: センサー読み取り (I2C)
   │  ├─ Task 2: 姿勢計算・PID制御 (相補フィルター)
   │  ├─ Task 3: モーター駆動 (PWM)
   │  └─ Task 4: UART送受信
   │
 ＝＝ I2C ＝＝         ＝＝ PWM ＝＝
   ▼                  ▼
[ MPU6050 ]         [ サーボモーター ]
