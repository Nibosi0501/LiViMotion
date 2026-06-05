# LiViMotion

**LiViMotion**（**Li**DAR + **Vi**sion + **Motion**）は、2D LiDAR と YOLO 姿勢推定を統合し、個人ごとの身体動作に基づいた床面映像インタラクションを実現するシステムです。

床面に投影された映像に対して、歩行者の足元位置や身体動作（手の挙上など）をリアルタイムに検出し、足跡エフェクトなどのインタラクティブな映像演出を行います。

## 目次

- [システム概要](#システム概要)
- [主な機能](#主な機能)
- [システム構成](#システム構成)
- [ディレクトリ構成](#ディレクトリ構成)
- [依存ライブラリ](#依存ライブラリ)
- [ビルド方法](#ビルド方法)
- [使い方](#使い方)
  - [1. キャリブレーション](#1-キャリブレーション)
  - [2. LiDAR による足位置検出・送信](#2-lidar-による足位置検出送信)
  - [3. インタラクションシステムの実行](#3-インタラクションシステムの実行)
- [通信プロトコル](#通信プロトコル)
- [ライセンス](#ライセンス)
- [コントリビュート](#コントリビュート)

## システム概要

LiViMotion は以下の 3 つのプロセスが UDP 通信で連携して動作します。

```
┌──────────────┐       UDP (port 12345)       ┌──────────────────────────┐
│  2D LiDAR    │ ──────────────────────────▶  │                          │
│  足位置検出   │                               │   LiViMotion 本体        │
└──────────────┘                               │   (Experiment/600x600)   │
                                               │                          │
┌──────────────┐       UDP (port 10001)        │  ・マルチオブジェクト追跡  │
│  YOLO 前方   │ ──────────────────────────▶  │  ・LiDAR-YOLO マッチング  │
│  カメラ      │                               │  ・足跡インタラクション    │
└──────────────┘                               │  ・OpenGL 床面映像描画    │
                                               │                          │
┌──────────────┐       UDP (port 10002)        │                          │
│  YOLO 後方   │ ──────────────────────────▶  │                          │
│  カメラ      │                               └──────────────────────────┘
└──────────────┘
```

## 主な機能

- **マルチセンサ統合**: 2D LiDAR の高精度な足位置検出と YOLO の姿勢推定（17 キーポイント）を組み合わせ、個人を正確に識別・追跡
- **リアルタイム追跡**: カルマンフィルタによる位置予測とハンガリアンアルゴリズムによる最適なデータ対応付け
- **身体動作認識**: YOLO キーポイントを用いた手の挙上などのジェスチャー検出
- **足跡インタラクション**: 歩行者の足元にリアルタイムでエフェクト（円・星・雪の結晶など）を投影
- **射影変換キャリブレーション**: カメラ映像と床面座標を正確に対応付けるグリッドベースのキャリブレーション

## システム構成

| コンポーネント | 技術 | 用途 |
|:---|:---|:---|
| 描画エンジン | OpenGL / GLUT | 床面映像のリアルタイムレンダリング |
| 画像処理 | OpenCV | キャリブレーション、画像変換、輪郭検出 |
| 音響 | OpenAL / ALUT | 空間音響の再生 |
| 姿勢推定 | YOLO (外部) | 17 キーポイントの人体姿勢推定 |
| 距離センサ | 2D LiDAR (外部) | 足位置のスキャン・検出 |
| 通信 | UDP ソケット | プロセス間データ転送 |
| 追跡 | カルマンフィルタ / ハンガリアンアルゴリズム | マルチオブジェクトトラッキング |

## ディレクトリ構成

```
LiViMotion/
├── Calibration/                    # カメラキャリブレーション
│   ├── Calibration.hpp             #   キャリブレーションクラス定義
│   ├── calibration.cpp             #   射影変換・座標変換の実装
│   └── create_calibration_600x600.cpp  #   キャリブレーションポイント生成ツール
│
├── LiDAR/                          # LiDAR 足位置検出
│   └── main_600x600.cpp            #   6m×6m 範囲のスキャン・検出処理
│
├── Udp/                            # UDP 通信モジュール
│   ├── SendUdp.hpp / send_udp.cpp  #   LiDAR データ送信
│   └── ReceiveUdp.hpp / receive_udp.cpp  #   LiDAR・YOLO データ受信
│
├── Experiment/600x600/             # メインのインタラクションシステム
│   ├── main.cpp                    #   エントリポイント・描画ループ
│   ├── Yolo/                       #   YOLO キーポイント処理・ジェスチャー認識
│   ├── Lidar/                      #   LiDAR データ構造
│   ├── Kalman/                     #   カルマンフィルタによる位置予測
│   └── Tracker/                    #   マルチオブジェクト追跡
│       ├── Hungarian/              #     ハンガリアンアルゴリズム（最適割当）
│       ├── Matching/               #     LiDAR-YOLO トラッカーのマッチング
│       └── FloorWindow/            #     足跡インタラクション・エフェクト管理
│
├── Movie/                          # テクスチャ・メディアリソース
│   ├── circle3.png
│   ├── star.png
│   └── snow.png
│
├── CONTRIBUTING.md                 # コントリビューションガイド
├── CODE_OF_CONDUCT.md              # 行動規範
└── LICENSE                         # GNU GPL-3.0
```

## 依存ライブラリ

| ライブラリ | 用途 |
|:---|:---|
| [OpenCV](https://opencv.org/) 4.x | 画像処理・カルマンフィルタ・射影変換 |
| [OpenGL](https://www.opengl.org/) / GLUT | リアルタイム描画 |
| [OpenAL](https://www.openal.org/) / ALUT | 音響再生 |

- **C++ 標準**: C++20
- **対応 OS**: macOS Apple Silicon（`-framework OpenGL -framework GLUT` を使用）

## ビルド方法

### キャリブレーションツール

```bash
g++ -O3 \
  -framework OpenGL -framework GLUT \
  -Wno-deprecated \
  `pkg-config --cflags --libs opencv4` \
  -mmacosx-version-min=13.0 -std=c++17 \
  Calibration/create_calibration_600x600.cpp
```

### インタラクションシステム本体

```bash
cd Experiment/600x600

g++ -O3 \
  -framework OpenGL -framework GLUT \
  -lalut -framework OpenAL \
  -Wno-deprecated \
  `pkg-config --cflags --libs opencv4` \
  -mmacosx-version-min=13.0 -std=c++20 \
  ../../Udp/receive_udp.cpp \
  ../../Calibration/calibration.cpp \
  ./Yolo/yolo.cpp \
  ./Lidar/lidar.cpp \
  ./Kalman/kalman.cpp \
  ./Tracker/tracker.cpp \
  ./Tracker/Hungarian/hungarian.cpp \
  ./Tracker/Matching/matching.cpp \
  ./Tracker/FloorWindow/floorWindow.cpp \
  ./Tracker/FloorWindow/movieController.cpp \
  main.cpp
```

## 使い方

### 1. キャリブレーション

プロジェクタとカメラの座標系を対応付けるキャリブレーションを行います。

1. `create_calibration_600x600` を実行し、OpenGL ウィンドウにグリッドを表示
2. カメラの画角を調整後、スペースキーでキャプチャ
3. カメラ映像上のグリッド交点（7×7 = 49 点）を左クリックで選択
   - 自動検出されない点は右クリックで手動指定可能
4. 任意のキーを押してキャリブレーションデータを保存
   - `front_calibration600x600.txt`（前方カメラ用）
   - `back_calibration600x600.txt`（後方カメラ用）

### 2. LiDAR による足位置検出・送信

`LiDAR/main_600x600.cpp` を参考に、使用する LiDAR の SDK に応じた足位置検出処理を実装してください。

> **Note**: LiDAR の SDK やドライバはライセンスの関係上、本リポジトリには含まれていません。

検出した足位置座標は `SendUdp` クラスを使って UDP で送信します。

```cpp
#include "../Udp/SendUdp.hpp"

snd::SendLidar sendLidar(12345, "127.0.0.1");

std::vector<std::pair<float, float>> points;
// ... 足位置の座標を points に追加 ...

sendLidar.send(points);
```

### 3. インタラクションシステムの実行

LiDAR プロセスおよび YOLO 推論プロセスが起動している状態で、`Experiment/600x600` のメインプログラムを実行します。

```bash
./a.out
```

| キー | 操作 |
|:---|:---|
| `F` | フルスクリーン表示 |
| `Q` / `Esc` | 終了 |

## 通信プロトコル

### LiDAR パケット形式

```
[人数: int32 (4 bytes)] [x: float, y: float] [x: float, y: float] ...
```

| フィールド | 型 | サイズ | 説明 |
|:---|:---|:---|:---|
| 人数 | `int32_t` | 4 bytes | 検出された人数（0 の場合は検出なし） |
| 座標 | `float` × 2 | 8 bytes × N | 各人の足位置 (x, y) |

### YOLO パケット形式

```
[人数: int32 (4 bytes)] [17 keypoints × (x, y, conf): float × 3] × N
```

| フィールド | 型 | サイズ | 説明 |
|:---|:---|:---|:---|
| 人数 | `int32_t` | 4 bytes | 検出された人数 |
| キーポイント | `float` × 3 × 17 | 204 bytes × N | 各人の 17 キーポイント (x, y, 信頼度) |

## ライセンス

このプロジェクトは [GNU General Public License v3.0](LICENSE) の下で公開されています。