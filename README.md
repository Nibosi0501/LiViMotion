# LiViMotion

<img width="3272" height="751" alt="header" src="https://github.com/user-attachments/assets/9a9b758b-53cc-4f2d-8028-9904e1481898" />

**LiViMotion**（**Li**DAR + **Vi**sion + **Motion**）は、2D LiDAR と Ultralytics YOLO26 を統合し、個人ごとの身体動作に基づいた床面映像インタラクションを実現するシステムです。

床面に投影された映像に対して、歩行者の足元位置や身体動作（手の挙上など）をリアルタイムに検出し、インタラクティブな映像演出を行います。

## 目次

- [主な機能](#主な機能)
- [システム構成](#システム構成)
- [ディレクトリ構成](#ディレクトリ構成)
- [依存ライブラリ](#依存ライブラリ)
- [使い方](#使い方)
  - [1. キャリブレーション](#1-キャリブレーション)
  - [2. LiDAR による足位置検出・送信](#2-lidar-による足位置検出送信)
  - [3. YOLO 姿勢推定の実行](#3-yolo-姿勢推定の実行)
  - [4. インタラクションシステムの実行](#4-インタラクションシステムの実行)
- [通信プロトコル](#通信プロトコル)
- [ライセンス](#ライセンス)

## 主な機能
<img width="1247" height="691" alt="system" src="https://github.com/user-attachments/assets/5359c41c-15f5-4b77-a4d9-5ae933c46a33" />

- **マルチセンサ統合**: 2D LiDAR の高精度な足位置検出と YOLO26 の姿勢推定（17 キーポイント）を組み合わせ、個人を正確に識別・追跡
- **リアルタイム追跡**: カルマンフィルタによる位置予測とハンガリアンアルゴリズムによる最適なデータ対応付け
- **身体動作認識**: YOLO26 キーポイントを用いた手の挙上などのジェスチャー検出
- **身体動作に基づいた床面映像インタラクション**: 歩行者の足元にリアルタイムでインタラクティブな映像（星・雪の結晶など）を投影
- **射影変換キャリブレーション**: カメラ映像と床面座標を正確に対応付けるグリッドベースのキャリブレーション

## システム構成

| コンポーネント | 技術 | 用途 |
|:---|:---|:---|
| 描画エンジン | OpenGL / GLUT | 床面映像のリアルタイムレンダリング |
| 画像処理 | OpenCV | キャリブレーション、画像変換、輪郭検出 |
| 姿勢推定 | YOLO26 | 17 キーポイントの人体姿勢推定 |
| 距離センサ | 2D LiDAR (外部) | 足位置のスキャン・検出 |
| 通信 | UDP ソケット | プロセス間データ転送 |
| 追跡 | カルマンフィルタ / ハンガリアンアルゴリズム | マルチオブジェクトトラッキング |

## ディレクトリ構成

```
LiViMotion/
├── include/                        # C++ .hpp
│   ├── calibration/                # カメラキャリブレーション
│   ├── core/
│   │   └── Hungarian.hpp           # ハンガリアンアルゴリズム（最適割当）
│   ├── kalman/                     # カルマンフィルタ
│   ├── lidar/                      # LiDAR データ構造
│   ├── network/                    # UDP 通信モジュール
│   ├── render/                     # 床面映像描画関連
│   ├── tracker/
│   │   │ Matching.hpp              # LiDAR-YOLO トラッカーのマッチング
│   │   └── Tracker.hpp             # マルチオブジェクト追跡
│   └──  yolo/                      # YOLO キーポイント処理・ジェスチャー認識
│
├── src/                            # C++ .cpp
│
├── apps/                           # 各種エントリポイント
│   ├── CreateCalibration/
│   │   └── create_calibration.cpp  # 事前キャリブレーション
│   ├── FloorMovie/
│   │   └── main_FloorMovie.cpp     # 床面映像とのインタラクション（例：動画）
│   ├── FootStamp/
│   │   └── main_FootStamp.cpp      # 床面映像とのインタラクション（例：足跡）
│   ├── LiDAR/                      # LiDAR 足位置検出
│   ├── TestLiDARData/
│   │   └── Test_LidarData.cpp      # ダミーLiDAR Data 送信プログラム
│   └── Yolo/
│       └── models/                 # YOLO26 姿勢推定モデル保存場所
│       └── yolo_pose_sender.py     # YOLO26 推論・UDP キーポイント送信
│
├── Movie/                          # テクスチャ・メディアリソース
│
├── docs
│   └──  doxyfiles/                 # 各種 Dxygen 設定ファイル
│
└── LICENSE                         # GNU AGPL-3.0
```

## 依存ライブラリ

### C++

| ライブラリ | 用途 |
|:---|:---|
| [OpenCV](https://opencv.org/) 5.x | 画像処理・カルマンフィルタ・射影変換 |
| [OpenGL](https://www.opengl.org/) / GLUT | リアルタイム描画 |

- **C++ 標準**: C++20
- **対応 OS**: macOS Apple Silicon（`-framework OpenGL -framework GLUT` を使用）

### Python（Yolo_Python）

| ライブラリ | 用途 |
|:---|:---|
| [Ultralytics](https://docs.ultralytics.com/) | YOLO26 姿勢推定モデルの推論 |
| [OpenCV](https://opencv.org/) (cv2) | カメラキャプチャ・画像描画 |
| [coremltools](https://apple.github.io/coremltools/docs-guides/) | CoreML 対応 |

- **Python**: 3.9 以上推奨
- **モデル**: [YOLO26-pose](https://docs.ultralytics.com/tasks/pose) を `models/` ディレクトリに配置（Apple Silicon (M シリーズ) の場合は CoreML 形式（`.mlpackage`）に変換）

## 使い方

### 1. キャリブレーション

プロジェクタとカメラの座標系を対応付けるキャリブレーションを行います。

1. `create_calibration.cpp` を実行し、OpenGL ウィンドウにグリッドを表示
2. カメラの画角を調整後、スペースキーを押してキャリブレーションモードに移行
3. カメラ映像上のグリッド交点（7×7 = 49 点）を左クリックで選択
   - 自動検出されない点は右クリックで手動指定可能
4. 任意のキーを押してキャリブレーションデータを保存
   - `front_calibration.txt`（前方カメラ用）
   - `back_calibration.txt`（後方カメラ用）

### 2. LiDAR による足位置検出・送信

`LiDAR/README.md` を参考に、使用する LiDAR の SDK に応じた足位置検出処理を実装してください。

> **Note**: LiDAR の SDK やドライバはライセンスの関係上、本リポジトリには含まれていません。

検出した足位置座標は `SendUdp` クラスを使って UDP で送信します。

```cpp
// exsample
#include "./Udp/SendUdp.hpp"

snd::SendLidar sendLidar(12345, "127.0.0.1");

std::vector<std::pair<float, float>> points;
// ... 足位置の座標を points に追加 ...

sendLidar.send(points);
```

### 3. YOLO 姿勢推定の実行

`apps/Yolo/yolo_pose_sender.py` を実行し、カメラ映像からリアルタイムで姿勢推定を行い、検出したキーポイントを UDP で送信します。

#### モデルの準備

1. [Ultralytics 公式サイト](https://docs.ultralytics.com/tasks/pose)から yolo26x-pose モデルをダウンロード
2. Apple Silicon (M シリーズ) の場合は CoreML 形式（`.mlpackage`）に変換
3. `apps/Yolo/models/` ディレクトリに配置

#### 実行
`yolo_pose_sender.py` 内の定数を環境に合わせて変更してください。

| 定数 | デフォルト値 | 説明 |
|:---|:---|:---|
| `IP` | `127.0.0.1` | 送信先 IP アドレス |
| `PORT` | `10001` | 送信先ポート番号（前方カメラ: `10001`、後方カメラ: `10002`） |

> **Note**: 複数台のカメラを使用する場合は、`yolo_pose_sender.py` を複数起動し、それぞれ異なるカメラデバイスと送信ポートを設定してください。

### 4. インタラクションシステムの実行

LiDAR プロセスおよび YOLO 推論プロセスが起動している状態で、`apps/` のメインプログラム(FloorMovie.cpp or FloorStamp.cpp)を実行します。

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

このプロジェクトは [GNU Affero General Public License v3.0](LICENSE) の下で公開されています。
