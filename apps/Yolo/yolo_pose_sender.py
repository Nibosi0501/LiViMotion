"""!
@file yolo_pose_sender.py
@brief カメラ映像から YOLO で人の姿勢を推定し、キーポイントを UDP で送信するプログラム

カメラの映像を 1 フレームずつ読み取り、YOLO (姿勢推定モデル) で人物のキーポイントを検出して、
信頼度の高い人物の分だけを UDP で送信する。送信先は、C++ 側の YOLO 受信
(rcv::RecvYolo) が待ち受けているポートである。

@par 使い方
カメラを接続し、モデルファイルを用意したうえで、プログラムを実行する。
@code
python yolo_pose_sender.py
@endcode
映像のウィンドウ ("Pose Estimation") が前面にあるときに q キーを押すと、プログラムを終了する。

@par 必要なもの
- Python のライブラリ: opencv-python (cv2)、ultralytics
- Homebrew のライブラリ(Apple M1の場合): OpenCV, CoreMLTools
- モデルファイル例: ./models/yolo11x-pose.mlpackage (実行するディレクトリからの相対パス)
- カメラ (デバイス番号例: 0)

@par 送信するデータ (UDP、宛先例: 127.0.0.1:10001)
| 位置             | 型        | 内容                                         |
|------------------|-----------|----------------------------------------------|
| 先頭 4 バイト    | int32     | 検出した人数 N (誰もいなければ 0)             |
| 続く N 人分      | float32 × 51 | 17 個のキーポイント × (x, y, 信頼度)          |

数値は、実行環境のバイトオーダー (ネイティブ) のまま送る。
受信側 (C++) も同じバイトオーダーで読む前提である。

@par キーポイントの番号 (COCO 形式)
0: 鼻、1: 左目、2: 右目、3: 左耳、4: 右耳、5: 左肩、6: 右肩、7: 左肘、8: 右肘、
9: 左手首、10: 右手首、11: 左腰、12: 右腰、13: 左膝、14: 右膝、15: 左足首、16: 右足首

Copyright (c) 2026 Maito Yasui
Released under the GNU AGPL-3.0 License.
https://github.com/Nibosi0501/LiViMotion/blob/main/LICENSE
"""
import cv2
from ultralytics import YOLO

import socket
import struct

## 送信先の IP アドレス (同じ PC 内の受信プログラム)
IP = '127.0.0.1'
## 送信先のポート番号。C++ 側の前方カメラの YOLO 受信 (RecvYolo) のポートと同じ
PORT = 10001

# ランドマーク間の接続定義
## 骨格の線で結ぶキーポイントの番号の組 (COCO 形式)。顔・腕・胴体・脚をつなぐ。draw_keypoints() で使う
JOINTS = [
    (0, 1), (0, 2), (1, 3), (2, 4), (5, 6), (5, 7), (6, 8), (7, 9),
    (8, 10), (11, 12), (11, 13), (12, 14),
    (13, 15), (14, 16), (5, 11), (6, 12)
]

def main():
    """!
    @brief カメラ映像から姿勢を推定し、結果を送信し続ける (メインループ)

    処理の流れ:
    -# UDP ソケットを作り、YOLO のモデル (./models/yolo11x-pose.mlpackage) を読み込み、カメラ (デバイス番号 0) を開く
    -# カメラからフレームを 1 枚読み取る
    -# モデルで姿勢を推定し、人物ごとのバウンディングボックス・信頼度・キーポイントを取り出す
    -# 人物の信頼度が 0.5 以上のものだけを選び、send_keypoints() で送信する
    -# フレームを "Pose Estimation" のウィンドウに表示する
    -# q キーが押されたら、ループを抜けて終了する

    カメラを開けない場合や、フレームを読み取れない場合は、メッセージを出力して終了する。
    
    @note YOLO のモデルは、Apple Silicon の場合は mlpackage 形式(CoreML)のモデルを使用する。
           Intel Mac の場合は pt 形式(PyTorch)のモデルを使う。
    """
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    # Apple Silicon の場合は、mlpackage 形式(CoreML)のモデルを使う。
    model = YOLO('./models/yolo11x-pose.mlpackage')
    # Intel Mac や Windows の場合は、pt 形式(PyTorch)のモデルを使う。
    #model = YOLO('./models/yolo11m-pose.pt')

    cap = cv2.VideoCapture(0)

    if not cap.isOpened():
        print("カメラを開くことができませんでした")
        return
    
    while True:
        ret, frame = cap.read()
        if not ret:
            print("フレームを読み取ることができませんでした")
            break

        results = model(frame, show=False, save=False, verbose=True)

        boxes = results[0].boxes.xywh
        confidences = results[0].boxes.conf
        keypoints_list = results[0].keypoints

        send_keypoints_data = []
        if boxes is not None:
            for conf, keypoints in zip(confidences, keypoints_list):
                if conf < 0.5:
                    continue
                send_keypoints_data.append(keypoints)
        
        send_keypoints(sock, send_keypoints_data)

        annotated_frame = frame.copy()
        '''
        if boxes is not None:
            for box, conf, keypoints in zip(boxes, confidences, keypoints_list):
                pass

                # annotated_frame = draw_bbox(annotated_frame, box, conf, isConf = True)

                annotated_frame = draw_keypoints(annotated_frame, keypoints, confidence=0.5, isKeypoint=True, isKeypointLine=True)
                '''
                
        cv2.imshow('Pose Estimation', annotated_frame)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

def draw_bbox(frame, box, conf, isConf = True):
    """!
    @brief 人物のバウンディングボックス (検出枠) を映像に描く

    緑の四角形を描く。isConf が True なら、枠の左上に信頼度 (小数第 2 位まで) も描く。

    @param frame  描画先の映像 (BGR の画像)。この画像に直接描き込まれる
    @param box    バウンディングボックス (中心の x、中心の y、幅、高さ) の順の値 (xywh 形式)
    @param conf   検出の信頼度
    @param isConf 信頼度の文字も描くか (既定: True)
    @return 描画後の映像 (引数 frame と同じ画像)
    """
    x, y, w, h = box
    x1, y1, x2, y2 = int(x - w / 2), int(y - h / 2), int(x + w / 2), int(y + h / 2)

    cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)

    if isConf:
        cv2.putText(frame, f'{conf:.2f}', (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
    
    return frame

def draw_keypoints(frame, keypoints, confidence, isKeypoint = True, isKeypointLine = True):
    """!
    @brief 1 人分のキーポイントと骨格の線を映像に描く

    - isKeypointLine が True: JOINTS で定義された組のうち、両端の信頼度が confidence 以上のものを緑の線でつなぐ
    - isKeypoint が True: 信頼度が confidence 以上のキーポイントを、赤い円で描く

    座標が (0, 0) のキーポイントは、検出できなかったものとして描かない。
    isKeypoint と isKeypointLine の両方が False の場合は、何も描かずに映像をそのまま返す。

    @param frame          描画先の映像 (BGR の画像)。この画像に直接描き込まれる
    @param keypoints      1 人分のキーポイント (ultralytics の Keypoints オブジェクト)
    @param confidence     描画するキーポイントの最低信頼度
    @param isKeypoint     キーポイントの円を描くか (既定: True)
    @param isKeypointLine 骨格の線を描くか (既定: True)
    @return 描画後の映像 (引数 frame と同じ画像)
    """
    if isKeypoint is False and isKeypointLine is False:
        return frame
    
    confs = keypoints.conf[0].tolist()
    xys = keypoints.xy[0].tolist()

    if isKeypointLine:
        for(start, end) in JOINTS:
            if confs[start] >= confidence and confs[end] >= confidence:
                start_point = (int(xys[start][0]), int(xys[start][1]))
                end_point = (int(xys[end][0]), int(xys[end][1]))
                if(start_point[0] == 0 and start_point[1] == 0) or (end_point[0] == 0 and end_point[1] == 0):
                    continue
                cv2.line(frame, start_point, end_point, (0, 255, 0), 2)
    
    if isKeypoint:
        for xy, conf in zip(xys, confs):
            if conf < confidence:
                continue

            x, y = int(xy[0]), int(xy[1])
            if x == 0 and y == 0:
                continue

            cv2.circle(frame, (x, y), 5, (0, 0, 255), -1)
    
    return frame

# -------------------------------------------------------
# データ送信処理
# -------------------------------------------------------
# プロトコル仕様:
# [ヘッダー] 人数 (int32)
# [ボディ]   各人のデータ:
#           - Keypoints (float32 * 51) : (x, y, conf) * 17点
# -------------------------------------------------------
def send_keypoints(sock, keypoints_list):
    """!
    @brief 検出した人物のキーポイントを、1 つの UDP パケットにまとめて送信する

    パケットは、先頭に人数 (int32)、続けて人数分のキーポイント
    (1 人あたり 17 点 × (x, y, 信頼度) の float32 = 51 個) を並べた形式である。
    人数が 0 の場合も、「誰も検出されていない」ことを伝えるために、ヘッダーだけのパケットを送る。
    送信先は、IP と PORT で指定した宛先である。

    @param sock           送信に使う UDP ソケット
    @param keypoints_list 送信する人物のキーポイントのリスト (ultralytics の Keypoints オブジェクトのリスト)

    @note パケットの大きさは、4 + 204 × 人数 バイトである。UDP の 1 パケットに入る範囲で使うこと。
    """
    num_persons = len(keypoints_list)
    
    packet = bytearray()
    packet.extend(struct.pack('i', num_persons))

    for keypoints in keypoints_list:
        confs = keypoints.conf[0].tolist()
        xys = keypoints.xy[0].tolist()

        for xy, conf in zip(xys, confs):
            packet.extend(struct.pack('3f', xy[0], xy[1], conf))
    
    sock.sendto(packet, (IP, PORT))
    #print(f"送信データ: {num_persons}人のキーポイントデータを送信")
    for i in range(num_persons):
        # print(f"  人物 {i}:")
        for j in range(17):
            idx = i * 17 + j
            x, y, conf = struct.unpack_from('3f', packet, 4 + idx * 12)
            #print(f"    キーポイント {j}: (x={x}, y={y}, conf={conf})")

if __name__ == "__main__":
    main()