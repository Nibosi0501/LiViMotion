# ----------------------------------------------------------------
# main.py
#
# YOLOv11x-poseを使用してリアルタイムで姿勢推定を行い、検出されたキーポイントデータをUDPで送信するサンプルコード
# 複数台のカメラを使用する場合は、main.pyを複数起動して、それぞれのカメラに対応するモデルと送信先IP/PORTを設定してください。
# Ultralytics YOLOv11 Pose Estimation Model を公式サイトからダウンロードして、models フォルダに保存してください。
#
# Copyright (c) 2026 Maito Yasui
# Released under the GNU GPL-3.0 License.
# https://github.com/Nibosi0501/LiViMotion/blob/main/LICENSE
# ----------------------------------------------------------------

import cv2
from ultralytics import YOLO

import socket
import struct

IP = '127.0.0.1'
PORT = 10001

# ランドマーク間の接続定義
JOINTS = [
    (0, 1), (0, 2), (1, 3), (2, 4), (5, 6), (5, 7), (6, 8), (7, 9),
    (8, 10), (11, 12), (11, 13), (12, 14),
    (13, 15), (14, 16), (5, 11), (6, 12)
]

def main():
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    model = YOLO('./models/yolo11x-pose.mlpackage')

    cap = cv2.VideoCapture(0)

    if not cap.isOpened():
        print("カメラを開くことができませんでした")
        return
    
    while True:
        ret, frame = cap.read()
        if not ret:
            print("フレームを読み取ることができませんでした")
            break

        results = model(frame, show=False, save=False, verbose=False)

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

        ''' デバッグ用: 検出結果をフレームに描画して表示
        annotated_frame = frame.copy()
        if boxes is not None:
            for box, conf, keypoints in zip(boxes, confidences, keypoints_list):
                pass

                annotated_frame = draw_bbox(annotated_frame, box, conf, isConf = True)

                annotated_frame = draw_keypoints(annotated_frame, keypoints, confidence=0.5, isKeypoint=True, isKeypointLine=True)
                
        cv2.imshow('Pose Estimation', annotated_frame)
        '''

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

def draw_bbox(frame, box, conf, isConf = True):
    x, y, w, h = box
    x1, y1, x2, y2 = int(x - w / 2), int(y - h / 2), int(x + w / 2), int(y + h / 2)

    cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)

    if isConf:
        cv2.putText(frame, f'{conf:.2f}', (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
    
    return frame

def draw_keypoints(frame, keypoints, confidence, isKeypoint = True, isKeypointLine = True):
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
    num_persons = len(keypoints_list)
    if num_persons < 0: # num_personが0の時は、誰も検出されていないことを示すために空のデータを送信する
        return
    
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
        print(f"  人物 {i}:")
        for j in range(17):
            idx = i * 17 + j
            x, y, conf = struct.unpack_from('3f', packet, 4 + idx * 12)
            print(f"    キーポイント {j}: (x={x}, y={y}, conf={conf})")

if __name__ == "__main__":
    main()