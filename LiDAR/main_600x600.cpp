// ----------------------------------------------------------------
// main_600x600.cpp
//
// 二次元LiDARで 6m x 6m の範囲をスキャンし、人の足位置を検出、送信.
// LiDARを使用するためのライブラリの権利関係により、LiDARのスキャン処理は記載していません。
// LiDARのスキャン処理は、使用するLiDARのSDKやAPIに応じて実装してください。
//
// Copyright (c) 2026 Maito Yasui
// Released under the GNU GPL-3.0 License.
// https://github.com/Nibosi0501/LiViMotion/blob/main/LICENSE
// ----------------------------------------------------------------

// LiDARのスキャン結果を二値画像として取得
// 膨張収縮処理、輪郭抽出、重心計算を行い、足位置を検出
/* OpenGL座標系に変換(必要であれば)
    float mapValue(const int x, const int in_min, const int in_max, const float out_min, const float out_max)
    {
        return static_cast<float>(x - in_min) * (out_max - out_min) / static_cast<float>(in_max - in_min) + out_min;
    }
*/

/* UDPで送信
    #include "../Udp/SendUdp.hpp"   // UDP送信クラスのヘッダファイル

    snd::SendLidar sendLidar(12345, "127.0.0.1");   // ポート番号と送信先IPアドレスを指定してSendLidarクラスのインスタンスを作成

    std::vector<std::pair<float, float>> points;    // 足位置の座標を格納するベクター

    // 足位置の座標をpointsベクターに追加

    sendLidar.send(points); // 足位置の座標をUDPで送信
*/