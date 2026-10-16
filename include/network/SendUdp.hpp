/**
 * @file SendUdp.hpp
 * @brief LiDAR の検出結果を UDP で送信するクラスの宣言
 *
 * @par パケット形式 (LiDAR)
 * | オフセット | 型      | 内容                 |
 * |-----------|---------|----------------------|
 * | 0         | int32_t | 検出点の数 N          |
 * | 4         | float×2 | (x, y) を N 個分      |
 *
 * @note 数値はホストのバイトオーダー (エンディアン変換なし) のまま送信する。
 * @see ReceiveUdp.hpp
 * 
 * Copyright (c) 2026 Maito Yasui
 * Released under the GNU AGPL-3.0 License.
 * https://github.com/Nibosi0501/LiViMotion/blob/main/LICENSE
 */
#pragma once

#include <string>
#include <vector>

/**
 * @namespace snd
 * @brief UDP 送信関連の名前空間
 */
namespace snd
{
    /**
     * @brief UDP 送信の共通基底クラス
     *
     * ソケットの作成、ノンブロッキング化、送信先への connect() を担当する。
     * 直接インスタンス化はできず、派生クラス経由でのみ利用する。
     */
    struct SendBase
    {
    protected:
        int sock; ///< UDP ソケットのファイルディスクリプタ (送信先に connect 済み)

        /**
         * @brief 送信先を指定して UDP ソケットを作成する
         * @param port 送信先ポート番号
         * @param ip   送信先 IPv4 アドレス (例: "127.0.0.1")
         * @note socket / fcntl / connect のいずれかが失敗した場合は std::exit(1) で終了する。
         */
        SendBase(const int port, const std::string &ip);

        /**
         * @brief ソケットを閉じる
         */
        ~SendBase();
    };

    /**
     * @brief LiDAR の検出点 (2D 座標) を送信するクラス
     */
    struct SendLidar : SendBase
    {
        /**
         * @brief コンストラクタ
         * @param port 送信先ポート番号
         * @param ip   送信先 IPv4 アドレス
         */
        SendLidar(const int port, const std::string &ip) : SendBase(port, ip) {}

        /**
         * @brief 検出点のリストを 1 パケットにまとめて送信する
         *
         * 検出点が 0 個の場合も「検出なし」を伝えるために送信する。
         * ソケットがノンブロッキングのため、送信バッファが一杯のときは送信せずに戻る。
         *
         * @param[in] points 送信する検出点 (x, y) のリスト
         * @retval true  送信に成功した
         * @retval false 送信に失敗した (EAGAIN / EWOULDBLOCK の場合は黙って false、
         *               それ以外のエラーは標準エラー出力にログを出す)
         *
         * @warning std::pair<float, float> のメモリ配置をそのままコピーして送信する。
         *          受信側 (RecvLidar) は float 2 個 = 8 byte を 1 点として読むため、
         *          両者のレイアウトが一致している前提で動作する。
         */
        bool send(const std::vector<std::pair<float, float>> &points);
    };
}
