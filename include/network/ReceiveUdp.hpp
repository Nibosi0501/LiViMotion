/**
 * @file ReceiveUdp.hpp
 * @brief LiDAR / YOLO からの UDP パケットを受信するクラスの宣言
 *
 * ノンブロッキングの UDP ソケットで受信し、キューに溜まったパケットのうち
 * 最新の 1 つだけを解析して返す。
 *
 * @par パケット形式 (LiDAR)
 * | オフセット | 型      | 内容                         |
 * |-----------|---------|------------------------------|
 * | 0         | int32_t | 検出点の数 N                  |
 * | 4         | float×2 | (x, y) を N 個分              |
 *
 * @par パケット形式 (YOLO)
 * | オフセット | 型        | 内容                                   |
 * |-----------|-----------|----------------------------------------|
 * | 0         | int32_t   | 検出人数 N                              |
 * | 4         | float×51  | 17 キーポイント × (x, y, conf) を N 人分 |
 *
 * @note 数値はホストのバイトオーダー (エンディアン変換なし) のまま扱う。
 * 
 * Copyright (c) 2026 Maito Yasui
 * Released under the GNU AGPL-3.0 License.
 * https://github.com/Nibosi0501/LiViMotion/blob/main/LICENSE
 */
#pragma once

#include <span>
#include <vector>

/**
 * @namespace rcv
 * @brief UDP 受信関連の名前空間
 */
namespace rcv
{
    /**
     * @brief UDP 受信の共通基底クラス
     *
     * ソケットの作成・bind・ノンブロッキング化と、最新パケットの取得を担当する。
     * 直接インスタンス化はできず、派生クラス経由でのみ利用する。
     */
    struct RecvBase
    {
    protected:
        int sock; ///< UDP ソケットのファイルディスクリプタ

        /**
         * @brief 指定ポートで待ち受ける UDP ソケットを作成する
         * @param port 待ち受けポート番号 (全インターフェース INADDR_ANY で bind)
         * @note socket / bind / fcntl のいずれかが失敗した場合は std::exit(1) で終了する。
         */
        RecvBase(const int port);

        /**
         * @brief ソケットを閉じる
         */
        ~RecvBase();

        /**
         * @brief ソケットに溜まっているパケットを読み捨て、最新の 1 つを取得する
         *
         * ノンブロッキングで recvfrom() を繰り返し、読み取れなくなるまで続ける。
         * 古いフレームを処理せず、常に最新の状態だけを使うための関数。
         *
         * @param[out] outBuffer 受信データの格納先。呼び出し側で十分なサイズを確保しておくこと
         *                       (サイズ = 1 回の recvfrom() で受け取れる最大バイト数)。
         * @return 最後に受信したパケットのバイト数。1 つも受信できなかった場合は -1
         */
        ssize_t receiveLatest(std::vector<uint8_t> &outBuffer);
    };

    /// LiDAR パケットのヘッダーサイズ (検出点数 int32_t) [byte]
    constexpr size_t LIDAR_HEADER_SIZE = sizeof(uint32_t);
    /// LiDAR パケットの 1 点あたりのボディサイズ (x, y の float 2 つ) [byte]
    constexpr size_t LIDAR_BODY_SIZE = 2 * sizeof(float); // x, y

    /**
     * @brief LiDAR の足元検出結果 (2D 座標) を受信するクラス
     */
    struct RecvLidar : RecvBase
    {
        /**
         * @brief コンストラクタ
         * @param port 待ち受けポート番号
         */
        RecvLidar(const int port) : RecvBase(port) {}

        /**
         * @brief 最新の LiDAR パケットを受信して解析する
         * @param[out] outPoints 検出点 (x, y) のリスト。成功時のみ更新される
         * @retval true  パケットを受信し、解析に成功した
         * @retval false 受信したパケットがない、または解析に失敗した
         */
        bool receive(std::vector<std::pair<float, float>> &outPoints);

        /**
         * @brief LiDAR パケットのバイト列を解析する
         * @param[in]  packet    受信したパケット全体
         * @param[out] outPoints 検出点 (x, y) のリスト。呼び出し時に一度クリアされる
         * @retval true  解析に成功した (検出点 0 個の場合も true)
         * @retval false ヘッダー不足・人数が負・サイズ不足のいずれか
         */
        bool parse_packet(std::span<const uint8_t> packet, std::vector<std::pair<float, float>> &outPoints);
    };

    /// YOLO パケットのヘッダーサイズ (検出人数 int32_t) [byte]
    constexpr size_t YOLO_HEADER_SIZE = sizeof(uint32_t);
    /// YOLO パケットの 1 人あたりのボディサイズ (17 キーポイント × (x, y, conf)) [byte]
    constexpr size_t YOLO_BODY_SIZE = 17 * 3 * sizeof(float);

    /**
     * @brief YOLO 姿勢推定のキーポイントを受信するクラス
     *
     * 1 人あたり 17 点 (COCO 形式) のキーポイントを (x, y, 信頼度) の組で受け取る。
     */
    struct RecvYolo : RecvBase
    {
        /**
         * @brief コンストラクタ
         * @param port 待ち受けポート番号
         */
        RecvYolo(const int port) : RecvBase(port) {}

        /**
         * @brief 最新の YOLO パケットを受信して解析する
         * @param[out] outKeypoints_list 人物ごとのキーポイント列 [人][キーポイント] = (x, y, conf)。
         *                               成功時のみ更新される
         * @retval true  パケットを受信し、解析に成功した
         * @retval false 受信したパケットがない、または解析に失敗した
         */
        bool receive(std::vector<std::vector<std::tuple<float, float, float>>> &outKeypoints_list);

        /**
         * @brief YOLO パケットのバイト列を解析する
         * @param[in]  packet            受信したパケット全体
         * @param[out] outKeypoints_list 人物ごとのキーポイント列。呼び出し時に一度クリアされる。
         *                               各人物は 17 個の (x, y, conf) を持つ
         * @retval true  解析に成功した (検出人数 0 の場合も true)
         * @retval false ヘッダー不足・人数が負・サイズ不足のいずれか
         */
        bool parse_packet(std::span<const uint8_t> packet, std::vector<std::vector<std::tuple<float, float, float>>> &outKeypoints_list);
    };
}
