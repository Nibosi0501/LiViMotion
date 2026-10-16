// ----------------------------------------------------------------
/**
 * @file Tracker.hpp
 * @brief LiDAR / YOLO の検出結果を時系列で追跡するトラッカーの宣言
 *
 * 毎フレームの検出結果 (足元位置) を、既存のトラックにハンガリアン法で対応付けて追跡する。
 * - 対応付けられたトラックは位置と履歴を更新する
 * - 対応付けられなかったトラックは見失ったフレーム数 (skippedFrames) を増やす
 * - 対応付けられなかった検出結果は新しいトラックとして登録する
 * - 長く見失ったトラックは deleteLostTrackers() で削除する
 *
 * @par 1 フレームの典型的な呼び出し順序
 * @code
 * tkr::updateTrackers(trackers, detections, idGen);
 * tkr::deleteLostTrackers(trackers);
 * @endcode
 *
 * Copyright (c) 2026 Maito Yasui
 * Released under the GNU AGPL-3.0 License.
 * https://github.com/Nibosi0501/LiViMotion/blob/main/LICENSE
 */
// ----------------------------------------------------------------

#pragma once
#include "../yolo/Yolo.hpp"
#include "../lidar/Lidar.hpp"

#include "../kalman/Kalman.hpp"

#include <chrono>

/**
 * @namespace tkr
 * @brief トラッキング (追跡) 関連の名前空間
 */
namespace tkr
{
    constexpr int MAX_HISTORY_SIZE = 5;                 ///< 1 トラックが保持する履歴の最大数
    constexpr int MAX_SKIPPED_FRAMES = 30;              ///< 見失ってもトラックを保持する最大フレーム数 (これを超えると削除)
    constexpr double EUCLIDEAN_LIDAR_THRESHOLD = 130.0; ///< LiDAR の対応付けを許す最大のユークリッド距離
    constexpr double EUCLIDEAN_YOLO_THRESHOLD = 130.0;  ///< YOLO の対応付けを許す最大のユークリッド距離
    /// LiDAR の対応付けの閾値 (距離の 2 乗)。コスト行列が 2 乗距離のため、こちらを比較に使う
    constexpr double SQUARED_EUCLIDEAN_LIDAR_THRESHOLD = EUCLIDEAN_LIDAR_THRESHOLD * EUCLIDEAN_LIDAR_THRESHOLD;
    /// YOLO の対応付けの閾値 (距離の 2 乗)。コスト行列が 2 乗距離のため、こちらを比較に使う
    constexpr double SQUARED_EUCLIDEAN_YOLO_THRESHOLD = EUCLIDEAN_YOLO_THRESHOLD * EUCLIDEAN_YOLO_THRESHOLD;

    /**
     * @brief トラックに割り当てる ID を生成する
     *
     * 0 から順に ID を発行し、10000 を超えたら 0 に戻る。
     * LiDAR / YOLO の両方で同じインスタンスを共有することも、別々に持つこともできる。
     */
    struct IDGenerator
    {
        int nextID = 0; ///< 次に発行する ID

        /**
         * @brief 新しい ID を発行する
         * @return 発行した ID (0 〜 10000)
         * @note 10000 を超えると 0 に戻るため、長時間動かすと生存中のトラックと
         *       ID が重複する可能性がある。
         */
        int generate()
        {
            int id = nextID++;
            if (nextID > 10000)
                nextID = 0;
            return id;
        }
    };

    /**
     * @brief LiDAR で検出した 1 人分の追跡情報
     */
    struct LidarTracker
    {
        int id;                          ///< トラックの ID
        lidar::Lidar currentLidar;       ///< 最新の検出位置
        int skippedFrames;               ///< 連続で対応付けられなかったフレーム数 (対応付けられると 0 に戻る)
        std::deque<lidar::Lidar> historys; ///< 位置の履歴 (最大 MAX_HISTORY_SIZE 件、古い順)

        // std::chrono::steady_clock::time_point lastAddHistoryTime;

        /**
         * @brief トラックを作成し、idGen から ID を発行する
         * @param[in,out] idGen          ID の発行元
         * @param[in]     _currentLidar  最初の検出位置
         * @param[in]     _skippedFrames 見失ったフレーム数の初期値 (既定: 0)
         * @param[in]     historys       履歴の初期値 (既定: 空)
         */
        LidarTracker(IDGenerator &idGen, const lidar::Lidar &_currentLidar, const int &_skippedFrames = 0, const std::deque<lidar::Lidar> &historys = std::deque<lidar::Lidar>());

        /**
         * @brief 履歴の末尾に位置を追加する
         *
         * 履歴が MAX_HISTORY_SIZE に達している場合は、最も古い履歴を捨ててから追加する。
         *
         * @param[in] lidar 追加する位置
         */
        void addHistory(const lidar::Lidar &lidar);
    };

    /**
     * @brief LiDAR のトラッカーを、今回の検出結果で更新する
     *
     * 各トラックの最新位置と検出点との 2 乗距離をコストとし、ハンガリアン法で対応付ける。
     * 2 乗距離が SQUARED_EUCLIDEAN_LIDAR_THRESHOLD 未満のペアだけを有効な対応とみなす。
     *
     * - 検出が 0 件: 全トラックの skippedFrames を増やして戻る
     * - トラックが 0 件: 検出ごとに新しいトラックを作る
     * - 有効に対応付けられたトラック: 最新位置を更新し、skippedFrames を 0 にする。
     *   履歴の最後の位置から一定以上 (距離 30 以上) 動いている場合のみ履歴に追加する
     * - 対応付けられなかったトラック: skippedFrames を増やす
     * - 対応付けられなかった検出: 新しいトラックとして登録する
     *
     * @param[in,out] lidarTrackers  更新対象のトラッカー一覧
     * @param[in]     currentLidars  今回のフレームの検出結果
     * @param[in,out] idGen          新規トラックの ID の発行元
     */
    void updateTrackers(std::vector<tkr::LidarTracker> &lidarTrackers, const std::vector<lidar::Lidar> &currentLidars, IDGenerator &idGen);

    /**
     * @brief 見失ったフレーム数が MAX_SKIPPED_FRAMES を超えた LiDAR トラックを削除する
     * @param[in,out] lidarTrackers 対象のトラッカー一覧
     */
    void deleteLostTrackers(std::vector<tkr::LidarTracker> &lidarTrackers);

    /**
     * @brief YOLO で検出した 1 人分の追跡情報 (カルマンフィルタ付き)
     */
    struct YoloTracker
    {
        int id;                         ///< トラックの ID
        yolo::Yolo currentYolo;         ///< 最新の検出結果
        int skippedFrames;              ///< 連続で対応付けられなかったフレーム数 (対応付けられると 0 に戻る)
        std::deque<yolo::Yolo> historys; ///< 検出結果の履歴 (最大 MAX_HISTORY_SIZE 件、古い順)

        kf::KalmanYolo kalmanYolo; ///< 足元位置を平滑化・予測するカルマンフィルタ

        /**
         * @brief トラックを作成し、idGen から ID を発行する
         *
         * カルマンフィルタは _currentYolo の足元位置で初期化される。
         *
         * @param[in,out] idGen          ID の発行元
         * @param[in]     _currentYolo   最初の検出結果
         * @param[in]     _skippedFrames 見失ったフレーム数の初期値 (既定: 0)
         * @param[in]     _historys      履歴の初期値 (既定: 空)
         */
        YoloTracker(IDGenerator &idGen, const yolo::Yolo &_currentYolo, const int &_skippedFrames = 0, const std::deque<yolo::Yolo> &_historys = std::deque<yolo::Yolo>());

        /**
         * @brief 履歴の末尾に検出結果を追加する
         *
         * 履歴が MAX_HISTORY_SIZE に達している場合は、最も古い履歴を捨ててから追加する。
         *
         * @param[in] yolo 追加する検出結果
         */
        void addHistory(const yolo::Yolo &yolo);
    };

    /**
     * @brief YOLO のトラッカーを、今回の検出結果で更新する
     *
     * 各トラックの最新の足元位置と検出結果の足元位置との 2 乗距離をコストとし、
     * ハンガリアン法で対応付ける。2 乗距離が SQUARED_EUCLIDEAN_YOLO_THRESHOLD 未満の
     * ペアだけを有効な対応とみなす。
     *
     * - 検出が 0 件: 全トラックの skippedFrames を増やし、カルマンフィルタの予測ステップを実行して戻る
     * - トラックが 0 件: 検出ごとに新しいトラックを作る
     * - 有効に対応付けられたトラック: 更新前の最新の検出結果を履歴に追加し、
     *   最新の検出結果を更新して skippedFrames を 0 にし、カルマンフィルタを update() する
     * - 対応付けられなかったトラック: skippedFrames を増やし、カルマンフィルタの予測ステップを実行する
     * - 対応付けられなかった検出: 新しいトラックとして登録する
     *
     * @param[in,out] yoloTrackers  更新対象のトラッカー一覧
     * @param[in]     currentYolos  今回のフレームの検出結果
     * @param[in,out] idGen         新規トラックの ID の発行元
     */
    void updateTrackers(std::vector<tkr::YoloTracker> &yoloTrackers, const std::vector<yolo::Yolo> &currentYolos, IDGenerator &idGen);

    /**
     * @brief 見失ったトラックと、無効なトラックを削除する
     *
     * 次のどちらかに当てはまる YOLO トラックを削除する。
     * - skippedFrames が MAX_SKIPPED_FRAMES を超えた
     * - 最新の検出結果の足元の信頼度 (footConf) が負 (座標が無効)
     *
     * @param[in,out] yoloTrackers 対象のトラッカー一覧
     */
    void deleteLostTrackers(std::vector<tkr::YoloTracker> &yoloTrackers);
}
