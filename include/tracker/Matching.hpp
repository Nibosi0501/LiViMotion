// ----------------------------------------------------------------
/**
 * @file Matching.hpp
 * @brief LiDAR トラッカーと YOLO トラッカーを対応付ける (マッチングする) 処理の宣言
 *
 * 同じ人物を LiDAR と YOLO の両方で追跡している場合に、2 つのトラッカーを 1 組のペア
 * (MatchedTracker) として結び付ける。
 * 前フレームでペアだったものは、ID が両方残っていて距離が近ければ優先して維持し、
 * 残りはハンガリアン法で新しく対応付ける。
 *
 * Copyright (c) 2026 Maito Yasui
 * Released under the GNU AGPL-3.0 License.
 * https://github.com/Nibosi0501/LiViMotion/blob/main/LICENSE
 */
// ----------------------------------------------------------------

#pragma once

#include "./Tracker.hpp"

/**
 * @namespace mtc
 * @brief LiDAR と YOLO のトラッカーのマッチング関連の名前空間
 */
namespace mtc
{
    /**
     * @brief LiDAR トラッカーと YOLO トラッカーを対応付けたペア
     *
     * 2 つのトラッカーはコピーして保持する (参照ではない)。
     * そのため、ペアを作った時点のスナップショットであり、元のトラッカーが更新されても反映されない。
     */
    struct MatchedTracker
    {
        float footX; ///< ペアの足元の X 座標 (LiDAR の最新位置を採用)
        float footY; ///< ペアの足元の Y 座標 (LiDAR の最新位置を採用)

        tkr::LidarTracker lidarTracker; ///< 対応付けられた LiDAR トラッカー
        tkr::YoloTracker yoloTracker;   ///< 対応付けられた YOLO トラッカー

        /**
         * @brief 2 つのトラッカーからペアを作る
         *
         * footX / footY には LiDAR トラッカーの最新位置を設定する。
         *
         * @param[in] _lidarTracker 対応付ける LiDAR トラッカー
         * @param[in] _yoloTracker  対応付ける YOLO トラッカー
         */
        MatchedTracker(const tkr::LidarTracker &_lidarTracker, const tkr::YoloTracker &_yoloTracker);
    };

    /**
     * @brief マッチング済みトラッカーの一覧を、最新のトラッカー群に合わせて更新する
     *
     * 処理の流れ:
     * -# LiDAR か YOLO のどちらかのトラッカーが 0 件なら、ペアをすべて削除して終了する
     * -# 前フレームのペアのうち、LiDAR・YOLO の両方の ID がまだ存在し、
     *    距離が閾値未満のものを優先して維持する
     * -# 残った未マッチのトラッカー同士で、ハンガリアン法により新しいペアを求める
     *    (距離が閾値未満のものだけ採用)
     * -# matchedTrackers を新しいペアの一覧で置き換える
     *
     * 距離は、LiDAR の足元位置と、YOLO のカルマンフィルタによる 4 フレーム先の予測位置との
     * 2 乗距離で評価する。
     *
     * @param[in,out] matchedTrackers マッチング済みトラッカーの一覧 (更新される)
     * @param[in]     lidarTrackers   現在の LiDAR トラッカー一覧
     * @param[in]     yoloTrackers    現在の YOLO トラッカー一覧
     */
    void updateMatchedTrackers(std::vector<mtc::MatchedTracker> &matchedTrackers, const std::vector<tkr::LidarTracker> &lidarTrackers, const std::vector<tkr::YoloTracker> &yoloTrackers);

    /**
     * @brief LiDAR トラッカーの ID から、対応するマッチング済みトラッカーを探す
     *
     * @param[in] lidarID         探したい LiDAR トラッカーの ID
     * @param[in] matchedTrackers マッチング済みトラッカーの一覧
     * @return 見つかったペアを指すイテレータ。見つからなければ matchedTrackers.end()
     */
    std::vector<MatchedTracker>::const_iterator getOtherMatchedTrackerIteratorFromLidarID(const int lidarID, const std::vector<mtc::MatchedTracker> &matchedTrackers);
}
