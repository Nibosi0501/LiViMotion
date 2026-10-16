// ----------------------------------------------------------------
/**
 * @file tracker.cpp
 * @brief LiDAR / YOLO のトラッカーの実装
 * @see Tracker.hpp
 *
 * Copyright (c) 2026 Maito Yasui
 * Released under the GNU AGPL-3.0 License.
 * https://github.com/Nibosi0501/LiViMotion/blob/main/LICENSE
 */
// ----------------------------------------------------------------

#include "../../include/tracker/Tracker.hpp"
#include "../../include/core/Hungarian.hpp"

namespace tkr
{
    /**
     * @details メンバを引数で初期化し、idGen.generate() で発行した ID を id に設定する。
     */
    LidarTracker::LidarTracker(IDGenerator &idGen, const lidar::Lidar &_currentLidar, const int &_skippedFrames, const std::deque<lidar::Lidar> &historys)
        : currentLidar(_currentLidar), skippedFrames(_skippedFrames), historys(historys)
    {
        id = idGen.generate();
        // lastAddHistoryTime = std::chrono::steady_clock::now();
    }

    /**
     * @details 履歴が MAX_HISTORY_SIZE 以上なら先頭 (最も古い要素) を削除してから、末尾に追加する。
     */
    void LidarTracker::addHistory(const lidar::Lidar &lidar)
    {
        if (historys.size() >= MAX_HISTORY_SIZE)
        {
            historys.pop_front();
        }

        historys.emplace_back(lidar);
    }

    /**
     * @details
     * 処理の流れ:
     * -# 検出が 0 件なら、全トラックの skippedFrames を増やして終了する
     * -# トラックが 0 件なら、全検出を新規トラックとして登録して終了する
     * -# 「トラック × 検出」の 2 乗距離のコスト行列を作り、Hungarian::Solve() で対応付ける
     * -# 各トラックについて、対応先があり 2 乗距離が閾値未満なら更新、そうでなければ skippedFrames を増やす
     * -# 使われなかった検出を新規トラックとして登録する
     *
     * 履歴への追加は、履歴の最後の位置との 2 乗距離が 900 (距離 30) 以上のときだけ行う。
     * 静止中に履歴が同じ位置で埋まるのを防ぐための処理。
     *
     * @note 新規トラックの作成時に addHistory() で履歴の先頭を入れているため、
     *       対応付け時に historys.back() を参照しても履歴が空になることはない。
     */
    void updateTrackers(std::vector<tkr::LidarTracker> &lidarTrackers, const std::vector<lidar::Lidar> &currentLidars, IDGenerator &idGen)
    {
        if (currentLidars.empty())
        {
            for (auto &tracker : lidarTrackers)
            {
                tracker.skippedFrames++;
            }
            return;
        }

        if (lidarTrackers.empty())
        {
            lidarTrackers.reserve(currentLidars.size());
            for (const auto &lidar : currentLidars)
            {
                lidarTrackers.emplace_back(idGen, lidar);
                lidarTrackers.back().addHistory(lidar);
            }
            return;
        }

        int nTracks = lidarTrackers.size();
        int nLidars = currentLidars.size();
        std::vector<std::vector<double>> costMatrix(nTracks, std::vector<double>(nLidars));
        for (int row = 0; row < nTracks; row++)
        {
            for (int col = 0; col < nLidars; col++)
            {
                costMatrix.at(row).at(col) = static_cast<double>(std::pow(lidarTrackers.at(row).currentLidar.footX - currentLidars.at(col).footX, 2) +
                                                                 std::pow(lidarTrackers.at(row).currentLidar.footY - currentLidars.at(col).footY, 2));
            }
        }

        std::vector<int> assignment;
        Hungarian::Solve(costMatrix, assignment);

        std::vector<bool> lidar_used(nLidars, false);
        for (int row = 0; row < nTracks; row++)
        {
            int col = assignment.at(row);
            if (col >= 0 && costMatrix.at(row).at(col) < SQUARED_EUCLIDEAN_LIDAR_THRESHOLD)
            {
                // 最新のLidar履歴点と今の観測点が２乗距離で閾値以上離れているならば、その点を履歴に追加する
                double dist = std::pow(lidarTrackers.at(row).historys.back().footX - currentLidars.at(col).footX, 2) +
                              std::pow(lidarTrackers.at(row).historys.back().footY - currentLidars.at(col).footY, 2);
                if (dist >= 900.0)
                {
                    lidarTrackers.at(row).addHistory(currentLidars.at(col));
                }

                lidarTrackers.at(row).currentLidar = currentLidars.at(col);
                lidarTrackers.at(row).skippedFrames = 0;
                lidar_used.at(col) = true;
            }
            else
            {
                lidarTrackers.at(row).skippedFrames++;
            }
        }

        for (int col = 0; col < nLidars; col++)
        {
            if (lidar_used.at(col) == false)
            {
                lidarTrackers.emplace_back(idGen, currentLidars.at(col));
                lidarTrackers.back().addHistory(currentLidars.at(col));
            }
        }
    }

    /**
     * @details skippedFrames が MAX_SKIPPED_FRAMES を超えたトラックを std::erase_if で削除する。
     */
    void deleteLostTrackers(std::vector<tkr::LidarTracker> &lidarTrackers)
    {
        std::erase_if(lidarTrackers, [](auto &tracker)
                      { return tracker.skippedFrames > MAX_SKIPPED_FRAMES; });
    }

    /**
     * @details
     * メンバを引数で初期化し、カルマンフィルタを _currentYolo の足元位置で初期化する。
     * ID は idGen.generate() で発行する。
     */
    YoloTracker::YoloTracker(IDGenerator &idGen, const yolo::Yolo &_currentYolo, const int &_skippedFrames, const std::deque<yolo::Yolo> &_historys)
        : currentYolo(_currentYolo), skippedFrames(_skippedFrames), historys(_historys), kalmanYolo(_currentYolo)
    {
        id = idGen.generate();
    }

    /**
     * @details 履歴が MAX_HISTORY_SIZE 以上なら先頭 (最も古い要素) を削除してから、末尾に追加する。
     */
    void YoloTracker::addHistory(const yolo::Yolo &yolo)
    {
        if (historys.size() >= MAX_HISTORY_SIZE)
        {
            historys.pop_front();
        }

        historys.emplace_back(yolo);
    }

    /**
     * @details
     * 処理の流れ:
     * -# 検出が 0 件なら、全トラックの skippedFrames を増やし、予測ステップだけ実行して終了する
     * -# トラックが 0 件なら、全検出を新規トラックとして登録して終了する
     * -# 「トラック × 検出」の足元位置の 2 乗距離のコスト行列を作り、Hungarian::Solve() で対応付ける
     * -# 各トラックについて:
     *    - 対応先があり 2 乗距離が閾値未満: 更新前の検出結果を履歴に追加してから最新の検出結果を更新し、
     *      skippedFrames を 0 にして、カルマンフィルタを補正する
     *    - それ以外: skippedFrames を増やし、カルマンフィルタの予測ステップだけ実行する
     * -# 使われなかった検出を新規トラックとして登録する
     *
     * @note LiDAR 版と異なり、履歴には「更新前の最新の検出結果」を毎フレーム追加する
     *       (移動距離による間引きは行わない)。
     * @note 対応付けには currentYolo.footX / footY を使い、カルマンフィルタの予測位置は使っていない。
     */
    void updateTrackers(std::vector<tkr::YoloTracker> &yoloTrackers, const std::vector<yolo::Yolo> &currentYolos, IDGenerator &idGen)
    {
        if (currentYolos.empty())
        {
            for (auto &tracker : yoloTrackers)
            {
                tracker.skippedFrames++;
                tracker.kalmanYolo.predict(); // 観測なしでも予測ステップを実行
            }
            return;
        }

        if (yoloTrackers.empty())
        {
            yoloTrackers.reserve(currentYolos.size());
            for (const auto &yolo : currentYolos)
            {
                yoloTrackers.emplace_back(idGen, yolo);
            }
            return;
        }

        int nTracks = yoloTrackers.size();
        int nYolos = currentYolos.size();
        std::vector<std::vector<double>> costMatrix(nTracks, std::vector<double>(nYolos));
        for (int row = 0; row < nTracks; row++)
        {
            for (int col = 0; col < nYolos; col++)
            {
                costMatrix.at(row).at(col) = static_cast<double>(std::pow(yoloTrackers.at(row).currentYolo.footX - currentYolos.at(col).footX, 2) +
                                                                 std::pow(yoloTrackers.at(row).currentYolo.footY - currentYolos.at(col).footY, 2));
            }
        }

        std::vector<int> assignment;
        Hungarian::Solve(costMatrix, assignment);

        std::vector<bool> yolo_used(nYolos, false);
        for (int row = 0; row < nTracks; row++)
        {
            int col = assignment.at(row);
            if (col >= 0 && costMatrix.at(row).at(col) < SQUARED_EUCLIDEAN_YOLO_THRESHOLD)
            {
                yoloTrackers.at(row).addHistory(yoloTrackers.at(row).currentYolo);

                yoloTrackers.at(row).currentYolo = currentYolos.at(col);
                yoloTrackers.at(row).skippedFrames = 0;
                yolo_used.at(col) = true;

                // Kalman Filter
                yoloTrackers.at(row).kalmanYolo.update(currentYolos.at(col));
            }
            else
            {
                yoloTrackers.at(row).skippedFrames++;
                yoloTrackers.at(row).kalmanYolo.predict(); // 観測なしでも予測ステップを実行
            }
        }

        for (int col = 0; col < nYolos; col++)
        {
            if (yolo_used.at(col) == false)
            {
                yoloTrackers.emplace_back(idGen, currentYolos.at(col));
            }
        }
    }

    /**
     * @details
     * skippedFrames が MAX_SKIPPED_FRAMES を超えたトラック、
     * または最新の検出結果の footConf が負のトラックを std::erase_if で削除する。
     */
    void deleteLostTrackers(std::vector<tkr::YoloTracker> &yoloTrackers)
    {
        std::erase_if(yoloTrackers, [](auto &tracker)
                      { return tracker.skippedFrames > MAX_SKIPPED_FRAMES || tracker.currentYolo.footConf < 0; });
    }
}
