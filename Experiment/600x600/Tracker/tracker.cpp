// ----------------------------------------------------------------
// tracker.cpp
//
// ユーザの足位置を追跡するためのクラスと関数の実装
//
// Copyright (c) 2026 Maito Yasui
// Released under the GNU GPL-3.0 License.
// https://github.com/Nibosi0501/LiViMotion/blob/main/LICENSE
// ----------------------------------------------------------------

#include "./Tracker.hpp"
#include "./Hungarian/Hungarian.hpp"

namespace tkr
{
    LidarTracker::LidarTracker(IDGenerator &idGen, const lidar::Lidar &_currentLidar, const int &_skippedFrames, const std::deque<lidar::Lidar> &historys)
        : currentLidar(_currentLidar), skippedFrames(_skippedFrames), historys(historys)
    {
        id = idGen.generate();
        // lastAddHistoryTime = std::chrono::steady_clock::now();
    }

    void LidarTracker::addHistory(const lidar::Lidar &lidar)
    {
        if (historys.size() >= MAX_HISTORY_SIZE)
        {
            historys.pop_front();
        }

        historys.emplace_back(lidar);
    }

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
                if (dist >= 2500.0f)
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

    void deleteLostTrackers(std::vector<tkr::LidarTracker> &lidarTrackers)
    {
        std::erase_if(lidarTrackers, [](auto &tracker)
                      { return tracker.skippedFrames > MAX_SKIPPED_FRAMES; });
    }

    YoloTracker::YoloTracker(IDGenerator &idGen, const yolo::Yolo &_currentYolo, const int &_skippedFrames, const std::deque<yolo::Yolo> &_historys)
        : currentYolo(_currentYolo), skippedFrames(_skippedFrames), historys(_historys), kalmanYolo(_currentYolo)
    {
        id = idGen.generate();
    }

    void YoloTracker::addHistory(const yolo::Yolo &yolo)
    {
        if (historys.size() >= MAX_HISTORY_SIZE)
        {
            historys.pop_front();
        }

        historys.emplace_back(yolo);
    }

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

    void deleteLostTrackers(std::vector<tkr::YoloTracker> &yoloTrackers)
    {
        std::erase_if(yoloTrackers, [](auto &tracker)
                      { return tracker.skippedFrames > MAX_SKIPPED_FRAMES || tracker.currentYolo.footConf < 0; });
    }
}