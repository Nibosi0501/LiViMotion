// ----------------------------------------------------------------
// matching.cpp
//
// LiDARトラッカーとYOLOトラッカーのマッチングを行う関数の実装
//
// Copyright (c) 2026 Maito Yasui
// Released under the GNU GPL-3.0 License.
// https://github.com/Nibosi0501/LiViMotion/blob/main/LICENSE
// ----------------------------------------------------------------

#include "./Matching.hpp"

#include "../Hungarian/Hungarian.hpp"

namespace mtc
{
    constexpr double EUCLIDEAN_MATCHING_THRESHOLD = 80.0; // 80.0の二乗
    constexpr double SQUARED_EUCLIDEAN_MATCHING_THRESHOLD = EUCLIDEAN_MATCHING_THRESHOLD * EUCLIDEAN_MATCHING_THRESHOLD;

    MatchedTracker::MatchedTracker(const tkr::LidarTracker &_lidarTracker, const tkr::YoloTracker &_yoloTracker)
        : lidarTracker(_lidarTracker), yoloTracker(_yoloTracker)
    {
        footX = lidarTracker.currentLidar.footX;
        footY = lidarTracker.currentLidar.footY;
    }

    void updateMatchedTrackers(std::vector<mtc::MatchedTracker> &matchedTrackers, const std::vector<tkr::LidarTracker> &lidarTrackers, const std::vector<tkr::YoloTracker> &yoloTrackers)
    {
        // もし、LiDARとYOLOのどちらのトラッカーも空の場合、マッチングされたトラッカーを全て削除して終了
        if (lidarTrackers.empty() || yoloTrackers.empty())
        {
            matchedTrackers.clear();
            return;
        }

        std::vector<MatchedTracker> newMatchedTrackers;
        std::vector<tkr::LidarTracker> unmatchedLidarTrackers = lidarTrackers;
        std::vector<tkr::YoloTracker> unmatchedYoloTrackers = yoloTrackers;
        for (auto &matchedTracker : matchedTrackers)
        {
            int matchedLidarID = matchedTracker.lidarTracker.id;
            int matchedYoloID = matchedTracker.yoloTracker.id;

            auto lidarIt = std::find_if(lidarTrackers.begin(), lidarTrackers.end(),
                                        [matchedLidarID](const tkr::LidarTracker &tracker)
                                        { return tracker.id == matchedLidarID; });
            auto yoloIt = std::find_if(yoloTrackers.begin(), yoloTrackers.end(),
                                       [matchedYoloID](const tkr::YoloTracker &tracker)
                                       { return tracker.id == matchedYoloID; });
            if (lidarIt != lidarTrackers.end() && yoloIt != yoloTrackers.end())
            {
                // もし、既存のマッチングされたトラッカーのペアと同一のIDを持つトラッカーが両方とも存在する場合、そのペアで優先的にマッチングを行う
                // double distance = std::pow(lidarIt->currentLidar.footX - yoloIt->currentYolo.footX, 2) + std::pow(lidarIt->currentLidar.footY - yoloIt->currentYolo.footY, 2);
                cv::Point2f futurePredictPoint;
                yoloIt->kalmanYolo.futurePredict(futurePredictPoint, 4);
                double distance = std::pow(lidarIt->currentLidar.footX - futurePredictPoint.x, 2) + std::pow(lidarIt->currentLidar.footY - yoloIt->currentYolo.footY, 2);
                if (distance < SQUARED_EUCLIDEAN_MATCHING_THRESHOLD)
                {
                    newMatchedTrackers.emplace_back(*lidarIt, *yoloIt);

                    // マッチングされたトラッカーは未マッチのトラッカーのリストから削除する
                    std::erase_if(unmatchedLidarTrackers, [matchedLidarID](const tkr::LidarTracker &tracker)
                                  { return tracker.id == matchedLidarID; });
                    std::erase_if(unmatchedYoloTrackers, [matchedYoloID](const tkr::YoloTracker &tracker)
                                  { return tracker.id == matchedYoloID; });
                }
            }
        }

        int nLidars = unmatchedLidarTrackers.size();
        int mYolos = unmatchedYoloTrackers.size();
        // もし、未マッチのトラッカーのどちらかが空の場合、マッチングされたトラッカーを更新して終了
        if (nLidars == 0 || mYolos == 0)
        {
            matchedTrackers.clear();
            matchedTrackers = newMatchedTrackers;
            return;
        }

        // 未マッチのトラッカーが両方とも存在する場合、コスト行列を作成してハンガリー法で最適なマッチングを求める
        std::vector<std::vector<double>> costMatrix(nLidars, std::vector<double>(mYolos));
        for (int row = 0; row < nLidars; row++)
        {
            for (int col = 0; col < mYolos; col++)
            {
                // costMatrix.at(row).at(col) = static_cast<double>(std::pow(unmatchedLidarTrackers.at(row).currentLidar.footX - unmatchedYoloTrackers.at(col).currentYolo.footX, 2) +
                //                                                  std::pow(unmatchedLidarTrackers.at(row).currentLidar.footY - unmatchedYoloTrackers.at(col).currentYolo.footY, 2));
                cv::Point2f futurePredictPoint;
                unmatchedYoloTrackers.at(col).kalmanYolo.futurePredict(futurePredictPoint, 4);
                costMatrix.at(row).at(col) = static_cast<double>(std::pow(unmatchedLidarTrackers.at(row).currentLidar.footX - futurePredictPoint.x, 2) +
                                                                 std::pow(unmatchedLidarTrackers.at(row).currentLidar.footY - unmatchedYoloTrackers.at(col).currentYolo.footY, 2));
            }
        }

        std::vector<int> assignment;
        Hungarian::Solve(costMatrix, assignment);

        std::vector<bool> yolo_used(mYolos, false);
        for (int row = 0; row < nLidars; row++)
        {
            int col = assignment.at(row);
            if (col >= 0 && costMatrix.at(row).at(col) < SQUARED_EUCLIDEAN_MATCHING_THRESHOLD)
            {
                newMatchedTrackers.emplace_back(unmatchedLidarTrackers.at(row), unmatchedYoloTrackers.at(col));
                yolo_used.at(col) = true;
            }
        }

        // マッチングされたトラッカーのリストを更新する
        matchedTrackers.clear();
        matchedTrackers = newMatchedTrackers;
    }

    std::vector<MatchedTracker>::const_iterator getOtherMatchedTrackerIteratorFromLidarID(const int lidarID, const std::vector<mtc::MatchedTracker> &matchedTracker)
    {
        return std::find_if(matchedTracker.begin(), matchedTracker.end(),
                            [lidarID](const MatchedTracker &tracker)
                            { return tracker.lidarTracker.id == lidarID; });
    }
}