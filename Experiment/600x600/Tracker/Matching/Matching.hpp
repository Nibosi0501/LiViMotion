// ----------------------------------------------------------------
// matching.hpp
//
// LiDARトラッカーとYOLOトラッカーのマッチングを行う関数の宣言
//
// Copyright (c) 2026 Maito Yasui
// Released under the GNU GPL-3.0 License.
// https://github.com/Nibosi0501/LiViMotion/blob/main/LICENSE
// ----------------------------------------------------------------

#pragma once

#include "../Tracker.hpp"

namespace mtc
{
    struct MatchedTracker
    {
        float footX;
        float footY;

        tkr::LidarTracker lidarTracker;
        tkr::YoloTracker yoloTracker;

        MatchedTracker(const tkr::LidarTracker &_lidarTracker, const tkr::YoloTracker &_yoloTracker);
    };

    void updateMatchedTrackers(std::vector<mtc::MatchedTracker> &matchedTrackers, const std::vector<tkr::LidarTracker> &lidarTrackers, const std::vector<tkr::YoloTracker> &yoloTrackers);

    std::vector<MatchedTracker>::const_iterator getOtherMatchedTrackerIteratorFromLidarID(const int lidarID, const std::vector<mtc::MatchedTracker> &matchedTrackers);
}