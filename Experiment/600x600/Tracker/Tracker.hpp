// ----------------------------------------------------------------
// tracker.hpp
//
// ユーザの足位置を追跡するためのクラスと関数の宣言
//
// Copyright (c) 2026 Maito Yasui
// Released under the GNU GPL-3.0 License.
// https://github.com/Nibosi0501/LiViMotion/blob/main/LICENSE
// ----------------------------------------------------------------

#pragma once
#include "../Yolo/Yolo.hpp"
#include "../Lidar/Lidar.hpp"

#include "../Kalman/Kalman.hpp"

#include <chrono>

namespace tkr
{
    constexpr int MAX_HISTORY_SIZE = 5;
    constexpr int MAX_SKIPPED_FRAMES = 30;
    constexpr double EUCLIDEAN_LIDAR_THRESHOLD = 130.0;
    constexpr double EUCLIDEAN_YOLO_THRESHOLD = 130.0;
    constexpr double SQUARED_EUCLIDEAN_LIDAR_THRESHOLD = EUCLIDEAN_LIDAR_THRESHOLD * EUCLIDEAN_LIDAR_THRESHOLD;
    constexpr double SQUARED_EUCLIDEAN_YOLO_THRESHOLD = EUCLIDEAN_YOLO_THRESHOLD * EUCLIDEAN_YOLO_THRESHOLD;

    struct IDGenerator
    {
        int nextID = 0;
        int generate()
        {
            int id = nextID++;
            if (nextID > 10000)
                nextID = 0;
            return id;
        }
    };

    struct LidarTracker
    {
        int id;
        int skippedFrames;
        lidar::Lidar currentLidar;
        std::deque<lidar::Lidar> historys;

        // std::chrono::steady_clock::time_point lastAddHistoryTime;

        LidarTracker(IDGenerator &idGen, const lidar::Lidar &_currentLidar, const int &_skippedFrames = 0, const std::deque<lidar::Lidar> &historys = std::deque<lidar::Lidar>());
        void addHistory(const lidar::Lidar &lidar);
    };

    void updateTrackers(std::vector<tkr::LidarTracker> &lidarTrackers, const std::vector<lidar::Lidar> &currentLidars, IDGenerator &idGen);
    void deleteLostTrackers(std::vector<tkr::LidarTracker> &lidarTrackers);

    struct YoloTracker
    {
        int id;
        int skippedFrames;
        yolo::Yolo currentYolo;
        std::deque<yolo::Yolo> historys;

        kf::KalmanYolo kalmanYolo;

        YoloTracker(IDGenerator &idGen, const yolo::Yolo &_currentYolo, const int &_skippedFrames = 0, const std::deque<yolo::Yolo> &_historys = std::deque<yolo::Yolo>());
        void addHistory(const yolo::Yolo &yolo);
    };

    void updateTrackers(std::vector<tkr::YoloTracker> &yoloTrackers, const std::vector<yolo::Yolo> &currentYolos, IDGenerator &idGen);
    void deleteLostTrackers(std::vector<tkr::YoloTracker> &yoloTrackers);
}