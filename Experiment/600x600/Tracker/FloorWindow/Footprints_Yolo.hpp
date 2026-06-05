// ----------------------------------------------------------------
// footprints_Yolo.hpp
//
// フロアウィンドウに足跡を描画するためのクラスと関数の宣言
//
// Copyright (c) 2026 Maito Yasui
// Released under the GNU GPL-3.0 License.
// https://github.com/Nibosi0501/LiViMotion/blob/main/LICENSE
// ----------------------------------------------------------------

#pragma once

#include "../../Yolo/Yolo.hpp"
#include "../Tracker.hpp"

#include <chrono>
#include <vector>
#include <random>

namespace win
{
    inline std::vector<std::tuple<double, double, double>> footPrintColors = {
        {1.0, 1.0, 1.0},
        {1.0, 1.0, 0.0},
        {1.0, 0.0, 1.0},
        {0.0, 1.0, 1.0},
        {1.0, 0.5, 0.5},
        {0.5, 1.0, 0.5},
        {0.5, 0.5, 1.0}};

    inline std::random_device rd;
    inline std::mt19937 gen(rd());
    inline std::uniform_int_distribution<> dis(0, 6);

    inline std::vector<float> angles = {0.0f, 30.0f, 60.0f, 90.0f, 120.0f, 150.0f, 180.0f};
    inline std::uniform_int_distribution<> angleDis(0, 6);

    struct FootPrintYolo
    {
        float centerX = 0.0f;
        float centerY = -1.0f;
        float angle = 0.0f;
        float size = 0.0f;
        float alpha = 1.5f;
        std::tuple<double, double, double> color = {1.0, 1.0, 1.0};
        int printTextureID = 0;
        std::chrono::steady_clock::time_point createdTime;

        FootPrintYolo(const tkr::YoloTracker &yoloTracker, float _angle, int _printTextureID, const std::tuple<double, double, double> &_color);

        void update();
    };

    void updateFootPrints(std::vector<FootPrintYolo> &footPrints);
    void deleteFootPrints(std::vector<FootPrintYolo> &footPrints);

    struct HumanYolo
    {
        int yoloID = -1;
        int printTextureID = 0;
        int printSE_ID = -1;

        // 前回スタンプ位置（変化検知用）
        float lastStampX = 0.0f;
        float lastStampY = 0.0f;
        float stepThreshold = 30.0f; // 何px進んだらスタンプするか

        std::chrono::steady_clock::time_point lastcontrolTime;

        HumanYolo(const int _yoloID);

        // footPrintsに追記するだけ。以降の管理はしない
        void stamp(const tkr::YoloTracker &yoloTracker,
                   std::vector<FootPrintYolo> &footPrints);
    };

    void updateHumans(const std::vector<tkr::YoloTracker> &yoloTrackers, std::vector<HumanYolo> &humans);

    void changeFootPrint(std::vector<HumanYolo> &humans, const std::vector<tkr::YoloTracker> &yoloTrackers, std::vector<FootPrintYolo> &footPrints);
}