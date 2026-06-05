// ----------------------------------------------------------------
// Yolo.hpp
//
// Yoloクラスの定義と関連する関数の宣言
//
// Copyright (c) 2026 Maito Yasui
// Released under the GNU GPL-3.0 License.
// https://github.com/Nibosi0501/LiViMotion/blob/main/LICENSE
// ----------------------------------------------------------------

#pragma once

#include <vector>

#include "../../../Calibration/Calibration.hpp"

namespace yolo
{
    struct Yolo
    {
        float footX;
        float footY;
        float footConf;
        std::vector<std::tuple<float, float, float>> keypoints;

        Yolo(const std::vector<std::tuple<float, float, float>> &_keypoints, const clb::Calibration &calibration);
        bool isLeftHandUp(const float conf) const;
        bool isRightHandUp(const float conf) const;

    private:
        void getFoot(float &x, float &y, float &conf) const;
        float mapValue(const float x, const float in_min, const float in_max, const float out_min, const float out_max) const;
    };

    bool isLeftHandUp(const std::vector<std::tuple<float, float, float>> &keypoints1, const std::vector<std::tuple<float, float, float>> &keypoints2, const float conf);
    bool isRightHandUp(const std::vector<std::tuple<float, float, float>> &keypoints1, const std::vector<std::tuple<float, float, float>> &keypoints2, const float conf);

    bool isRightHandUpInHistory(const std::deque<Yolo> &historys, const float conf);
    bool isRightHandUpInHistory(const std::deque<Yolo> &historys1, const std::deque<Yolo> &historys2, const float conf);
}