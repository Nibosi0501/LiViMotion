// ----------------------------------------------------------------
// Kalman.hpp
//
// カルマンフィルタのためのヘッダーファイル
//
// Copyright (c) 2026 Maito Yasui
// Released under the GNU GPL-3.0 License.
// https://github.com/Nibosi0501/LiViMotion/blob/main/LICENSE
// ----------------------------------------------------------------

#pragma once

#include <opencv2/opencv.hpp>

#include "../Yolo/Yolo.hpp"

namespace kf
{
    struct KalmanYolo
    {
        cv::KalmanFilter kf;        // OpenCVのカルマンフィルタクラス
        cv::Point2f predictedPoint; // 予測された位置

        KalmanYolo(const yolo::Yolo &observedYolo);
        void update(const yolo::Yolo &observedYolo); // 観測値を用いてカルマンフィルタを更新
        void predict();                              // 観測なしの予測ステップ

        // 将来の予測ステップ (numFrames フレーム先を予測)
        void futurePredict(cv::Point2f &futurePredictPoint, const int numFrames) const;
    };
}