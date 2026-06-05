// ----------------------------------------------------------------
// Calibration.hpp
//
// カメラキャリブレーションのためのクラス定義
//
// Copyright (c) 2026 Maito Yasui
// Released under the GNU GPL-3.0 License.
// https://github.com/Nibosi0501/LiViMotion/blob/main/LICENSE
// ----------------------------------------------------------------

#pragma once

#include <opencv2/opencv.hpp>

namespace clb
{
    struct Quad
    {
        cv::Point2f topLeft;
        cv::Point2f topRight;
        cv::Point2f bottomLeft;
        cv::Point2f bottomRight;
    };

    struct Calibration
    {
        std::vector<std::vector<Quad>> calibrationQuads;
        std::vector<std::vector<Quad>> transformQuads;
        int cols;
        int rows;
        int windowWidth;
        int windowHeight;
        std::string attribute;
        Calibration(const std::string &filename, const int _cols, const int _rows, const int _windowWidth, const int _windowHeight, const std::string &_attribute);

        bool isPointInQuad(const cv::Point2f &targetPoint, const Quad &quad) const;
        bool transformPoint(const cv::Point2f &targetPoint, cv::Point2f &outPoint) const;

        // 確認描画用の関数
        void transformImage(const cv::Mat &inputImage, cv::Mat &outImage) const;
    };
}