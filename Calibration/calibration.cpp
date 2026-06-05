// ----------------------------------------------------------------
// calibration.cpp
//
// カメラキャリブレーションのためのクラス実装
//
// Copyright (c) 2026 Maito Yasui
// Released under the GNU GPL-3.0 License.
// https://github.com/Nibosi0501/LiViMotion/blob/main/LICENSE
// ----------------------------------------------------------------

#include <fstream>
#include <iostream>

#include "./Calibration.hpp"

namespace clb
{
    Calibration::Calibration(const std::string &filename, const int _cols, const int _rows, const int _windowWidth, const int _windowHeight, const std::string &_attribute)
        : cols(_cols), rows(_rows), windowWidth(_windowWidth), windowHeight(_windowHeight), attribute(_attribute)
    {
        calibrationQuads.resize(rows - 1);
        transformQuads.resize(rows - 1);
        for (int i = 0; i < rows - 1; i++)
        {
            calibrationQuads.at(i).resize(cols - 1);
            transformQuads.at(i).resize(cols - 1);
        }

        std::ifstream file(filename);
        if (!file.is_open())
        {
            std::cerr << "キャリブレーションファイルが開けません: " << filename << std::endl;
            std::cerr << "キャリブレーションファイルのパスは、一連のプログラムを実行するmain()関数のあるプログラムがあるディレクトリからの相対パスである必要があります。" << std::endl;
            std::cerr << "main()関数がないプログラムでclb::Calibration calib()する際にも、指定するファイルのパスは、main()関数があるプログラムからの相対パスである必要があります。" << std::endl;
            std::cerr << "ただし、#include の際には、Calibration.hppをincludeするプログラムからの相対パスで指定してください。" << std::endl;
            std::cerr << "また、コンパイルコマンドに指定するcalibration.cppのパスは、main()関数があるプログラムからの相対パスで指定してください。" << std::endl;
            return;
        }

        std::vector<std::vector<cv::Point2f>> gridPoints(rows, std::vector<cv::Point2f>(cols));
        for (int row = 0; row < rows; row++)
        {
            for (int col = 0; col < cols; col++)
            {
                file >> gridPoints[row][col].x >> gridPoints[row][col].y;
            }
        }
        // グリッド点から四角形を構成する
        for (int row = 0; row < rows - 1; row++)
        {
            for (int col = 0; col < cols - 1; col++)
            {
                calibrationQuads[row][col].topLeft = gridPoints[row][col];
                calibrationQuads[row][col].topRight = gridPoints[row][col + 1];
                calibrationQuads[row][col].bottomRight = gridPoints[row + 1][col + 1];
                calibrationQuads[row][col].bottomLeft = gridPoints[row + 1][col];
            }
        }

        // キャリブレーション後の四角形座標を生成
        Quad transformQuad;
        for (int row = 0; row < rows - 1; row++)
        {
            for (int col = 0; col < cols - 1; col++)
            {
                float x0 = (static_cast<float>(windowWidth) / (cols - 1)) * col;
                float y0 = (static_cast<float>(windowHeight) / (rows - 1)) * row;
                float x1 = (static_cast<float>(windowWidth) / (cols - 1)) * (col + 1);
                float y1 = (static_cast<float>(windowHeight) / (rows - 1)) * row;
                float x2 = (static_cast<float>(windowWidth) / (cols - 1)) * (col + 1);
                float y2 = (static_cast<float>(windowHeight) / (rows - 1)) * (row + 1);
                float x3 = (static_cast<float>(windowWidth) / (cols - 1)) * col;
                float y3 = (static_cast<float>(windowHeight) / (rows - 1)) * (row + 1);

                transformQuad.topLeft = cv::Point2f(x0, y0);
                transformQuad.topRight = cv::Point2f(x1, y1);
                transformQuad.bottomRight = cv::Point2f(x2, y2);
                transformQuad.bottomLeft = cv::Point2f(x3, y3);
                transformQuads.at(row).at(col) = transformQuad;
            }
        }
    }

    bool Calibration::isPointInQuad(const cv::Point2f &targetPoint, const Quad &quad) const
    {
        std::vector<cv::Point2f> quadPoints = {
            cv::Point2f(quad.topLeft),
            cv::Point2f(quad.topRight),
            cv::Point2f(quad.bottomRight),
            cv::Point2f(quad.bottomLeft)};

        int result = cv::pointPolygonTest(quadPoints, targetPoint, false);

        if (result >= 0)
        {
            // 点は四角形内または境界線上にある
            return true;
        }
        else
        {
            // 点は四角形外にある
            return false;
        }
    }

    bool Calibration::transformPoint(const cv::Point2f &targetPoint, cv::Point2f &outPoint) const
    {
        for (int row = 0; row < rows - 1; row++)
        {
            for (int col = 0; col < cols - 1; col++)
            {
                if (isPointInQuad(targetPoint, calibrationQuads.at(row).at(col)))
                {
                    std::vector<cv::Point2f> calibrationPoints = {
                        cv::Point2f(calibrationQuads.at(row).at(col).topLeft),
                        cv::Point2f(calibrationQuads.at(row).at(col).topRight),
                        cv::Point2f(calibrationQuads.at(row).at(col).bottomRight),
                        cv::Point2f(calibrationQuads.at(row).at(col).bottomLeft)};
                    std::vector<cv::Point2f> transformPoints = {
                        cv::Point2f(transformQuads.at(row).at(col).topLeft),
                        cv::Point2f(transformQuads.at(row).at(col).topRight),
                        cv::Point2f(transformQuads.at(row).at(col).bottomRight),
                        cv::Point2f(transformQuads.at(row).at(col).bottomLeft)};

                    cv::Mat Homography = cv::getPerspectiveTransform(calibrationPoints, transformPoints);

                    std::vector<cv::Point2f> calibrationPointVec(1);
                    calibrationPointVec.at(0) = targetPoint;
                    std::vector<cv::Point2f> transformPointVec(1);
                    cv::perspectiveTransform(calibrationPointVec, transformPointVec, Homography);
                    outPoint = transformPointVec.at(0);
                    return true;
                }
            }
        }
        return false; // どの四角形にも属さない場合は変換できない
    }

    void Calibration::transformImage(const cv::Mat &inputImage, cv::Mat &outImage) const
    {
        for (int row = 0; row < rows - 1; row++)
        {
            for (int col = 0; col < cols - 1; col++)
            {
                std::vector<cv::Point2f> calibrationPoints = {
                    cv::Point2f(calibrationQuads.at(row).at(col).topLeft),
                    cv::Point2f(calibrationQuads.at(row).at(col).topRight),
                    cv::Point2f(calibrationQuads.at(row).at(col).bottomRight),
                    cv::Point2f(calibrationQuads.at(row).at(col).bottomLeft)};
                std::vector<cv::Point2f> transformPoints = {
                    cv::Point2f(transformQuads.at(row).at(col).topLeft),
                    cv::Point2f(transformQuads.at(row).at(col).topRight),
                    cv::Point2f(transformQuads.at(row).at(col).bottomRight),
                    cv::Point2f(transformQuads.at(row).at(col).bottomLeft)};

                cv::Mat Homography = cv::getPerspectiveTransform(calibrationPoints, transformPoints);
                cv::Mat warpedImage;
                cv::warpPerspective(inputImage, warpedImage, Homography, outImage.size(), cv::INTER_LINEAR, cv::BORDER_CONSTANT);

                // マスクを作成して重ね合わせ
                cv::Mat mask = cv::Mat::zeros(outImage.size(), CV_8UC1);
                std::vector<std::vector<cv::Point>> fillContAll;
                std::vector<cv::Point> fillCont;
                fillCont.push_back(transformPoints[0]);
                fillCont.push_back(transformPoints[1]);
                fillCont.push_back(transformPoints[2]);
                fillCont.push_back(transformPoints[3]);
                fillContAll.push_back(fillCont);
                cv::fillPoly(mask, fillContAll, cv::Scalar(255));
                warpedImage.copyTo(outImage, mask);
            }
        }
    }
}