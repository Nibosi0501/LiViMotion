/**
 * @file calibration.cpp
 * @brief clb::Calibration の実装
 * @see Calibration.hpp
 * 
 * Copyright (c) 2026 Maito Yasui
 * Released under the GNU AGPL-3.0 License.
 * https://github.com/Nibosi0501/LiViMotion/blob/main/LICENSE
 */
#include <fstream>
#include <iostream>

#include "../../include/calibration/Calibration.hpp"

namespace clb
{
    /**
     * @details
     * 処理の流れ:
     * -# calibrationQuads / transformQuads を (rows - 1) × (cols - 1) にリサイズする
     * -# ファイルから rows × cols 個のグリッド点 (x y) を行優先で読み込む
     * -# 隣り合う 4 点から calibrationQuads を構成する
     * -# ウィンドウを (cols - 1) × (rows - 1) 等分した格子から transformQuads を生成する
     *
     * ファイルが開けない場合はエラーを出力して return する (四角形は初期値のまま)。
     */
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
            std::cerr << "キャリブレーションファイルのパスは、実行ファイル(a.out)があるディレクトリからの相対パスである必要があります。" << std::endl;
            std::cerr << "main()関数がないプログラムでclb::Calibration calib()する際にも、指定するファイルのパスは、実行ファイルからの相対パスである必要があります。" << std::endl;
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

    /**
     * @details
     * 四角形の 4 頂点を多角形として cv::pointPolygonTest() に渡して判定する。
     * 距離計算は行わず (measureDist = false)、内外のみを符号で判定する。
     * 戻り値は 内部: +1、境界上: 0、外部: -1 なので、0 以上なら「含まれる」と扱う。
     *
     * @param[in] targetPoint 判定する点
     * @param[in] quad        判定対象の四角形
     * @retval true  内部または境界線上
     * @retval false 外部
     */
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

    /**
     * @details
     * 全四角形を行優先で走査し、targetPoint を含む最初の四角形を見つける。
     * その四角形の calibrationQuads → transformQuads のホモグラフィを
     * cv::getPerspectiveTransform() で求め、cv::perspectiveTransform() で点を変換する。
     *
     * @param[in]  targetPoint 変換する点 (計測座標系)
     * @param[out] outPoint    変換後の点 (ウィンドウ座標系)
     * @retval true  変換に成功
     * @retval false どの四角形にも含まれない
     */
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

    /**
     * @details
     * 四角形ごとに以下を繰り返す:
     * -# ホモグラフィを求め、入力画像全体を outImage と同じサイズへ射影変換する
     * -# transformQuads の領域だけを白で塗ったマスクを作る
     * -# マスク領域のみ outImage にコピーして重ね合わせる
     *
     * @param[in]     inputImage 変換元の画像 (計測座標系)
     * @param[in,out] outImage   出力先。呼び出し前にサイズを確保しておくこと
     */
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
