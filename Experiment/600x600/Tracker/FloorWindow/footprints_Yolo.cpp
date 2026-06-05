// ----------------------------------------------------------------
// footprints_Yolo.cpp
//
// フロアウィンドウに足跡を描画するためのクラスと関数の実装
//
// Copyright (c) 2026 Maito Yasui
// Released under the GNU GPL-3.0 License.
// https://github.com/Nibosi0501/LiViMotion/blob/main/LICENSE
// ----------------------------------------------------------------

#include "./Footprints_Yolo.hpp"

namespace win
{
    HumanYolo::HumanYolo(const int _yoloID)
    {
        yoloID = _yoloID;
    }

    void updateHumans(const std::vector<tkr::YoloTracker> &yoloTrackers, std::vector<HumanYolo> &humans)
    {
        // yoloTrackersに存在するyoloIDを持つHumanがhumansに存在しない場合はhumansに追加
        for (const auto &tracker : yoloTrackers)
        {
            if (tracker.skippedFrames > 5)
            {
                continue;
            }
            auto it = std::find_if(humans.begin(), humans.end(), [&tracker](const HumanYolo &human)
                                   { return human.yoloID == tracker.id; });
            if (it == humans.end())
            {
                humans.emplace_back(tracker.id);
            }
        }

        // humansに存在するyoloIDがyoloTrackersに存在しない場合はhumansから削除
        std::erase_if(humans, [&yoloTrackers](const HumanYolo &human)
                      { return std::none_of(yoloTrackers.begin(), yoloTrackers.end(), [&human](const tkr::YoloTracker &tracker)
                                            { return tracker.id == human.yoloID; }); });
    }

    void changeFootPrint(std::vector<HumanYolo> &humans, const std::vector<tkr::YoloTracker> &yoloTrackers, std::vector<FootPrintYolo> &footPrints)
    {
        if (humans.empty() || yoloTrackers.empty())
        {
            return;
        }

        for (auto &human : humans)
        {
            int yoloID = human.yoloID;
            auto yoloIt = std::find_if(yoloTrackers.begin(), yoloTrackers.end(),
                                       [yoloID](const tkr::YoloTracker &tracker)
                                       { return tracker.id == yoloID; });
            if (yoloIt == yoloTrackers.end())
            {
                continue;
            }

            if (yoloIt != yoloTrackers.end())
            {
                // yoloItのYoloTrackerを見てテクスチャIDを決定
                bool isLeft = yoloIt->currentYolo.isLeftHandUp(0.5f);
                bool isRight = yoloIt->currentYolo.isRightHandUp(0.5f);
                if (isLeft && isRight)
                {
                    auto now = std::chrono::steady_clock::now();
                    double elasped = std::chrono::duration<double>(now - human.lastcontrolTime).count();
                    if (elasped < 1.0)
                    {
                        continue;
                    }

                    human.printTextureID = 2;
                    human.stamp(*yoloIt, footPrints);

                    human.lastcontrolTime = std::chrono::steady_clock::now();
                }
                else if (isLeft)
                {
                    auto now = std::chrono::steady_clock::now();
                    double elasped = std::chrono::duration<double>(now - human.lastcontrolTime).count();
                    if (elasped < 1.0)
                    {
                        continue;
                    }

                    human.printTextureID = 0;
                    human.stamp(*yoloIt, footPrints);

                    human.lastcontrolTime = std::chrono::steady_clock::now();
                }
                else if (isRight)
                {
                    auto now = std::chrono::steady_clock::now();
                    double elasped = std::chrono::duration<double>(now - human.lastcontrolTime).count();
                    if (elasped < 1.0)
                    {
                        continue;
                    }

                    human.printTextureID = 1;
                    human.stamp(*yoloIt, footPrints);

                    human.lastcontrolTime = std::chrono::steady_clock::now();
                }
            }
        }
    }

    void win::HumanYolo::stamp(const tkr::YoloTracker &yoloTracker,
                               std::vector<FootPrintYolo> &footPrints)
    {
        cv::Point2f predictedPoint;
        yoloTracker.kalmanYolo.futurePredict(predictedPoint, 10);

        float angle = angles.at(angleDis(gen));

        int randomIndex = dis(gen);

        footPrints.emplace_back(yoloTracker, angle, printTextureID, footPrintColors.at(randomIndex));
    }

    FootPrintYolo::FootPrintYolo(const tkr::YoloTracker &yoloTracker, const float _angle, const int _printTextureID, const std::tuple<double, double, double> &color)
    {
        // Yolo生データ
        centerX = yoloTracker.currentYolo.footX;
        centerY = yoloTracker.currentYolo.footY;

        // KalmanFilter予測値
        // centerX = yoloTracker.kalmanYolo.predictedPoint.x;
        // centerY = yoloTracker.kalmanYolo.predictedPoint.y;

        // kalmanYoloの任意のフレーム先の予測点を描画
        // cv::Point2f futurePredictPoint;
        // yoloTracker.kalmanYolo.futurePredict(futurePredictPoint, 10);
        // centerX = futurePredictPoint.x;
        // centerY = futurePredictPoint.y;

        angle = _angle;
        this->printTextureID = _printTextureID;
        this->color = color;
    }

    void FootPrintYolo::update()
    {
        // 足跡のサイズを時間経過とともに増加させる
        size += 4.0f;
        alpha -= 0.015f;

        angle += 1.0f;
        if (angle >= 360.0f)
        {
            angle = 0.0f;
        }
    }

    void updateFootPrints(std::vector<FootPrintYolo> &footPrints)
    {
        for (auto &footPrint : footPrints)
        {
            footPrint.update();
        }
    }

    void deleteFootPrints(std::vector<FootPrintYolo> &footPrints)
    {
        // アルファ値が0になった足跡を削除
        std::erase_if(footPrints, [](const FootPrintYolo &footPrint)
                      { return footPrint.alpha <= 0.0f; });
    }
}