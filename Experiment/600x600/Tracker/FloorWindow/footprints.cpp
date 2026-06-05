// ----------------------------------------------------------------
// footprints.cpp
//
// フロアウィンドウに足跡を描画するためのクラスと関数の実装
//
// Copyright (c) 2026 Maito Yasui
// Released under the GNU GPL-3.0 License.
// https://github.com/Nibosi0501/LiViMotion/blob/main/LICENSE
// ----------------------------------------------------------------

#include "./Footprints.hpp"

namespace win
{
    Human::Human(const int _lidarID)
    {
        lidarID = _lidarID;
    }

    void updateHumans(const std::vector<tkr::LidarTracker> &lidarTrackers, std::vector<Human> &humans)
    {
        // lidarTrackersに存在するlidarIDを持つHumanがhumansに存在しない場合はhumansに追加
        for (const auto &tracker : lidarTrackers)
        {
            if (tracker.skippedFrames > 5)
            {
                continue;
            }
            auto it = std::find_if(humans.begin(), humans.end(), [&tracker](const Human &human)
                                   { return human.lidarID == tracker.id; });
            if (it == humans.end())
            {
                humans.emplace_back(tracker.id);
            }
        }

        // humansに存在するlidarIDがlidarTrackersに存在しない場合はhumansから削除
        std::erase_if(humans, [&lidarTrackers](const Human &human)
                      { return std::none_of(lidarTrackers.begin(), lidarTrackers.end(), [&human](const tkr::LidarTracker &tracker)
                                            { return tracker.id == human.lidarID; }); });
    }

    void changeFootPrint(std::vector<Human> &humans, const std::vector<mtc::MatchedTracker> &matchedTrackers1, const std::vector<mtc::MatchedTracker> &matchedTrackers2, std::vector<FootPrint> &footPrints)
    {
        if (humans.empty() || (matchedTrackers1.empty() && matchedTrackers2.empty()))
        {
            return;
        }

        for (auto &human : humans)
        {
            int lidarID = human.lidarID;
            auto matchedIt1 = std::find_if(matchedTrackers1.begin(), matchedTrackers1.end(),
                                           [lidarID](const mtc::MatchedTracker &tracker)
                                           { return tracker.lidarTracker.id == lidarID; });
            auto matchedIt2 = std::find_if(matchedTrackers2.begin(), matchedTrackers2.end(),
                                           [lidarID](const mtc::MatchedTracker &tracker)
                                           { return tracker.lidarTracker.id == lidarID; });
            if (matchedIt1 == matchedTrackers1.end() && matchedIt2 == matchedTrackers2.end())
            {
                continue;
            }

            if (matchedIt1 != matchedTrackers1.end() && matchedIt2 != matchedTrackers2.end())
            {
                // 両方のYoloTrackerを見てテクスチャIDを決定
            }
            else if (matchedIt1 != matchedTrackers1.end())
            {
                // matchedIt1のYoloTrackerを見てテクスチャIDを決定
                bool isLeft = matchedIt1->yoloTracker.currentYolo.isLeftHandUp(0.5f);
                bool isRight = matchedIt1->yoloTracker.currentYolo.isRightHandUp(0.5f);
                if (isLeft && isRight)
                {
                    auto now = std::chrono::steady_clock::now();
                    double elasped = std::chrono::duration<double>(now - human.lastcontrolTime).count();
                    if (elasped < 1.0)
                    {
                        continue;
                    }

                    human.printTextureID = 2;
                    human.stamp(matchedIt1->lidarTracker.currentLidar, footPrints);

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
                    human.stamp(matchedIt1->lidarTracker.currentLidar, footPrints);

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
                    human.stamp(matchedIt1->lidarTracker.currentLidar, footPrints);

                    human.lastcontrolTime = std::chrono::steady_clock::now();
                }
            }
            else if (matchedIt2 != matchedTrackers2.end())
            {
                // matchedIt2のYoloTrackerを見てテクスチャIDを決定
            }
        }
    }

    void win::Human::stamp(const lidar::Lidar &lidar,
                           std::vector<FootPrint> &footPrints)
    {
        float angle = angles.at(angleDis(gen));

        int randomIndex = dis(gen);
        footPrints.emplace_back(lidar, angle, printTextureID, footPrintColors.at(randomIndex));
    }

    FootPrint::FootPrint(const lidar::Lidar &lidar, const float _angle, const int _printTextureID, const std::tuple<double, double, double> &color)
    {
        centerX = lidar.footX;
        centerY = lidar.footY;
        angle = _angle;
        this->printTextureID = _printTextureID;
        this->color = color;
    }

    void FootPrint::update()
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

    void updateFootPrints(std::vector<FootPrint> &footPrints)
    {
        for (auto &footPrint : footPrints)
        {
            footPrint.update();
        }
    }

    void deleteFootPrints(std::vector<FootPrint> &footPrints)
    {
        // アルファ値が0になった足跡を削除
        std::erase_if(footPrints, [](const FootPrint &footPrint)
                      { return footPrint.alpha <= 0.0f; });
    }
}