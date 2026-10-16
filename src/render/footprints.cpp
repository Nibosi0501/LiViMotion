/**
 * @file footprints.cpp
 * @brief 足跡エフェクト (FootPrint) と、足跡を出す人 (Human) の処理の実装
 * @see Footprints.hpp
 *
 * Copyright (c) 2026 Maito Yasui
 * Released under the GNU AGPL-3.0 License.
 * https://github.com/Nibosi0501/LiViMotion/blob/main/LICENSE
 */
#include "../../include/render/Footprints.hpp"

// #include "../../Save/Save.hpp"

// #include <fstream>

namespace win
{
    /**
     * @details lidarID のみを設定する。それ以外のメンバは宣言時の初期値のままになる。
     */
    Human::Human(const int _lidarID)
    {
        lidarID = _lidarID;
    }

    /**
     * @details
     * 処理の流れ:
     * -# 見失ったフレーム数 (skippedFrames) が 5 を超えるトラッカーを除き、
     *    対応する Human がないトラッカーの ID で Human を追加する
     * -# 対応するトラッカーが 1 つも存在しない Human を削除する
     *    (追加時は skippedFrames を見るが、削除時は見ないため、skippedFrames が 5 を超えても
     *    トラッカーが残っている間は Human は残る)
     *
     * @param[in]     lidarTrackers LiDAR トラッカーの一覧
     * @param[in,out] humans        Human の一覧
     */
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

    /**
     * @details
     * Human が空、または 2 系統のマッチング結果がどちらも空なら、何もせず戻る。
     *
     * 各 Human について、次の処理を行う。
     * -# lidarID を持つマッチング済みトラッカーを、2 系統から探す。どちらにもなければ次の Human へ進む
     * -# 見つかり方に応じて、手の挙げ方を判定する (信頼度の閾値はすべて 0.6)
     *    - 両方に見つかった: yolo::isLeftHandUp() / yolo::isRightHandUp() の 2 組版で判定する
     *    - 片方だけ見つかった: その YOLO の最新の検出結果 (currentYolo) で判定する
     * -# 手の挙げ方でテクスチャの種類を決める (両手: 2、左手のみ: 0、右手のみ: 1)。
     *    どちらの手も挙がっていなければ何もしない
     * -# 前回の足跡から 2 秒未満なら、次の Human へ進む (クールダウン)
     * -# printTextureID を設定して stamp() で足跡を追加し、lastcontrolTime を現在時刻に更新する
     *
     * 足跡の位置は、見つかったマッチング済みトラッカーの LiDAR の最新位置を使う
     * (両方に見つかった場合は 1 系統目のもの)。
     *
     * @note 3 つの分岐 (両方・1 系統目のみ・2 系統目のみ) は、判定の入力元以外はほぼ同じ処理になっている。
     *
     * @param[in,out] humans            Human の一覧
     * @param[in]     matchedTrackers1  1 系統目のマッチング済みトラッカー
     * @param[in]     matchedTrackers2  2 系統目のマッチング済みトラッカー
     * @param[in,out] footPrints        足跡の一覧 (追加される)
     */
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
                bool isLeft = yolo::isLeftHandUp(matchedIt1->yoloTracker.currentYolo.keypoints, matchedIt2->yoloTracker.currentYolo.keypoints, 0.6f);
                bool isRight = yolo::isRightHandUp(matchedIt1->yoloTracker.currentYolo.keypoints, matchedIt2->yoloTracker.currentYolo.keypoints, 0.6f);
                if (isLeft && isRight)
                {
                    auto now = std::chrono::steady_clock::now();
                    double elasped = std::chrono::duration<double>(now - human.lastcontrolTime).count();
                    if (elasped < 2.0)
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
                    if (elasped < 2.0)
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
                    if (elasped < 2.0)
                    {
                        continue;
                    }

                    human.printTextureID = 1;
                    human.stamp(matchedIt1->lidarTracker.currentLidar, footPrints);

                    human.lastcontrolTime = std::chrono::steady_clock::now();
                }
            }
            else if (matchedIt1 != matchedTrackers1.end())
            {
                // matchedIt1のYoloTrackerを見てテクスチャIDを決定
                bool isLeft = matchedIt1->yoloTracker.currentYolo.isLeftHandUp(0.6f);
                bool isRight = matchedIt1->yoloTracker.currentYolo.isRightHandUp(0.6f);
                if (isLeft && isRight)
                {
                    auto now = std::chrono::steady_clock::now();
                    double elasped = std::chrono::duration<double>(now - human.lastcontrolTime).count();
                    if (elasped < 2.0)
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
                    if (elasped < 2.0)
                    {
                        continue;
                    }

                    // human.printTextureID += 1;
                    // if (human.printTextureID >= 4)
                    // {
                    //     human.printTextureID = 0;
                    // }

                    human.printTextureID = 0;
                    human.stamp(matchedIt1->lidarTracker.currentLidar, footPrints);

                    // matchedIt1->lidarTracker.currentLidar.footX, matchedIt1->lidarTracker.currentLidar.footY
                    // matchedIt1->yoloTracker.currentYolo.footX, matchedIt1->yoloTracker.currentYolo.footY

                    // cv::Point2f predictedPoint;
                    // matchedIt1->yoloTracker.kalmanYolo.futurePredict(predictedPoint, 4);

                    // std::ofstream outFile("./100.txt", std::ios::app);
                    // outFile << "(" << matchedIt1->lidarTracker.currentLidar.footX << ", " << matchedIt1->lidarTracker.currentLidar.footY << "), "
                    //         << "(" << matchedIt1->yoloTracker.currentYolo.footX << ", " << matchedIt1->yoloTracker.currentYolo.footY << "), "
                    //         << "(" << predictedPoint.x << ", " << predictedPoint.y << ")"
                    //         << std::endl;
                    // outFile.close();

                    human.lastcontrolTime = std::chrono::steady_clock::now();
                }
                else if (isRight)
                {
                    auto now = std::chrono::steady_clock::now();
                    double elasped = std::chrono::duration<double>(now - human.lastcontrolTime).count();
                    if (elasped < 2.0)
                    {
                        continue;
                    }

                    human.printTextureID = 1;
                    human.stamp(matchedIt1->lidarTracker.currentLidar, footPrints);

                    // cv::Point2f predictedPoint;
                    // matchedIt1->yoloTracker.kalmanYolo.futurePredict(predictedPoint, 4);

                    // std::ofstream outFile("./100.txt", std::ios::app);
                    // outFile << "(" << matchedIt1->lidarTracker.currentLidar.footX << ", " << matchedIt1->lidarTracker.currentLidar.footY << "), "
                    //         << "(" << matchedIt1->yoloTracker.currentYolo.footX << ", " << matchedIt1->yoloTracker.currentYolo.footY << "), "
                    //         << "(" << predictedPoint.x << ", " << predictedPoint.y << ")"
                    //         << std::endl;
                    // outFile.close();

                    human.lastcontrolTime = std::chrono::steady_clock::now();
                }
            }
            else if (matchedIt2 != matchedTrackers2.end())
            {
                // matchedIt2のYoloTrackerを見てテクスチャIDを決定
                bool isLeft = matchedIt2->yoloTracker.currentYolo.isLeftHandUp(0.6f);
                bool isRight = matchedIt2->yoloTracker.currentYolo.isRightHandUp(0.6f);
                if (isLeft && isRight)
                {
                    auto now = std::chrono::steady_clock::now();
                    double elasped = std::chrono::duration<double>(now - human.lastcontrolTime).count();
                    if (elasped < 2.0)
                    {
                        continue;
                    }

                    human.printTextureID = 2;
                    human.stamp(matchedIt2->lidarTracker.currentLidar, footPrints);

                    human.lastcontrolTime = std::chrono::steady_clock::now();
                }
                else if (isLeft)
                {
                    auto now = std::chrono::steady_clock::now();
                    double elasped = std::chrono::duration<double>(now - human.lastcontrolTime).count();
                    if (elasped < 2.0)
                    {
                        continue;
                    }

                    human.printTextureID = 0;
                    human.stamp(matchedIt2->lidarTracker.currentLidar, footPrints);

                    human.lastcontrolTime = std::chrono::steady_clock::now();
                }
                else if (isRight)
                {
                    auto now = std::chrono::steady_clock::now();
                    double elasped = std::chrono::duration<double>(now - human.lastcontrolTime).count();
                    if (elasped < 2.0)
                    {
                        continue;
                    }

                    human.printTextureID = 1;
                    human.stamp(matchedIt2->lidarTracker.currentLidar, footPrints);

                    human.lastcontrolTime = std::chrono::steady_clock::now();
                }
            }
        }
    }

    /**
     * @details
     * 足跡の向きを angles から、色を footPrintColors から、それぞれ乱数で 1 つ選び、
     * lidar の足元位置に足跡を末尾へ追加する。
     * 追加後の足跡の管理 (更新・削除) は行わない。
     *
     * @note 前回のスタンプ位置からの移動距離で足跡を間引く処理は、現在コメントアウトされている。
     *       そのため lastStampX / lastStampY / stepThreshold は使われていない。
     *
     * @param[in]     lidar      足跡を置く位置 (LiDAR の足元位置)
     * @param[in,out] footPrints 足跡の一覧 (末尾に追加される)
     */
    void win::Human::stamp(const lidar::Lidar &lidar,
                           std::vector<FootPrint> &footPrints)
    {
        // save::saveFootPrints("0x500.txt", lidar.footX, lidar.footY);
        //  現在位置を取得（lidarから）
        //  float cx = lidar.footX;
        //  float cy = lidar.footY;

        // // 前回スタンプ位置からの移動距離
        // float dx = cx - lastStampX;
        // float dy = cy - lastStampY;
        // float dist = std::sqrt(dx * dx + dy * dy);

        // // 閾値を超えたときだけスタンプ
        // if (dist < stepThreshold)
        //     return;

        // 進行方向をangleに
        float angle = angles.at(angleDis(gen));

        int randomIndex = dis(gen);

        footPrints.emplace_back(lidar, angle, printTextureID, footPrintColors.at(randomIndex));

        // // 次のスタンプ基準位置を更新
        // lastStampX = cx;
        // lastStampY = cy;
    }

    /**
     * @details
     * 位置を lidar の足元位置に、向き・テクスチャの種類・色を引数の値に設定する。
     * size・alpha・createdTime は宣言時の初期値のままになる。
     *
     * @param[in] lidar            足跡を置く位置 (LiDAR の足元位置)
     * @param[in] _angle           足跡の初期角度 [度]
     * @param[in] _printTextureID  足跡のテクスチャの種類
     * @param[in] color            足跡の色 (R, G, B)
     */
    FootPrint::FootPrint(const lidar::Lidar &lidar, const float _angle, const int _printTextureID, const std::tuple<double, double, double> &color)
    {
        centerX = lidar.footX;
        centerY = lidar.footY;
        angle = _angle;
        this->printTextureID = _printTextureID;
        this->color = color;
    }

    /**
     * @details
     * 1 フレームごとに、size を 4.0 増やし、alpha を 0.015 減らし、angle を 1 度回転させる。
     * angle が 360 度以上になったら 0 度に戻す。
     */
    void FootPrint::update()
    {
        // 足跡のサイズを時間経過とともに増加させる
        size += 4.0f;
        alpha -= 0.015f;
        // if (size < 250.0f)
        // {
        //     size += 4.0f; // サイズの増加速度を調整

        //     if (size > 250.0f)
        //     {
        //         createdTime = std::chrono::steady_clock::now();
        //     }
        // }
        // else
        // {
        //     // サイズが最大になったら、アルファ値を時間経過とともに減少させる
        //     // auto now = std::chrono::steady_clock::now();
        //     // double elapsed = std::chrono::duration<double>(now - createdTime).count();
        //     // if (elapsed > 4.0)
        //     // {
        //     //     alpha -= 0.0025f;
        //     // }

        //     return;
        // }

        angle += 1.0f;
        if (angle >= 360.0f)
        {
            angle = 0.0f;
        }
    }

    /**
     * @details すべての足跡に対して FootPrint::update() を呼ぶ。
     */
    void updateFootPrints(std::vector<FootPrint> &footPrints)
    {
        for (auto &footPrint : footPrints)
        {
            footPrint.update();
        }
    }

    /**
     * @details alpha が 0 以下の足跡を std::erase_if で削除する。
     */
    void deleteFootPrints(std::vector<FootPrint> &footPrints)
    {
        // アルファ値が0になった足跡を削除
        std::erase_if(footPrints, [](const FootPrint &footPrint)
                      { return footPrint.alpha <= 0.0f; });
    }
}
