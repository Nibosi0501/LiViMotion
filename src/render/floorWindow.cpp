// ----------------------------------------------------------------
/**
 * @file floorWindow.cpp
 * @brief 床面ウィンドウの位置・角度の計算と、ジェスチャーによる動画操作の実装
 * @see FloorWindow.hpp
 *
 * Copyright (c) 2026 Maito Yasui
 * Copyright (c) 2026 Shinsei Yamada
 * Released under the GNU AGPL-3.0 License.
 * https://github.com/Nibosi0501/LiViMotion/blob/main/LICENSE
 */
// ----------------------------------------------------------------

#include "../../include/render/FloorWindow.hpp"

#include <cmath>
#include <numbers>

namespace win
{
    /**
     * @details
     * 処理の流れ:
     * -# MovieController を lidarTracker.id で構築する (成功・失敗にかかわらず構築される)
     * -# 履歴が 5 件でなければ lidarID を -1 にして戻る
     * -# calculateFloorWindow() で予測位置と角度を求める。失敗したら lidarID を -1 にして戻る
     * -# lidarID・現在位置・現在角度に結果を設定する
     * -# 人の足元から予測位置への単位ベクトルを targetVecX / targetVecY に設定する
     *    (距離が 0 の場合は設定しない)
     * -# targetAngle に求めた角度を設定する
     *
     * @note 現在位置 (centerX, centerY) は、最初から予測位置に置かれる。
     *       targetCenterX / targetCenterY は、このコンストラクタでは設定されない。
     */
    FloorWindow::FloorWindow(const tkr::LidarTracker &lidarTracker) : movie(lidarTracker.id)
    {
        std::deque<lidar::Lidar> historys = lidarTracker.historys;
        if (historys.size() != 5)
        {
            lidarID = -1;
            return;
        }

        float winPosX, winPosY, winAngle;
        if (!calculateFloorWindow(historys, winPosX, winPosY, winAngle))
        {
            lidarID = -1;
            return;
        }

        lidarID = lidarTracker.id;
        angle = winAngle;
        centerX = winPosX;
        centerY = winPosY;

        // lidarTracker.currentLidar から winPosX, winPosY までの正規化されたベクトルを計算して targetVecX, targetVecY に格納
        float dx = winPosX - lidarTracker.currentLidar.footX;
        float dy = winPosY - lidarTracker.currentLidar.footY;
        float length = std::hypot(dx, dy);
        if (length > 0.0f)
        {
            targetVecX = dx / length;
            targetVecY = dy / length;
        }

        targetAngle = winAngle;
    }

    /**
     * @details
     * 処理は大きく 3 段階に分かれる。
     *
     * 1. 新規作成・目標の更新 (履歴が 5 件そろっているトラッカーのみ)
     *    - ウィンドウが 0 件なら、各トラッカーのウィンドウを作成して終了する
     *    - 対応するウィンドウがあれば、予測位置への単位ベクトル (targetVec) と
     *      目標角度 (targetAngle) を更新する
     *    - なければ新規作成する
     *
     * 2. 位置の更新 (ウィンドウごと)
     *    - 対応する LiDAR トラッカーがある場合:
     *      人の足元から targetVec の向きに 130 離れた点を新しい目標位置とする
     *      (現在位置から 2 より大きく離れている場合のみ更新する)。
     *      その後、人の足元を原点とする極座標に変換し、角度と距離をそれぞれ
     *      10 % ずつ目標に近づける。これにより、曲がるときに円弧に沿って動く。
     *      足元に近すぎる場合 (半径が 1e-3 以下) は、線形補間に切り替える
     *    - 対応するトラッカーがない場合: 現在位置を目標位置に向けて線形補間 (10 %) する
     *
     * 3. 角度の更新 (ウィンドウごと)
     *    現在角度と目標角度の差を [-180, 180) に正規化し、
     *    1 フレームあたり最大 ±10 度に制限して近づける。最後に現在角度を [-180, 180) に正規化する。
     *
     * @param[in]     lidarTrackers LiDAR トラッカーの一覧
     * @param[in,out] floorWindows  床面ウィンドウの一覧
     *
     * @note 履歴の件数 5 は tkr::MAX_HISTORY_SIZE と同じ値だが、ここでは数値を直接書いている。
     * @note 補間係数 (0.1)・足元からの距離 (130)・位置更新の閾値 (2) は固定値で、定数化されていない。
     */
    void updateFloorWindows(const std::vector<tkr::LidarTracker> &lidarTrackers, std::vector<FloorWindow> &floorWindows)
    {
        if (lidarTrackers.empty())
        {
            return;
        }

        if (floorWindows.empty())
        {
            for (const auto &lidarTracker : lidarTrackers)
            {
                if (lidarTracker.historys.size() != 5)
                {
                    continue;
                }
                floorWindows.emplace_back(lidarTracker);
            }
            return;
        }

        for (const auto &lidarTracker : lidarTrackers)
        {
            if (lidarTracker.historys.size() != 5)
            {
                continue;
            }
            auto windowIt = std::find_if(floorWindows.begin(), floorWindows.end(), [&lidarTracker](const FloorWindow &window)
                                         { return window.lidarID == lidarTracker.id; });
            if (windowIt != floorWindows.end())
            {
                float winPosX, winPosY, winAngle;
                if (!calculateFloorWindow(lidarTracker.historys, winPosX, winPosY, winAngle))
                {
                    continue;
                }

                // lidarTracker.currentLidar から winPosX, winPosY までの正規化されたベクトルを計算して windowIt->targetVecX, windowIt->targetVecY に格納
                float dx = winPosX - lidarTracker.currentLidar.footX;
                float dy = winPosY - lidarTracker.currentLidar.footY;
                float length = std::hypot(dx, dy);
                if (length > 0.0f)
                {
                    windowIt->targetVecX = dx / length;
                    windowIt->targetVecY = dy / length;
                }

                windowIt->targetAngle = winAngle;
            }
            else
            {
                floorWindows.emplace_back(lidarTracker);
            }
        }
        for (auto &window : floorWindows)
        {
            // 位置と角度を徐々に更新
            auto lidarIt = std::find_if(lidarTrackers.begin(), lidarTrackers.end(), [&window](const tkr::LidarTracker &tracker)
                                        { return tracker.id == window.lidarID; });
            if (lidarIt != lidarTrackers.end())
            {
                float lidarX = lidarIt->currentLidar.footX;
                float lidarY = lidarIt->currentLidar.footY;

                // ターゲット位置を更新
                float moveVecX = window.targetVecX * 130.0f;
                float moveVecY = window.targetVecY * 130.0f;
                float newCenterX = lidarX + moveVecX;
                float newCenterY = lidarY + moveVecY;
                float dist = std::hypot(newCenterX - window.centerX, newCenterY - window.centerY);
                if (dist > 2.0f)
                {
                    window.targetCenterX = newCenterX;
                    window.targetCenterY = newCenterY;
                }

                // lidar基準の極座標に変換して、角度・距離をそれぞれ補間
                // （直線時はlerpとほぼ同じ結果、カーブ時は円弧上を移動する）
                float curOffX = window.centerX - lidarX;
                float curOffY = window.centerY - lidarY;
                float curRadius = std::hypot(curOffX, curOffY);

                float tgtOffX = window.targetCenterX - lidarX;
                float tgtOffY = window.targetCenterY - lidarY;
                float tgtRadius = std::hypot(tgtOffX, tgtOffY);

                float lerpFactor = 0.1f;

                if (curRadius > 1e-3f && tgtRadius > 1e-3f)
                {
                    float curAngleRad = std::atan2(curOffY, curOffX);
                    float tgtAngleRad = std::atan2(tgtOffY, tgtOffX);

                    // 最短角度差を [-π, π] に正規化
                    float angleDiffRad = tgtAngleRad - curAngleRad;
                    angleDiffRad = std::fmod(angleDiffRad + std::numbers::pi_v<float>, 2.0f * std::numbers::pi_v<float>);
                    if (angleDiffRad < 0.0f)
                    {
                        angleDiffRad += 2.0f * std::numbers::pi_v<float>;
                    }
                    angleDiffRad -= std::numbers::pi_v<float>;

                    // 角度を補間（カーブに沿う）＋ 距離も補間（ラグを維持）
                    float newAngleRad = curAngleRad + angleDiffRad * lerpFactor;
                    float newRadius = std::lerp(curRadius, tgtRadius, lerpFactor);

                    window.centerX = lidarX + std::cos(newAngleRad) * newRadius;
                    window.centerY = lidarY + std::sin(newAngleRad) * newRadius;
                }
                else
                {
                    // lidarとwindowが近すぎる場合は線形補間にフォールバック
                    window.centerX = std::lerp(window.centerX, window.targetCenterX, lerpFactor);
                    window.centerY = std::lerp(window.centerY, window.targetCenterY, lerpFactor);
                }
            }
            else
            {
                // lidarが見つからない場合は従来の線形補間
                float lerpFactor = 0.1f;
                window.centerX = std::lerp(window.centerX, window.targetCenterX, lerpFactor);
                window.centerY = std::lerp(window.centerY, window.targetCenterY, lerpFactor);
            }

            // ウィンドウの表示角度の補間（変更なし）
            float angleDiff = std::fmod(window.targetAngle - window.angle + 180.0f, 360.0f);
            if (angleDiff < 0)
            {
                angleDiff += 360.0f;
            }
            angleDiff -= 180.0f;

            float deltaAngle = std::clamp(angleDiff, -10.0f, 10.0f);
            window.angle += deltaAngle;

            window.angle = std::fmod(window.angle + 180.0f, 360.0f);
            if (window.angle < 0)
            {
                window.angle += 360.0f;
            }
            window.angle -= 180.0f;
        }
    }

    /**
     * @details
     * - トラッカーが 0 件なら、すべてのウィンドウを削除して終了する
     * - ウィンドウが 0 件なら何もしない
     * - それ以外は、対応するトラッカーの ID が存在しないウィンドウと、lidarID が -1 のウィンドウを削除する
     */
    void deleteFloorWindows(const std::vector<tkr::LidarTracker> &lidarTrackers, std::vector<FloorWindow> &floorWindows)
    {
        if (lidarTrackers.empty())
        {
            floorWindows.clear();
            return;
        }
        if (floorWindows.empty())
        {
            return;
        }

        std::erase_if(floorWindows, [&lidarTrackers](const FloorWindow &window)
                      { return std::none_of(lidarTrackers.begin(), lidarTrackers.end(), [&window](const tkr::LidarTracker &tracker)
                                            { return tracker.id == window.lidarID; }) ||
                               window.lidarID == -1; });
    }

    /**
     * @details
     * 履歴が 5 件でなければ false を返す。
     * 5 件なら、古い順に 0 番目 (p0)・2 番目 (p2)・4 番目 (p4) の位置を取り出し、
     * isStraight() (閾値 0.9) で直線かどうかを判定する。
     * - 直線: predictPositionOnStraight() で求める
     * - 直線でない: predictPositionOnLagrange() で求める
     *
     * @param[in]  historys  LiDAR の位置履歴 (5 件)
     * @param[out] winPosX   ウィンドウの予測位置の X 座標
     * @param[out] winPosY   ウィンドウの予測位置の Y 座標
     * @param[out] winAngle  ウィンドウの表示角度 [度]
     * @retval true  計算に成功した
     * @retval false 履歴が 5 件ではない
     */
    bool calculateFloorWindow(const std::deque<lidar::Lidar> &historys, float &winPosX, float &winPosY, float &winAngle)
    {
        if (historys.size() != 5)
        {
            return false;
        }

        std::pair<float, float> p0(historys.at(0).footX, historys.at(0).footY);
        std::pair<float, float> p2(historys.at(2).footX, historys.at(2).footY);
        std::pair<float, float> p4(historys.at(4).footX, historys.at(4).footY);
        float posX, posY, angle;
        if (isStraight(p0, p2, p4, 0.9f))
        {
            predictPositionOnStraight(p2, p4, posX, posY, angle);
        }
        else
        {
            predictPositionOnLagrange(p0, p2, p4, posX, posY, angle);
        }

        winPosX = posX;
        winPosY = posY;
        winAngle = angle;
        return true;
    }

    /**
     * @details
     * 2 つの移動ベクトル (p2→p4 と p0→p2) のコサイン類似度を計算し、threshold を超えれば true を返す。
     * どちらかのベクトルの長さが 1e-6 未満 (ほぼ静止) の場合は、類似度を 0 として扱う。
     * そのため、閾値が 0 以上であれば、静止している場合は直線とは判定されず、曲線の予測側に回る。
     *
     * @param[in] p0         履歴の 0 番目の位置
     * @param[in] p2         履歴の 2 番目の位置
     * @param[in] p4         履歴の 4 番目の位置
     * @param[in] threshold  コサイン類似度の閾値
     * @retval true  コサイン類似度が threshold を超えている
     * @retval false 超えていない
     */
    bool isStraight(const std::pair<float, float> &p0, const std::pair<float, float> &p2, const std::pair<float, float> &p4, const float threshold)
    {
        float dx1 = p4.first - p2.first;
        float dy1 = p4.second - p2.second;
        float dx2 = p2.first - p0.first;
        float dy2 = p2.second - p0.second;
        float length1 = std::hypot(dx1, dy1);
        float length2 = std::hypot(dx2, dy2);

        float cosSim = -1.0f;
        if (length1 < 1e-6 || length2 < 1e-6)
        {
            cosSim = 0.0f;
        }
        else
        {
            cosSim = (dx1 * dx2 + dy1 * dy2) / (length1 * length2);
        }

        return cosSim > threshold;
    }

    /**
     * @details
     * 1. p2 から p4 への方向を単位ベクトル (dx, dy) にする (長さが 0 なら (0, 0) とする)
     * 2. p4 から 150 先の点を予測位置とする
     * 3. 角度は atan2(dy, dx) を度に変換し、90 度引いた値とする
     *
     * @param[in]  p2        履歴の 2 番目の位置
     * @param[in]  p4        履歴の 4 番目 (最新) の位置
     * @param[out] winPosX   ウィンドウの予測位置の X 座標
     * @param[out] winPosY   ウィンドウの予測位置の Y 座標
     * @param[out] winAngle  ウィンドウの表示角度 [度]
     */
    void predictPositionOnStraight(const std::pair<float, float> &p2, const std::pair<float, float> &p4, float &winPosX, float &winPosY, float &winAngle)
    {
        float dx = p4.first - p2.first;
        float dy = p4.second - p2.second;
        float length = std::hypot(dx, dy);
        if (length > 0.0f)
        {
            dx /= length;
            dy /= length;
        }
        else
        {
            dx = 0.0f;
            dy = 0.0f;
        }

        float lineLength = 150;
        winPosX = p4.first + dx * lineLength;
        winPosY = p4.second + dy * lineLength;

        winAngle = std::atan2(dy, dx) * 180.0f / std::numbers::pi_v<float> - 90.0f;
    }

    /**
     * @details
     * p0 (t = 0)、p2 (t = 0.5)、p4 (t = 1) を通る 2 次のラグランジュ補間曲線
     * \f$ P(t) = L_0(t) p_0 + L_1(t) p_2 + L_2(t) p_4 \f$ を使う。
     * - \f$ L_0 = 2(t - 0.5)(t - 1) \f$
     * - \f$ L_1 = -4t(t - 1) \f$
     * - \f$ L_2 = 2t(t - 0.5) \f$
     *
     * t > 1 の範囲は p4 より先の外挿になる。p4 からの距離が 150 (許容誤差 5) になる t を、
     * 次のルールで最大 20 回探索する。
     * - 距離が目標より短ければ t を step (初期値 0.1) だけ増やす
     * - 長ければ t を step の半分だけ減らし、step を半分にする
     * - t は毎回 1.01 〜 3.0 の範囲に制限する (初期値は 1.1)
     *
     * 求めた t で calculateAngle() を呼んで角度を計算する。
     *
     * @note 探索が収束しなかった場合、予測位置は最後のループで計算した候補 (更新前の t) を使い、
     *       角度は更新後の t で計算する。わずかに t がずれる可能性がある。
     * @note 関数内のコメント (「t=0でp2」「t=0.5でp1」「t=1でp0」) は、実際のコードの対応
     *       (p0・p2・p4) と一致していない。上記の対応がコードの実際の動作である。
     *
     * @param[in]  p0        履歴の 0 番目の位置
     * @param[in]  p2        履歴の 2 番目の位置
     * @param[in]  p4        履歴の 4 番目 (最新) の位置
     * @param[out] winPosX   ウィンドウの予測位置の X 座標
     * @param[out] winPosY   ウィンドウの予測位置の Y 座標
     * @param[out] winAngle  ウィンドウの表示角度 [度]
     */
    void predictPositionOnLagrange(const std::pair<float, float> &p0, const std::pair<float, float> &p2, const std::pair<float, float> &p4, float &winPosX, float &winPosY, float &winAngle)
    {
        double targetDistance = 150;
        double t = 1.1;                       // 初期値
        double step = 0.1;                    // t値の調整ステップ
        std::pair<float, float> candidatePos; // 予測位置の候補

        // t値を調整してp0から目標距離になる位置を見つける
        for (int i = 0; i < 20; i++)
        {
            // ラグランジュ基底関数の計算
            double L0 = 2.0 * (t - 0.5) * (t - 1.0); // t=0でp2
            double L1 = -4.0 * t * (t - 1.0);        // t=0.5でp1
            double L2 = 2.0 * t * (t - 0.5);         // t=1でp0

            candidatePos.first = L0 * p0.first + L1 * p2.first + L2 * p4.first;
            candidatePos.second = L0 * p0.second + L1 * p2.second + L2 * p4.second;

            float dist = std::hypot(candidatePos.first - p4.first, candidatePos.second - p4.second);
            if (std::abs(dist - targetDistance) < 5.0f)
            {
                break;
            }

            if (dist < targetDistance)
            {
                t += step;
            }
            else
            {
                t -= step * 0.5;
                step *= 0.5;
            }
            t = std::clamp(t, 1.01, 3.0);
        }

        winPosX = candidatePos.first;
        winPosY = candidatePos.second;

        calculateAngle(p0, p2, p4, t, winAngle);
    }

    /**
     * @details
     * 曲線 \f$ P(t) \f$ の導関数から、パラメータ t での接線ベクトルを求める。
     * - \f$ L_0' = 2(2t - 1.5) \f$
     * - \f$ L_1' = -4(2t - 1) \f$
     * - \f$ L_2' = 2(2t - 0.5) \f$
     *
     * 接線ベクトルの長さが 0.1 より大きければ、正規化して atan2 から角度 [度] を求め、
     * 90 度を引いた値を winAngle とする。0.1 以下なら winAngle は 0 度にする。
     *
     * @param[in]  p0        履歴の 0 番目の位置
     * @param[in]  p2        履歴の 2 番目の位置
     * @param[in]  p4        履歴の 4 番目の位置
     * @param[in]  t         接線を求める曲線のパラメータ
     * @param[out] winAngle  ウィンドウの表示角度 [度]
     */
    void calculateAngle(const std::pair<float, float> &p0, const std::pair<float, float> &p2, const std::pair<float, float> &p4, const double t, float &winAngle)
    {
        // パネル位置での曲線の接線ベクトルから角度を計算
        // 最終t値での接線ベクトルを計算
        double dL0 = 2.0 * (2.0 * t - 1.5);  // L0の導関数
        double dL1 = -4.0 * (2.0 * t - 1.0); // L1の導関数
        double dL2 = 2.0 * (2.0 * t - 0.5);  // L2の導関数

        std::pair<float, float> tangentVec;
        tangentVec.first = dL0 * p0.first + dL1 * p2.first + dL2 * p4.first;
        tangentVec.second = dL0 * p0.second + dL1 * p2.second + dL2 * p4.second;

        float length = std::hypot(tangentVec.first, tangentVec.second);
        if (length > 0.1f)
        {
            std::pair<float, float> normTangentVec = {tangentVec.first / length, tangentVec.second / length};
            winAngle = std::atan2(normTangentVec.second, normTangentVec.first) * 180.0f / std::numbers::pi_v<float> - 90.0f;
        }
        else
        {
            winAngle = 0.0f;
        }
    }

    /**
     * @details
     * 各ウィンドウについて、lidarID を持つマッチング済みトラッカーを 2 系統から探す。
     *
     * - どちらにも見つからない: 何もしない
     * - 両方に見つかった: 2 つの YOLO を組み合わせた判定を、信頼度の閾値 0.6 で行う
     * - 片方だけ見つかった: その YOLO だけで判定する
     *   (切り替え・巻き戻しは閾値 0.9、一時停止は閾値 0.7)
     *
     * いずれの場合も、判定の優先順位は 切り替え → 巻き戻し → 一時停止 の順で、
     * 最初に当てはまった操作だけを行って次のウィンドウへ進む。
     * どれにも当てはまらなければ resumeMovie() で再生を再開する。
     *
     * @note 巻き戻し・切り替えは MovieController 側のクールダウンで連続操作が抑えられている。
     */
    void changeMovies(std::vector<win::FloorWindow> &floorWindows, const std::vector<mtc::MatchedTracker> &matchedTrackers1, const std::vector<mtc::MatchedTracker> &matchedTrackers2)
    {
        for (auto &window : floorWindows)
        {
            int lidarID = window.lidarID;
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
                if (isMovieChange(matchedIt1->yoloTracker, matchedIt2->yoloTracker, 0.6f))
                {
                    window.movie.changeMovieID();
                    continue;
                }
                if (isMovieReset(matchedIt1->yoloTracker, matchedIt2->yoloTracker, 0.6f))
                {
                    window.movie.resetMovie();
                    continue;
                }
                if (isMoviePause(matchedIt1->yoloTracker, matchedIt2->yoloTracker, 0.6f))
                {
                    window.movie.pauseMovie();
                    continue;
                }
                else
                {
                    window.movie.resumeMovie();
                    continue;
                }
            }
            else if (matchedIt1 != matchedTrackers1.end())
            {
                if (isMovieChange(matchedIt1->yoloTracker, 0.9f))
                {
                    window.movie.changeMovieID();
                    continue;
                }
                if (isMovieReset(matchedIt1->yoloTracker, 0.9f))
                {
                    window.movie.resetMovie();
                    continue;
                }
                if (isMoviePause(matchedIt1->yoloTracker, 0.7f))
                {
                    window.movie.pauseMovie();
                    continue;
                }
                else
                {
                    window.movie.resumeMovie();
                    continue;
                }
            }
            else if (matchedIt2 != matchedTrackers2.end())
            {
                if (isMovieChange(matchedIt2->yoloTracker, 0.9f))
                {
                    window.movie.changeMovieID();
                    continue;
                }
                if (isMovieReset(matchedIt2->yoloTracker, 0.9f))
                {
                    window.movie.resetMovie();
                    continue;
                }
                if (isMoviePause(matchedIt2->yoloTracker, 0.7f))
                {
                    window.movie.pauseMovie();
                    continue;
                }
                else
                {
                    window.movie.resumeMovie();
                    continue;
                }
            }
        }
    }

    /**
     * @details 最新の検出結果で、左手と右手の両方が挙がっていれば true を返す。
     */
    bool isMovieChange(const tkr::YoloTracker &tracker, const float conf)
    {
        bool isLeftHandUp = tracker.currentYolo.isLeftHandUp(conf);
        bool isRightHandUp = tracker.currentYolo.isRightHandUp(conf);
        if (isLeftHandUp && isRightHandUp)
        {
            return true;
        }
        return false;
    }

    /**
     * @details
     * 2 つのトラッカーの最新のキーポイントを使い、yolo::isLeftHandUp() と yolo::isRightHandUp() の
     * 2 組版で判定する。両方が true なら true を返す。
     */
    bool isMovieChange(const tkr::YoloTracker &tracker1, const tkr::YoloTracker &tracker2, const float conf)
    {
        bool isLeftHandUp = yolo::isLeftHandUp(tracker1.currentYolo.keypoints, tracker2.currentYolo.keypoints, conf);
        bool isRightHandUp = yolo::isRightHandUp(tracker1.currentYolo.keypoints, tracker2.currentYolo.keypoints, conf);
        if (isLeftHandUp && isRightHandUp)
        {
            return true;
        }
        return false;
    }

    /**
     * @details 最新の検出結果で、左手が挙がっていれば true を返す。
     */
    bool isMoviePause(const tkr::YoloTracker &tracker, const float conf)
    {
        bool isLeftHandUp = tracker.currentYolo.isLeftHandUp(conf);
        if (isLeftHandUp)
        {
            return true;
        }
        return false;
    }

    /**
     * @details 2 つのトラッカーの最新のキーポイントを使い、2 組版の yolo::isLeftHandUp() で判定する。
     */
    bool isMoviePause(const tkr::YoloTracker &tracker1, const tkr::YoloTracker &tracker2, const float conf)
    {
        bool isLeftHandUp = yolo::isLeftHandUp(tracker1.currentYolo.keypoints, tracker2.currentYolo.keypoints, conf);
        if (isLeftHandUp)
        {
            return true;
        }
        return false;
    }

    /**
     * @details
     * 最新の検出結果で右手が挙がっていなければ false を返す。
     * 挙がっていれば、履歴のすべてのフレームでも右手が挙がっているか
     * (yolo::isRightHandUpInHistory()) を判定して結果を返す。
     * 履歴が空の場合は false になる。
     */
    bool isMovieReset(const tkr::YoloTracker &tracker, const float conf)
    {
        bool isRightHandUp = tracker.currentYolo.isRightHandUp(conf);
        if (isRightHandUp == false)
        {
            return false;
        }
        bool isReset = false;
        isReset = yolo::isRightHandUpInHistory(tracker.historys, conf);
        if (isReset)
        {
            return true;
        }
        return false;
    }

    /**
     * @details
     * 2 つのトラッカーの履歴を使って yolo::isRightHandUpInHistory() の 2 組版で判定する。
     * 最新の検出結果 (currentYolo) は判定に使わない。
     *
     * @note 同じ判定 (isRightHandUpInHistory) を 2 回続けて呼んでおり、2 回目は 1 回目と同じ結果になる。
     *       1 回で済ませても動作は変わらない。
     */
    bool isMovieReset(const tkr::YoloTracker &tracker1, const tkr::YoloTracker &tracker2, const float conf)
    {
        bool isRightHandUp = yolo::isRightHandUpInHistory(tracker1.historys, tracker2.historys, conf);
        if (isRightHandUp == false)
        {
            return false;
        }
        bool isReset = false;
        isReset = yolo::isRightHandUpInHistory(tracker1.historys, tracker2.historys, conf);
        if (isReset)
        {
            return true;
        }
        return false;
    }
}
