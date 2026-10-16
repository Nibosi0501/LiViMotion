// ----------------------------------------------------------------
/**
 * @file yolo.cpp
 * @brief yolo::Yolo および挙手判定関数の実装
 * @see Yolo.hpp
 *
 * Copyright (c) 2026 Maito Yasui
 * Released under the GNU AGPL-3.0 License.
 * https://github.com/Nibosi0501/LiViMotion/blob/main/LICENSE
 */
// ----------------------------------------------------------------

#include "../../include/yolo/Yolo.hpp"

namespace yolo
{
    /**
     * @details
     * 処理の流れ:
     * -# getFoot() で足元の画像座標と信頼度を求める
     * -# calibration.transformPoint() でウィンドウ座標に変換する
     * -# calibration.attribute に応じて軸の入れ替え・反転とスケール変換を行う
     *    - "front": X を 0〜600 → 300〜-300、Y を 0〜600 → 600〜0 に変換 (変換結果を標準出力に表示)
     *    - "back" : 変換後の y を X に (0〜600 → 300〜-300)、x を Y に (0〜600 → 0〜600) 割り当てる
     *    - その他 : "front" と同じ変換 (標準出力への表示なし)
     * -# footX / footY が有効範囲 (X: -300〜300, Y: 0〜600) を外れた場合は footConf を -1 にする
     *
     * 変換に失敗した場合は footConf を -1 にし、footX / footY は設定しない。
     */
    Yolo::Yolo(const std::vector<std::tuple<float, float, float>> &_keypoints, const clb::Calibration &calibration) : keypoints(_keypoints)
    {
        float x, y, conf;
        getFoot(x, y, conf);

        cv::Point2f transformedPoint;
        if (calibration.transformPoint(cv::Point2f(x, y), transformedPoint))
        {
            if (calibration.attribute == "front")
            {
                footX = mapValue(transformedPoint.x, 0.0f, 600.0f, 300.0f, -300.0f);
                footY = mapValue(transformedPoint.y, 0.0f, 600.0f, 600.0f, 0.0f);
            }
            else if (calibration.attribute == "back")
            {
                footX = mapValue(transformedPoint.y, 0.0f, 600.0f, 300.0f, -300.0f);
                footY = mapValue(transformedPoint.x, 0.0f, 600.0f, 0.0f, 600.0f);
            }
            else
            {
                footX = mapValue(transformedPoint.x, 0.0f, 600.0f, 300.0f, -300.0f);
                footY = mapValue(transformedPoint.y, 0.0f, 600.0f, 600.0f, 0.0f);
            }
            footConf = conf;

            if (footX < -300.0f || footX > 300.0f || footY < 0.0f || footY > 600.0f)
            {
                footConf = -1; // 座標が範囲外の場合は信頼度を-1に設定
            }

        }
        else
        {
            footConf = -1;
        }
    }

    /**
     * @details
     * 左足首 (15) と右足首 (16) の信頼度がともに 0.4 を超える場合は座標・信頼度の平均を返す。
     * そうでない場合は、信頼度の高いほうの足首をそのまま返す
     * (両方とも低い場合でも、高いほうを返す点に注意)。
     *
     * @param[out] x    足元の x 座標 (画像座標)
     * @param[out] y    足元の y 座標 (画像座標)
     * @param[out] conf 足元の信頼度
     */
    void Yolo::getFoot(float &x, float &y, float &conf) const
    {
        auto [leftAnkleX, leftAnkleY, leftAnkleConf] = keypoints.at(15);
        auto [rightAnkleX, rightAnkleY, rightAnkleConf] = keypoints.at(16);

        if (leftAnkleConf > 0.4f && rightAnkleConf > 0.4f)
        {
            x = (leftAnkleX + rightAnkleX) / 2.0f;
            y = (leftAnkleY + rightAnkleY) / 2.0f;
            conf = (leftAnkleConf + rightAnkleConf) / 2.0f;
        }
        else if (leftAnkleConf > rightAnkleConf)
        {
            x = leftAnkleX;
            y = leftAnkleY;
            conf = leftAnkleConf;
        }
        else
        {
            x = rightAnkleX;
            y = rightAnkleY;
            conf = rightAnkleConf;
        }
    }

    /**
     * @details
     * (x - in_min) / (in_max - in_min) で入力範囲内の割合を求め、
     * それを出力範囲に当てはめる。クリップは行わない。
     *
     * @param[in] x       変換する値
     * @param[in] in_min  入力範囲の最小値
     * @param[in] in_max  入力範囲の最大値
     * @param[in] out_min 出力範囲の最小値
     * @param[in] out_max 出力範囲の最大値
     * @return 変換後の値
     */
    float Yolo::mapValue(const float x, const float in_min, const float in_max, const float out_min, const float out_max) const
    {
        return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
    }

    /**
     * @details
     * 左肘 (7) と左手首 (9) のどちらかの信頼度が conf 未満なら false。
     * そうでなければ、手首の y が肘の y より小さい (画像上で上にある) とき true。
     *
     * @param[in] conf 判定に必要な最低信頼度
     * @retval true  左手が上がっている
     * @retval false 上がっていない、または信頼度が足りない
     */
    bool Yolo::isLeftHandUp(const float conf) const
    {
        // auto [leftShoulderX, leftShoulderY, leftShoulderConf] = keypoints.at(5);
        auto [leftElbowX, leftElbowY, leftElbowConf] = keypoints.at(7);
        auto [leftWristX, leftWristY, leftWristConf] = keypoints.at(9);
        if (leftElbowConf < conf || leftWristConf < conf)
        {
            return false;
        }
        return leftWristY < leftElbowY;
    }

    /**
     * @details
     * 右肘 (8) と右手首 (10) のどちらかの信頼度が conf 未満なら false。
     * そうでなければ、手首の y が肘の y より小さい (画像上で上にある) とき true。
     *
     * @param[in] conf 判定に必要な最低信頼度
     * @retval true  右手が上がっている
     * @retval false 上がっていない、または信頼度が足りない
     */
    bool Yolo::isRightHandUp(const float conf) const
    {
        auto [rightElbowX, rightElbowY, rightElbowConf] = keypoints.at(8);
        auto [rightWristX, rightWristY, rightWristConf] = keypoints.at(10);
        if (rightElbowConf < conf || rightWristConf < conf)
        {
            return false;
        }
        return rightWristY < rightElbowY;
    }

    /**
     * @details
     * 左肩 (5) と左手首 (9) の信頼度が 2 組分 (計 4 点) すべて conf 以上であることを確認する。
     * 左肩の信頼度が高いほうの組を選び、その組の肩と手首の y を比較して、
     * 手首が肩より上 (y が小さい) なら true を返す。
     *
     * @param[in] keypoints1 1 組目のキーポイント
     * @param[in] keypoints2 2 組目のキーポイント
     * @param[in] conf       判定に必要な最低信頼度
     * @retval true  左手が上がっている
     * @retval false 上がっていない、または信頼度が足りない
     */
    bool isLeftHandUp(const std::vector<std::tuple<float, float, float>> &keypoints1, const std::vector<std::tuple<float, float, float>> &keypoints2, const float conf)
    {
        auto [leftShoulderX1, leftShoulderY1, leftShoulderConf1] = keypoints1.at(5);
        auto [leftShoulderX2, leftShoulderY2, leftShoulderConf2] = keypoints2.at(5);

        auto [leftWristX1, leftWristY1, leftWristConf1] = keypoints1.at(9);
        auto [leftWristX2, leftWristY2, leftWristConf2] = keypoints2.at(9);

        if (leftShoulderConf1 < conf || leftShoulderConf2 < conf || leftWristConf1 < conf || leftWristConf2 < conf)
        {
            return false;
        }

        float shoulderY, wristY;
        if (leftShoulderConf1 > leftShoulderConf2)
        {
            shoulderY = leftShoulderY1;
            wristY = leftWristY1;
        }
        else
        {
            shoulderY = leftShoulderY2;
            wristY = leftWristY2;
        }

        return wristY < shoulderY;
    }

    /**
     * @details
     * 右肩 (6) と右手首 (10) の信頼度が 2 組分 (計 4 点) すべて conf 以上であることを確認する。
     * 右肩の信頼度が高いほうの組を選び、その組の肩と手首の y を比較して、
     * 手首が肩より上 (y が小さい) なら true を返す。
     *
     * @param[in] keypoints1 1 組目のキーポイント
     * @param[in] keypoints2 2 組目のキーポイント
     * @param[in] conf       判定に必要な最低信頼度
     * @retval true  右手が上がっている
     * @retval false 上がっていない、または信頼度が足りない
     */
    bool isRightHandUp(const std::vector<std::tuple<float, float, float>> &keypoints1, const std::vector<std::tuple<float, float, float>> &keypoints2, const float conf)
    {
        auto [rightShoulderX1, rightShoulderY1, rightShoulderConf1] = keypoints1.at(6);
        auto [rightShoulderX2, rightShoulderY2, rightShoulderConf2] = keypoints2.at(6);

        auto [rightWristX1, rightWristY1, rightWristConf1] = keypoints1.at(10);
        auto [rightWristX2, rightWristY2, rightWristConf2] = keypoints2.at(10);

        if (rightShoulderConf1 < conf || rightShoulderConf2 < conf || rightWristConf1 < conf || rightWristConf2 < conf)
        {
            return false;
        }

        float shoulderY, wristY;
        if (rightShoulderConf1 > rightShoulderConf2)
        {
            shoulderY = rightShoulderY1;
            wristY = rightWristY1;
        }
        else
        {
            shoulderY = rightShoulderY2;
            wristY = rightWristY2;
        }

        return wristY < shoulderY;
    }

    /**
     * @details
     * 履歴が空の場合は false。それ以外は、全フレームで Yolo::isRightHandUp(conf) が
     * true のときだけ true を返す (1 フレームでも false があれば即座に false)。
     *
     * @param[in] historys 時系列の姿勢履歴
     * @param[in] conf     判定に必要な最低信頼度
     * @retval true  全フレームで右手が上がっている
     * @retval false 履歴が空、または 1 フレームでも上がっていない
     */
    bool isRightHandUpInHistory(const std::deque<Yolo> &historys, const float conf)
    {
        if (historys.empty())
        {
            return false;
        }

        for (const auto &history : historys)
        {
            if (history.isRightHandUp(conf) == false)
            {
                return false;
            }
        }
        return true;
    }

    /**
     * @details
     * 処理の流れ:
     * -# 両方の履歴が空なら false
     * -# 先頭から短いほうの長さ (minSize) までは、同じ添字のフレーム同士を
     *    2 組版の isRightHandUp() で判定する
     * -# 長いほうの履歴の残りのフレームは、Yolo::isRightHandUp() で単独判定する
     *
     * 1 フレームでも false があれば即座に false を返す。
     *
     * @param[in] historys1 1 つ目の時系列の姿勢履歴
     * @param[in] historys2 2 つ目の時系列の姿勢履歴
     * @param[in] conf      判定に必要な最低信頼度
     * @retval true  全フレームで右手が上がっている
     * @retval false 両方の履歴が空、または 1 フレームでも上がっていない
     */
    bool isRightHandUpInHistory(const std::deque<Yolo> &historys1, const std::deque<Yolo> &historys2, const float conf)
    {
        int size1 = historys1.size();
        int size2 = historys2.size();
        if (size1 == 0 && size2 == 0)
        {
            return false;
        }
        int minSize = std::min(size1, size2);
        for (int i = 0; i < minSize; i++)
        {
            if (yolo::isRightHandUp(historys1.at(i).keypoints, historys2.at(i).keypoints, conf) == false)
            {
                return false;
            }
        }
        if (size1 > size2)
        {
            for (int i = minSize; i < size1; i++)
            {
                if (historys1.at(i).isRightHandUp(conf) == false)
                {
                    return false;
                }
            }
        }
        else if (size2 > size1)
        {
            for (int i = minSize; i < size2; i++)
            {
                if (historys2.at(i).isRightHandUp(conf) == false)
                {
                    return false;
                }
            }
        }
        return true;
    }
}
