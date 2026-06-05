// ----------------------------------------------------------------
// Yolo.cpp
//
// Yoloクラスの実装と関連する関数の定義
//
// Copyright (c) 2026 Maito Yasui
// Released under the GNU GPL-3.0 License.
// https://github.com/Nibosi0501/LiViMotion/blob/main/LICENSE
// ----------------------------------------------------------------

#include "./Yolo.hpp"

namespace yolo
{
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

    float Yolo::mapValue(const float x, const float in_min, const float in_max, const float out_min, const float out_max) const
    {
        return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
    }

    bool Yolo::isLeftHandUp(const float conf) const
    {
        auto [leftElbowX, leftElbowY, leftElbowConf] = keypoints.at(7);
        auto [leftWristX, leftWristY, leftWristConf] = keypoints.at(9);
        if (leftElbowConf < conf || leftWristConf < conf)
        {
            return false;
        }
        return leftWristY < leftElbowY;
    }
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