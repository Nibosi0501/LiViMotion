/**
 * @file kalman.cpp
 * @brief kf::KalmanYolo の実装
 * @see Kalman.hpp
 * 
 * Copyright (c) 2026 Maito Yasui
 * Released under the GNU AGPL-3.0 License.
 * https://github.com/Nibosi0501/LiViMotion/blob/main/LICENSE
 */
#include "../../include/kalman/Kalman.hpp"

namespace kf
{
    /**
     * @details
     * 状態 4 次元 (x, y, vx, vy)・観測 2 次元 (x, y)・制御入力なしでフィルタを初期化する。
     *
     * 遷移行列は、位置が速度分だけ進み、速度が alpha 倍に減衰するモデル。
     * alpha を 1 未満にすると、静止したときに速度が早く 0 に近づく。
     * alpha が小さいほど速く止まるが、動きへの追従は落ちる (0.7〜0.9 が目安)。
     *
     * @param[in] observedYolo 初期位置を与える YOLO の検出結果
     */
    KalmanYolo::KalmanYolo(const yolo::Yolo &observedYolo)
    {
        kf.init(4, 2, 0);

        // 速度減衰モデル: alpha < 1.0 で静止時の復帰を速くする
        // alpha が小さいほど速く止まるが、動きへの追従も落ちる (0.7〜0.9 推奨)
        float alpha = 0.8f;
        kf.transitionMatrix = cv::Mat_<float>({4, 4}, {1, 0, 1, 0,
                                               0, 1, 0, 1,
                                               0, 0, alpha, 0,
                                               0, 0, 0, alpha});

        kf.measurementMatrix = cv::Mat_<float>({2, 4}, {1, 0, 0, 0,
                                                0, 1, 0, 0});

        setIdentity(kf.processNoiseCov, cv::Scalar(1e-2));

        setIdentity(kf.measurementNoiseCov, cv::Scalar(1e-1));

        setIdentity(kf.errorCovPost, cv::Scalar(1));

        kf.statePost = cv::Mat_<float>({4, 1}, {observedYolo.footX, observedYolo.footY, 0.0f, 0.0f});
        predictedPoint = cv::Point2f(observedYolo.footX, observedYolo.footY);
    }

    /**
     * @details
     * -# kf.predict() で状態を 1 フレーム進め、その位置を predictedPoint に保存する
     * -# 観測値 (footX, footY) を使って kf.correct() で状態を補正する
     *
     * @param[in] observedYolo 今回のフレームの YOLO の検出結果
     */
    void KalmanYolo::update(const yolo::Yolo &observedYolo)
    {
        cv::Mat prediction = kf.predict();
        predictedPoint = cv::Point2f(prediction.at<float>(0), prediction.at<float>(1));

        cv::Mat measurement = cv::Mat_<float>({2, 1}, {observedYolo.footX, observedYolo.footY});
        kf.correct(measurement);
    }

    /**
     * @details kf.predict() のみを実行し、予測位置を predictedPoint に保存する。補正は行わない。
     */
    void KalmanYolo::predict()
    {
        cv::Mat prediction = kf.predict();
        predictedPoint = cv::Point2f(prediction.at<float>(0), prediction.at<float>(1));
    }

    /**
     * @details
     * 現在の事後状態 kf.statePost のコピーに遷移行列を numFrames 回掛けて、
     * 先の状態を求める。コピーに対して計算するため、フィルタ本体の状態は変化しない。
     * プロセスノイズは考慮しない (決定論的な外挿)。
     *
     * @param[out] futurePredictPoint 予測された位置 (x, y)
     * @param[in]  numFrames          何フレーム先を予測するか
     */
    void KalmanYolo::futurePredict(cv::Point2f &futurePredictPoint, const int numFrames) const
    {
        cv::Mat futureState = kf.statePost.clone();
        for (int i = 0; i < numFrames; i++)
        {
            futureState = kf.transitionMatrix * futureState;
        }
        futurePredictPoint = cv::Point2f(futureState.at<float>(0), futureState.at<float>(1));
    }
}
