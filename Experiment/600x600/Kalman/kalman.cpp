// ----------------------------------------------------------------
// kalman.cpp
//
// カルマンフィルタの実装
//
// Copyright (c) 2026 Maito Yasui
// Released under the GNU GPL-3.0 License.
// https://github.com/Nibosi0501/LiViMotion/blob/main/LICENSE
// ----------------------------------------------------------------

#include "./Kalman.hpp"

namespace kf
{
    KalmanYolo::KalmanYolo(const yolo::Yolo &observedYolo)
    {
        // カルマンフィルタの初期化
        kf.init(4, 2, 0);

        // 速度減衰モデル: alpha < 1.0 で静止時の復帰を速くする
        // alpha が小さいほど速く止まるが、動きへの追従も落ちる
        float alpha = 0.8f;

        // 状態遷移行列
        kf.transitionMatrix = (cv::Mat_<float>(4, 4) << 1, 0, 1, 0,
                               0, 1, 0, 1,
                               0, 0, alpha, 0,
                               0, 0, 0, alpha);

        // 観測行列
        kf.measurementMatrix = (cv::Mat_<float>(2, 4) << 1, 0, 0, 0,
                                0, 1, 0, 0);

        // プロセスノイズ共分散行列
        setIdentity(kf.processNoiseCov, cv::Scalar(1e-2));

        // 観測ノイズ共分散行列
        setIdentity(kf.measurementNoiseCov, cv::Scalar(1e-1));

        // 事後誤差共分散行列
        setIdentity(kf.errorCovPost, cv::Scalar(1));

        // 初期状態
        kf.statePost = (cv::Mat_<float>(4, 1) << observedYolo.footX, observedYolo.footY, 0, 0);

        // 初期予測位置
        predictedPoint = cv::Point2f(observedYolo.footX, observedYolo.footY);
    }

    // 観測値を用いてカルマンフィルタを更新
    void KalmanYolo::update(const yolo::Yolo &observedYolo)
    {
        cv::Mat prediction = kf.predict();                                              // 予測ステップ
        predictedPoint = cv::Point2f(prediction.at<float>(0), prediction.at<float>(1)); // 予測位置を更新

        cv::Mat measurement = (cv::Mat_<float>(2, 1) << observedYolo.footX, observedYolo.footY); // 観測値を作成
        kf.correct(measurement);                                                                 // 更新ステップ
    }

    // 観測なしの予測ステップ
    void KalmanYolo::predict()
    {
        cv::Mat prediction = kf.predict();                                              // 予測ステップ
        predictedPoint = cv::Point2f(prediction.at<float>(0), prediction.at<float>(1)); // 予測位置を更新
    }

    // 将来の予測ステップ (numFrames フレーム先を予測)
    void KalmanYolo::futurePredict(cv::Point2f &futurePredictPoint, const int numFrames) const
    {
        cv::Mat futureState = kf.statePost.clone(); // 現在の状態をコピー
        for (int i = 0; i < numFrames; i++)
        {
            // 予測ステップを繰り返す
            futureState = kf.transitionMatrix * futureState;
        }

        // 予測された将来の位置を futurePredictPoint に格納
        futurePredictPoint = cv::Point2f(futureState.at<float>(0), futureState.at<float>(1));
    }
}