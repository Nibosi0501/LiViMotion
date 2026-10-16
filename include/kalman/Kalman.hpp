/**
 * @file Kalman.hpp
 * @brief YOLO で検出した足元位置を追跡するカルマンフィルタクラスの宣言
 *
 * 足元の 2D 位置 (x, y) を観測とし、位置と速度を状態とする等速モデル
 * (速度は減衰する) のカルマンフィルタで平滑化・予測を行う。
 *
 * @par 状態ベクトル
 * \f$ [x, y, v_x, v_y]^T \f$ (4 次元)
 *
 * @par 観測ベクトル
 * \f$ [x, y]^T \f$ (2 次元)
 * 
 * Copyright (c) 2026 Maito Yasui
 * Released under the GNU AGPL-3.0 License.
 * https://github.com/Nibosi0501/LiViMotion/blob/main/LICENSE
 */
#pragma once

#include <opencv2/opencv.hpp>

#include "../yolo/Yolo.hpp"

/**
 * @namespace kf
 * @brief カルマンフィルタ関連の名前空間
 */
namespace kf
{
    /**
     * @brief YOLO の足元位置を追跡するカルマンフィルタ
     *
     * 1 人分の追跡に 1 インスタンスを使う。
     * 観測が得られたフレームでは update()、得られなかったフレームでは predict() を
     * 毎フレーム 1 回呼び出す。
     */
    struct KalmanYolo
    {
        cv::KalmanFilter kf;       ///< OpenCV のカルマンフィルタ本体 (状態 4 次元・観測 2 次元)
        cv::Point2f predictedPoint; ///< 直近の予測ステップで得られた予測位置 (x, y)

        /**
         * @brief 最初の観測値でフィルタを初期化する
         *
         * 状態の位置を観測値の足元位置、速度を 0 に設定する。
         * 各行列は次の値で初期化される。
         * - 遷移行列: 位置は速度を加算、速度は減衰係数 alpha (= 0.8) 倍
         * - 観測行列: 位置 (x, y) のみを観測
         * - プロセスノイズ共分散: 1e-2 × 単位行列
         * - 観測ノイズ共分散: 1e-1 × 単位行列
         * - 事後誤差共分散: 単位行列
         *
         * @param[in] observedYolo 初期位置を与える YOLO の検出結果 (footX, footY を使用)
         */
        KalmanYolo(const yolo::Yolo &observedYolo);

        /**
         * @brief 予測ステップと補正ステップを実行する (観測があるフレーム用)
         *
         * predict() で状態を 1 フレーム進めて predictedPoint を更新した後、
         * 観測値 (footX, footY) で状態を補正する。
         *
         * @param[in] observedYolo 今回のフレームの YOLO の検出結果
         *
         * @note predictedPoint は補正前の予測位置である。補正後の推定位置は kf.statePost にある。
         */
        void update(const yolo::Yolo &observedYolo);

        /**
         * @brief 予測ステップのみを実行する (観測がないフレーム用)
         *
         * 状態を 1 フレーム進めて predictedPoint を更新する。補正は行わない。
         */
        void predict(); // 観測なしの予測ステップ

        /**
         * @brief 現在の状態から、指定フレーム先の位置を予測する
         *
         * 現在の事後状態 (kf.statePost) に遷移行列を numFrames 回掛けて求める。
         * フィルタ本体の状態は変更しない。
         *
         * @param[out] futurePredictPoint 予測された位置 (x, y)
         * @param[in]  numFrames          何フレーム先を予測するか
         */
        void futurePredict(cv::Point2f &futurePredictPoint, const int numFrames) const;
    };
}
