// ----------------------------------------------------------------
/**
 * @file MovieController.hpp
 * @brief 床面に投影する動画の再生・切り替え・緊急表示を管理するクラスの宣言
 *
 * 複数の動画を OpenCV の cv::VideoCapture で保持し、1 フレームずつ読み出して
 * OpenGL のテクスチャとして転送する。
 * 一時停止・再開・巻き戻し・動画の切り替えに加え、緊急時には動画の代わりに
 * 矢印画像 (避難誘導用) を表示する。
 *
 * @note 巻き戻し・切り替えは、連続して操作されないようにクールダウン
 *       (MOVIE_CONTROL_COOLDOWN_SECONDS) を設けている。
 *
 * Copyright (c) 2026 Maito Yasui
 * Released under the GNU AGPL-3.0 License.
 * https://github.com/Nibosi0501/LiViMotion/blob/main/LICENSE
 */
// ----------------------------------------------------------------

#pragma once

#include <opencv2/opencv.hpp>
#include <chrono>
#include <vector>

/**
 * @namespace mctl
 * @brief 動画の再生制御関連の名前空間
 */
namespace mctl
{
    /// 動画の巻き戻し・切り替えの操作を受け付けない時間 (クールダウン) [秒]
    constexpr int MOVIE_CONTROL_COOLDOWN_SECONDS = 3;

    /**
     * @brief 動画の再生制御と、OpenGL テクスチャへの転送を行うクラス
     */
    struct MovieController
    {
        int movieID; ///< 現在再生中の動画を識別する ID。0 〜 videos.size() - 1 の範囲
        bool isPaused; ///< 動画が一時停止中かどうかのフラグ
        int lidarID; ///< 動画フレームを転送する OpenGL のテクスチャ ID (名前は lidarID だが、テクスチャ ID として使う)


        /// 最後に巻き戻し・切り替えを行った時刻 (クールダウンの判定に使う)
        std::chrono::steady_clock::time_point lastControlMovieTime;

        std::vector<cv::VideoCapture> videos; ///< 読み込み済みの動画 (添字が movieID に対応)
        std::vector<double> videoFrameCounts; ///< 各動画のフレーム数に関する値 (添字は videos と同じ)

        /// 緊急時用の矢印画像
        cv::Mat arrowImage;

        /**
         * @brief コントローラを初期化し、動画と矢印画像を読み込む
         *
         * - クールダウンを「経過済み」の状態にして、起動直後から操作できるようにする
         * - 動画 3 本 (dance / movie / cat) を読み込む
         * - lidarID のテクスチャの拡大・縮小フィルタを GL_NEAREST に設定する
         * - 緊急時用の矢印画像 (exit.png) を読み込む
         *
         * @param[in] lidarID 動画フレームを転送する OpenGL のテクスチャ ID
         *
         * @note ファイルのパスはすべて相対パスで、main() のあるプログラムからの相対パスで指定する。
         *       読み込みに失敗した場合はエラーを出力するだけで、処理は続行する。
         * @note OpenGL のコンテキストが作成済みの状態で呼び出すこと。
         */
        MovieController(const int lidarID);

        /// 緊急状態かどうかのフラグ。true の間は動画を更新せず、矢印画像を表示する
        bool isEmergency = false;

        /**
         * @brief 巻き戻し・切り替えがクールダウン中かどうかを判定する
         * @param[in] cooldownSeconds クールダウンの長さ [秒]
         * @retval true  最後の操作から cooldownSeconds 秒が経っていない (操作を受け付けない)
         * @retval false クールダウンが終わっている (操作できる)
         */
        bool isMovieControlCooldown(const int cooldownSeconds) const;

        /**
         * @brief クールダウンの起点を現在時刻にリセットする
         */
        void resetMovieControlCooldown();

        /**
         * @brief 動画ファイルを開いて、動画の一覧の末尾に追加する
         *
         * @param[in] newMovieID 追加する動画の ID (0 以上で、未使用であること)
         * @param[in] videoPath  動画ファイルのパス (main() のあるプログラムからの相対パス)
         * @retval true  追加に成功した
         * @retval false ID が不正・使用済み、または動画を開けなかった
         *
         * @note 動画は常に videos の末尾に追加され、実際の ID は追加順 (添字) で決まる。
         *       newMovieID には、現在の videos.size() を指定すること。
         */
        bool openVideo(const int newMovieID, const std::string &videoPath);

        /**
         * @brief 現在の状態に応じて、テクスチャに転送する内容を更新する (毎フレーム呼ぶ)
         *
         * - 緊急状態: 動画は更新せず、矢印画像をテクスチャに表示する
         * - 一時停止中: 現在のフレームを再取得してテクスチャに転送する
         * - 通常: 次のフレームを読み出してテクスチャに転送する。動画の最後まで来たら先頭に戻る
         *
         * @param[in] isEmergency 緊急状態かどうか。内部で setEmergency() に渡される
         * @retval true  更新に成功した (緊急状態の場合も true)
         * @retval false movieID が不正、またはフレームを読み込めなかった
         */
        bool updateMovieFrame(const bool isEmergency);

        /**
         * @brief 動画を一時停止する
         */
        void pauseMovie();

        /**
         * @brief 動画の再生を再開する
         */
        void resumeMovie();

        /**
         * @brief 現在の動画を先頭に巻き戻す
         *
         * クールダウン中は何もしない。巻き戻した場合はクールダウンをリセットする。
         */
        void resetMovie();

        /**
         * @brief 次の動画に切り替える
         *
         * 最後の動画の次は先頭の動画に戻る (循環)。
         * クールダウン中は何もしない。切り替えた場合はクールダウンをリセットする。
         */
        void changeMovieID();

        /**
         * @brief 緊急状態のフラグを切り替える
         *
         * - false → true: 矢印画像をテクスチャに転送し、フラグを true にする。
         *   矢印画像が読み込まれていない場合はエラーを出力し、フラグは変更しない
         * - true → false: フラグを false に戻す (テクスチャは次の updateMovieFrame() で動画に戻る)
         * - 状態が変わらない場合は何もしない
         *
         * @param[in] isEmergency 新しい緊急状態
         */
        void setEmergency(const bool isEmergency);
    };
}
