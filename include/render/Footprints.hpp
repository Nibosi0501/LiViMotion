/**
 * @file Footprints.hpp
 * @brief 手を挙げた人の足元に出す「足跡」エフェクトの宣言
 *
 * 人が手を挙げると、その人の足元 (LiDAR の位置) に足跡のエフェクトを 1 つ出す。
 * 足跡は時間とともに大きくなりながら回転し、だんだん薄くなって消える。
 * 手の挙げ方によって、足跡のテクスチャが変わる。
 *
 * @par 手の挙げ方と足跡のテクスチャ
 * | 手の挙げ方 | printTextureID |
 * |------------|----------------|
 * | 左手のみ   | 0              |
 * | 右手のみ   | 1              |
 * | 両手       | 2              |
 *
 * @par 1 フレームの典型的な呼び出し順序
 * @code
 * win::updateHumans(lidarTrackers, humans);
 * win::changeFootPrint(humans, matchedTrackers1, matchedTrackers2, footPrints);
 * win::updateFootPrints(footPrints);
 * win::deleteFootPrints(footPrints);
 * @endcode
 * 
 * Copyright (c) 2026 Maito Yasui
 * Released under the GNU AGPL-3.0 License.
 * https://github.com/Nibosi0501/LiViMotion/blob/main/LICENSE
 */

#pragma once

#include "../lidar/Lidar.hpp"
#include "../tracker/Tracker.hpp"
#include "../tracker/Matching.hpp"

#include <chrono>
#include <vector>
#include <random>

/**
 * @namespace win
 * @brief 床面への描画 (ウィンドウ・足跡) 関連の名前空間
 */
namespace win
{
    /// 足跡の色の候補 (R, G, B。各 0.0 〜 1.0)。足跡を作るたびにランダムに 1 色を選ぶ
    inline std::vector<std::tuple<double, double, double>> footPrintColors = {
        {1.0, 1.0, 1.0},
        {1.0, 1.0, 0.0},
        {1.0, 0.0, 1.0},
        {0.0, 1.0, 1.0},
        {1.0, 0.5, 0.5},
        {0.5, 1.0, 0.5},
        {0.5, 0.5, 1.0}};

    inline std::random_device rd; ///< 乱数の種を作るための非決定的な乱数生成器
    inline std::mt19937 gen(rd()); ///< 足跡の色・角度を選ぶための乱数生成器 (メルセンヌ・ツイスタ)
    /// footPrintColors の添字 (0 〜 6) を一様に選ぶ分布。footPrintColors の要素数 (7) に合わせてある
    inline std::uniform_int_distribution<> dis(0, 6);

    /// 足跡の初期角度の候補 [度]。足跡を作るたびにランダムに 1 つ選ぶ
    inline std::vector<float> angles = {0.0f, 30.0f, 60.0f, 90.0f, 120.0f, 150.0f, 180.0f};
    /// angles の添字 (0 〜 6) を一様に選ぶ分布。angles の要素数 (7) に合わせてある
    inline std::uniform_int_distribution<> angleDis(0, 6);

    /**
     * @brief 床に表示される足跡エフェクト 1 つ分
     *
     * 作られた直後は size が 0 で、update() を呼ぶたびに大きくなり、回転し、薄くなる。
     * alpha が 0 以下になったら deleteFootPrints() で削除される。
     */
    struct FootPrint
    {
        float centerX = 0.0f;  ///< 足跡の中心の X 座標
        float centerY = -1.0f; ///< 足跡の中心の Y 座標
        float angle = 0.0f;    ///< 足跡の向き [度] (0 以上 360 未満)
        float size = 0.0f;     ///< 足跡の大きさ。update() のたびに増える
        float alpha = 1.5f;    ///< 足跡の不透明度。update() のたびに減り、0 以下で削除される
        /// 足跡の色 (R, G, B。各 0.0 〜 1.0)
        std::tuple<double, double, double> color = {1.0, 1.0, 1.0};
        int printTextureID = 0; ///< 足跡のテクスチャの種類 (0: 左手, 1: 右手, 2: 両手)
        /// 足跡を作った時刻。コンストラクタでは設定されず、現在は使われていない
        std::chrono::steady_clock::time_point createdTime;

        /**
         * @brief 足跡を作る
         *
         * @param[in] lidar            足跡を置く位置 (LiDAR の足元位置)
         * @param[in] _angle           足跡の初期角度 [度]
         * @param[in] _printTextureID  足跡のテクスチャの種類
         * @param[in] _color           足跡の色 (R, G, B)
         */
        FootPrint(const lidar::Lidar &lidar, float _angle, int _printTextureID, const std::tuple<double, double, double> &_color);

        /**
         * @brief 足跡を 1 フレーム分だけ進める
         *
         * - size を 4.0 増やす
         * - alpha を 0.015 減らす (初期値 1.5 から約 100 フレームで 0 以下になる)
         * - angle を 1.0 度回転させる (360 度に達したら 0 度に戻す)
         */
        void update();
    };

    /**
     * @brief すべての足跡を 1 フレーム分だけ進める
     * @param[in,out] footPrints 足跡の一覧
     */
    void updateFootPrints(std::vector<FootPrint> &footPrints);

    /**
     * @brief alpha が 0 以下になった足跡を削除する
     * @param[in,out] footPrints 足跡の一覧
     */
    void deleteFootPrints(std::vector<FootPrint> &footPrints);

    /**
     * @brief 足跡を出す 1 人分の状態
     *
     * 1 人の LiDAR トラッカーに 1 つ対応する。
     * 足跡の種類 (テクスチャ) と、連続して足跡を出さないためのクールダウンの時刻を持つ。
     */
    struct Human
    {
        int lidarID = -1;       ///< 対応する LiDAR トラッカーの ID
        int printTextureID = 0; ///< 次に出す足跡のテクスチャの種類 (0: 左手, 1: 右手, 2: 両手)
        int printSE_ID = -1;    ///< 効果音の ID (-1 は未設定。現在このファイルでは使われていない)

        // 前回スタンプ位置（変化検知用）
        float lastStampX = 0.0f; ///< 前回足跡を出した位置の X 座標 (現在は使われていない)
        float lastStampY = 0.0f; ///< 前回足跡を出した位置の Y 座標 (現在は使われていない)
        float stepThreshold = 30.0f; ///< 何px進んだらスタンプするか (現在は使われていない)

        /// 最後に足跡を出した時刻 (クールダウンの判定に使う)
        std::chrono::steady_clock::time_point lastcontrolTime;

        /**
         * @brief LiDAR トラッカーの ID を指定して初期化する
         * @param[in] _lidarID 対応する LiDAR トラッカーの ID
         */
        Human(const int _lidarID);

        /**
         * @brief 現在の printTextureID の足跡を、指定位置に 1 つ追加する
         *
         * 足跡の向きと色は、候補 (angles / footPrintColors) からランダムに選ぶ。
         * footPrints に追加するだけで、追加した足跡のその後の管理は行わない。
         *
         * @param[in]     lidar      足跡を置く位置 (LiDAR の足元位置)
         * @param[in,out] footPrints 足跡の一覧 (末尾に追加される)
         */
        // footPrintsに追記するだけ。以降の管理はしない
        void stamp(const lidar::Lidar &lidar,
                   std::vector<FootPrint> &footPrints);
    };

    /**
     * @brief LiDAR トラッカーに合わせて、Human の一覧を更新する
     *
     * - 見失ったフレーム数が 5 以下のトラッカーのうち、対応する Human がないものを追加する
     * - 対応するトラッカーが存在しなくなった Human を削除する
     *
     * @param[in]     lidarTrackers LiDAR トラッカーの一覧
     * @param[in,out] humans        Human の一覧
     */
    void updateHumans(const std::vector<tkr::LidarTracker> &lidarTrackers, std::vector<Human> &humans);

    /**
     * @brief 手の挙げ方に応じて、足跡を追加する
     *
     * 各 Human について、lidarID を持つマッチング済みトラッカーを 2 系統から探す。
     * - 両方に見つかった: 2 つの YOLO のキーポイントを組み合わせて判定する
     * - 片方だけ見つかった: その YOLO だけで判定する
     * - どちらにもない: その Human は何もしない
     *
     * 判定は、すべて信頼度の閾値 0.6 で行い、手の挙げ方でテクスチャの種類を決めて足跡を追加する
     * (両手: 2, 左手のみ: 0, 右手のみ: 1)。
     * 足跡は、LiDAR の最新位置 (両方見つかった場合は 1 系統目のもの) に置く。
     * 同じ Human が連続して足跡を出さないよう、前回から 2 秒以上経つまでは追加しない。
     *
     * @param[in,out] humans            Human の一覧
     * @param[in]     matchedTrackers1  1 系統目のマッチング済みトラッカー
     * @param[in]     matchedTrackers2  2 系統目のマッチング済みトラッカー
     * @param[in,out] footPrints        足跡の一覧 (追加される)
     */
    void changeFootPrint(std::vector<Human> &humans, const std::vector<mtc::MatchedTracker> &matchedTrackers1, const std::vector<mtc::MatchedTracker> &matchedTrackers2, std::vector<FootPrint> &footPrints);
}
