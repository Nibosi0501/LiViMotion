// ----------------------------------------------------------------
/**
 * @file FloorWindow.hpp
 * @brief 人の進行方向の先に投影する「床面ウィンドウ」の位置・角度の計算と、動画操作の判定の宣言
 *
 * LiDAR トラッカーの位置履歴から、人の少し先 (進行方向) にウィンドウを置く位置と向きを予測する。
 * ウィンドウは目標位置・目標角度に向かって滑らかに動き、各ウィンドウは動画を再生する。
 * さらに、YOLO で検出した手の挙げ方 (ジェスチャー) に応じて、動画の切り替え・巻き戻し・一時停止・再開を行う。
 *
 * @par 1 フレームの典型的な呼び出し順序
 * @code
 * win::updateFloorWindows(lidarTrackers, floorWindows);
 * win::deleteFloorWindows(lidarTrackers, floorWindows);
 * win::changeMovies(floorWindows, matchedTrackers1, matchedTrackers2);
 * @endcode
 *
 * @par ジェスチャーと動画操作
 * | 動作         | ジェスチャー                               |
 * |--------------|--------------------------------------------|
 * | 動画の切り替え | 両手を挙げる                               |
 * | 巻き戻し     | 右手を挙げ続ける (履歴のすべてのフレームで) |
 * | 一時停止     | 左手を挙げる                               |
 * | 再開         | 上記のいずれにも当てはまらない             |
 * 優先順位は、切り替え > 巻き戻し > 一時停止 > 再開。
 *
 * @note 角度はすべて度 (degree) で扱い、ウィンドウの表示角度は [-180, 180) に正規化される。
 * @note 位置・距離は LiDAR の座標系と同じ単位で扱う。
 *
 * Copyright (c) 2026 Maito Yasui
 * Copyright (c) 2026 Shinsei Yamada
 * Released under the GNU AGPL-3.0 License.
 * https://github.com/Nibosi0501/LiViMotion/blob/main/LICENSE
 */
// ----------------------------------------------------------------

#pragma once

#include <vector>
#include <chrono>

#include "../lidar/Lidar.hpp"
#include "../tracker/Tracker.hpp"
#include "../tracker/Matching.hpp"
#include "./MovieController.hpp"

/**
 * @namespace win
 * @brief 床面ウィンドウ関連の名前空間
 */
namespace win
{
    /**
     * @brief 人に追従して床に投影される 1 枚のウィンドウ (動画の表示領域)
     *
     * 1 人の LiDAR トラッカーに 1 つ対応する。
     * 現在位置 (centerX, centerY) と現在角度 (angle) は、目標値に向かって毎フレーム少しずつ近づく。
     */
    struct FloorWindow
    {
        float centerX = 0.0f;       ///< ウィンドウの現在の中心 X 座標
        float centerY = 0.0f;       ///< ウィンドウの現在の中心 Y 座標
        float angle = 0.0f;         ///< ウィンドウの現在の表示角度 [度] ([-180, 180) に正規化)
        float targetCenterX = 0.0f; ///< ウィンドウの目標の中心 X 座標
        float targetCenterY = 0.0f; ///< ウィンドウの目標の中心 Y 座標
        float targetVecX = 0.0f;    ///< 人の足元からウィンドウの予測位置への単位ベクトルの X 成分
        float targetVecY = 0.0f;    ///< 人の足元からウィンドウの予測位置への単位ベクトルの Y 成分
        float targetAngle = 0.0f;   ///< ウィンドウの目標の表示角度 [度]
        int lidarID = -1;           ///< 対応する LiDAR トラッカーの ID。初期化に失敗した場合は -1

        mctl::MovieController movie; ///< このウィンドウに表示する動画の制御 (テクスチャ ID にはトラッカーの ID を使う)

        /**
         * @brief LiDAR トラッカーから床面ウィンドウを作る
         *
         * トラッカーの履歴が 5 件そろっていて、位置・角度を計算できた場合に初期化する。
         * 履歴が足りない、または計算に失敗した場合は lidarID を -1 にして戻る。
         *
         * @param[in] lidarTracker 対応させる LiDAR トラッカー
         *
         * @note MovieController は引数のトラッカーの ID で必ず構築されるため、
         *       生成のたびに動画ファイルの読み込みが行われる。
         */
        FloorWindow(const tkr::LidarTracker &lidarTracker);
    };

    /**
     * @brief LiDAR トラッカーに合わせて、床面ウィンドウの目標値を更新し、現在値を目標に近づける
     *
     * 処理の流れ:
     * -# トラッカーが 0 件なら何もせず戻る
     * -# ウィンドウが 0 件なら、履歴が 5 件そろっているトラッカーごとに新規作成して戻る
     * -# 履歴が 5 件そろっている各トラッカーについて、対応するウィンドウがあれば
     *    目標の向き (targetVec) と目標角度 (targetAngle) を更新し、なければ新規作成する
     * -# 各ウィンドウの現在位置・現在角度を、目標に向かって少しだけ動かす
     *    (位置は 1 フレームあたり差の 10 %、角度は 1 フレームあたり最大 10 度)
     *
     * @param[in]     lidarTrackers LiDAR トラッカーの一覧
     * @param[in,out] floorWindows  床面ウィンドウの一覧
     */
    void updateFloorWindows(const std::vector<tkr::LidarTracker> &lidarTrackers, std::vector<FloorWindow> &floorWindows);

    /**
     * @brief 対応する LiDAR トラッカーがなくなったウィンドウを削除する
     *
     * - トラッカーが 0 件: すべてのウィンドウを削除する
     * - ウィンドウが 0 件: 何もしない
     * - それ以外: 対応するトラッカーの ID が存在しないウィンドウ、または lidarID が -1 のウィンドウを削除する
     *
     * @param[in]     lidarTrackers LiDAR トラッカーの一覧
     * @param[in,out] floorWindows  床面ウィンドウの一覧
     */
    void deleteFloorWindows(const std::vector<tkr::LidarTracker> &lidarTrackers, std::vector<FloorWindow> &floorWindows);

    /**
     * @brief 手の挙げ方に応じて、各ウィンドウの動画を操作する
     *
     * 各ウィンドウの lidarID に対応するマッチング済みトラッカー (LiDAR と YOLO のペア) を、
     * 2 系統のマッチング結果から探し、見つかり方によって判定の方法と閾値を変える。
     *
     * | 見つかった系統 | 判定                                                     | 信頼度の閾値                       |
     * |----------------|----------------------------------------------------------|------------------------------------|
     * | 両方           | 2 つの YOLO のキーポイントを組み合わせて判定             | 0.6                                |
     * | どちらか一方   | 見つかった側の YOLO だけで判定                           | 切り替え・巻き戻し 0.9、一時停止 0.7 |
     * | どちらも無し   | 何もしない                                               | -                                  |
     *
     * 動作の優先順位は、切り替え > 巻き戻し > 一時停止 > 再開。
     *
     * @param[in,out] floorWindows      床面ウィンドウの一覧
     * @param[in]     matchedTrackers1  1 系統目のマッチング済みトラッカー (例: 前方カメラ側)
     * @param[in]     matchedTrackers2  2 系統目のマッチング済みトラッカー (例: 後方カメラ側)
     */
    void changeMovies(std::vector<win::FloorWindow> &floorWindows, const std::vector<mtc::MatchedTracker> &matchedTrackers1, const std::vector<mtc::MatchedTracker> &matchedTrackers2);

    /**
     * @brief 位置の履歴から、ウィンドウを置く位置と表示角度を求める
     *
     * 履歴の 5 点のうち、古い順に 0 番目・2 番目・4 番目の 3 点を使う。
     * 3 点がほぼ一直線なら直線で外挿し (predictPositionOnStraight)、
     * そうでなければ曲線で外挿する (predictPositionOnLagrange)。
     *
     * @param[in]  historys  LiDAR の位置履歴 (ちょうど 5 件であること)
     * @param[out] winPosX   ウィンドウの予測位置の X 座標
     * @param[out] winPosY   ウィンドウの予測位置の Y 座標
     * @param[out] winAngle  ウィンドウの表示角度 [度]
     * @retval true  計算に成功した
     * @retval false 履歴が 5 件ではない
     * @author Maito Yasui
     * @author Shinsei Yamada
     */
    bool calculateFloorWindow(const std::deque<lidar::Lidar> &historys, float &winPosX, float &winPosY, float &winAngle);

    /**
     * @brief ラグランジュ補間の曲線の接線から、ウィンドウの表示角度を求める
     *
     * 曲線上のパラメータ t での接線ベクトルを求め、その向きから角度 [度] を計算する。
     * 進行方向に対して 90 度ずらした角度を返す (atan2 の結果から 90 度引く)。
     * 接線ベクトルの長さが 0.1 以下の場合は 0 度にする。
     *
     * @param[in]  p0        履歴の 0 番目の位置 (パラメータ t = 0 に対応)
     * @param[in]  p2        履歴の 2 番目の位置 (パラメータ t = 0.5 に対応)
     * @param[in]  p4        履歴の 4 番目の位置 (パラメータ t = 1 に対応)
     * @param[in]  t         接線を求める曲線のパラメータ
     * @param[out] winAngle  ウィンドウの表示角度 [度]
     * @author Maito Yasui
     * @author Shinsei Yamada
     */
    void calculateAngle(const std::pair<float, float> &p0, const std::pair<float, float> &p2, const std::pair<float, float> &p4, const double t, float &winAngle);

    /**
     * @brief 3 点がほぼ一直線上 (同じ向きに進んでいる) かどうかを判定する
     *
     * ベクトル p2→p4 と p0→p2 のコサイン類似度が threshold を超えていれば直線とみなす。
     * どちらかのベクトルの長さが極端に小さい (ほぼ静止) 場合は、類似度を 0 として扱う。
     *
     * @param[in] p0         履歴の 0 番目の位置
     * @param[in] p2         履歴の 2 番目の位置
     * @param[in] p4         履歴の 4 番目の位置
     * @param[in] threshold  直線とみなすコサイン類似度の閾値 (例: 0.9)
     * @retval true  コサイン類似度が threshold を超えている
     * @retval false 超えていない、またはほぼ静止している (閾値が 0 未満の場合を除く)
     * @author Maito Yasui
     * @author Shinsei Yamada
     */
    bool isStraight(const std::pair<float, float> &p0, const std::pair<float, float> &p2, const std::pair<float, float> &p4, const float threshold);

    /**
     * @brief 直線の動きを仮定して、ウィンドウの位置と角度を求める
     *
     * p2 から p4 へ向かう単位ベクトルの方向に、p4 から 150 離れた位置を予測位置とする。
     * 角度は進行方向から 90 度引いた値 [度]。p2 と p4 が同じ位置の場合、方向ベクトルは 0 として扱い、予測位置は p4 になる。
     *
     * @param[in]  p2        履歴の 2 番目の位置
     * @param[in]  p4        履歴の 4 番目 (最新) の位置
     * @param[out] winPosX   ウィンドウの予測位置の X 座標
     * @param[out] winPosY   ウィンドウの予測位置の Y 座標
     * @param[out] winAngle  ウィンドウの表示角度 [度]
     * @author Maito Yasui
     * @author Shinsei Yamada
     */
    void predictPositionOnStraight(const std::pair<float, float> &p2, const std::pair<float, float> &p4, float &winPosX, float &winPosY, float &winAngle);

    /**
     * @brief 曲がっている動きを仮定して、ラグランジュ補間でウィンドウの位置と角度を求める
     *
     * p0 (t = 0)・p2 (t = 0.5)・p4 (t = 1) を通る 2 次曲線を作り、
     * t を 1 より大きい範囲で探索して、p4 から約 150 (許容誤差 5) 離れた曲線上の点を予測位置とする。
     * 探索は最大 20 回で、t は 1.01 〜 3.0 の範囲に制限する。
     * 角度は、その位置での接線から calculateAngle() で求める。
     *
     * @param[in]  p0        履歴の 0 番目の位置
     * @param[in]  p2        履歴の 2 番目の位置
     * @param[in]  p4        履歴の 4 番目 (最新) の位置
     * @param[out] winPosX   ウィンドウの予測位置の X 座標
     * @param[out] winPosY   ウィンドウの予測位置の Y 座標
     * @param[out] winAngle  ウィンドウの表示角度 [度]
     * @author Maito Yasui
     * @author Shinsei Yamada
     */
    void predictPositionOnLagrange(const std::pair<float, float> &p0, const std::pair<float, float> &p2, const std::pair<float, float> &p4, float &winPosX, float &winPosY, float &winAngle);

    /**
     * @brief 動画を切り替えるジェスチャー (両手を挙げる) かを判定する (1 系統版)
     * @param[in] tracker  判定対象の YOLO トラッカー (最新の検出結果を使う)
     * @param[in] conf     判定に必要な最低信頼度
     * @retval true  左右の手がともに挙がっている
     * @retval false それ以外
     */
    bool isMovieChange(const tkr::YoloTracker &tracker, const float conf);

    /**
     * @brief 動画を切り替えるジェスチャー (両手を挙げる) かを判定する (2 系統版)
     *
     * 2 つのトラッカーの最新のキーポイントを組み合わせて判定する。
     *
     * @param[in] tracker1  1 つ目の YOLO トラッカー
     * @param[in] tracker2  2 つ目の YOLO トラッカー
     * @param[in] conf      判定に必要な最低信頼度
     * @retval true  左右の手がともに挙がっている
     * @retval false それ以外
     */
    bool isMovieChange(const tkr::YoloTracker &tracker1, const tkr::YoloTracker &tracker2, const float conf);

    /**
     * @brief 動画を一時停止するジェスチャー (左手を挙げる) かを判定する (1 系統版)
     * @param[in] tracker  判定対象の YOLO トラッカー (最新の検出結果を使う)
     * @param[in] conf     判定に必要な最低信頼度
     * @retval true  左手が挙がっている
     * @retval false それ以外
     */
    bool isMoviePause(const tkr::YoloTracker &tracker, const float conf);

    /**
     * @brief 動画を一時停止するジェスチャー (左手を挙げる) かを判定する (2 系統版)
     *
     * 2 つのトラッカーの最新のキーポイントを組み合わせて判定する。
     *
     * @param[in] tracker1  1 つ目の YOLO トラッカー
     * @param[in] tracker2  2 つ目の YOLO トラッカー
     * @param[in] conf      判定に必要な最低信頼度
     * @retval true  左手が挙がっている
     * @retval false それ以外
     */
    bool isMoviePause(const tkr::YoloTracker &tracker1, const tkr::YoloTracker &tracker2, const float conf);

    /**
     * @brief 動画を巻き戻すジェスチャー (右手を挙げ続ける) かを判定する (1 系統版)
     *
     * 最新の検出結果で右手が挙がっていて、かつ履歴のすべてのフレームでも右手が挙がっている場合に true。
     *
     * @param[in] tracker  判定対象の YOLO トラッカー
     * @param[in] conf     判定に必要な最低信頼度
     * @retval true  右手が挙がり続けている
     * @retval false それ以外
     */
    bool isMovieReset(const tkr::YoloTracker &tracker, const float conf);

    /**
     * @brief 動画を巻き戻すジェスチャー (右手を挙げ続ける) かを判定する (2 系統版)
     *
     * 2 つのトラッカーの履歴を組み合わせて、すべてのフレームで右手が挙がっているかを判定する。
     *
     * @param[in] tracker1  1 つ目の YOLO トラッカー
     * @param[in] tracker2  2 つ目の YOLO トラッカー
     * @param[in] conf      判定に必要な最低信頼度
     * @retval true  右手が挙がり続けている
     * @retval false それ以外
     */
    bool isMovieReset(const tkr::YoloTracker &tracker1, const tkr::YoloTracker &tracker2, const float conf);
}
