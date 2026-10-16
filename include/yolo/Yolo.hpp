// ----------------------------------------------------------------
/**
 * @file Yolo.hpp
 * @brief YOLO 姿勢推定のキーポイントから足元位置・挙手を判定するクラスと関数の宣言
 *
 * 1 人分の 17 点キーポイント (COCO 形式) を保持し、
 * - 足首の位置から床面上の足元座標 (キャリブレーション後) を求める
 * - 肘・手首・肩の位置関係から左右の挙手を判定する
 * - 過去フレームの履歴 (std::deque) から挙手が継続しているかを判定する
 *
 * @par 使用する COCO キーポイントのインデックス
 * | 番号 | 部位   |
 * |------|--------|
 * | 5    | 左肩   |
 * | 6    | 右肩   |
 * | 7    | 左肘   |
 * | 8    | 右肘   |
 * | 9    | 左手首 |
 * | 10   | 右手首 |
 * | 15   | 左足首 |
 * | 16   | 右足首 |
 *
 * @note 画像座標系は y 軸が下向きのため、「手が上」は y 値が小さいことを意味する。
 *
 * Copyright (c) 2026 Maito Yasui
 * Released under the GNU AGPL-3.0 License.
 * https://github.com/Nibosi0501/LiViMotion/blob/main/LICENSE
 */
// ----------------------------------------------------------------

#pragma once

#include <vector>

#include "../calibration/Calibration.hpp"

/**
 * @namespace yolo
 * @brief YOLO 姿勢推定結果の処理に関する名前空間
 */
namespace yolo
{
    /**
     * @brief 1 人分の姿勢推定結果と、そこから求めた足元位置
     *
     * コンストラクタで足元位置 (footX, footY) と信頼度 (footConf) を計算する。
     * 足元位置が求められなかった場合や範囲外の場合は footConf が -1 になる。
     */
    struct Yolo
    {
        float footX;    ///< 足元の X 座標 (キャリブレーション・スケール変換後、有効範囲 -250 〜 250)
        float footY;    ///< 足元の Y 座標 (キャリブレーション・スケール変換後、有効範囲 0 〜 280)
        float footConf; ///< 足元位置の信頼度。座標が無効 (変換失敗・範囲外) の場合は -1
        /// 17 点のキーポイント。各要素は (x, y, 信頼度)
        std::vector<std::tuple<float, float, float>> keypoints;

        /**
         * @brief キーポイントから足元位置を計算して初期化する
         *
         * 足首のキーポイントから足元の画像座標を求め、calibration でウィンドウ座標に変換し、
         * calibration.attribute ("front" / "back" / それ以外) に応じて座標軸の向きとスケールを合わせる。
         *
         * @param[in] _keypoints  17 点のキーポイント (x, y, 信頼度)
         * @param[in] calibration 画像座標からウィンドウ座標への変換に使うキャリブレーション
         *
         * @note 変換に失敗した場合は footConf が -1 になり、footX / footY は設定されない (未初期化)。
         *       使用前に footConf を確認すること。
         */
        Yolo(const std::vector<std::tuple<float, float, float>> &_keypoints, const clb::Calibration &calibration);
        // void isHandsUp(bool &isLeftHandUp, bool &isRightHandUp) const;

        /**
         * @brief 左手が上がっているか判定する
         *
         * 左肘 (7) と左手首 (9) の信頼度がともに conf 以上で、
         * かつ手首が肘より上にある場合に挙手とみなす。
         *
         * @param[in] conf 判定に必要な最低信頼度
         * @retval true  左手が上がっている
         * @retval false 上がっていない、または信頼度が足りない
         */
        bool isLeftHandUp(const float conf) const;

        /**
         * @brief 右手が上がっているか判定する
         *
         * 右肘 (8) と右手首 (10) の信頼度がともに conf 以上で、
         * かつ手首が肘より上にある場合に挙手とみなす。
         *
         * @param[in] conf 判定に必要な最低信頼度
         * @retval true  右手が上がっている
         * @retval false 上がっていない、または信頼度が足りない
         */
        bool isRightHandUp(const float conf) const;

    private:
        /**
         * @brief 足首のキーポイントから足元の画像座標と信頼度を求める
         *
         * 左右の足首 (15, 16) の信頼度がともに 0.4 を超える場合は 2 点の平均を、
         * そうでなければ信頼度の高いほうの足首をそのまま採用する。
         *
         * @param[out] x    足元の x 座標 (画像座標)
         * @param[out] y    足元の y 座標 (画像座標)
         * @param[out] conf 足元の信頼度
         */
        void getFoot(float &x, float &y, float &conf) const;

        /**
         * @brief 値を区間 [in_min, in_max] から [out_min, out_max] へ線形変換する
         *
         * out_min > out_max を指定すると軸を反転できる。範囲外の値はクリップしない。
         *
         * @param[in] x       変換する値
         * @param[in] in_min  入力範囲の最小値
         * @param[in] in_max  入力範囲の最大値
         * @param[in] out_min 出力範囲の最小値 (in_min に対応)
         * @param[in] out_max 出力範囲の最大値 (in_max に対応)
         * @return 変換後の値
         */
        float mapValue(const float x, const float in_min, const float in_max, const float out_min, const float out_max) const;
    };

    // void isHandsUp(const std::vector<std::tuple<float, float, float>> &keypoints1, const std::vector<std::tuple<float, float, float>> &keypoints2, const float conf, bool &isLeftHandUp, bool &isRightHandUp);

    /**
     * @brief 2 組のキーポイントを使って左手が上がっているか判定する
     *
     * 左肩 (5) と左手首 (9) の信頼度が 4 点すべて conf 以上であることを要求する。
     * 左肩の信頼度が高いほうのキーポイントを採用し、手首が肩より上にあれば挙手とみなす。
     *
     * @param[in] keypoints1 1 組目のキーポイント (例: 一方の視点)
     * @param[in] keypoints2 2 組目のキーポイント (例: 別の視点)
     * @param[in] conf       判定に必要な最低信頼度
     * @retval true  左手が上がっている
     * @retval false 上がっていない、または信頼度が足りない
     */
    bool isLeftHandUp(const std::vector<std::tuple<float, float, float>> &keypoints1, const std::vector<std::tuple<float, float, float>> &keypoints2, const float conf);

    /**
     * @brief 2 組のキーポイントを使って右手が上がっているか判定する
     *
     * 右肩 (6) と右手首 (10) の信頼度が 4 点すべて conf 以上であることを要求する。
     * 右肩の信頼度が高いほうのキーポイントを採用し、手首が肩より上にあれば挙手とみなす。
     *
     * @param[in] keypoints1 1 組目のキーポイント (例: 一方の視点)
     * @param[in] keypoints2 2 組目のキーポイント (例: 別の視点)
     * @param[in] conf       判定に必要な最低信頼度
     * @retval true  右手が上がっている
     * @retval false 上がっていない、または信頼度が足りない
     */
    bool isRightHandUp(const std::vector<std::tuple<float, float, float>> &keypoints1, const std::vector<std::tuple<float, float, float>> &keypoints2, const float conf);

    /**
     * @brief 履歴の全フレームで右手が上がっているか判定する
     *
     * @param[in] historys 時系列の姿勢履歴
     * @param[in] conf     判定に必要な最低信頼度
     * @retval true  履歴が空でなく、全フレームで右手が上がっている
     * @retval false 履歴が空、または 1 フレームでも上がっていない
     */
    bool isRightHandUpInHistory(const std::deque<Yolo> &historys, const float conf);

    /**
     * @brief 2 つの履歴を組み合わせて、全フレームで右手が上がっているか判定する
     *
     * 先頭から短いほうの長さまでは、同じ添字のフレーム同士を
     * isRightHandUp(keypoints1, keypoints2, conf) で判定する。
     * 長いほうの残りのフレームは、そのフレーム単独で判定する。
     *
     * @param[in] historys1 1 つ目の時系列の姿勢履歴
     * @param[in] historys2 2 つ目の時系列の姿勢履歴
     * @param[in] conf      判定に必要な最低信頼度
     * @retval true  全フレームで右手が上がっている
     * @retval false 両方の履歴が空、または 1 フレームでも上がっていない
     */
    bool isRightHandUpInHistory(const std::deque<Yolo> &historys1, const std::deque<Yolo> &historys2, const float conf);
}
