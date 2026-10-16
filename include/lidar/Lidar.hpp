// ----------------------------------------------------------------
/**
 * @file Lidar.hpp
 * @brief LiDAR で検出した足元位置を保持するクラスの宣言
 *
 * Copyright (c) 2026 Maito Yasui
 * Released under the GNU AGPL-3.0 License.
 * https://github.com/Nibosi0501/LiViMotion/blob/main/LICENSE
 */
// ----------------------------------------------------------------

#pragma once

/**
 * @namespace lidar
 * @brief LiDAR の検出結果に関する名前空間
 */
namespace lidar
{
    /**
     * @brief LiDAR で検出した 1 人分の足元位置
     */
    struct Lidar
    {
        float footX; ///< 足元の X 座標
        float footY; ///< 足元の Y 座標

        /**
         * @brief 足元の座標を指定して初期化する
         * @param[in] _x 足元の X 座標
         * @param[in] _y 足元の Y 座標
         */
        Lidar(const float &_x, const float &_y);
    };
}
