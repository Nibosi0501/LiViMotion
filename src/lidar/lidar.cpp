// ----------------------------------------------------------------
/**
 * @file lidar.cpp
 * @brief lidar::Lidar の実装
 * @see Lidar.hpp
 *
 * Copyright (c) 2026 Maito Yasui
 * Released under the GNU AGPL-3.0 License.
 * https://github.com/Nibosi0501/LiViMotion/blob/main/LICENSE
 */
// ----------------------------------------------------------------

#include "../../include/lidar/Lidar.hpp"

namespace lidar
{
    /**
     * @details メンバ初期化子リストで footX / footY に引数の値をそのまま設定する。
     *
     * @param[in] _x 足元の X 座標
     * @param[in] _y 足元の Y 座標
     */
    Lidar::Lidar(const float &_x, const float &_y) : footX(_x), footY(_y) {}
}
