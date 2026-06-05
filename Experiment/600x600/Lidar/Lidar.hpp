// ----------------------------------------------------------------
// Lidar.hpp
//
// 2D LiDARのデータ構造とコンストラクタの宣言
//
// Copyright (c) 2026 Maito Yasui
// Released under the GNU GPL-3.0 License.
// https://github.com/Nibosi0501/LiViMotion/blob/main/LICENSE
// ----------------------------------------------------------------

#pragma once

namespace lidar
{
    struct Lidar
    {
        float footX;
        float footY;

        Lidar(const float &_x, const float &_y);
    };
}