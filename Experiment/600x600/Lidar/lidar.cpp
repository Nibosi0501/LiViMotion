// ----------------------------------------------------------------
// lidar.cpp
//
// 2D LiDARのデータ構造とコンストラクタの実装
//
// Copyright (c) 2026 Maito Yasui
// Released under the GNU GPL-3.0 License.
// https://github.com/Nibosi0501/LiViMotion/blob/main/LICENSE
// ----------------------------------------------------------------

#include "./Lidar.hpp"

namespace lidar
{
    Lidar::Lidar(const float &_x, const float &_y) : footX(_x), footY(_y) {}
}