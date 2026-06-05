// ----------------------------------------------------------------
// SendUdp.hpp
//
// UDP で LIDAR のデータを送信するクラスの宣言
//
// Copyright (c) 2026 Maito Yasui
// Released under the GNU GPL-3.0 License.
// https://github.com/Nibosi0501/LiViMotion/blob/main/LICENSE
// ----------------------------------------------------------------

#pragma once

#include <string>
#include <vector>

namespace snd
{
    struct SendBase
    {
    protected:
        int sock;

        SendBase(const int port, const std::string &ip);
        ~SendBase();
    };

    struct SendLidar : SendBase
    {
        SendLidar(const int port, const std::string &ip) : SendBase(port, ip) {}

        bool send(const std::vector<std::pair<float, float>> &points);
    };
}