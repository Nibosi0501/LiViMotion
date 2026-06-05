// ----------------------------------------------------------------
// ReceiveUdp.hpp
//
// UDP で LIDAR と YOLO のデータを受信するクラスの宣言
//
// Copyright (c) 2026 Maito Yasui
// Released under the GNU GPL-3.0 License.
// https://github.com/Nibosi0501/LiViMotion/blob/main/LICENSE
// ----------------------------------------------------------------

#pragma once

#include <span>
#include <vector>

namespace rcv
{
    struct RecvBase
    {
    protected:
        int sock;
        RecvBase(const int port);
        ~RecvBase();
        ssize_t receiveLatest(std::vector<uint8_t> &outBuffer);
    };

    constexpr size_t LIDAR_HEADER_SIZE = sizeof(uint32_t);
    constexpr size_t LIDAR_BODY_SIZE = 2 * sizeof(float); // x, y

    struct RecvLidar : RecvBase
    {
        RecvLidar(const int port) : RecvBase(port) {}

        bool receive(std::vector<std::pair<float, float>> &outPoints);
        bool parse_packet(std::span<const uint8_t> packet, std::vector<std::pair<float, float>> &outPoints);
    };

    constexpr size_t YOLO_HEADER_SIZE = sizeof(uint32_t);
    constexpr size_t YOLO_BODY_SIZE = 17 * 3 * sizeof(float);

    struct RecvYolo : RecvBase
    {
        RecvYolo(const int port) : RecvBase(port) {}

        bool receive(std::vector<std::vector<std::tuple<float, float, float>>> &outKeypoints_list);
        bool parse_packet(std::span<const uint8_t> packet, std::vector<std::vector<std::tuple<float, float, float>>> &outKeypoints_list);
    };
}