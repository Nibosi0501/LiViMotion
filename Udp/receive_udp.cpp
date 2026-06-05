// ----------------------------------------------------------------
// receive_udp.cpp
//
// UDP で LIDAR と YOLO のデータを受信するクラスの実装
//
// Copyright (c) 2026 Maito Yasui
// Released under the GNU GPL-3.0 License.
// https://github.com/Nibosi0501/LiViMotion/blob/main/LICENSE
// ----------------------------------------------------------------

#include <iostream>
#include <netinet/in.h>
#include <unistd.h>
#include <fcntl.h>

#include "./ReceiveUdp.hpp"

namespace rcv
{
    RecvBase::RecvBase(const int port)
    {
        sock = socket(AF_INET, SOCK_DGRAM, 0);
        if (sock < 0)
        {
            std::cerr << "socket() failed\n";
            std::exit(1);
        }

        sockaddr_in addr{};
        addr.sin_family = AF_INET;
        addr.sin_port = htons(port);
        addr.sin_addr.s_addr = INADDR_ANY;

        if (bind(sock, reinterpret_cast<sockaddr *>(&addr), sizeof(addr)) < 0)
        {
            std::cerr << "bind() failed\n";
            close(sock);
            std::exit(1);
        }

        int flags = fcntl(sock, F_GETFL, 0);
        if (flags < 0)
        {
            std::cerr << "fcntl F_GETFL failed\n";
            close(sock);
            std::exit(1);
        }
        if (fcntl(sock, F_SETFL, flags | O_NONBLOCK) < 0)
        {
            std::cerr << "fcntl F_SETFL or O_NONBLOCK failed\n";
            close(sock);
            std::exit(1);
        }
    }

    RecvBase::~RecvBase()
    {
        close(sock);
    }

    ssize_t RecvBase::receiveLatest(std::vector<uint8_t> &outBuffer)
    {
        ssize_t last_n = -1;
        while (true)
        {
            ssize_t n = recvfrom(sock, outBuffer.data(), outBuffer.size(), 0, nullptr, nullptr);
            if (n <= 0)
            {
                break;
            }
            last_n = n;
        }
        return last_n;
    }

    bool RecvLidar::receive(std::vector<std::pair<float, float>> &outPoints)
    {
        std::vector<uint8_t> buf(65535);
        ssize_t last_n = receiveLatest(buf);
        if (last_n > 0)
        {
            std::span<const uint8_t> packet(buf.data(), last_n);
            return parse_packet(packet, outPoints);
        }
        return false;
    }

    bool RecvLidar::parse_packet(std::span<const uint8_t> packet, std::vector<std::pair<float, float>> &outPoints)
    {
        if (packet.size() < LIDAR_HEADER_SIZE)
        {
            std::cerr << "LIDAR から受信したパケットのヘッダーサイズが不正です" << std::endl;
            return false;
        }

        int32_t num_persons;
        std::memcpy(&num_persons, packet.data(), sizeof(int32_t));
        if (num_persons < 0)
        {
            std::cerr << "LIDAR から受信したパケットの人数が不正です: " << num_persons << std::endl;
            return false;
        }

        size_t expected = LIDAR_HEADER_SIZE + num_persons * LIDAR_BODY_SIZE;
        if (packet.size() < expected)
        {
            std::cerr << "LIDAR から受信したパケットのサイズが不正です" << std::endl;
            return false;
        }

        std::span<const uint8_t> body = packet.subspan(LIDAR_HEADER_SIZE);

        outPoints.clear();
        // 事前に必要なサイズを確保しておく
        outPoints.reserve(num_persons); // 後にemplace_back するので resize ではなく reserve を使う

        for (int i = 0; i < num_persons; i++)
        {
            float vals[2]; // x, y
            std::memcpy(vals, body.data(), sizeof(vals));
            outPoints.emplace_back(vals[0], vals[1]);
            body = body.subspan(sizeof(vals));
        }
        return true;
    }

    bool RecvYolo::receive(std::vector<std::vector<std::tuple<float, float, float>>> &outKeypoints_list)
    {
        std::vector<uint8_t> buf(65535);
        ssize_t last_n = receiveLatest(buf);
        if (last_n > 0)
        {
            std::span<const uint8_t> packet(buf.data(), last_n);
            return parse_packet(packet, outKeypoints_list);
        }
        return false;
    }

    bool RecvYolo::parse_packet(std::span<const uint8_t> packet, std::vector<std::vector<std::tuple<float, float, float>>> &outKeypoints_list)
    {
        if (packet.size() < YOLO_HEADER_SIZE)
        {
            std::cerr << "YOLO から受信したパケットのヘッダーサイズが不正です" << std::endl;
            return false;
        }

        int32_t num_persons;
        std::memcpy(&num_persons, packet.data(), sizeof(int32_t));
        if (num_persons < 0)
        {
            std::cerr << "YOLO から受信したパケットの人数が不正です: " << num_persons << std::endl;
            return false;
        }

        size_t expected = YOLO_HEADER_SIZE + num_persons * YOLO_BODY_SIZE;
        if (packet.size() < expected)
        {
            std::cerr << "YOLO から受信したパケットのサイズが不正です" << std::endl;
            return false;
        }

        // subspan でヘッダーをスキップしてボディだけのビューを作成
        std::span<const uint8_t> body = packet.subspan(YOLO_HEADER_SIZE);

        outKeypoints_list.clear();
        // 事前に必要なサイズを確保しておく
        outKeypoints_list.resize(num_persons); // 後に at(i).emplace_back するので reserve ではなく resize を使う

        for (int i = 0; i < num_persons; i++)
        {
            for (int j = 0; j < 17; j++)
            {
                float vals[3]; // x, y, conf
                std::memcpy(vals, body.data(), sizeof(vals));
                outKeypoints_list.at(i).emplace_back(vals[0], vals[1], vals[2]);
                body = body.subspan(sizeof(vals)); // 次のポイントへ移動
            }
        }
        return true;
    }
}