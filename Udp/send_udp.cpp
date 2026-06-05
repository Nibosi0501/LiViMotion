// ----------------------------------------------------------------
// send_udp.cpp
//
// UDP で LIDAR のデータを送信するクラスの実装
//
// Copyright (c) 2026 Maito Yasui
// Released under the GNU GPL-3.0 License.
// https://github.com/Nibosi0501/LiViMotion/blob/main/LICENSE
// ----------------------------------------------------------------

#include <arpa/inet.h>
#include <fcntl.h>
#include <iostream>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include "./SendUdp.hpp"

namespace snd
{
    SendBase::SendBase(const int port, const std::string &ip)
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
        addr.sin_addr.s_addr = inet_addr(ip.c_str());

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

        if (connect(sock, reinterpret_cast<sockaddr *>(&addr), sizeof(addr)) < 0)
        {
            std::cerr << "connect() failed\n";
            close(sock);
            std::exit(1);
        }
    }
    SendBase::~SendBase()
    {
        close(sock);
    }

    bool SendLidar::send(const std::vector<std::pair<float, float>> &points)
    {
        const int32_t num_points = static_cast<int32_t>(points.size());
        if (num_points < 0) // num_points == 0は、検出なしを意味するため、送信する
        {
            return false; // 送信するポイントがない場合は何もしない
        }
        const std::size_t payload_size = num_points * sizeof(std::pair<float, float>);
        const std::size_t totalSize = sizeof(num_points) + payload_size;

        std::vector<uint8_t> buf(totalSize);
        std::memcpy(buf.data(), &num_points, sizeof(num_points));
        std::memcpy(buf.data() + sizeof(num_points), points.data(), payload_size);

        // SendLidar::send() と ::send()は別物
        // ::send() は<sys.socket.h>の関数
        const ssize_t sent = ::send(sock, buf.data(), totalSize, 0);
        if (sent < 0)
        {
            if (errno == EAGAIN || errno == EWOULDBLOCK)
            {
                // ソケットが一時的に送信できない場合は無視して次回に期待
                return false;
            }
            else
            {
                // その他のエラーはログに出力
                std::cerr << "send() failed: " << strerror(errno) << "\n";
                return false;
            }
        }
        return true;
    }
}