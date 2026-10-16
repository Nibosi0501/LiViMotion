/**
 * @file send_udp.cpp
 * @brief UDP 送信クラス (SendBase / SendLidar) の実装
 * @see SendUdp.hpp
 * 
 * Copyright (c) 2026 Maito Yasui
 * Released under the GNU AGPL-3.0 License.
 * https://github.com/Nibosi0501/LiViMotion/blob/main/LICENSE
 */
#include <arpa/inet.h>
#include <fcntl.h>
#include <iostream>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include "../../include/network/SendUdp.hpp"

namespace snd
{
    /**
     * @details
     * 処理の流れ:
     * -# UDP ソケットを作成する
     * -# O_NONBLOCK を設定してノンブロッキングにする
     * -# 送信先アドレスに connect() する (以降は ::send() で宛先指定が不要になる)
     *
     * いずれかが失敗した場合はエラーメッセージを出力して std::exit(1) する。
     *
     * @param port 送信先ポート番号
     * @param ip   送信先 IPv4 アドレス (inet_addr で変換される)
     */
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

    /**
     * @brief ソケットを閉じる
     */
    SendBase::~SendBase()
    {
        close(sock);
    }

    /**
     * @details
     * 送信バッファを「ヘッダー (検出点数 int32_t) + 検出点の生データ」の順に組み立て、
     * 1 回の ::send() で送信する。
     * 検出点数 0 は「検出なし」を表すため、そのまま送信する。
     *
     * エラー処理:
     * - EAGAIN / EWOULDBLOCK: 一時的に送れないだけなので、ログを出さずに false を返す
     * - それ以外: strerror(errno) をログ出力して false を返す
     *
     * @note SendLidar::send() とグローバルの ::send() (<sys/socket.h>) は別の関数。
     *       ここでは明示的に ::send() を呼んでいる。
     *
     * @param[in] points 送信する検出点 (x, y) のリスト
     * @retval true  送信に成功した
     * @retval false 送信に失敗した
     */
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
