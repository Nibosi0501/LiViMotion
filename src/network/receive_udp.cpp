/**
 * @file receive_udp.cpp
 * @brief UDP 受信クラス (RecvBase / RecvLidar / RecvYolo) の実装
 * @see ReceiveUdp.hpp
 * 
 * Copyright (c) 2026 Maito Yasui
 * Released under the GNU AGPL-3.0 License.
 * https://github.com/Nibosi0501/LiViMotion/blob/main/LICENSE
 */
#include <iostream>
#include <netinet/in.h>
#include <unistd.h>
#include <fcntl.h>

#include "../../include/network/ReceiveUdp.hpp"

namespace rcv
{
    /**
     * @details
     * 処理の流れ:
     * -# UDP ソケットを作成する
     * -# INADDR_ANY の指定ポートに bind する
     * -# O_NONBLOCK を設定してノンブロッキングにする
     *
     * いずれかが失敗した場合はエラーメッセージを出力して std::exit(1) する。
     */
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

    /**
     * @brief ソケットを閉じる
     */
    RecvBase::~RecvBase()
    {
        close(sock);
    }

    /**
     * @details
     * ノンブロッキングソケットに対して recvfrom() を繰り返し、
     * 戻り値が 0 以下 (キューが空、またはエラー) になった時点で終了する。
     * 同じバッファに上書きし続けるため、最終的に最新パケットのみが残る。
     *
     * @param[out] outBuffer 受信バッファ (サイズは呼び出し側で確保)
     * @return 最後に受信したパケットのバイト数。受信できなかった場合は -1
     */
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

    /**
     * @details
     * UDP の最大ペイロード (65535 byte) のバッファを用意し、
     * receiveLatest() で最新パケットを取得して parse_packet() に渡す。
     *
     * @param[out] outPoints 検出点 (x, y) のリスト
     * @retval true  受信・解析ともに成功
     * @retval false パケットなし、または解析失敗
     */
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

    /**
     * @details
     * 検証の順序:
     * -# パケットサイズがヘッダー (4 byte) 以上であること
     * -# 検出点数 (int32_t) が 0 以上であること
     * -# パケットサイズが「ヘッダー + 点数 × 8 byte」以上であること
     *
     * 検証を通過したら、ボディを 8 byte (float × 2) ずつ読み出して outPoints に追加する。
     * 読み出しは memcpy 経由で行い、アライメント違反を避けている。
     *
     * @param[in]  packet    受信したパケット全体
     * @param[out] outPoints 検出点 (x, y) のリスト (先頭でクリアされる)
     * @retval true  解析成功
     * @retval false 検証に失敗 (理由は標準エラー出力に出力)
     */
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

    /**
     * @details
     * UDP の最大ペイロード (65535 byte) のバッファを用意し、
     * receiveLatest() で最新パケットを取得して parse_packet() に渡す。
     *
     * @param[out] outKeypoints_list 人物ごとのキーポイント列
     * @retval true  受信・解析ともに成功
     * @retval false パケットなし、または解析失敗
     */
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

    /**
     * @details
     * 検証の順序:
     * -# パケットサイズがヘッダー (4 byte) 以上であること
     * -# 検出人数 (int32_t) が 0 以上であること
     * -# パケットサイズが「ヘッダー + 人数 × 204 byte (17 点 × 3 float)」以上であること
     *
     * 検証を通過したら、1 人につき 17 個の (x, y, conf) を順に読み出す。
     * ヘッダーは subspan でスキップし、ボディだけのビューを進めながら読み進める。
     *
     * @param[in]  packet            受信したパケット全体
     * @param[out] outKeypoints_list 人物ごとのキーポイント列 (先頭でクリアされる)
     * @retval true  解析成功
     * @retval false 検証に失敗 (理由は標準エラー出力に出力)
     */
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
