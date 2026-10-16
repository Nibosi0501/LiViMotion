// ----------------------------------------------------------------
/**
 * @file movieController.cpp
 * @brief mctl::MovieController の実装
 * @see MovieController.hpp
 *
 * Copyright (c) 2026 Maito Yasui
 * Released under the GNU AGPL-3.0 License.
 * https://github.com/Nibosi0501/LiViMotion/blob/main/LICENSE
 */
// ----------------------------------------------------------------

#include "../../include/render/MovieController.hpp"

#include <GLUT/glut.h>

namespace mctl
{
    /**
     * @details
     * 処理の流れ:
     * -# lastControlMovieTime を 10 秒前に設定し、起動直後はクールダウン中にならないようにする
     * -# openVideo() で動画 3 本 (dance.mov / movie.mov / cat.mov) を読み込む
     * -# lidarID のテクスチャを bind し、拡大・縮小フィルタを GL_NEAREST に設定する
     * -# 矢印画像 exit.png を cv::IMREAD_COLOR で読み込む (読み込めなければエラーを出力)
     *
     * @param[in] lidarID 動画フレームを転送する OpenGL のテクスチャ ID
     *
     * @note cv::IMREAD_COLOR は 3 チャンネル (BGR) で読み込むため、画像の透過情報 (アルファ) は失われる。
     */
    MovieController::MovieController(const int lidarID) : movieID(0), isPaused(false), lidarID(lidarID)
    {
        lastControlMovieTime = std::chrono::steady_clock::now() - std::chrono::seconds(10);

        openVideo(movieID, "./Movie/dance.mov");
        openVideo(movieID + 1, "./Movie/movie.mov");
        openVideo(movieID + 2, "./Movie/cat.mov");

        glBindTexture(GL_TEXTURE_2D, lidarID);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);

        // 背景透過ありで読み込み
        arrowImage = cv::imread("./Movie/exit.png", cv::IMREAD_COLOR);
        if (arrowImage.empty())
        {
            std::cerr << "矢印画像を読み込めませんでした" << std::endl;
            std::cerr << "画像ファイルのパスは、実行ファイルからの相対パスで指定してください" << std::endl;
        }
    }

    /**
     * @details lastControlMovieTime を現在時刻 (steady_clock) に更新する。
     */
    void MovieController::resetMovieControlCooldown()
    {
        lastControlMovieTime = std::chrono::steady_clock::now();
    }

    /**
     * @details 現在時刻と lastControlMovieTime の差 (秒) が cooldownSeconds 未満なら true を返す。
     */
    bool MovieController::isMovieControlCooldown(const int cooldownSeconds) const
    {
        auto now = std::chrono::steady_clock::now();
        double elapsed = std::chrono::duration<double>(now - lastControlMovieTime).count();
        return elapsed < cooldownSeconds;
    }

    /**
     * @details
     * 処理の流れ:
     * -# setEmergency() で緊急状態を反映する
     * -# 緊急状態なら、動画は更新せず true を返す (矢印画像は setEmergency() でテクスチャに転送済み)
     * -# movieID が範囲外ならエラーを出力して false を返す
     * -# 一時停止中: videos[movieID].retrieve() で現在のフレームを取り出す。
     *    空なら先頭に戻して再度取り出し、それでも空ならエラーを出力して false を返す
     * -# 通常: 次のフレームを読み出す。空 (動画の最後) なら先頭に戻して再度読み出す
     * -# 得られたフレームを BGR のまま glTexImage2D() で lidarID のテクスチャに転送する
     *
     * @param[in] isEmergency 緊急状態かどうか
     * @retval true  更新に成功した (緊急状態の場合も true)
     * @retval false movieID が不正、またはフレームを読み込めなかった
     *
     * @note 一時停止中の retrieve() は、直前に grab() された内容を取り出す関数である。
     *       先頭に戻した (set) 直後の retrieve() は grab() を挟んでいないため、
     *       期待したフレームが取れない可能性がある。
     */
    bool MovieController::updateMovieFrame(const bool isEmergency)
    {
        this->setEmergency(isEmergency);
        if (isEmergency)
        {
            // 緊急事態の場合は動画を更新せずに矢印画像を表示する
            // std::cout << "緊急事態のため動画を更新せずに矢印画像を表示します" << std::endl;
            return true;
        }

        if (movieID < 0 || static_cast<std::size_t>(movieID) >= videos.size())
        {
            std::cerr << "無効なmovieID: " << movieID << std::endl;
            std::cerr << "movieIDは0から" << videos.size() - 1 << "の範囲で指定してください" << std::endl;
            return false;
        }

        cv::Mat inputFrame;
        if (isPaused)
        {
            // 動画が一時停止中の場合は、現在のフレームを返す
            videos.at(movieID).retrieve(inputFrame);
            if (inputFrame.empty())
            {
                // 動画の最後で静止した場合は、最初のフレームを返す
                videos.at(movieID).set(cv::CAP_PROP_POS_FRAMES, 0);
                videos.at(movieID).retrieve(inputFrame);
                if (inputFrame.empty())
                {
                    std::cerr << "動画のフレームを読み込めませんでした: " << movieID << std::endl;
                    return false;
                }
            }

            glBindTexture(GL_TEXTURE_2D, lidarID);
            glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, inputFrame.cols, inputFrame.rows, 0, GL_BGR, GL_UNSIGNED_BYTE, inputFrame.data);

            return true;
        }

        videos.at(movieID) >> inputFrame;
        if (inputFrame.empty())
        {
            // 動画の最後まで再生したら、最初に戻る
            videos.at(movieID).set(cv::CAP_PROP_POS_FRAMES, 0);
            videos.at(movieID) >> inputFrame;
            if (inputFrame.empty())
            {
                std::cerr << "動画のフレームを読み込めませんでした: " << movieID << std::endl;
                return false;
            }
        }

        glBindTexture(GL_TEXTURE_2D, lidarID);
        glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, inputFrame.cols, inputFrame.rows, 0, GL_BGR, GL_UNSIGNED_BYTE, inputFrame.data);
        return true;
    }

    /**
     * @details
     * movieID が範囲外ならエラーを出力して戻る。クールダウン中も何もしない。
     * それ以外は、現在の動画の再生位置を先頭 (フレーム 0) に戻し、クールダウンをリセットする。
     */
    void MovieController::resetMovie()
    {
        if (movieID < 0 || static_cast<std::size_t>(movieID) >= videos.size())
        {
            std::cerr << "無効なmovieID: " << movieID << std::endl;
            std::cerr << "movieIDは0から" << videos.size() - 1 << "の範囲で指定してください" << std::endl;
            return;
        }

        if (isMovieControlCooldown(MOVIE_CONTROL_COOLDOWN_SECONDS))
        {
            return;
        }

        videos.at(movieID).set(cv::CAP_PROP_POS_FRAMES, 0);

        resetMovieControlCooldown();
    }

    /**
     * @details
     * 処理の流れ:
     * -# newMovieID が負、または既に使用済み (videos.size() 未満) ならエラーを出力して false を返す
     * -# videoPath の動画を開く。開けなければエラーを出力して false を返す
     * -# videos と videoFrameCounts の末尾に追加して true を返す
     *
     * @param[in] newMovieID 追加する動画の ID
     * @param[in] videoPath  動画ファイルのパス
     * @retval true  追加に成功した
     * @retval false ID が不正・使用済み、または動画を開けなかった
     */
    bool MovieController::openVideo(const int newMovieID, const std::string &videoPath)
    {
        if (newMovieID < 0)
        {
            std::cerr << "無効なmovieID: " << newMovieID << std::endl;
            return false;
        }
        if (static_cast<std::size_t>(newMovieID) < videos.size())
        {
            std::cerr << "このmovieIDは既に使用されています: " << newMovieID << std::endl;
            return false;
        }
        cv::VideoCapture newVideo;
        newVideo.open(videoPath);
        if (!newVideo.isOpened())
        {
            std::cerr << "動画を開けませんでした: " << videoPath << std::endl;
            std::cerr << "動画ファイルのパスは、main()関数のあるcppファイルからの相対パスで指定してください" << std::endl;
            return false;
        }
        videos.push_back(newVideo);

        videoFrameCounts.push_back(newVideo.get(cv::CAP_PROP_FRAME_COUNT));

        return true;
    }

    /**
     * @details isPaused を true にする。実際のフレームの固定は updateMovieFrame() で行われる。
     */
    void MovieController::pauseMovie()
    {
        isPaused = true;
    }

    /**
     * @details isPaused を false にする。
     */
    void MovieController::resumeMovie()
    {
        isPaused = false;
    }

    /**
     * @details
     * クールダウン中は何もしない。そうでなければ次の処理を行う。
     * -# 現在の動画の cv::CAP_PROP_FRAME_COUNT の値を videoFrameCounts に保存する
     * -# movieID を 1 つ進める (最後の動画の次は 0 に戻る)
     * -# 切り替え先の動画に対して、保存してある値を cv::CAP_PROP_FRAME_COUNT として設定する
     * -# クールダウンをリセットする
     *
     * @note 再生位置を保存・復元する意図に見えるが、使っているプロパティは再生位置
     *       (cv::CAP_PROP_POS_FRAMES) ではなく総フレーム数 (cv::CAP_PROP_FRAME_COUNT) である。
     *       総フレーム数は通常書き込みできないため、切り替え先の動画は直前の再生位置から再開される。
     */
    void MovieController::changeMovieID()
    {
        if (isMovieControlCooldown(MOVIE_CONTROL_COOLDOWN_SECONDS))
        {
            return;
        }

        videoFrameCounts.at(movieID) = videos.at(movieID).get(cv::CAP_PROP_FRAME_COUNT);

        if (static_cast<std::size_t>(movieID) == videos.size() - 1)
        {
            movieID = 0;
        }
        else
        {
            movieID += 1;
        }
        videos.at(movieID).set(cv::CAP_PROP_FRAME_COUNT, videoFrameCounts.at(movieID));

        resetMovieControlCooldown();
    }

    /**
     * @details
     * 状態の変化に応じて動作が分かれる。
     * - true → false: フラグを false にして戻る
     * - false → true: 矢印画像が空ならエラーを出力して戻る (フラグは false のまま)。
     *   空でなければ、矢印画像を glTexImage2D() で lidarID のテクスチャに転送し、フラグを true にする
     * - 状態が変わらない場合は何もしない
     *
     * @param[in] isEmergency 新しい緊急状態
     */
    void MovieController::setEmergency(const bool isEmergency)
    {
        if (this->isEmergency == true && isEmergency == false)
        {
            this->isEmergency = false;
            return;
        }
        if (this->isEmergency == false && isEmergency == true)
        {
            if (arrowImage.empty())
            {
                std::cerr << "矢印画像が読み込まれていないため、緊急事態の表示ができません" << std::endl;
                return;
            }
            glBindTexture(GL_TEXTURE_2D, lidarID);
            glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, arrowImage.cols, arrowImage.rows, 0, GL_BGR, GL_UNSIGNED_BYTE, arrowImage.data);
            this->isEmergency = true;
        }
    }
}
