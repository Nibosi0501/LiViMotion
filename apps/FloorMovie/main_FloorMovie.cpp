//g++ -O3 -framework OpenGL -framework GLUT -DGL_SILENCE_DEPRECATION `pkg-config --cflags --libs opencv5` -std=c++20 -Wall -Wextra ./src/network/*.cpp ./src/calibration/*.cpp ./src/yolo/*.cpp ./src/lidar/*.cpp ./src/kalman/*.cpp ./src/core/*.cpp ./src/tracker/*.cpp ./src/render/*.cpp ./apps/FloorMovie/main_FloorMovie.cpp

// ----------------------------------------------------------------
/// @file main_FloorMovie.cpp
/// @brief 床面に動画ウィンドウを投影するプログラムのエントリーポイント
///
/// LiDAR と YOLO (前方・後方カメラ) の検出結果を UDP で受信し、人の足元の少し先に
/// 動画のウィンドウを投影する。手の挙げ方 (ジェスチャー) で動画の切り替え・巻き戻し・一時停止ができ、
/// 緊急時には動画の代わりに出口への矢印を表示する。
/// 描画は GLUT (OpenGL) で行い、データの受信と追跡は約 33 ms ごとのタイマー処理 (timer()) で行う。
///
/// @par ビルド方法
/// このファイルは main() を持つ単独のプログラムである。
/// main() を持つ他のファイル (Test_LidarData.cpp など) とは同時にコンパイルしないこと。
/// @code
/// g++ -O3 -framework OpenGL -framework GLUT -DGL_SILENCE_DEPRECATION `pkg-config --cflags --libs opencv5` -std=c++20 -Wall -Wextra ./src/network/*.cpp ./src/calibration/*.cpp ./src/yolo/*.cpp ./src/lidar/*.cpp ./src/kalman/*.cpp ./src/core/*.cpp ./src/tracker/*.cpp ./src/render/*.cpp ./apps/FloorMovie/main_FloorMovie.cpp
/// @endcode
///
/// @par 受信ポート (UDP)
/// | ポート | データ         |
/// |--------|----------------|
/// | 12345  | LiDAR          |
/// | 10001  | YOLO (前方)    |
/// | 10002  | YOLO (後方)    |
///
/// @par キー操作
/// | キー              | 動作                              |
/// |-------------------|-----------------------------------|
/// | q / Q / ESC       | プログラムを終了する              |
/// | f / F             | フルスクリーンにする              |
/// | e / E             | 緊急事態の開始・解除を切り替える  |
///
/// @par 実行時に必要なファイル
/// - ./Calibration/front_calibration600x600.txt, ./Calibration/back_calibration600x600.txt
/// - ./Movie/arrow3.png (緊急時の矢印画像)
/// - MovieController が読み込む動画・画像 (../../Movie/ 以下)
///
/// @par 座標系
/// - 検出範囲 (LiDAR・YOLO のウィンドウ座標): X が -300 〜 300、Y が 0 〜 600
/// - 描画範囲 (OpenGL の正射影): X が 546 〜 -546 (左右反転)、Y が 0 〜 691
///
/// Copyright (c) 2026 Maito Yasui
/// Released under the GNU AGPL-3.0 License.
/// https://github.com/Nibosi0501/LiViMotion/blob/main/LICENSE
// ----------------------------------------------------------------

#include <iostream>
#include <numbers>
#include <chrono>

#include <opencv2/opencv.hpp>
#include <GLUT/glut.h>
//#include <AL/alut.h> //OpenAL

#include <cstdlib>

#include "../../include/network/ReceiveUdp.hpp"
#include "../../include/calibration/Calibration.hpp"

#include "../../include/yolo/Yolo.hpp"
#include "../../include/lidar/Lidar.hpp"
#include "../../include/kalman/Kalman.hpp"
#include "../../include/tracker/Tracker.hpp"
#include "../../include/tracker/Matching.hpp"
#include "../../include/render/FloorWindow.hpp"
#include "../../include/render/MovieController.hpp"

// GLUT
void initGL();
void display0();
void reshape0(int w, int h);
void timer(int value);
void keyboard0(unsigned char key, int x, int y);

// OpenAL
//void initAL();

template <typename T>
void drawRawData(const T &data, const float r, const float g, const float b);

template <typename T>
void drawTracker(const T &tracker, const float r, const float g, const float b, const bool isID, const bool isHistory);

void drawTrackerID(const int id);

template <typename T>
void drawHistorys(const std::deque<T> &historys, const float r, const float g, const float b);

void drawMatchedTracker(const mtc::MatchedTracker &tracker, const bool isYolo, const bool isID);
void drawDoubleMatchedTrackers(const std::vector<mtc::MatchedTracker> &matchedTrackers1, const std::vector<mtc::MatchedTracker> &matchedTrackers2, const bool isYolo);

void drawFloorWindow(const win::FloorWindow &window);

/// GLUT のウィンドウ ID の配列 (ウィンドウは 1 つだけ)
int winID[1];

/// 前方カメラ用のキャリブレーション (7×7 のグリッド点、600×600 のウィンドウ座標へ変換、属性 "front")
clb::Calibration calibFront("./Calibration/front_calibration600x600.txt", 7, 7, 600, 600, "front");
/// 後方カメラ用のキャリブレーション (7×7 のグリッド点、600×600 のウィンドウ座標へ変換、属性 "back")
clb::Calibration calibBack("./Calibration/back_calibration600x600.txt", 7, 7, 600, 600, "back");

/// LiDAR の UDP 受信 (ポート 12345)
rcv::RecvLidar recvLidar(12345);
/// 最新のフレームの LiDAR 検出結果 (検出範囲外の点は除外済み)
std::vector<lidar::Lidar> currentLidars;

/// 前方カメラの YOLO の UDP 受信 (ポート 10001)
rcv::RecvYolo recvYoloFront(10001);
/// 最新のフレームの前方カメラの YOLO 検出結果 (足元の信頼度が負のものは除外済み)
std::vector<yolo::Yolo> currentYoloFronts;

/// 後方カメラの YOLO の UDP 受信 (ポート 10002)
rcv::RecvYolo recvYoloBack(10002);
/// 最新のフレームの後方カメラの YOLO 検出結果 (足元の信頼度が負のものは除外済み)
std::vector<yolo::Yolo> currentYoloBacks;

/// LiDAR トラッカーの ID の発行元
tkr::IDGenerator lidarIDGen;
/// 前方カメラの YOLO トラッカーの ID の発行元
tkr::IDGenerator frontIDGen;
/// 後方カメラの YOLO トラッカーの ID の発行元
tkr::IDGenerator backIDGen;

/// LiDAR のトラッカー一覧
std::vector<tkr::LidarTracker> lidarTrackers;
/// 前方カメラの YOLO トラッカー一覧
std::vector<tkr::YoloTracker> yoloFrontTrackers;
/// 後方カメラの YOLO トラッカー一覧
std::vector<tkr::YoloTracker> yoloBackTrackers;

/// LiDAR トラッカーと前方カメラの YOLO トラッカーのマッチング結果
std::vector<mtc::MatchedTracker> matchedLidar2FrontTrackers;
/// LiDAR トラッカーと後方カメラの YOLO トラッカーのマッチング結果
std::vector<mtc::MatchedTracker> matchedLidar2BackTrackers;

/// 床に投影している動画ウィンドウの一覧
std::vector<win::FloorWindow> floorWindows;

/// 緊急事態かどうか (true の間は動画の代わりに出口への矢印を表示する)
bool isEmergency = false;

//ALuint source0; // 音源0

/**
 * @brief プログラムのエントリーポイント
 *
 * GLUT を初期化し、ウィンドウとコールバックを設定 (initGL()) して、イベントループに入る。
 * glutMainLoop() は戻らないため、通常は return 0 に到達しない (終了は keyboard0() の exit(0))。
 *
 * @param[in] argc コマンドライン引数の数 (GLUT に渡される)
 * @param[in] argv コマンドライン引数 (GLUT に渡される)
 * @return 0
 *
 * @note UDP の受信ソケットやキャリブレーションは、main() の前にグローバル変数として構築される。
 *       ソケットの作成・bind に失敗した場合は、main() に入る前に終了する。
 */
int main(int argc, char *argv[])
{
    // alutInit(&argc, argv);
    // initAL();

    glutInit(&argc, argv);
    initGL();

    glutMainLoop();

    return 0;
}

/*
void initAL()
{
    ALuint buffer;
    alGenBuffers(1, &buffer);

    buffer = alutCreateBufferFromFile("../../Movie/test.wav");
    alGenSources(1, &source0);
    alSourcei(source0, AL_BUFFER, buffer);

    alSourcei(source0, AL_LOOPING, AL_TRUE);
    alSourcef(source0, AL_REFERENCE_DISTANCE, 10.0);
    alSourcef(source0, AL_PITCH, 1.0);
    alSourcef(source0, AL_GAIN, 1.0);

    ALfloat orient[] = {0.0, 0.0, -1.0, 0.0, 1.0, 0.0};
    alListenerfv(AL_ORIENTATION, orient);
    alListener3f(AL_POSITION, 0.0, 0.0, 20.0);
}
*/

/**
 * @brief OpenGL と GLUT のウィンドウを初期化する
 *
 * 処理の流れ:
 * -# ダブルバッファ・深度バッファ付きの 800×500 のウィンドウを作成する
 * -# 描画・リサイズ・キー入力・タイマーのコールバックを登録する (タイマーは 33 ms)
 * -# 背景色、深度テスト、アルファブレンド、アルファテストを設定する
 * -# 緊急時用の矢印画像 (./Movie/arrow3.png) を読み込み、テクスチャ ID 10001 に転送する
 *
 * @note 矢印画像が読み込めない場合はエラーを出力するが、そのままテクスチャ転送に進む。
 * @note テクスチャ ID 10001 は、動画のテクスチャ (LiDAR トラッカーの ID: 0 〜 10000) と
 *       重ならない値として使っている。
 */
void initGL()
{
    glutInitDisplayMode(GLUT_RGBA | GLUT_DEPTH | GLUT_DOUBLE);
    glutInitWindowSize(800, 500);
    winID[0] = glutCreateWindow("LiDAR and YOLO Points");

    glutDisplayFunc(display0);
    glutReshapeFunc(reshape0);
    glutKeyboardFunc(keyboard0);
    glutTimerFunc(33, timer, 0);

    for (int i = 0; i < 1; i++)
    {
        glutSetWindow(winID[i]);

        glClearColor(0.0f, 0.0f, 0.0f, 1.0f);
        glEnable(GL_DEPTH_TEST);
        glEnable(GL_BLEND);
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
        glEnable(GL_ALPHA_TEST);
        glAlphaFunc(GL_GREATER, 0.01);
    }

    cv::Mat arrawImage = cv::imread("./Movie/arrow3.png", cv::IMREAD_UNCHANGED);
    if (arrawImage.empty())
    {
        std::cerr << "矢印画像を読み込めませんでした" << std::endl;
        std::cerr << "画像ファイルのパスは、実行ファイルからの相対パスで指定してください" << std::endl;
    }

    glBindTexture(GL_TEXTURE_2D, 10001);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, arrawImage.cols, arrawImage.rows, 0, GL_BGRA, GL_UNSIGNED_BYTE, arrawImage.data);
}

/**
 * @brief 描画コールバック: 画面を描画する
 *
 * 描画内容:
 * -# 検出範囲 (X: -300 〜 300、Y: 0 〜 600) の枠
 * -# 各床面ウィンドウ (drawFloorWindow())
 * -# 検出範囲の外側の床面映像を隠す黒い四角形 (左側・右側・上側)
 *
 * 検出点やトラッカーの描画は、デバッグ用にコメントアウトされている。
 */
void display0()
{
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glLoadIdentity();

    // スキャン範囲の枠を描画

    glBegin(GL_LINE_LOOP);
    glColor3f(0.3f, 0.3f, 0.3f);
    glVertex3f(300.0f, 0.0f, 0.0f);
    glVertex3f(300.0f, 600.0f, 0.0f);
    glVertex3f(-300.0f, 600.0f, 0.0f);
    glVertex3f(-300.0f, 0.0f, 0.0f);
    glEnd();

    // for (const auto &lidar : currentLidars)
    // {
    //     drawRawData(lidar, 1.0f, 0.0f, 0.0f);
    // }

    // for (const auto &yoloTracker : yoloFrontTrackers)
    // {
    //     drawTracker(yoloTracker, 0.0f, 0.0f, 1.0f, false, false);
    // }

    // for (const auto &yoloTracker : yoloBackTrackers)
    // {
    //     drawTracker(yoloTracker, 0.0f, 1.0f, 0.0f, false, false);
    // }

    // for (const auto &lidarTracker : lidarTrackers)
    // {
    //     drawTracker(lidarTracker, 1.0f, 1.0f, 1.0f, false, false);
    // }

    // drawDoubleMatchedTrackers(matchedLidar2FrontTrackers, matchedLidar2BackTrackers, true);

    for (const auto &floorWindow : floorWindows)
    {
        drawFloorWindow(floorWindow);
    }

    // for (const auto &yolo : currentYoloFronts)
    // {
    //     drawRawData(yolo, 0.0f, 0.0f, 1.0f);
    // }

    // yoloTrackersのkalmanYoloの予測点を描画

    /*
    for (const auto &yoloTracker : yoloFrontTrackers)
    {
        cv::Point2f predictedPoint = yoloTracker.kalmanYolo.predictedPoint;
        glPushMatrix();
        glTranslated(predictedPoint.x, predictedPoint.y, 0.0f);
        glScaled(10, 10, 0.0);
        glColor3f(0.0f, 1.0f, 0.0f);
        glutSolidSphere(1.0, 36, 18);
        glPopMatrix();
    }
        */

    // yoloTrackersのkalmanYoloの任意のフレーム先の予測点を描画

    // for (const auto &yoloTracker : yoloFrontTrackers)
    // {
    //     cv::Point2f futurePredictPoint;
    //     yoloTracker.kalmanYolo.futurePredict(futurePredictPoint, 4);
    //     glPushMatrix();
    //     // glTranslated(futurePredictPoint.x, futurePredictPoint.y, 0.0f);
    //     glTranslated(futurePredictPoint.x, yoloTracker.currentYolo.footY, 0.0f);
    //     glScaled(10, 10, 0.0);
    //     glColor3f(0.0f, 1.0f, 0.0f);
    //     glutSolidSphere(1.0, 36, 18);
    //     glPopMatrix();
    // }

    // スキャン範囲外の床面映像を塗りつぶす
    glBegin(GL_QUADS);
    glColor3f(0.0f, 0.0f, 0.0f);
    glVertex3f(-546.0f, 0.0f, 1.0f);
    glVertex3f(-301.0f, 0.0f, 1.0f);
    glVertex3d(-301.0f, 600.0f, 1.0f);
    glVertex3f(-546.0f, 600.0f, 1.0f);
    glEnd();
    glBegin(GL_QUADS);
    glColor3f(0.0f, 0.0f, 0.0f);
    glVertex3f(546.0f, 0.0f, 1.0f);
    glVertex3f(301.0f, 0.0f, 1.0f);
    glVertex3d(301.0f, 600.0f, 1.0f);
    glVertex3f(546.0f, 600.0f, 1.0f);
    glEnd();
    glBegin(GL_QUADS);
    glColor3f(0.0f, 0.0f, 0.0f);
    glVertex3f(546.0f, 601.0f, 1.0f);
    glVertex3f(-546.0f, 601.0f, 1.0f);
    glVertex3d(-546.0f, 691.0f, 1.0f);
    glVertex3f(546.0f, 691.0f, 1.0f);
    glEnd();

    // 出口座標を描画
    /*
        glPushMatrix();
        glTranslated(-546.0f, 691.0f, 2.0f);
        glScaled(10, 10, 0.0);
        glColor3f(1.0f, 1.0f, 1.0f);
        glutSolidSphere(1.0, 36, 18);
        glPopMatrix();
        */

    glutSwapBuffers();
}

/**
 * @brief リサイズコールバック: ビューポートと投影を設定する
 *
 * 正射影の範囲を X: 546 〜 -546 (左右反転)、Y: 0 〜 691 にする。
 *
 * @param[in] w ウィンドウの幅 [pixel]
 * @param[in] h ウィンドウの高さ [pixel]
 */
void reshape0(int w, int h)
{
    glViewport(0, 0, w, h);
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    glOrtho(546.0, -546.0, 0.0, 691.0, -3.0, 3.0);
    glMatrixMode(GL_MODELVIEW);
}

/**
 * @brief タイマーコールバック: データの受信・追跡・動画の更新を行う (約 33 ms ごと)
 *
 * 処理の流れ:
 * -# 次のタイマーを登録し、再描画を要求する
 * -# LiDAR を受信できたら、検出範囲外の点を除いて検出結果を作り、LiDAR トラッカーを更新・整理する
 * -# 前方カメラの YOLO を受信できたら、キャリブレーションを通して検出結果を作り、
 *    足元の信頼度が負のものを除いて、前方の YOLO トラッカーを更新・整理する
 * -# 後方カメラの YOLO も同様に処理する
 * -# いずれかを受信できたら、LiDAR と前方・後方の YOLO のマッチングと、床面ウィンドウの更新・削除を行う
 * -# YOLO のいずれかを受信できたら、手の挙げ方に応じて動画を操作する (win::changeMovies())
 * -# 各ウィンドウの動画のフレームを更新する
 * -# 緊急事態中に、前方・後方の YOLO トラッカーがどちらも空になったら、緊急事態を自動で解除する
 *
 * 受信できなかった系統は、前回の状態のまま処理する。
 *
 * @param[in] value 未使用 (glutTimerFunc() に渡した値)
 */
void timer([[maybe_unused]] int value)
{
    glutSetWindow(winID[0]);
    glutTimerFunc(33, timer, 0);
    glutPostRedisplay();

    std::vector<std::pair<float, float>> recvLidarPoints;
    bool gotLidar = recvLidar.receive(recvLidarPoints);
    if (gotLidar)
    {
        currentLidars.clear();
        if (recvLidarPoints.empty())
        {
            // std::cout << "No LiDAR points" << std::endl;
        }
        else
        {
            currentLidars.reserve(recvLidarPoints.size());
            for (const auto &[x, y] : recvLidarPoints)
            {
                if (x < -300.0f || x > 300.0f || y < 0.0f || y > 600.0f)
                {
                    continue; // 座標が範囲外のものはリストに追加しない
                }
                currentLidars.emplace_back(x, y);
            }
            // std::cout << "x: " << currentLidars.front().footX << ", y: " << currentLidars.front().footY << std::endl;
        }
        tkr::updateTrackers(lidarTrackers, currentLidars, lidarIDGen);
        tkr::deleteLostTrackers(lidarTrackers);
    }

    std::vector<std::vector<std::tuple<float, float, float>>> recvYoloFrontKeypointsList;
    bool gotYoloFront = recvYoloFront.receive(recvYoloFrontKeypointsList);
    if (gotYoloFront)
    {
        currentYoloFronts.clear();
        if (recvYoloFrontKeypointsList.empty())
        {
            // そのフレームでYOLOはキーポイントを検出しなかった
            // std::cout << "No YOLO keypoints Front Camera" << std::endl;
        }
        else
        {
            currentYoloFronts.reserve(recvYoloFrontKeypointsList.size());
            for (const auto &keypoints : recvYoloFrontKeypointsList)
            {
                currentYoloFronts.emplace_back(keypoints, calibFront);
                if (currentYoloFronts.back().footConf < 0)
                {
                    currentYoloFronts.pop_back(); // 信頼度が-1のものはリストから削除
                }

                // std::cout << "Front Camera - footX: " << currentYoloFronts.back().footX << ", footY: " << currentYoloFronts.back().footY << ", conf: " << currentYoloFronts.back().footConf << std::endl;
            }
        }
        tkr::updateTrackers(yoloFrontTrackers, currentYoloFronts, frontIDGen);
        tkr::deleteLostTrackers(yoloFrontTrackers);
    }

    std::vector<std::vector<std::tuple<float, float, float>>> recvYoloBackKeypointsList;
    bool gotYoloBack = recvYoloBack.receive(recvYoloBackKeypointsList);
    if (gotYoloBack)
    {
        currentYoloBacks.clear();
        if (recvYoloBackKeypointsList.empty())
        {
            // std::cout << "No YOLO keypoints Back Camera" << std::endl;
        }
        else
        {
            currentYoloBacks.reserve(recvYoloBackKeypointsList.size());
            for (const auto &keypoints : recvYoloBackKeypointsList)
            {
                currentYoloBacks.emplace_back(keypoints, calibBack);
                if (currentYoloBacks.back().footConf < 0)
                {
                    currentYoloBacks.pop_back(); // 信頼度が-1のものはリストから削除
                }
            }
        }
        tkr::updateTrackers(yoloBackTrackers, currentYoloBacks, backIDGen);
        tkr::deleteLostTrackers(yoloBackTrackers);
    }

    glutSetWindow(winID[0]);
    if (gotLidar || gotYoloFront || gotYoloBack)
    {
        mtc::updateMatchedTrackers(matchedLidar2FrontTrackers, lidarTrackers, yoloFrontTrackers);
        mtc::updateMatchedTrackers(matchedLidar2BackTrackers, lidarTrackers, yoloBackTrackers);

        win::updateFloorWindows(lidarTrackers, floorWindows);
        win::deleteFloorWindows(lidarTrackers, floorWindows);
    }
    if (gotYoloFront || gotYoloBack)
    {
        win::changeMovies(floorWindows, matchedLidar2FrontTrackers, matchedLidar2BackTrackers);
    }

    for (auto &window : floorWindows)
    {
        window.movie.updateMovieFrame(isEmergency);
    }

    // 緊急事態の時、yoloTrackersが空の時は、緊急事態を解除する
    if (isEmergency)
    {
        if (yoloFrontTrackers.empty() && yoloBackTrackers.empty())
        {
            isEmergency = false;
            std::cout << "全ての人が部屋から出ました" << std::endl;
            std::cout << "緊急事態解除！" << std::endl;
            //alSourceStop(source0);
            for (auto &window : floorWindows)
            {
                window.movie.setEmergency(isEmergency);
            }
        }
    }
}

/**
 * @brief キーボードコールバック: キー入力に応じて動作する
 *
 * | キー        | 動作                                                              |
 * |-------------|-------------------------------------------------------------------|
 * | ESC / q / Q | プログラムを終了する                                              |
 * | f / F       | フルスクリーンにする                                              |
 * | e / E       | 緊急事態の開始・解除を切り替え、全ウィンドウの動画に反映する      |
 *
 * @param[in] key 押されたキー
 * @param[in] x   キーを押したときのマウスの X 座標 (未使用)
 * @param[in] y   キーを押したときのマウスの Y 座標 (未使用)
 */
void keyboard0(unsigned char key, [[maybe_unused]] int x, [[maybe_unused]] int y)
{
    switch (key)
    {
    case 27:
    case 'q':
    case 'Q':
        exit(0);
    case 'f':
    case 'F':
        glutFullScreen();
        break;
    case 'e':
    case 'E':
        if (isEmergency == false)
        {
            isEmergency = true;
            std::cout << "緊急事態発生！" << std::endl;
            //alSourcePlay(source0);
        }
        else
        {
            isEmergency = false;
            std::cout << "緊急事態解除！" << std::endl;
            //alSourceStop(source0);
        }

        for (auto &window : floorWindows)
        {
            window.movie.setEmergency(isEmergency);
        }
        break;
    default:
        break;
    }
}

// 床面映像を描画する関数群
/**
 * @brief マッチング済みトラッカーを描画する (デバッグ用)
 *
 * LiDAR トラッカーを赤、YOLO トラッカーを青で描画する。
 *
 * @param[in] tracker  描画するマッチング済みトラッカー
 * @param[in] isYolo   YOLO トラッカーも描画するか
 * @param[in] isID     LiDAR トラッカーの ID も描画するか
 */
void drawMatchedTracker(const mtc::MatchedTracker &tracker, const bool isYolo, const bool isID)
{
    drawTracker(tracker.lidarTracker, 1.0f, 0.0f, 0.0f, isID, false);
    if (isYolo)
    {
        drawTracker(tracker.yoloTracker, 0.0f, 0.0f, 1.0f, false, false);
    }
}

/**
 * @brief 2 系統の YOLO の両方とマッチングした LiDAR トラッカーを描画する (デバッグ用)
 *
 * 片方の系統にしかマッチングしていない LiDAR トラッカーは描画しない。
 * 両方にマッチングした LiDAR トラッカーは緑 (ID 付き)、YOLO トラッカーは青で描画する。
 * どちらかの一覧が空の場合は何もしない。
 *
 * @param[in] matchedTrackers1 1 系統目のマッチング済みトラッカー
 * @param[in] matchedTrackers2 2 系統目のマッチング済みトラッカー
 * @param[in] isYolo           YOLO トラッカーも描画するか
 */
void drawDoubleMatchedTrackers(const std::vector<mtc::MatchedTracker> &matchedTrackers1, const std::vector<mtc::MatchedTracker> &matchedTrackers2, const bool isYolo)
{
    if (matchedTrackers1.empty() || matchedTrackers2.empty())
    {
        return;
    }

    std::vector<mtc::MatchedTracker> baseTrackers;
    std::vector<mtc::MatchedTracker> otherTrackers;
    if (matchedTrackers1.size() >= matchedTrackers2.size())
    {
        baseTrackers = matchedTrackers1;
        otherTrackers = matchedTrackers2;
    }
    else
    {
        baseTrackers = matchedTrackers2;
        otherTrackers = matchedTrackers1;
    }

    for (const auto &baseTracker : baseTrackers)
    {
        int lidarID = baseTracker.lidarTracker.id;
        auto it = std::find_if(otherTrackers.begin(), otherTrackers.end(), [lidarID](const mtc::MatchedTracker &tracker)
                               { return tracker.lidarTracker.id == lidarID; });

        if (it == otherTrackers.end())
        {
            // lidarTrackerは1つのYoloTrackerとしか紐付いていない
            continue;
        }
        else
        {
            // lidarTrackerは２つのYoloTrackerと紐付いている
            drawTracker(baseTracker.lidarTracker, 0.0f, 1.0f, 0.0f, true, false);
            if (isYolo)
            {
                drawTracker(baseTracker.yoloTracker, 0.0f, 0.0f, 1.0f, false, false);
                drawTracker(it->yoloTracker, 0.0f, 0.0f, 1.0f, false, false);
            }
        }
    }
}

/**
 * @brief トラッカーの現在位置を球で描画する (デバッグ用)
 *
 * 検出範囲外の位置や、足元の信頼度が負の YOLO は描画しない。
 *
 * @tparam T      tkr::LidarTracker または tkr::YoloTracker
 * @param[in] tracker    描画するトラッカー
 * @param[in] r          色の赤成分 (0.0 〜 1.0)
 * @param[in] g          色の緑成分 (0.0 〜 1.0)
 * @param[in] b          色の青成分 (0.0 〜 1.0)
 * @param[in] isID       トラッカーの ID も描画するか
 * @param[in] isHistory  位置の履歴も描画するか
 *
 * @note T が上記以外の型の場合は、エラーを出力して何も描画しない。
 */
template <typename T>
void drawTracker(const T &tracker, const float r, const float g, const float b, const bool isID, const bool isHistory)
{
    float footX, footY;
    if constexpr (std::is_same_v<T, tkr::LidarTracker>)
    {
        footX = tracker.currentLidar.footX;
        footY = tracker.currentLidar.footY;
    }
    else if constexpr (std::is_same_v<T, tkr::YoloTracker>)
    {
        footX = tracker.currentYolo.footX;
        footY = tracker.currentYolo.footY;

        if (tracker.currentYolo.footConf < 0)
        {
            return;
        }
    }
    else
    {
        std::cerr << "不明な型のTrackerが渡されました。" << std::endl;
        std::cerr << "drawTracker関数はtkr::LidarTrackerまたはtkr::YoloTrackerのインスタンスを受け取ることができます。" << std::endl;
        return;
    }

    if (footX < -300.0f || footX > 300.0f || footY < 0.0f || footY > 600.0f)
    {
        return;
    }

    glPushMatrix();
    glColor3f(r, g, b); // 文字の色の設定をする際には、glRasterPos2fの前にglColor3fを呼び出す必要がある.
    glTranslated(footX, footY, 0);
    glScaled(10.0, 10.0, 0.0);
    glutSolidSphere(1.0, 36, 18);
    if (isID)
    {
        glRasterPos2f(0.0f, 15.0f);
        drawTrackerID(tracker.id);
    }
    glPopMatrix();

    if (isHistory)
    {
        drawHistorys(tracker.historys, r, g, b);
    }
}

/**
 * @brief 現在のラスタ位置に、トラッカーの ID を文字で描画する
 *
 * 呼び出す前に glRasterPos2f() で位置を設定しておくこと。ID が負の場合は何も描画しない。
 *
 * @param[in] id 描画するトラッカーの ID
 */
void drawTrackerID(const int id)
{
    if (id < 0)
    {
        return;
    }

    std::string idStr = std::to_string(id);
    for (const char &c : idStr)
    {
        glutBitmapCharacter(GLUT_BITMAP_HELVETICA_18, c);
    }
}

/**
 * @brief 位置の履歴を、球で描画する (デバッグ用)
 *
 * @tparam T      lidar::Lidar または yolo::Yolo
 * @param[in] historys  描画する履歴
 * @param[in] r         色の赤成分 (0.0 〜 1.0)
 * @param[in] g         色の緑成分 (0.0 〜 1.0)
 * @param[in] b         色の青成分 (0.0 〜 1.0)
 *
 * @note T が上記以外の型の場合は、エラーを出力して何も描画しない。
 */
template <typename T>
void drawHistorys(const std::deque<T> &historys, const float r, const float g, const float b)
{
    if constexpr (!(std::is_same_v<T, lidar::Lidar> || std::is_same_v<T, yolo::Yolo>))
    {
        std::cerr << "不明な型のhistorysが渡されました。" << std::endl;
        std::cerr << "drawHistorys関数はlidar::Lidarまたはyolo::Yoloのdequeを受け取ることができます。" << std::endl;
        return;
    }

    for (const auto &history : historys)
    {
        drawRawData(history, r, g, b);
    }
}

/**
 * @brief 検出結果 (LiDAR または YOLO) の足元位置を、球で描画する (デバッグ用)
 *
 * 検出範囲外の位置や、足元の信頼度が負の YOLO は描画しない。
 *
 * @tparam T      lidar::Lidar または yolo::Yolo
 * @param[in] data  描画する検出結果
 * @param[in] r     色の赤成分 (0.0 〜 1.0)
 * @param[in] g     色の緑成分 (0.0 〜 1.0)
 * @param[in] b     色の青成分 (0.0 〜 1.0)
 *
 * @note T が上記以外の型の場合は、エラーを出力して何も描画しない。
 */
template <typename T>
void drawRawData(const T &data, const float r, const float g, const float b)
{
    float footX, footY, footConf;
    if constexpr (std::is_same_v<T, lidar::Lidar>)
    {
        footX = data.footX;
        footY = data.footY;
    }
    else if constexpr (std::is_same_v<T, yolo::Yolo>)
    {
        footX = data.footX;
        footY = data.footY;
        footConf = data.footConf;
        if (footConf < 0)
        {
            return;
        }
    }
    else
    {
        std::cerr << "不明な型のデータが渡されました." << std::endl;
        std::cerr << "drawRawData()関数はlidar::Lidarまたはyolo::Yoloのインスタンスを受け取ることができます." << std::endl;
        return;
    }

    if (footX < -300.0f || footX > 300.0f || footY < 0.0f || footY > 600.0f)
    {
        return;
    }

    glPushMatrix();
    glTranslated(footX, footY, 0);
    glScaled(10.0, 10.0, 0.0);
    glColor3f(r, g, b);
    glutSolidSphere(1.0, 36, 18);
    glPopMatrix();
}

/**
 * @brief 床面ウィンドウ 1 枚分を描画する
 *
 * 描画内容:
 * -# ウィンドウの動画 (96×60 の四角形に、動画のテクスチャを貼って回転させて描画する)
 * -# 他の人のウィンドウの方向を示す印。この人から他の人までの距離が 300 未満の場合のみ、
 *    ウィンドウの中心から他の人の方向へ 60 離れた位置に点を描く。
 *    距離が 150 未満なら赤、150 以上 300 未満なら黄色
 * -# 緊急事態の場合は、ウィンドウの横に出口 (-546, 691) の方向を向いた矢印の画像
 *
 * lidarID が -1 のウィンドウは描画しない。
 *
 * @param[in] window 描画する床面ウィンドウ
 */
void drawFloorWindow(const win::FloorWindow &window)
{
    if (window.lidarID == -1)
    {
        return;
    }
    glPushMatrix();
    glTranslated(window.centerX, window.centerY, 0);
    glRotatef(window.angle, 0, 0, 1);
    glScaled(16 * 6, 10 * 6, 0.0);
    glColor3f(1.0f, 1.0f, 1.0f);
    glEnable(GL_TEXTURE_2D);
    glBindTexture(GL_TEXTURE_2D, window.movie.lidarID);
    glBegin(GL_QUADS);

    glTexCoord2f(1.0f, 0.0f);
    glVertex2f(-0.5f, 0.5f);

    glTexCoord2f(0.0f, 0.0f);
    glVertex2f(0.5f, 0.5f);

    glTexCoord2f(0.0f, 1.0f);
    glVertex2f(0.5f, -0.5f);

    glTexCoord2f(1.0f, 1.0f);
    glVertex2f(-0.5f, -0.5f);

    glEnd();
    glDisable(GL_TEXTURE_2D);
    glPopMatrix();

    auto lidarIt = std::find_if(lidarTrackers.begin(), lidarTrackers.end(), [&window](const tkr::LidarTracker &tracker)
                                { return tracker.id == window.lidarID; });

    // window.centerX, window.centerYとlidarIt->currentLidar.footX, lidarIt->currentLidar.footYを結ぶベクトルを計算
    for (const auto &w : floorWindows)
    {
        if (w.lidarID == -1)
        {
            continue;
        }
        if (w.lidarID == window.lidarID)
        {
            continue;
        }
        auto otherLidarIt = std::find_if(lidarTrackers.begin(), lidarTrackers.end(), [&w](const tkr::LidarTracker &tracker)
                                         { return tracker.id == w.lidarID; });
        if (otherLidarIt == lidarTrackers.end() || lidarIt == lidarTrackers.end())
        {
            continue;
        }

        float dx = otherLidarIt->currentLidar.footX - lidarIt->currentLidar.footX; // 出口座標とLiDARの足位置を結ぶベクトルのx成分
        float dy = otherLidarIt->currentLidar.footY - lidarIt->currentLidar.footY; // 出口座標とLiDARの足位置を結ぶベクトルのy成分
        // ベクトル(dx, dy)を正規化する
        float length = std::sqrt(dx * dx + dy * dy);
        if (length > 300.0f)
        {
            continue;
        }
        if (length > 0.0f)
        {
            dx /= length;
            dy /= length;
        }
        // lidarIt->currentLidar.footX, Y からdx, dyの方向に長さ20の座標を計算
        float arrowX = window.centerX + dx * 60.0f;
        float arrowY = window.centerY + dy * 60.0f;
        glPushMatrix();
        glTranslated(arrowX, arrowY, 1.0);
        glScaled(5.0, 5.0, 0.0);
        if (length < 150.0f)
        {
            glColor3f(1.0f, 0.0f, 0.0f);
        }
        else if (length < 300.0f)
        {
            glColor3f(1.0f, 1.0f, 0.0f);
        }
        glutSolidSphere(1.0, 36, 18);
        glPopMatrix();
    }

    if (isEmergency)
    {
        glPushMatrix();
        glTranslated(window.centerX, window.centerY, 0.9);
        glRotatef(window.angle, 0, 0, 1);  // ① メイン画像と同じ回転を適用
        glTranslated(20.0, 10.0, 0.0);     // ② ローカル座標で右に移動（値は調整してください）
        glRotatef(-window.angle, 0, 0, 1); // ③ メイン画像の回転を打ち消す

        // ここからは元のコードと同じ（矢印の独自回転）
        float exitPosX = -546.0f;
        float exitPosY = 691.0f;
        auto lidarIt = std::find_if(lidarTrackers.begin(), lidarTrackers.end(), [&window](const tkr::LidarTracker &tracker)
                                    { return tracker.id == window.lidarID; });
        float angleDeg = 0.0f;
        if (lidarIt != lidarTrackers.end())
        {
            float dx = exitPosX - lidarIt->currentLidar.footX;
            float dy = exitPosY - lidarIt->currentLidar.footY;
            angleDeg = std::atan2(dy, dx) * 180.0f / std::numbers::pi_v<float> - 90.0f;
        }
        glRotatef(angleDeg, 0, 0, 1); // ④ 出口方向への回転

        glScaled(55.4 * 0.4, 78.3 * 0.4, 0.0);
        glColor3f(1.0f, 1.0f, 1.0f);
        glEnable(GL_TEXTURE_2D);
        glBindTexture(GL_TEXTURE_2D, 10001);
        glBegin(GL_QUADS);

        glTexCoord2f(1.0f, 0.0f);
        glVertex2f(-0.5f, 0.5f);

        glTexCoord2f(0.0f, 0.0f);
        glVertex2f(0.5f, 0.5f);

        glTexCoord2f(0.0f, 1.0f);
        glVertex2f(0.5f, -0.5f);

        glTexCoord2f(1.0f, 1.0f);
        glVertex2f(-0.5f, -0.5f);

        glEnd();
        glDisable(GL_TEXTURE_2D);
        glPopMatrix();
    }
}