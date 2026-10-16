//g++ -O3 -framework OpenGL -framework GLUT -DGL_SILENCE_DEPRECATION `pkg-config --cflags --libs opencv5` -std=c++20 -Wall -Wextra ./src/network/*.cpp ./src/calibration/*.cpp ./src/yolo/*.cpp ./src/lidar/*.cpp ./src/kalman/*.cpp ./src/core/*.cpp ./src/tracker/*.cpp ./src/render/*.cpp ./apps/FootStamp/main_FootStamp.cpp

// ----------------------------------------------------------------
/// @file main_FootStamp.cpp
/// @brief 手を挙げた人の足元に足跡エフェクトを出すプログラムのエントリーポイント
///
/// LiDAR と YOLO (前方・後方カメラ) の検出結果を UDP で受信して人を追跡し、
/// 人が手を挙げると、その人の足元 (LiDAR の位置) に足跡のエフェクトを出す。
/// 足跡は時間とともに大きくなりながら回転し、薄くなって消える。
/// 描画は GLUT (OpenGL) で行い、データの受信と追跡は約 33 ms ごとのタイマー処理 (timer()) で行う。
///
/// @par ビルド方法
/// このファイルは main() を持つ単独のプログラムである。
/// main() を持つ他のファイル (main_FloorMovie.cpp や Test_LidarData.cpp など) とは同時にコンパイルしないこと。
/// @code
/// g++ -O3 -framework OpenGL -framework GLUT -DGL_SILENCE_DEPRECATION `pkg-config --cflags --libs opencv5` -std=c++20 -Wall -Wextra ./src/network/*.cpp ./src/calibration/*.cpp ./src/yolo/*.cpp ./src/lidar/*.cpp ./src/kalman/*.cpp ./src/core/*.cpp ./src/tracker/*.cpp ./src/render/*.cpp ./apps/FootStamp/main_FootStamp.cpp
/// @endcode
///
/// @par 受信ポート (UDP)
/// | ポート | データ         |
/// |--------|----------------|
/// | 12345  | LiDAR          |
/// | 10001  | YOLO (前方)    |
/// | 10002  | YOLO (後方)    |
///
/// @par 手の挙げ方と足跡の種類
/// | 手の挙げ方 | テクスチャ ID | 画像        |
/// |------------|---------------|-------------|
/// | 左手のみ   | 0             | circle3.png |
/// | 右手のみ   | 1             | star.png    |
/// | 両手       | 2             | snow.png    |
/// | (使用なし) | 3             | sakura.png  |
///
/// @par キー操作
/// | キー        | 動作                 |
/// |-------------|----------------------|
/// | q / Q / ESC | プログラムを終了する |
/// | f / F       | フルスクリーンにする |
///
/// @par 実行時に必要なファイル (実行するディレクトリからの相対パス)
/// - ./Calibration/front_calibration600x600.txt, ./Calibration/back_calibration600x600.txt
/// - ./Movie/circle3.png, ./Movie/star.png, ./Movie/snow.png, ./Movie/sakura.png
///
/// @par 座標系
/// - 検出範囲 (LiDAR・YOLO のウィンドウ座標): X が -300 〜 300、Y が 0 〜 600
/// - 描画範囲 (OpenGL の正射影): X が 546 〜 -546 (左右反転)、Y が 0 〜 691
///
/// @note 緊急事態の処理 (キー e / E、床面ウィンドウ、動画) は、このプログラムではコメントアウトされている。
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

// 足跡インタラクション用 Yolo + LiDAR
#include "../../include/render/Footprints.hpp"

// 足跡インタラクション用 Yoloのみ
// #include "./Tracker/FloorWindow/Footprints_Yolo.hpp"

// 実験記録用
// #include "./Save/Save.hpp"

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

// std::vector<win::FloorWindow> floorWindows;

// 足跡インタラクション用 Yolo + LiDAR
/// 画面に表示している足跡エフェクトの一覧
std::vector<win::FootPrint> footPrints;
/// 足跡を出す人 (LiDAR トラッカーに対応) の一覧
std::vector<win::Human> humans;

// 足跡インタラクション用 Yoloのみ
// std::vector<win::FootPrintYolo> footPrints;
// std::vector<win::HumanYolo> humans;

/// 緊急事態かどうか (このプログラムでは、緊急事態の処理がコメントアウトされているため使われていない)
bool isEmergency = false;

// ALuint source0; // 音源0

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

    // テスト用の足跡を追加 （lidarの位置にスタンプされるかの確認）
    // footPrints.emplace_back(lidar::Lidar(0.0f, 300.0f), 0.0f, 0, std::tuple<double, double, double>{1.0, 0.0, 0.0});
    // footPrints.emplace_back(lidar::Lidar(20.0f, 300.0f), 0.0f, 0, std::tuple<double, double, double>{0.0, 0.0, 1.0});

    glutMainLoop();

    return 0;
}

// void initAL()
// {
//     ALuint buffer;
//     alGenBuffers(1, &buffer);

//     buffer = alutCreateBufferFromFile("../../Movie/test.wav");
//     alGenSources(1, &source0);
//     alSourcei(source0, AL_BUFFER, buffer);

//     alSourcei(source0, AL_LOOPING, AL_TRUE);
//     alSourcef(source0, AL_REFERENCE_DISTANCE, 10.0);
//     alSourcef(source0, AL_PITCH, 1.0);
//     alSourcef(source0, AL_GAIN, 1.0);

//     ALfloat orient[] = {0.0, 0.0, -1.0, 0.0, 1.0, 0.0};
//     alListenerfv(AL_ORIENTATION, orient);
//     alListener3f(AL_POSITION, 0.0, 0.0, 20.0);
// }

/**
 * @brief OpenGL と GLUT のウィンドウを初期化し、足跡のテクスチャを読み込む
 *
 * 処理の流れ:
 * -# ダブルバッファ・深度バッファ付きの 800×500 のウィンドウを作成する
 * -# 描画・リサイズ・キー入力・タイマーのコールバックを登録する (タイマーは 33 ms)
 * -# 背景色、深度テスト、アルファブレンド、アルファテストを設定する
 * -# 足跡の画像 4 枚を読み込み、透過付き (BGRA) のテクスチャとして転送する
 *    - テクスチャ ID 0: ./Movie/circle3.png
 *    - テクスチャ ID 1: ./Movie/star.png
 *    - テクスチャ ID 2: ./Movie/snow.png
 *    - テクスチャ ID 3: ./Movie/sakura.png
 *
 * @note 画像が読み込めない場合はエラーを出力するが、そのままテクスチャ転送に進む。
 * @note テクスチャ ID は、win::Human の printTextureID (0: 左手、1: 右手、2: 両手) と対応している。
 *       ID 3 (桜) は、現在は足跡の種類として使われていない。
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

    cv::Mat circleImage = cv::imread("./Movie/circle3.png", cv::IMREAD_UNCHANGED);
    if (circleImage.empty())
    {
        std::cerr << "円の画像を読み込めませんでした" << std::endl;
        std::cerr << "画像ファイルのパスは、実行ファイルからの相対パスで指定してください" << std::endl;
    }

    glBindTexture(GL_TEXTURE_2D, 0);
    glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
    glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, circleImage.cols, circleImage.rows, 0, GL_BGRA, GL_UNSIGNED_BYTE, circleImage.data);

    cv::Mat starImage = cv::imread("./Movie/star.png", cv::IMREAD_UNCHANGED);
    if (starImage.empty())
    {
        std::cerr << "星の画像を読み込めませんでした" << std::endl;
        std::cerr << "画像ファイルのパスは、実行ファイルからの相対パスで指定してください" << std::endl;
    }

    glBindTexture(GL_TEXTURE_2D, 1);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, starImage.cols, starImage.rows, 0, GL_BGRA, GL_UNSIGNED_BYTE, starImage.data);

    cv::Mat snowImage = cv::imread("./Movie/snow.png", cv::IMREAD_UNCHANGED);
    if (snowImage.empty())
    {
        std::cerr << "雪の画像を読み込めませんでした" << std::endl;
        std::cerr << "画像ファイルのパスは、実行ファイルからの相対パスで指定してください" << std::endl;
    }

    glBindTexture(GL_TEXTURE_2D, 2);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, snowImage.cols, snowImage.rows, 0, GL_BGRA, GL_UNSIGNED_BYTE, snowImage.data);

    cv::Mat sakuraImage = cv::imread("./Movie/sakura.png", cv::IMREAD_UNCHANGED);
    if (sakuraImage.empty())
    {
        std::cerr << "桜の画像を読み込めませんでした" << std::endl;
        std::cerr << "画像ファイルのパスは、実行ファイルからの相対パスで指定してください" << std::endl;
    }

    glBindTexture(GL_TEXTURE_2D, 3);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, sakuraImage.cols, sakuraImage.rows, 0, GL_BGRA, GL_UNSIGNED_BYTE, sakuraImage.data);
}

/**
 * @brief 描画コールバック: 画面を描画する
 *
 * 描画内容:
 * -# 検出範囲 (X: -300 〜 300、Y: 0 〜 600) の枠
 * -# 足跡 (footPrints) をすべて。足跡ごとに、テクスチャ・色と不透明度・位置・回転・大きさ (正方形) を
 *    反映して描画する。深度バッファへの書き込みは、足跡の描画中だけ無効にする
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

    // glBegin(GL_LINES);
    // glColor3f(0.3f, 0.3f, 0.3f);

    // glVertex3f(-300.0f, 100.0f, 0.0f);
    // glVertex3f(300.0f, 100.0f, 0.0f);

    // glVertex3f(-300.0f, 300.0f, 0.0f);
    // glVertex3f(300.0f, 300.0f, 0.0f);

    // glVertex3f(-300.0f, 500.0f, 0.0f);
    // glVertex3f(300.0f, 500.0f, 0.0f);
    // glEnd();

    // glPointSize(5.0f);
    // glBegin(GL_POINTS);
    // glColor3f(0.3f, 0.3f, 0.3f);

    // // glVertex3f(-200.0f, 100.0f, -1.0f);
    // glVertex3f(0.0f, 100.0f, -1.0f);
    // // glVertex3f(200.0f, 100.0f, -1.0f);

    // // glVertex3f(-200.0f, 300.0f, -1.0f);
    // glVertex3f(0.0f, 300.0f, -1.0f);
    // // glVertex3f(200.0f, 300.0f, -1.0f);

    // // glVertex3f(-200.0f, 500.0f, -1.0f);
    // glVertex3f(0.0f, 500.0f, -1.0f);
    // // glVertex3f(200.0f, 500.0f, -1.0f);

    // glEnd();

    // 足跡を描画
    for (const auto &footPrint : footPrints)
    {
        glDepthMask(GL_FALSE); // 深度バッファへの書き込みを無効化
        glEnable(GL_TEXTURE_2D);
        glBindTexture(GL_TEXTURE_2D, footPrint.printTextureID);
        // glBindTexture(GL_TEXTURE_2D, 1);
        auto [r, g, b] = footPrint.color;
        glColor4d(r, g, b, footPrint.alpha);
        // glColor4d(0.0, 1.0, 0.0, 0.5);

        glPushMatrix();
        glTranslated(footPrint.centerX, footPrint.centerY, 0.0f);
        glRotated(footPrint.angle, 0.0, 0.0, 1.0);
        glScaled(footPrint.size, footPrint.size, 1.0f); // 正方形

        glBegin(GL_QUADS); // 物体開始(四角形)
        glTexCoord2d(0.0, 0.0);
        glVertex3d(-0.5, 0.5, 0.0);
        glTexCoord2d(0.0, 1.0);
        glVertex3d(-0.5, -0.5, 0.0);
        glTexCoord2d(1.0, 1.0);
        glVertex3d(0.5, -0.5, 0.0);
        glTexCoord2d(1.0, 0.0);
        glVertex3d(0.5, 0.5, 0.0);
        glEnd(); // 物体終了

        glPopMatrix();
        glDisable(GL_TEXTURE_2D);
        glDepthMask(GL_TRUE); // 深度バッファへの書き込みを有効化
    }

    // for (const auto &lidar : currentLidars)
    // {
    //     drawRawData(lidar, 1.0f, 0.0f, 0.0f);
    // }

    // for (const auto &yoloTracker : yoloFrontTrackers)
    // {
    //     drawTracker(yoloTracker, 0.0f, 0.0f, 1.0f, false, false);
    // }

    /*
    for (const auto &yoloTracker : yoloBackTrackers)
    {
        // drawTracker(yoloTracker, 0.0f, 0.0f, 1.0f, false, false);
    }
    */

    // for (const auto &lidarTracker : lidarTrackers)
    // {
    //     drawTracker(lidarTracker, 1.0f, 1.0f, 1.0f, false, false);
    // }

    // drawDoubleMatchedTrackers(matchedLidar2FrontTrackers, matchedLidar2BackTrackers, true);

    // for (const auto &floorWindow : floorWindows)
    // {
    //     drawFloorWindow(floorWindow);
    // }

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
    // glOrtho(546.0, -546.0, 0.0, 691.0, -3.0, 3.0);
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
    // glOrtho(270.0, -270.0, 0.0, 310.0, -3.0, 3.0);
    glOrtho(546.0, -546.0, 0.0, 691.0, -3.0, 3.0);
    glMatrixMode(GL_MODELVIEW);
}

/**
 * @brief タイマーコールバック: データの受信・追跡・足跡の更新を行う (約 33 ms ごと)
 *
 * 処理の流れ:
 * -# 次のタイマーを登録し、再描画を要求する
 * -# LiDAR を受信できたら、検出範囲外の点を除いて検出結果を作り、LiDAR トラッカーを更新・整理する
 * -# 前方カメラの YOLO を受信できたら、キャリブレーションを通して検出結果を作り、
 *    足元の信頼度が負のものを除いて、前方の YOLO トラッカーを更新・整理する
 * -# 後方カメラの YOLO も同様に処理する
 * -# いずれかを受信できたら、LiDAR と前方・後方の YOLO のマッチングを更新する
 * -# 毎フレーム、Human の一覧を更新し (win::updateHumans())、手の挙げ方に応じて足跡を追加する
 *    (win::changeFootPrint())
 * -# 毎フレーム、すべての足跡を 1 フレーム分だけ進め (win::updateFootPrints())、
 *    消えた足跡を削除する (win::deleteFootPrints())
 *
 * 受信できなかった系統は、前回の状態のまま処理する。
 * 足跡は 1 フレームごとに不透明度が 0.015 ずつ減るため、約 100 フレーム (約 3.3 秒) で消える。
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

        // win::updateFloorWindows(lidarTrackers, floorWindows);
        // win::deleteFloorWindows(lidarTrackers, floorWindows);
    }

    // if (gotYoloFront || gotYoloBack)
    // {
    //     win::changeMovies(floorWindows, matchedLidar2FrontTrackers, matchedLidar2BackTrackers);
    // }

    // for (auto &window : floorWindows)
    // {
    //     window.movie.updateMovieFrame(isEmergency);
    // }

    // 緊急事態の時、yoloTrackersが空の時は、緊急事態を解除する
    // if (isEmergency)
    // {
    //     if (yoloFrontTrackers.empty() && yoloBackTrackers.empty())
    //     {
    //         isEmergency = false;
    //         std::cout << "全ての人が部屋から出ました" << std::endl;
    //         std::cout << "緊急事態解除！" << std::endl;
    //         alSourceStop(source0);
    //         for (auto &window : floorWindows)
    //         {
    //             window.movie.setEmergency(isEmergency);
    //         }
    //     }
    // }

    // 足跡インタラクションの更新 Yolo + LiDAR
    win::updateHumans(lidarTrackers, humans);
    win::changeFootPrint(humans, matchedLidar2FrontTrackers, matchedLidar2BackTrackers, footPrints);

    // 足跡インタラクションの更新 Yoloのみ
    // win::updateHumans(yoloFrontTrackers, humans);
    // win::changeFootPrint(humans, yoloFrontTrackers, footPrints);

    // for (auto &human : humans)
    // {
    //     // 位置が変わったときだけ内部で判定してスタンプ
    //     auto lidarIt = std::find_if(lidarTrackers.begin(), lidarTrackers.end(), [&human](const tkr::LidarTracker &tracker)
    //                                 { return tracker.id == human.lidarID; });
    //     if (lidarIt != lidarTrackers.end())
    //     {
    //         human.stamp(lidarIt->currentLidar, footPrints);
    //     }
    // }

    // 足跡インタラクションの更新
    win::updateFootPrints(footPrints);
    win::deleteFootPrints(footPrints);

    // for (const auto &yoloTracker : yoloFrontTrackers)
    // {
    //     // cv::Point2f predictedPoint;
    //     // yoloTracker.kalmanYolo.futurePredict(predictedPoint, 10);
    //     // save::saveFootPrints("500.txt", predictedPoint.x, predictedPoint.y);

    //     save::saveFootPrints("500.txt", yoloTracker.currentYolo.footX, yoloTracker.currentYolo.footY);
    // }
    // for (const auto &lidarTracker : lidarTrackers)
    // {
    //     save::saveFootPrints("500.txt", lidarTracker.currentLidar.footX, lidarTracker.currentLidar.footY);
    // }
}

/**
 * @brief キーボードコールバック: キー入力に応じて動作する
 *
 * | キー        | 動作                 |
 * |-------------|----------------------|
 * | ESC / q / Q | プログラムを終了する |
 * | f / F       | フルスクリーンにする |
 *
 * 緊急事態の切り替え (e / E) は、このプログラムではコメントアウトされている。
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
    // case 'e':
    // case 'E':
    //     if (isEmergency == false)
    //     {
    //         isEmergency = true;
    //         std::cout << "緊急事態発生！" << std::endl;
    //         alSourcePlay(source0);
    //     }
    //     else
    //     {
    //         isEmergency = false;
    //         std::cout << "緊急事態解除！" << std::endl;
    //         alSourceStop(source0);
    //     }

    //     for (auto &window : floorWindows)
    //     {
    //         window.movie.setEmergency(isEmergency);
    //     }
    //     break;
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