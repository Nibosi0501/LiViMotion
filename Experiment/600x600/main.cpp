// ----------------------------------------------------------------
// main.cpp
//
// LiViMotion - 個人ごとの身体動作に基づいた床面映像とのインタラクションシステム
//
// Copyright (c) 2026 Maito Yasui
// Released under the GNU GPL-3.0 License.
// https://github.com/Nibosi0501/LiViMotion/blob/main/LICENSE
// ----------------------------------------------------------------

/* コンパイルコマンド例:
g++ -O3 -framework OpenGL -framework GLUT -lalut -framework OpenAL -Wno-deprecated `pkg-config --cflags --libs opencv4` -mmacosx-version-min=13.0 -std=c++20 ../../Udp/receive_udp.cpp ../../Calibration/calibration.cpp ./Yolo/yolo.cpp ./Lidar/lidar.cpp ./Kalman/kalman.cpp ./Tracker/tracker.cpp ./Tracker/Hungarian/hungarian.cpp ./Tracker/Matching/matching.cpp ./Tracker/FloorWindow/floorWindow.cpp ./Tracker/FloorWindow/movieController.cpp main.cpp
*/

#include <iostream>
#include <numbers>
#include <chrono>

#include <opencv2/opencv.hpp>
#include <GLUT/glut.h>
#include <AL/alut.h> //OpenAL

#include <cstdlib>

#include "../../Udp/ReceiveUdp.hpp"
#include "../../Calibration/Calibration.hpp"

#include "./Yolo/Yolo.hpp"
#include "./Lidar/Lidar.hpp"
#include "./Kalman/Kalman.hpp"
#include "./Tracker/Tracker.hpp"
#include "./Tracker/Matching/Matching.hpp"

// 足跡インタラクション用 Yolo + LiDAR
#include "./Tracker/FloorWindow/Footprints.hpp"

// 足跡インタラクション用 Yoloのみ
// #include "./Tracker/FloorWindow/Footprints_Yolo.hpp"

// GLUT
void initGL();
void display0();
void reshape0(int w, int h);
void timer(int value);
void keyboard0(unsigned char key, int x, int y);

// OpenAL
void initAL();

template <typename T>
void drawRawData(const T &data, const float r, const float g, const float b);

template <typename T>
void drawTracker(const T &tracker, const float r, const float g, const float b, const bool isID, const bool isHistory);

void drawTrackerID(const int id);

template <typename T>
void drawHistorys(const std::deque<T> &historys, const float r, const float g, const float b);

void drawMatchedTracker(const mtc::MatchedTracker &tracker, const bool isYolo, const bool isID);
void drawDoubleMatchedTrackers(const std::vector<mtc::MatchedTracker> &matchedTrackers1, const std::vector<mtc::MatchedTracker> &matchedTrackers2, const bool isYolo);

int winID[1];

clb::Calibration calibFront("../../Calibration/front_calibration600x600.txt", 7, 7, 600, 600, "front");
clb::Calibration calibBack("../../Calibration/back_calibration600x600.txt", 7, 7, 600, 600, "back");

rcv::RecvLidar recvLidar(12345);
std::vector<lidar::Lidar> currentLidars;

rcv::RecvYolo recvYoloFront(10001);
std::vector<yolo::Yolo> currentYoloFronts;

rcv::RecvYolo recvYoloBack(10002);
std::vector<yolo::Yolo> currentYoloBacks;

tkr::IDGenerator lidarIDGen;
tkr::IDGenerator frontIDGen;
tkr::IDGenerator backIDGen;

std::vector<tkr::LidarTracker> lidarTrackers;
std::vector<tkr::YoloTracker> yoloFrontTrackers;
std::vector<tkr::YoloTracker> yoloBackTrackers;

std::vector<mtc::MatchedTracker> matchedLidar2FrontTrackers;
std::vector<mtc::MatchedTracker> matchedLidar2BackTrackers;

// 足跡インタラクション用 Yolo + LiDAR
std::vector<win::FootPrint> footPrints;
std::vector<win::Human> humans;

// 足跡インタラクション用 Yoloのみ
// std::vector<win::FootPrintYolo> footPrints;
// std::vector<win::HumanYolo> humans;

bool isEmergency = false;

ALuint source0; // 音源0

int main(int argc, char *argv[])
{
    alutInit(&argc, argv);
    initAL();

    glutInit(&argc, argv);
    initGL();

    // テスト用の足跡を追加 （lidarの位置にスタンプされるかの確認）
    // footPrints.emplace_back(lidar::Lidar(0.0f, 300.0f), 0.0f, 0, std::tuple<double, double, double>{1.0, 0.0, 0.0});
    // footPrints.emplace_back(lidar::Lidar(20.0f, 300.0f), 0.0f, 0, std::tuple<double, double, double>{0.0, 0.0, 1.0});

    glutMainLoop();

    return 0;
}

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

    cv::Mat circleImage = cv::imread("../../Movie/circle3.png", cv::IMREAD_UNCHANGED);
    if (circleImage.empty())
    {
        std::cerr << "円の画像を読み込めませんでした" << std::endl;
        std::cerr << "画像ファイルのパスは、main()関数のあるcppファイルからの相対パスで指定してください" << std::endl;
    }

    glBindTexture(GL_TEXTURE_2D, 0);
    glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
    glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, circleImage.cols, circleImage.rows, 0, GL_BGRA, GL_UNSIGNED_BYTE, circleImage.data);

    cv::Mat starImage = cv::imread("../../Movie/star.png", cv::IMREAD_UNCHANGED);
    if (starImage.empty())
    {
        std::cerr << "星の画像を読み込めませんでした" << std::endl;
        std::cerr << "画像ファイルのパスは、main()関数のあるcppファイルからの相対パスで指定してください" << std::endl;
    }

    glBindTexture(GL_TEXTURE_2D, 1);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, starImage.cols, starImage.rows, 0, GL_BGRA, GL_UNSIGNED_BYTE, starImage.data);

    cv::Mat snowImage = cv::imread("../../Movie/snow.png", cv::IMREAD_UNCHANGED);
    if (snowImage.empty())
    {
        std::cerr << "雪の画像を読み込めませんでした" << std::endl;
        std::cerr << "画像ファイルのパスは、main()関数のあるcppファイルからの相対パスで指定してください" << std::endl;
    }

    glBindTexture(GL_TEXTURE_2D, 2);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, snowImage.cols, snowImage.rows, 0, GL_BGRA, GL_UNSIGNED_BYTE, snowImage.data);
}

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

    // 足跡を描画
    for (const auto &footPrint : footPrints)
    {
        glDepthMask(GL_FALSE); // 深度バッファへの書き込みを無効化
        glEnable(GL_TEXTURE_2D);
        glBindTexture(GL_TEXTURE_2D, footPrint.printTextureID);
        auto [r, g, b] = footPrint.color;
        glColor4d(r, g, b, footPrint.alpha);

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

    glutSwapBuffers();
}

void reshape0(int w, int h)
{
    glViewport(0, 0, w, h);
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    glOrtho(546.0, -546.0, 0.0, 691.0, -3.0, 3.0);
    glMatrixMode(GL_MODELVIEW);
}

void timer(int value)
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
    }

    // 足跡インタラクションの更新 Yolo + LiDAR
    win::updateHumans(lidarTrackers, humans);
    win::changeFootPrint(humans, matchedLidar2FrontTrackers, matchedLidar2BackTrackers, footPrints);

    // 足跡インタラクションの更新 Yoloのみ
    // win::updateHumans(yoloFrontTrackers, humans);
    // win::changeFootPrint(humans, yoloFrontTrackers, footPrints);

    // 足跡インタラクションの更新
    win::updateFootPrints(footPrints);
    win::deleteFootPrints(footPrints);
}

void keyboard0(unsigned char key, int x, int y)
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
    default:
        break;
    }
}

// 床面映像を描画する関数群
void drawMatchedTracker(const mtc::MatchedTracker &tracker, const bool isYolo, const bool isID)
{
    drawTracker(tracker.lidarTracker, 1.0f, 0.0f, 0.0f, isID, false);
    if (isYolo)
    {
        drawTracker(tracker.yoloTracker, 0.0f, 0.0f, 1.0f, false, false);
    }
}

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