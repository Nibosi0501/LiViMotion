// ----------------------------------------------------------------
// create_calibration_600x600.cpp
//
// カメラキャリブレーション用のポイントを生成するプログラム.
// このプログラムは、カメラからの映像を表示し、ユーザーがグリッド点をクリックしてキャリブレーションポイントを指定できるようにします.
// 指定されたポイントは、"front_calibration600x600.txt" または "back_calibration600x600.txt" というテキストファイルに保存されます。
// なお、本プログラムは、6m x 6m の床面映像を想定. それ以外のサイズの映像を使用する場合は、各種パラメータを適宜調整してください.
//
// Copyright (c) 2026 Maito Yasui
// Released under the GNU GPL-3.0 License.
// https://github.com/Nibosi0501/LiViMotion/blob/main/LICENSE
// ----------------------------------------------------------------

/* コンパイルコマンド例 (macOS):
g++ -O3 -framework OpenGL -framework GLUT -Wno-deprecated `pkg-config --cflags --libs opencv4` -mmacosx-version-min=13.0 -std=c++17 create_calibration2.cpp
*/

#include <opencv2/opencv.hpp>
#include <GLUT/glut.h>

#include <iostream>
#include <fstream>

constexpr int CAMERA_DEVICE = 0;
std::string FILE_PREFIX = "front_";
// std::string FILE_PREFIX = "back_";

void initGL();
void display();
void timer(int value);
void reshape(int w, int h);
void keyboard(unsigned char key, int x, int y);

int winID[1];

cv::VideoCapture capture;
cv::Mat calibrationImage;
std::vector<cv::Point2d> corners;
std::vector<cv::Point2d> calibrationCorners;

// カメラの画角調節中かどうかのフラグ
bool adjustingFOV = true;

void calibrateCamera();
void onMouse(int event, int x, int y, int flags, void *userdata);

bool saveCalibrationPoint(const std::string &filename);

int main(int argc, char *argv[])
{
    glutInit(&argc, argv);

    // 初期設定
    initGL();

    capture.open(CAMERA_DEVICE);
    if (!capture.isOpened())
    {
        std::cerr << "カメラが開けません。" << std::endl;
        return -1;
    }

    glutMainLoop();

    return 0;
}

void initGL()
{
    glutInitDisplayMode(GLUT_RGBA | GLUT_DEPTH | GLUT_DOUBLE);
    glutInitWindowSize(800, 500);
    winID[0] = glutCreateWindow("CG");

    glutDisplayFunc(display);
    glutReshapeFunc(reshape);
    glutKeyboardFunc(keyboard);
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
}

void display()
{
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glLoadIdentity();

    for (int row = 0; row < 7; row++)
    {
        for (int col = 0; col < 7; col++)
        {
            glPushMatrix();
            glColor3f(1.0f, 1.0f, 0.0f);
            glTranslated(-300.0 + 100.0 * row, 100.0 * col, 2.0);
            glScaled(10.0, 10.0, 0.0);
            glutSolidSphere(1.0, 36, 18);
            glPopMatrix();
        }
    }

    glPushMatrix();
    glColor3f(1.0f, 0.0f, 0.0f);
    glTranslated(0.0, 300.0, 2.0);
    glScaled(10.0, 10.0, 0.0);
    glutSolidSphere(1.0, 36, 18);
    glPopMatrix();

    glutSwapBuffers();
}

void reshape(int w, int h)
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

    if (adjustingFOV)
    {
        capture >> calibrationImage;
        if (calibrationImage.empty())
        {
            std::cerr << "カメラから画像が取得できません。" << std::endl;
            std::abort();
        }
    }
    else
    {
        calibrateCamera();
    }

    cv::imshow("Input Image", calibrationImage);
}

void calibrateCamera()
{
    cv::imwrite("calibration_600x600.png", calibrationImage);

    cv::Mat grayImage;
    cv::cvtColor(calibrationImage, grayImage, cv::COLOR_BGR2GRAY);
    cv::Mat binImage;
    cv::threshold(grayImage, binImage, 128, 255, cv::THRESH_BINARY | cv::THRESH_OTSU);

    std::vector<std::vector<cv::Point>> counters;
    cv::findContours(binImage, counters, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    corners.clear();
    for (int i = 0; i < counters.size(); i++)
    {
        double area = cv::contourArea(counters.at(i));
        if (area < 0)
        {
            continue;
        }

        cv::Moments mu = cv::moments(counters.at(i));
        if (mu.m00 == 0)
        {
            continue;
        }
        corners.emplace_back(mu.m10 / mu.m00, mu.m01 / mu.m00);
        cv::circle(calibrationImage, corners.at(corners.size() - 1), 5, cv::Scalar(0, 255, 0), -1);
    }

    cv::imshow("Input Image", calibrationImage);
    cv::setMouseCallback("Input Image", onMouse, nullptr);

    std::cout << "画像上をクリックしてポイントを指定してください. 指定後に任意のキーを押して下さい." << std::endl;
    cv::waitKey(0);

    if (!saveCalibrationPoint(FILE_PREFIX + "calibration600x600.txt"))
    {
        std::cerr << "キャリブレーションポイントの保存に失敗しました。" << std::endl;
        std::abort();
    }

    std::cout << "キャリブレーション完了。" << std::endl;
    std::exit(0);
}

bool saveCalibrationPoint(const std::string &filename)
{
    std::ofstream file(filename);
    if (!file.is_open())
    {
        std::cerr << "ファイルを開けません: " << filename << std::endl;
        return false;
    }

    for (const auto &point : calibrationCorners)
    {
        file << point.x << " " << point.y << "\n";
    }
    file.close();
    std::cout << "ポイントを保存しました: " << filename << std::endl;

    if (calibrationCorners.size() != 49)
    {
        std::cerr << "警告: キャリブレーションポイントが49個ではありません。保存されたポイント数: " << calibrationCorners.size() << std::endl;
    }

    return true;
}

void onMouse(int event, int x, int y, int flags, void *userdata)
{
    if (event == cv::EVENT_LBUTTONDOWN || event == cv::EVENT_RBUTTONDOWN)
    {
        if (calibrationCorners.size() >= 49)
        {
            std::cout << "これ以上点を選択できません。" << std::endl;
            return;
        }
    }

    cv::Point2d addPoint(-1.0, -1.0);
    if (event == cv::EVENT_LBUTTONDOWN)
    {
        double minDist = std::numeric_limits<double>::max();
        int closestIndex = -1;
        for (int i = 0; i < corners.size(); i++)
        {
            double dist = cv::norm(corners.at(i) - cv::Point2d(x, y));
            if (dist < minDist)
            {
                minDist = dist;
                closestIndex = i;
            }
        }
        if (closestIndex == -1)
        {
            std::cerr << "点が見つかりませんでした。" << std::endl;
            return;
        }
        addPoint = corners.at(closestIndex);
    }
    else if (event == cv::EVENT_RBUTTONDOWN)
    {
        addPoint = cv::Point2d(x, y);
    }

    if (addPoint == cv::Point2d(-1.0, -1.0))
    {
        return;
    }

    bool alreadySelected = false;
    for (const auto &pt : calibrationCorners)
    {
        if (pt == addPoint)
        {
            alreadySelected = true;
            break;
        }
    }
    if (alreadySelected)
    {
        std::cout << "この点は既に選択されています。" << std::endl;
        return;
    }

    calibrationCorners.emplace_back(addPoint);
    std::cout << "キャリブレーション用コーナー追加: " << addPoint << std::endl;

    // 追加したコーナーを表示
    if (event == cv::EVENT_LBUTTONDOWN)
    {
        cv::circle(calibrationImage, addPoint, 10, cv::Scalar(0, 0, 255), 2);
    }
    else if (event == cv::EVENT_RBUTTONDOWN)
    {
        cv::circle(calibrationImage, addPoint, 10, cv::Scalar(255, 0, 0), 2);
    }

    cv::Mat annotationImage = calibrationImage.clone();
    cv::putText(annotationImage, "Corners: " + std::to_string(calibrationCorners.size()) + "/" + std::to_string(49),
                cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 1.0,
                cv::Scalar(255, 255, 255), 2);

    cv::imshow("Input Image", annotationImage);
}

void keyboard(unsigned char key, int x, int y)
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
    case 'c':
    case 'C':
        imwrite("calibration_600x600.png", calibrationImage);
        break;

    case ' ':
        adjustingFOV = false;
        break;

    default:
        break;
    }
}