// g++ -O3 -framework OpenGL -framework GLUT -DGL_SILENCE_DEPRECATION `pkg-config --cflags --libs opencv5` -std=c++20 -Wall -Wextra ./src/network/send_udp.cpp ./apps/TestLiDARData/Test_LidarData.cpp

/// @file Test_LidarData.cpp
/// @brief LiDAR の検出データを疑似的に送信するテスト用プログラム
///
/// ウィンドウ上でマウスをドラッグして赤い点 (人の足元の位置) を動かし、
/// その位置を LiDAR のデータとして UDP で送信する。
/// 実際の LiDAR がなくても、受信側のプログラム (main_FloorMovie.cpp や main_FootStamp.cpp など) を
/// 動作確認できる。
///
/// @par ビルド方法
/// このファイルは main() を持つ単独のプログラムで、送信側の send_udp.cpp だけをリンクすればよい。
/// main() を持つ他のファイル (main_FloorMovie.cpp や main_FootStamp.cpp など) とは同時にコンパイルしないこと。
/// @code
/// g++ -O3 -framework OpenGL -framework GLUT -DGL_SILENCE_DEPRECATION `pkg-config --cflags --libs opencv5` -std=c++20 -Wall -Wextra ./src/network/send_udp.cpp ./apps/TestLiDARData/Test_LidarData.cpp
/// @endcode
///
/// @par 使い方
/// -# 受信側のプログラムを起動しておく
/// -# このプログラムを起動し、ウィンドウ内でマウスの左ボタンを押しながら動かす
/// -# 赤い点の位置が、約 33 ms ごとに LiDAR の検出点 (1 点) として送信される
///
/// @par 送信先
/// 127.0.0.1 (同じ PC) のポート 12345 (受信側の RecvLidar が待ち受けるポート)
///
/// @par キー操作
/// | キー        | 動作                 |
/// |-------------|----------------------|
/// | q / Q / ESC | プログラムを終了する |
///
/// @par 座標系
/// 受信側の LiDAR の検出範囲 (X が -300 〜 300、Y が 0 〜 600) と同じ座標系で送信する。
/// 描画の正射影は X が 300 〜 -300 (左右反転)、Y が 0 〜 600。
///
/// Copyright (c) 2026 Maito Yasui
/// Released under the GNU AGPL-3.0 License.
/// https://github.com/Nibosi0501/LiViMotion/blob/main/LICENSE
// ----------------------------------------------------------------

#include <iostream>

#include <opencv2/opencv.hpp>
#include <GLUT/glut.h>

#include "../../include/network/SendUdp.hpp"

void initGL();
void display();
void reshape(int w, int h);
void timer(int value);
void keyboard(unsigned char key, int x, int y);
void mouse(int button, int state, int x, int y);
void motion(int x, int y);

/// GLUT のウィンドウ ID の配列 (ウィンドウは 1 つだけ)
int winID[1];
/// マウスの左ボタンを押している間 (ドラッグ中) は true
bool isDragging = false;

/// 現在の人の位置 (LiDAR 座標系)。初期値 (0, -1) は検出範囲 (Y: 0 〜 600) の外なので、ドラッグするまで受信側では無視される
cv::Point2f currentPos(0, -1); // 現在の人の位置

/// LiDAR データの送信 (送信先 127.0.0.1、ポート 12345)
snd::SendLidar sendLidar(12345, "127.0.0.1");

/**
 * @brief プログラムのエントリーポイント
 *
 * GLUT を初期化し、ウィンドウとコールバックを設定 (initGL()) して、イベントループに入る。
 * glutMainLoop() は戻らないため、通常は return 0 に到達しない (終了は keyboard() の exit(0))。
 *
 * @param[in] argc コマンドライン引数の数 (GLUT に渡される)
 * @param[in] argv コマンドライン引数 (GLUT に渡される)
 * @return 0
 *
 * @note 送信ソケットは、main() の前にグローバル変数 sendLidar として構築される。
 */
int main(int argc, char *argv[])
{
    glutInit(&argc, argv);
    initGL();

    glutMainLoop();

    return 0;
}

/**
 * @brief OpenGL と GLUT のウィンドウを初期化する
 *
 * 処理の流れ:
 * -# ダブルバッファ・深度バッファ付きの 600×600 のウィンドウを作成する
 * -# 描画・リサイズ・キー入力・マウスのボタン・マウスのドラッグ・タイマーのコールバックを登録する
 *    (タイマーは 33 ms)
 * -# 背景色、深度テスト、アルファブレンド、アルファテストを設定する
 */
void initGL()
{
    glutInitDisplayMode(GLUT_RGBA | GLUT_DEPTH | GLUT_DOUBLE);
    glutInitWindowSize(600, 600);
    winID[0] = glutCreateWindow("Lagrange");

    glutDisplayFunc(display);
    glutReshapeFunc(reshape);
    glutKeyboardFunc(keyboard);
    glutMouseFunc(mouse);
    glutMotionFunc(motion);
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

/**
 * @brief 描画コールバック: 現在の人の位置を赤い球で描画する
 */
void display()
{
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glLoadIdentity();

    // currentPos
    glPushMatrix();
    glColor3f(1.0f, 0.0f, 0.0f);
    glTranslated(currentPos.x, currentPos.y, 0.0);
    glScaled(10.0, 10.0, 0.0);
    glutSolidSphere(1.0, 36, 18);
    glPopMatrix();

    glutSwapBuffers();
}

/**
 * @brief リサイズコールバック: ビューポートと投影を設定する
 *
 * 正射影の範囲を X: 300 〜 -300 (左右反転)、Y: 0 〜 600 にする。
 *
 * @param[in] w ウィンドウの幅 [pixel]
 * @param[in] h ウィンドウの高さ [pixel]
 */
void reshape(int w, int h)
{
    glViewport(0, 0, w, h);
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    glOrtho(300.0, -300.0, 0.0, 600.0, -3.0, 3.0);
    glMatrixMode(GL_MODELVIEW);
}

/**
 * @brief タイマーコールバック: 現在位置を LiDAR のデータとして送信する (約 33 ms ごと)
 *
 * 次のタイマーを登録して再描画を要求し、currentPos を 1 点だけ含むデータを UDP で送信する。
 * ドラッグしていない間も、最後の位置を送り続ける。
 *
 * @param[in] value 未使用 (glutTimerFunc() に渡した値)
 */
void timer([[maybe_unused]] int value)
{
    glutSetWindow(winID[0]);
    glutTimerFunc(33, timer, 0);
    glutPostRedisplay();

    // currentPosを送信
    std::vector<std::pair<float, float>> points = {{currentPos.x, currentPos.y}};
    sendLidar.send(points);
}

/**
 * @brief キーボードコールバック: キー入力に応じて動作する
 *
 * ESC / q / Q でプログラムを終了する。それ以外のキーは何もしない。
 *
 * @param[in] key 押されたキー
 * @param[in] x   キーを押したときのマウスの X 座標 (未使用)
 * @param[in] y   キーを押したときのマウスの Y 座標 (未使用)
 */
void keyboard(unsigned char key, [[maybe_unused]] int x, [[maybe_unused]] int y)
{
    switch (key)
    {
    case 27:
    case 'q':
    case 'Q':
        exit(0);
    default:
        break;
    }
}

/**
 * @brief マウスのボタンコールバック: 左ボタンの押下・解放でドラッグ状態を切り替える
 *
 * 左ボタンを押したら isDragging を true に、離したら false にする。それ以外のボタンは無視する。
 *
 * @param[in] button 操作されたボタン (GLUT_LEFT_BUTTON など)
 * @param[in] state  ボタンの状態 (GLUT_DOWN または GLUT_UP)
 * @param[in] x      ボタンを操作したときのマウスの X 座標 (未使用)
 * @param[in] y      ボタンを操作したときのマウスの Y 座標 (未使用)
 */
void mouse(int button, int state, [[maybe_unused]] int x, [[maybe_unused]] int y)
{
    if (button == GLUT_LEFT_BUTTON && state == GLUT_DOWN)
    {
        isDragging = true;
    }
    else if (button == GLUT_LEFT_BUTTON && state == GLUT_UP)
    {
        isDragging = false;
    }
}

/**
 * @brief マウスのドラッグコールバック: ドラッグ中、現在位置をマウスの位置に更新する
 *
 * ウィンドウ座標 (原点が左上) を LiDAR 座標系に変換する。
 * - X: 300 - x (左右反転)
 * - Y: 600 - y (上下反転)
 *
 * @param[in] x マウスの X 座標 [pixel]
 * @param[in] y マウスの Y 座標 [pixel]
 *
 * @note この変換は、ウィンドウの大きさが 600×600 のときだけ正しい。
 *       ウィンドウをリサイズすると、点の位置とマウスの位置がずれる。
 */
void motion(int x, int y)
{
    if (isDragging)
    {
        currentPos = cv::Point2f(300 - x, 600 - y);
    }
}