// ----------------------------------------------------------------
/// @file create_calibration.cpp
/// @brief カメラキャリブレーション用の格子点 (7×7 = 49 点) を作成するプログラム
///
/// カメラからの映像を表示し、ユーザーが格子点をクリックしてキャリブレーションポイントを指定する。
/// 指定されたポイントは、"front_calibration600x600.txt" または "back_calibration600x600.txt" という
/// テキストファイルに保存される。
/// 本プログラムは、6 m × 6 m の床面映像 (600 × 600 の座標) を想定している。
/// それ以外の大きさの映像を使用する場合は、各種パラメータ (格子の間隔・個数など) を適宜調整すること。
///
/// @par ビルド方法
/// このファイルは main() を持つ単独のプログラムで、他のソースファイルは必要ない。
/// main() を持つ他のファイルとは同時にコンパイルしないこと。
/// @code
/// g++ -O3 -framework OpenGL -framework GLUT -DGL_SILENCE_DEPRECATION `pkg-config --cflags --libs opencv5` -std=c++20 ./apps/CreateCalibration/create_calibration.cpp
/// @endcode
///
/// @par 使い方
/// -# カメラの映像が、OpenCV のウィンドウ "Input Image" に表示される。
///    GLUT のウィンドウには、7×7 の黄色い格子点と、中心の赤い点が描画される
///    (床に投影してカメラで撮影する想定)
/// -# 格子点が映像にすべて映るように、カメラの画角を調整する
/// -# GLUT のウィンドウでスペースキーを押し、画角の調整を終えてキャリブレーションに進む
/// -# 映像から格子点の候補が検出され、緑の点で表示される。"Input Image" のウィンドウで、格子点を順にクリックする
///    - 左クリック: クリックした位置に最も近い格子点の候補を選ぶ (赤い円が付く)
///    - 右クリック: クリックした位置をそのまま選ぶ (青い円が付く)。検出できなかった点を補うときに使う
/// -# 49 点を選んだら、任意のキーを押す。ファイルに保存してプログラムを終了する
///
/// @note 点を選ぶ順番が、そのまま保存される順番になる。clb::Calibration は、ファイルを行優先
///       (1 行目の点を順に、次の行へ) で読み込むので、その並びに合うようにクリックすること。
///
/// @par 保存されるファイル
/// | ファイル                           | 内容                                                       |
/// |------------------------------------|------------------------------------------------------------|
/// | (FILE_PREFIX)calibration600x600.txt | 選んだ点の座標。1 行に 1 点、「x y」の形式 (空白区切り)    |
/// | calibration_600x600.png            | キャリブレーションに使った映像 (保存のたびに上書き)        |
///
/// FILE_PREFIX が "front_" なら front_calibration600x600.txt、"back_" なら back_calibration600x600.txt になる。
///
/// @par キー操作 (GLUT のウィンドウ)
/// | キー        | 動作                                                         |
/// |-------------|--------------------------------------------------------------|
/// | スペース    | 画角の調整を終えて、キャリブレーションに進む                 |
/// | c / C       | 現在の映像を calibration_600x600.png に保存する              |
/// | f / F       | フルスクリーンにする                                         |
/// | q / Q / ESC | プログラムを終了する                                         |
///
/// @par 座標系 (GLUT のウィンドウに描画する格子点)
/// - 格子点: X が -300 〜 300、Y が 0 〜 600 の範囲に、100 間隔で 7×7 個
/// - 描画範囲 (OpenGL の正射影): X が 546 〜 -546 (左右反転)、Y が 0 〜 691
///
/// Copyright (c) 2026 Maito Yasui
/// Released under the GNU AGPL-3.0 License.
/// https://github.com/Nibosi0501/LiViMotion/blob/main/LICENSE
// ----------------------------------------------------------------

/* コンパイルコマンド例 (macOS):
g++ -O3 -framework OpenGL -framework GLUT -Wno-deprecated `pkg-config --cflags --libs opencv4` -mmacosx-version-min=13.0 -std=c++17 create_calibration2.cpp
*/

#include <opencv2/opencv.hpp>
#include <opencv2/geometry.hpp> // OpenCV5.0のバグに対応するため

#include <GLUT/glut.h>

#include <iostream>
#include <fstream>

/// 使用するカメラのデバイス番号
constexpr int CAMERA_DEVICE = 0;
/// 保存するファイル名の先頭に付ける文字列 ("front_" または "back_")。前方・後方カメラのどちらかに合わせて切り替える
std::string FILE_PREFIX = "front_";
// std::string FILE_PREFIX = "back_";

void initGL();
void display();
void timer(int value);
void reshape(int w, int h);
void keyboard(unsigned char key, int x, int y);

/// GLUT のウィンドウ ID の配列 (ウィンドウは 1 つだけ)
int winID[1];

/// カメラからの映像の取得
cv::VideoCapture capture;
/// キャリブレーションに使う映像 (選んだ点の印も、この画像に描き込まれる)
cv::Mat calibrationImage;
/// 映像から検出した、格子点の候補の座標 (輪郭の重心)
std::vector<cv::Point2d> corners;
/// ユーザーが選んだ、キャリブレーション用の点の座標 (選んだ順)
std::vector<cv::Point2d> calibrationCorners;

/// カメラの画角調整中かどうかのフラグ。true の間は映像を更新し続け、スペースキーで false になる
// カメラの画角調節中かどうかのフラグ
bool adjustingFOV = true;

void calibrateCamera();
void onMouse(int event, int x, int y, int flags, void *userdata);

bool saveCalibrationPoint(const std::string &filename);

/**
 * @brief プログラムのエントリーポイント
 *
 * GLUT を初期化してウィンドウを作り (initGL())、カメラを開いて、イベントループに入る。
 * カメラが開けない場合は、エラーを出力して -1 で終了する。
 * glutMainLoop() は戻らないため、通常は return 0 に到達しない
 * (終了は calibrateCamera() または keyboard() の終了処理)。
 *
 * @param[in] argc コマンドライン引数の数 (GLUT に渡される)
 * @param[in] argv コマンドライン引数 (GLUT に渡される)
 * @retval 0  通常は到達しない
 * @retval -1 カメラを開けなかった
 */
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

/**
 * @brief OpenGL と GLUT のウィンドウを初期化する
 *
 * ダブルバッファ・深度バッファ付きの 800×500 のウィンドウ ("CG") を作成し、
 * 描画・リサイズ・キー入力・タイマー (33 ms) のコールバックを登録する。
 * あわせて、背景色、深度テスト、アルファブレンド、アルファテストを設定する。
 */
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

/**
 * @brief 描画コールバック: 格子点を描画する
 *
 * 次の点を描画する。
 * - 黄色の球: X が -300 〜 300、Y が 0 〜 600 の範囲に、100 間隔で 7×7 個 (計 49 個)
 * - 赤い球: (0, 300)。格子の中心
 */
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

/**
 * @brief リサイズコールバック: ビューポートと投影を設定する
 *
 * 正射影の範囲を X: 546 〜 -546 (左右反転)、Y: 0 〜 691 にする。
 *
 * @param[in] w ウィンドウの幅 [pixel]
 * @param[in] h ウィンドウの高さ [pixel]
 */
void reshape(int w, int h)
{
    glViewport(0, 0, w, h);
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    glOrtho(546.0, -546.0, 0.0, 691.0, -3.0, 3.0);
    glMatrixMode(GL_MODELVIEW);
}

/**
 * @brief タイマーコールバック: 映像の取得と、キャリブレーションの開始を行う (約 33 ms ごと)
 *
 * 次のタイマーを登録して再描画を要求した後、状態に応じて次のように動作する。
 * - 画角の調整中 (adjustingFOV が true): カメラから映像を 1 フレーム取得する。取得できなければ異常終了する
 * - 調整が終わった後: calibrateCamera() を呼ぶ (この関数は、点の選択と保存のあとにプログラムを終了する)
 *
 * 最後に、映像を "Input Image" のウィンドウに表示する。
 *
 * @param[in] value 未使用 (glutTimerFunc() に渡した値)
 */
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

/**
 * @brief 映像から格子点の候補を検出し、ユーザーが点を選んだ後、ファイルに保存して終了する
 *
 * 処理の流れ:
 * -# 現在の映像を calibration_600x600.png に保存する
 * -# グレースケールにして、大津の方法で二値化する
 * -# 外側の輪郭を検出し、輪郭ごとの重心を格子点の候補 (corners) とする。候補は緑の点で映像に描く
 * -# "Input Image" のウィンドウにマウスのコールバック (onMouse()) を登録する
 * -# 何かキーが押されるまで待つ (cv::waitKey(0))。この間に、ユーザーが格子点をクリックして選ぶ
 * -# 選んだ点を保存する (saveCalibrationPoint())。失敗したら異常終了する
 * -# 完了のメッセージを表示して、プログラムを終了する
 *
 * @note この関数は戻らない (std::exit(0) または std::abort() で終了する)。
 * @note 輪郭の面積の判定は area < 0 だが、面積は 0 以上なので、この条件は成り立たない。
 *       そのため、小さなノイズの輪郭も候補に含まれる。
 */
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

/**
 * @brief 選んだ点の座標を、テキストファイルに保存する
 *
 * calibrationCorners の点を、1 行に 1 点、「x y」の形式 (空白区切り) で書き込む。
 * 点の数が 49 個でない場合は、警告を出力するが、保存はそのまま行う。
 *
 * @param[in] filename 保存するファイルのパス (実行するディレクトリからの相対パス)
 * @retval true  保存できた (点の数が 49 個でない場合も含む)
 * @retval false ファイルを開けなかった
 */
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

/**
 * @brief マウスのコールバック: クリックされた点を、キャリブレーション用の点に追加する
 *
 * - 左クリック: クリック位置に最も近い格子点の候補 (corners) を選ぶ。選んだ点には赤い円を描く
 * - 右クリック: クリック位置をそのまま選ぶ。選んだ点には青い円を描く
 *
 * 既に 49 点を選んでいる場合は、これ以上選べない。
 * 既に選んだ点と同じ座標の点は、追加しない。
 * 追加したら、選んだ点の数 ("Corners: 選んだ数/49") を映像に重ねて表示する。
 *
 * @param[in] event    マウスのイベントの種類 (cv::EVENT_LBUTTONDOWN など)
 * @param[in] x        クリックした位置の X 座標 [pixel]
 * @param[in] y        クリックした位置の Y 座標 [pixel]
 * @param[in] flags    イベントのフラグ (未使用)
 * @param[in] userdata cv::setMouseCallback() に渡したデータ (未使用)
 */
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

/**
 * @brief キーボードコールバック: キー入力に応じて動作する
 *
 * | キー        | 動作                                            |
 * |-------------|-------------------------------------------------|
 * | ESC / q / Q | プログラムを終了する                            |
 * | f / F       | フルスクリーンにする                            |
 * | c / C       | 現在の映像を calibration_600x600.png に保存する |
 * | スペース    | 画角の調整を終える (adjustingFOV を false にする) |
 *
 * @param[in] key 押されたキー
 * @param[in] x   キーを押したときのマウスの X 座標 (未使用)
 * @param[in] y   キーを押したときのマウスの Y 座標 (未使用)
 */
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