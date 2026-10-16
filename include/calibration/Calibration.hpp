/**
 * @file Calibration.hpp
 * @brief グリッド点を用いた区分的射影変換によるキャリブレーションクラスの宣言
 *
 * 計測座標系 (LiDAR / カメラなど) 上に並んだ格子点 (cols × rows) を読み込み、
 * 隣接する 4 点で囲まれた四角形ごとに、ウィンドウ座標系の均等な格子へ射影変換する。
 * 全体を 1 つのホモグラフィで変換するよりも、レンズ歪みや投影面の歪みに追従しやすい。
 *
 * @par 座標系
 * - calibrationQuads : 計測座標系 (キャリブレーションファイルの点) 上の四角形
 * - transformQuads   : ウィンドウ座標系 (windowWidth × windowHeight を等分した格子) 上の四角形
 * 
 * Copyright (c) 2026 Maito Yasui
 * Released under the GNU AGPL-3.0 License.
 * https://github.com/Nibosi0501/LiViMotion/blob/main/LICENSE
 */
#ifndef CALIBRATION_H
#define CALIBRATION_H

#include <opencv2/opencv.hpp>

#include <opencv2/geometry.hpp> // OpenCV5.0のバグで <opencv2/opencv.hpp> で　cv::pointPolygonTest, cv::getPerspectiveTransform が使えないため、geometry.hppを追加

/**
 * @namespace clb
 * @brief キャリブレーション関連の名前空間
 */
namespace clb
{
    /**
     * @brief 4 頂点で表される四角形
     *
     * 頂点は左上から時計回り (topLeft → topRight → bottomRight → bottomLeft) の順で
     * 扱われる。
     */
    struct Quad
    {
        cv::Point2f topLeft;     ///< 左上の頂点
        cv::Point2f topRight;    ///< 右上の頂点
        cv::Point2f bottomLeft;  ///< 左下の頂点
        cv::Point2f bottomRight; ///< 右下の頂点
    };

    /**
     * @brief グリッド点ベースの区分的射影変換によるキャリブレーション
     *
     * コンストラクタでキャリブレーションファイルを読み込み、
     * (cols - 1) × (rows - 1) 個の四角形について変換元 (計測座標系) と
     * 変換先 (ウィンドウ座標系) の対応を構築する。
     * 点の変換は transformPoint()、画像の変換は transformImage() で行う。
     */
    struct Calibration
    {
        /// 変換元の四角形。[行][列] で (rows - 1) × (cols - 1) 個。計測座標系
        std::vector<std::vector<Quad>> calibrationQuads;
        /// 変換先の四角形。[行][列] で (rows - 1) × (cols - 1) 個。ウィンドウ座標系
        std::vector<std::vector<Quad>> transformQuads;
        int cols;           ///< グリッド点の列数 (四角形の列数は cols - 1)
        int rows;           ///< グリッド点の行数 (四角形の行数は rows - 1)
        int windowWidth;    ///< 変換先ウィンドウの幅 [pixel]
        int windowHeight;   ///< 変換先ウィンドウの高さ [pixel]
        std::string attribute; ///< このキャリブレーションを識別するための属性文字列

        /**
         * @brief キャリブレーションファイルを読み込み、変換用の四角形を構築する
         *
         * @par ファイル形式
         * 空白区切りのテキストで、グリッド点を行優先 (左上 → 右へ → 次の行) で
         * 「x y」の順に rows × cols 個並べる。
         *
         * @param filename      キャリブレーションファイルのパス
         *                      (main() のあるプログラムからの相対パス)
         * @param _cols         グリッド点の列数 (2 以上)
         * @param _rows         グリッド点の行数 (2 以上)
         * @param _windowWidth  変換先ウィンドウの幅 [pixel]
         * @param _windowHeight 変換先ウィンドウの高さ [pixel]
         * @param _attribute    識別用の属性文字列
         *
         * @warning ファイルが開けない場合はエラーを出力して return するだけで、例外は投げない。
         *          その場合 calibrationQuads / transformQuads は初期値 (全頂点が (0, 0)) のままとなり、
         *          transformPoint() は常に false を返す。
         */
        Calibration(const std::string &filename, const int _cols, const int _rows, const int _windowWidth, const int _windowHeight, const std::string &_attribute);

        /**
         * @brief 点が四角形の内部または境界上にあるか判定する
         * @param[in] targetPoint 判定する点
         * @param[in] quad        判定対象の四角形
         * @retval true  点が四角形の内部、または境界線上にある
         * @retval false 点が四角形の外にある
         */
        bool isPointInQuad(const cv::Point2f &targetPoint, const Quad &quad) const;

        /**
         * @brief 計測座標系の点を、ウィンドウ座標系の点に変換する
         *
         * targetPoint を含む calibrationQuads の四角形を探し、
         * その四角形に対応するホモグラフィで変換する。
         * 複数の四角形の境界上にある場合は、先に見つかった (行・列の小さい) 四角形を使う。
         *
         * @param[in]  targetPoint 変換する点 (計測座標系)
         * @param[out] outPoint    変換後の点 (ウィンドウ座標系)。変換できた場合のみ更新される
         * @retval true  変換に成功した
         * @retval false どの四角形にも含まれず、変換できなかった
         */
        bool transformPoint(const cv::Point2f &targetPoint, cv::Point2f &outPoint) const;

        /**
         * @brief キャリブレーション結果の確認用に、画像を四角形ごとに射影変換して合成する
         *
         * 各四角形について、入力画像全体を射影変換し、対応する transformQuads の
         * 領域だけをマスクで切り出して outImage に重ねる。
         *
         * @param[in]     inputImage 変換元の画像 (計測座標系)
         * @param[in,out] outImage   変換結果の出力先。呼び出し前に
         *                           windowWidth × windowHeight 相当のサイズで確保しておくこと
         *                           (このサイズが warpPerspective の出力サイズになる)。
         *                           四角形外の領域は変更されない
         */
        // 確認描画用の関数
        void transformImage(const cv::Mat &inputImage, cv::Mat &outImage) const;
    };
}

#endif
