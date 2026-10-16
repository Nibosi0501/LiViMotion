///////////////////////////////////////////////////////////////////////////////
// Hungarian.h: Header file for Class HungarianAlgorithm.
//
// This is a C++ wrapper with slight modification of a hungarian algorithm implementation by Markus Buehren.
// The original implementation is a few mex-functions for use in MATLAB, found here:
// http://www.mathworks.com/matlabcentral/fileexchange/6543-functions-for-the-rectangular-assignment-problem
//
// Both this code and the orignal code are published under the BSD license.
// by Cong Ma, 2016
//
// Copyright (c) 2014, Markus Buehren
// All rights reserved.
//
// Copyright (c) 2019 Delta Group, City Univeristy of Hong Kong
// Copyright (c) 2019 Dr. Li Shuaicheng
// Released under the MIT License. https://github.com/deepomicslab/SuperTAD/blob/master/LICENSE.md
//
// modified by Maito Yasui on 2026-10-16
//
// Copyright (c) 2026 Maito Yasui
// This file incorporates code licensed under the BSD and MIT Licenses (see above).
// Modifications and additions by Maito Yasui are released under the GNU AGPL-3.0.
// https://github.com/Nibosi0501/LiViMotion/blob/main/LICENSE

/**
 * @file Hungarian.hpp
 * @brief ハンガリアン法 (Munkres 法) による割り当て問題ソルバーの宣言
 *
 * コスト行列から、合計コストが最小になる「行と列の 1 対 1 の割り当て」を求める。
 * 行数と列数が異なる長方行列にも対応する。
 * 追跡処理で、既存のトラックと新しい検出結果を対応付ける用途などに使う。
 *
 * @par 使い方
 * @code
 * std::vector<std::vector<double>> cost = {{1.0, 5.0}, {4.0, 2.0}};
 * std::vector<int> assignment;
 * double total = Hungarian::Solve(cost, assignment);
 * // assignment[0] == 0, assignment[1] == 1, total == 3.0
 * @endcode
 *
 * @note Solve() 以外の関数 (assignmentoptimal, step2a〜step5 など) は
 *       アルゴリズムの内部処理であり、通常は直接呼び出さない。
 */

#pragma once

#include <iostream>
#include <vector>
#include <stdlib.h>
#include <cfloat>
#include <cmath>

/**
 * @namespace Hungarian
 * @brief ハンガリアン法による割り当て問題の解法
 *
 * 内部の行列はすべて「列優先 (要素 (i, j) は添字 i + nOfRows * j)」の 1 次元配列で扱う。
 */
namespace Hungarian
{
    /**
     * @brief コスト行列から最小コストの割り当てを求める (外部向けのラッパー関数)
     *
     * 行列を 1 次元配列に詰め替えて assignmentoptimal() を呼び出し、結果を Assignment に格納する。
     *
     * @param[in]  DistMatrix コスト行列。DistMatrix[行][列]。すべての要素が非負であること。
     *                        行数 ≥ 1 かつ列数 ≥ 1 で、全行が同じ長さであること
     * @param[out] Assignment 各行に割り当てられた列番号 (0 始まり)。
     *                        割り当てられなかった行 (行数 > 列数の場合) は -1。
     *                        呼び出し時にクリアされる
     * @return 割り当てられた要素のコストの合計
     *
     * @warning 空の行列 (行数 0) を渡すと DistMatrix[0] への不正アクセスになる。
     */
    double Solve(std::vector<std::vector<double>> &DistMatrix, std::vector<int> &Assignment);

    /**
     * @brief Munkres 法で最適な割り当てを求める本体
     *
     * 行列の縮約と初期の星付けを行った後、step2b から始まる反復処理で解を求める。
     *
     * @param[out] assignment    各行に割り当てられた列番号 (行数分の配列)。未割り当ては -1
     * @param[out] cost          割り当ての合計コスト
     * @param[in]  distMatrixIn  コスト行列 (列優先の 1 次元配列、要素数 nOfRows × nOfColumns)
     * @param[in]  nOfRows       行数
     * @param[in]  nOfColumns    列数
     */
    void assignmentoptimal(int *assignment, double *cost, double *distMatrixIn, int nOfRows, int nOfColumns);

    /**
     * @brief 星付きゼロの位置から、割り当てベクトルを組み立てる
     *
     * @param[out] assignment  各行に割り当てられた列番号
     * @param[in]  starMatrix  星付きゼロの位置を表す行列 (列優先)
     * @param[in]  nOfRows     行数
     * @param[in]  nOfColumns  列数
     *
     * @note マクロ ONE_INDEXING を定義すると、列番号は 1 始まり (MATLAB 形式) になる。
     */
    void buildassignmentvector(int *assignment, bool *starMatrix, int nOfRows, int nOfColumns);

    /**
     * @brief 割り当て結果から合計コストを計算する
     *
     * 割り当てられた行 (列番号が 0 以上) の元のコストだけを合計し、cost に加算する。
     *
     * @param[in]     assignment 各行に割り当てられた列番号
     * @param[in,out] cost       合計コスト (呼び出し側で 0 に初期化しておくこと)
     * @param[in]     distMatrix 元のコスト行列 (列優先)
     * @param[in]     nOfRows    行数
     */
    void computeassignmentcost(int *assignment, double *cost, double *distMatrix, int nOfRows);

    /**
     * @brief ステップ 2a: 星付きゼロを含む列をすべてカバーする
     *
     * 処理後は step2b へ進む。
     *
     * @param[out]    assignment     割り当て結果
     * @param[in]     distMatrix     作業用のコスト行列 (列優先)
     * @param[in,out] starMatrix     星付きゼロの行列
     * @param[in,out] newStarMatrix  step4 で使う一時的な星行列
     * @param[in,out] primeMatrix    プライム付きゼロの行列
     * @param[in,out] coveredColumns 列のカバー状態
     * @param[in,out] coveredRows    行のカバー状態
     * @param[in]     nOfRows        行数
     * @param[in]     nOfColumns     列数
     * @param[in]     minDim         行数と列数の小さいほう (求める割り当ての数)
     */
    void step2a(int *assignment, double *distMatrix, bool *starMatrix, bool *newStarMatrix, bool *primeMatrix, bool *coveredColumns, bool *coveredRows, int nOfRows, int nOfColumns, int minDim);

    /**
     * @brief ステップ 2b: カバーされた列の数を調べ、終了か継続かを判定する
     *
     * カバーされた列の数が minDim に等しければ、割り当てが完成したとして
     * buildassignmentvector() で結果を作る。そうでなければ step3 へ進む。
     *
     * @param[out]    assignment     割り当て結果
     * @param[in]     distMatrix     作業用のコスト行列 (列優先)
     * @param[in,out] starMatrix     星付きゼロの行列
     * @param[in,out] newStarMatrix  step4 で使う一時的な星行列
     * @param[in,out] primeMatrix    プライム付きゼロの行列
     * @param[in,out] coveredColumns 列のカバー状態
     * @param[in,out] coveredRows    行のカバー状態
     * @param[in]     nOfRows        行数
     * @param[in]     nOfColumns     列数
     * @param[in]     minDim         行数と列数の小さいほう
     */
    void step2b(int *assignment, double *distMatrix, bool *starMatrix, bool *newStarMatrix, bool *primeMatrix, bool *coveredColumns, bool *coveredRows, int nOfRows, int nOfColumns, int minDim);

    /**
     * @brief ステップ 3: カバーされていないゼロにプライムを付ける
     *
     * カバーされていないゼロを探してプライムを付ける。
     * - その行に星付きゼロがなければ step4 へ進む
     * - あれば、その行をカバーして星付きゼロの列のカバーを外し、探索を続ける
     *
     * カバーされていないゼロがなくなったら step5 へ進む。
     *
     * @param[out]    assignment     割り当て結果
     * @param[in]     distMatrix     作業用のコスト行列 (列優先)
     * @param[in,out] starMatrix     星付きゼロの行列
     * @param[in,out] newStarMatrix  step4 で使う一時的な星行列
     * @param[in,out] primeMatrix    プライム付きゼロの行列
     * @param[in,out] coveredColumns 列のカバー状態
     * @param[in,out] coveredRows    行のカバー状態
     * @param[in]     nOfRows        行数
     * @param[in]     nOfColumns     列数
     * @param[in]     minDim         行数と列数の小さいほう
     */
    void step3(int *assignment, double *distMatrix, bool *starMatrix, bool *newStarMatrix, bool *primeMatrix, bool *coveredColumns, bool *coveredRows, int nOfRows, int nOfColumns, int minDim);

    /**
     * @brief ステップ 4: 星とプライムを交互にたどり、星付きゼロを 1 つ増やす
     *
     * 見つかったプライム付きゼロを起点に、同じ列の星付きゼロ、同じ行のプライム付きゼロ、…と
     * 交互にたどる。たどった経路上で、星を外してプライムを星に置き換える。
     * その後プライムをすべて消し、行のカバーを外して step2a へ進む。
     *
     * @param[out]    assignment     割り当て結果
     * @param[in]     distMatrix     作業用のコスト行列 (列優先)
     * @param[in,out] starMatrix     星付きゼロの行列
     * @param[in,out] newStarMatrix  更新後の星を作るための一時的な星行列
     * @param[in,out] primeMatrix    プライム付きゼロの行列
     * @param[in,out] coveredColumns 列のカバー状態
     * @param[in,out] coveredRows    行のカバー状態
     * @param[in]     nOfRows        行数
     * @param[in]     nOfColumns     列数
     * @param[in]     minDim         行数と列数の小さいほう
     * @param[in]     row            経路の起点となるプライム付きゼロの行
     * @param[in]     col            経路の起点となるプライム付きゼロの列
     */
    void step4(int *assignment, double *distMatrix, bool *starMatrix, bool *newStarMatrix, bool *primeMatrix, bool *coveredColumns, bool *coveredRows, int nOfRows, int nOfColumns, int minDim, int row, int col);

    /**
     * @brief ステップ 5: 行列を調整して新しいゼロを作る
     *
     * カバーされていない要素の最小値 h を求め、カバーされた行に h を加え、
     * カバーされていない列から h を引く。その後 step3 へ戻る。
     *
     * @param[out]    assignment     割り当て結果
     * @param[in,out] distMatrix     作業用のコスト行列 (列優先)
     * @param[in,out] starMatrix     星付きゼロの行列
     * @param[in,out] newStarMatrix  step4 で使う一時的な星行列
     * @param[in,out] primeMatrix    プライム付きゼロの行列
     * @param[in,out] coveredColumns 列のカバー状態
     * @param[in,out] coveredRows    行のカバー状態
     * @param[in]     nOfRows        行数
     * @param[in]     nOfColumns     列数
     * @param[in]     minDim         行数と列数の小さいほう
     */
    void step5(int *assignment, double *distMatrix, bool *starMatrix, bool *newStarMatrix, bool *primeMatrix, bool *coveredColumns, bool *coveredRows, int nOfRows, int nOfColumns, int minDim);

    /**
     * @brief Hungarian.cpp とのリンクを確認するためのテスト関数
     *
     * 標準出力に確認メッセージを表示する。
     *
     * @return 常に true
     */
    bool isLink();
}
