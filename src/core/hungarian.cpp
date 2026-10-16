///////////////////////////////////////////////////////////////////////////////
// Hungarian.cpp: Implementation file for Class HungarianAlgorithm.
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
// modified by wang mengbo on 2020-07-30
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
 * @file hungarian.cpp
 * @brief ハンガリアン法 (Munkres 法) による割り当て問題ソルバーの実装
 * @see Hungarian.hpp
 *
 * @par アルゴリズムの流れ
 * -# assignmentoptimal(): 行 (または列) ごとに最小値を引き、ゼロに星を付ける
 * -# step2b(): カバーされた列数が minDim なら終了。そうでなければ step3 へ
 * -# step3(): カバーされていないゼロにプライムを付ける。星がなければ step4、
 *    ゼロが尽きたら step5 へ
 * -# step4(): 星とプライムの交互経路をたどって星を 1 つ増やし、step2a へ
 * -# step5(): 行列を調整して新しいゼロを作り、step3 へ戻る
 *
 * @note 各ステップは関数の再帰呼び出しで次のステップへ進む実装になっている。
 *       行列が大きいと呼び出しが深くなる点に注意。
 * @note ゼロ判定は DBL_EPSILON を許容誤差とした fabs() 比較で行う。
 * 
 * @par ライセンス
 * ライセンスの全文は、このファイルの冒頭のコメントを参照。
 */

#include <iostream>
#include <vector>
#include <stdlib.h>
#include <cfloat>
#include <cmath>

#include "../../include/core/Hungarian.hpp"

//********************************************************//
// A single function wrapper for solving assignment problem.
//********************************************************//
/**
 * @details
 * 処理の流れ:
 * -# DistMatrix を列優先の 1 次元配列 (添字 i + nRows * j) に詰め替える
 * -# assignmentoptimal() を呼んで最適割り当てを求める
 * -# 結果を Assignment にコピーし、確保した配列を解放して合計コストを返す
 *
 * @param[in]  DistMatrix コスト行列 (DistMatrix[行][列]、非負、空でないこと)
 * @param[out] Assignment 各行に割り当てられた列番号。未割り当ては -1
 * @return 割り当てられた要素のコストの合計
 */
double Hungarian::Solve(std::vector<std::vector<double>> &DistMatrix, std::vector<int> &Assignment)
{
    unsigned int nRows = DistMatrix.size();
    unsigned int nCols = DistMatrix[0].size();

    double *distMatrixIn = new double[nRows * nCols];
    int *assignment = new int[nRows];
    double cost = 0.0;

    // Fill in the distMatrixIn. Mind the index is "i + nRows * j".
    // Here the cost matrix of size MxN is defined as a double precision array of N*M elements.
    // In the solving functions matrices are seen to be saved MATLAB-internally in row-order.
    // (i.e. the matrix [1 2; 3 4] will be stored as a vector [1 3 2 4], NOT [1 2 3 4]).
    for (unsigned int i = 0; i < nRows; i++)
        for (unsigned int j = 0; j < nCols; j++)
            distMatrixIn[i + nRows * j] = DistMatrix[i][j];

    // call solving function
    Hungarian::assignmentoptimal(assignment, &cost, distMatrixIn, nRows, nCols);

    Assignment.clear();
    for (unsigned int r = 0; r < nRows; r++)
        Assignment.push_back(assignment[r]);

    delete[] distMatrixIn;
    delete[] assignment;
    return cost;
}

//********************************************************//
// Solve optimal solution for assignment problem using Munkres algorithm, also known as Hungarian Algorithm.
//********************************************************//
/**
 * @details
 * 処理の流れ:
 * -# assignment を -1 で初期化し、作業用のコスト行列と、カバー・星・プライム用のフラグ配列を確保する
 * -# 前処理 (行数 ≤ 列数なら各行、行数 > 列数なら各列から最小値を引く) を行う
 * -# ステップ 1 と 2a: 縮約後のゼロのうち、同じ行・列に星がないものに星を付け、
 *    該当する列 (行数 > 列数の場合は行も) をカバーする
 * -# step2b() 以降の反復処理で解を求める
 * -# 元のコスト行列を使って合計コストを計算し、確保したメモリを解放する
 *
 * @param[out] assignment    各行に割り当てられた列番号 (行数分の配列)
 * @param[out] cost          割り当ての合計コスト
 * @param[in]  distMatrixIn  コスト行列 (列優先の 1 次元配列)
 * @param[in]  nOfRows       行数
 * @param[in]  nOfColumns    列数
 */
void Hungarian::assignmentoptimal(int *assignment, double *cost, double *distMatrixIn, int nOfRows, int nOfColumns)
{
    double *distMatrix, *distMatrixTemp, *distMatrixEnd, *columnEnd, value, minValue;
    bool *coveredColumns, *coveredRows, *starMatrix, *newStarMatrix, *primeMatrix;
    int nOfElements, minDim, row, col;

    /* initialization */
    *cost = 0;
    for (row = 0; row < nOfRows; row++)
        assignment[row] = -1;

    /* generate working copy of distance Matrix */
    /* check if all matrix elements are positive */
    nOfElements = nOfRows * nOfColumns;
    distMatrix = (double *)malloc(nOfElements * sizeof(double));
    distMatrixEnd = distMatrix + nOfElements;

    for (row = 0; row < nOfElements; row++)
    {
        value = distMatrixIn[row];
        //		if (value < 0)
        //			cerr << "All matrix elements have to be non-negative." << endl;
        distMatrix[row] = value;
    }

    /* memory allocation */
    coveredColumns = (bool *)calloc(nOfColumns, sizeof(bool));
    coveredRows = (bool *)calloc(nOfRows, sizeof(bool));
    starMatrix = (bool *)calloc(nOfElements, sizeof(bool));
    primeMatrix = (bool *)calloc(nOfElements, sizeof(bool));
    newStarMatrix = (bool *)calloc(nOfElements, sizeof(bool)); /* used in step4 */

    /* preliminary steps */
    if (nOfRows <= nOfColumns)
    {
        minDim = nOfRows;

        for (row = 0; row < nOfRows; row++)
        {
            /* find the smallest element in the row */
            distMatrixTemp = distMatrix + row;
            minValue = *distMatrixTemp;
            distMatrixTemp += nOfRows;
            while (distMatrixTemp < distMatrixEnd)
            {
                value = *distMatrixTemp;
                if (value < minValue)
                    minValue = value;
                distMatrixTemp += nOfRows;
            }

            /* subtract the smallest element from each element of the row */
            distMatrixTemp = distMatrix + row;
            while (distMatrixTemp < distMatrixEnd)
            {
                *distMatrixTemp -= minValue;
                distMatrixTemp += nOfRows;
            }
        }

        /* Steps 1 and 2a */
        for (row = 0; row < nOfRows; row++)
            for (col = 0; col < nOfColumns; col++)
                if (fabs(distMatrix[row + nOfRows * col]) < DBL_EPSILON)
                    if (!coveredColumns[col])
                    {
                        starMatrix[row + nOfRows * col] = true;
                        coveredColumns[col] = true;
                        break;
                    }
    }
    else /* if(nOfRows > nOfColumns) */
    {
        minDim = nOfColumns;

        for (col = 0; col < nOfColumns; col++)
        {
            /* find the smallest element in the column */
            distMatrixTemp = distMatrix + nOfRows * col;
            columnEnd = distMatrixTemp + nOfRows;

            minValue = *distMatrixTemp++;
            while (distMatrixTemp < columnEnd)
            {
                value = *distMatrixTemp++;
                if (value < minValue)
                    minValue = value;
            }

            /* subtract the smallest element from each element of the column */
            distMatrixTemp = distMatrix + nOfRows * col;
            while (distMatrixTemp < columnEnd)
                *distMatrixTemp++ -= minValue;
        }

        /* Steps 1 and 2a */
        for (col = 0; col < nOfColumns; col++)
            for (row = 0; row < nOfRows; row++)
                if (fabs(distMatrix[row + nOfRows * col]) < DBL_EPSILON)
                    if (!coveredRows[row])
                    {
                        starMatrix[row + nOfRows * col] = true;
                        coveredColumns[col] = true;
                        coveredRows[row] = true;
                        break;
                    }
        for (row = 0; row < nOfRows; row++)
            coveredRows[row] = false;
    }

    /* move to step 2b */
    Hungarian::step2b(assignment, distMatrix, starMatrix, newStarMatrix, primeMatrix, coveredColumns, coveredRows, nOfRows, nOfColumns, minDim);

    /* compute cost and remove invalid assignments */
    Hungarian::computeassignmentcost(assignment, cost, distMatrixIn, nOfRows);

    /* free allocated memory */
    free(distMatrix);
    free(coveredColumns);
    free(coveredRows);
    free(starMatrix);
    free(primeMatrix);
    free(newStarMatrix);

    return;
}

/********************************************************/
/**
 * @details
 * 星付きゼロを行ごとに探し、見つかった列番号を assignment[行] に設定する。
 * 星のない行は変更されない (assignmentoptimal() で設定した -1 のまま)。
 */
void Hungarian::buildassignmentvector(int *assignment, bool *starMatrix, int nOfRows, int nOfColumns)
{
    int row, col;

    for (row = 0; row < nOfRows; row++)
        for (col = 0; col < nOfColumns; col++)
            if (starMatrix[row + nOfRows * col])
            {
#ifdef ONE_INDEXING
                assignment[row] = col + 1; /* MATLAB-Indexing */
#else
                assignment[row] = col;
#endif
                break;
            }
}

/********************************************************/
/**
 * @details
 * assignment[行] が 0 以上の行について、元のコスト行列の要素 (行, assignment[行]) を
 * *cost に加算する。縮約後ではなく元の行列を使うため、実際のコストの合計になる。
 */
void Hungarian::computeassignmentcost(int *assignment, double *cost, double *distMatrix, int nOfRows)
{
    int row, col;

    for (row = 0; row < nOfRows; row++)
    {
        col = assignment[row];
        if (col >= 0)
            *cost += distMatrix[row + nOfRows * col];
    }
}

/********************************************************/
/**
 * @details
 * 各列を調べ、星付きゼロを含む列を coveredColumns でカバー済みにする。
 * その後 step2b() へ進む。
 */
void Hungarian::step2a(int *assignment, double *distMatrix, bool *starMatrix, bool *newStarMatrix, bool *primeMatrix, bool *coveredColumns, bool *coveredRows, int nOfRows, int nOfColumns, int minDim)
{
    bool *starMatrixTemp, *columnEnd;
    int col;

    /* cover every column containing a starred zero */
    for (col = 0; col < nOfColumns; col++)
    {
        starMatrixTemp = starMatrix + nOfRows * col;
        columnEnd = starMatrixTemp + nOfRows;
        while (starMatrixTemp < columnEnd)
        {
            if (*starMatrixTemp++)
            {
                coveredColumns[col] = true;
                break;
            }
        }
    }

    /* move to step 3 */
    Hungarian::step2b(assignment, distMatrix, starMatrix, newStarMatrix, primeMatrix, coveredColumns, coveredRows, nOfRows, nOfColumns, minDim);
}

/********************************************************/
/**
 * @details
 * カバーされた列の数を数える。minDim に等しければ割り当てが完成しているので
 * buildassignmentvector() で結果を作って終了する。そうでなければ step3() へ進む。
 */
void Hungarian::step2b(int *assignment, double *distMatrix, bool *starMatrix, bool *newStarMatrix, bool *primeMatrix, bool *coveredColumns, bool *coveredRows, int nOfRows, int nOfColumns, int minDim)
{
    int col, nOfCoveredColumns;

    /* count covered columns */
    nOfCoveredColumns = 0;
    for (col = 0; col < nOfColumns; col++)
        if (coveredColumns[col])
            nOfCoveredColumns++;

    if (nOfCoveredColumns == minDim)
    {
        /* algorithm finished */
        Hungarian::buildassignmentvector(assignment, starMatrix, nOfRows, nOfColumns);
    }
    else
    {
        /* move to step 3 */
        Hungarian::step3(assignment, distMatrix, starMatrix, newStarMatrix, primeMatrix, coveredColumns, coveredRows, nOfRows, nOfColumns, minDim);
    }
}

/********************************************************/
/**
 * @details
 * カバーされていないゼロがある限り、次を繰り返す。
 * -# そのゼロにプライムを付ける
 * -# 同じ行に星付きゼロがなければ、step4() へ進んで終了する
 * -# あれば、その行をカバーし、星付きゼロの列のカバーを外して探索を続ける
 *
 * カバーされていないゼロがなくなったら step5() へ進む。
 */
void Hungarian::step3(int *assignment, double *distMatrix, bool *starMatrix, bool *newStarMatrix, bool *primeMatrix, bool *coveredColumns, bool *coveredRows, int nOfRows, int nOfColumns, int minDim)
{
    bool zerosFound;
    int row, col, starCol;

    zerosFound = true;
    while (zerosFound)
    {
        zerosFound = false;
        for (col = 0; col < nOfColumns; col++)
            if (!coveredColumns[col])
                for (row = 0; row < nOfRows; row++)
                    if ((!coveredRows[row]) && (fabs(distMatrix[row + nOfRows * col]) < DBL_EPSILON))
                    {
                        /* prime zero */
                        primeMatrix[row + nOfRows * col] = true;

                        /* find starred zero in current row */
                        for (starCol = 0; starCol < nOfColumns; starCol++)
                            if (starMatrix[row + nOfRows * starCol])
                                break;

                        if (starCol == nOfColumns) /* no starred zero found */
                        {
                            /* move to step 4 */
                            Hungarian::step4(assignment, distMatrix, starMatrix, newStarMatrix, primeMatrix, coveredColumns, coveredRows, nOfRows, nOfColumns, minDim, row, col);
                            return;
                        }
                        else
                        {
                            coveredRows[row] = true;
                            coveredColumns[starCol] = false;
                            zerosFound = true;
                            break;
                        }
                    }
    }

    /* move to step 5 */
    Hungarian::step5(assignment, distMatrix, starMatrix, newStarMatrix, primeMatrix, coveredColumns, coveredRows, nOfRows, nOfColumns, minDim);
}

/********************************************************/
/**
 * @details
 * 処理の流れ:
 * -# starMatrix を newStarMatrix にコピーし、起点のプライム付きゼロ (row, col) に星を付ける
 * -# 同じ列の星付きゼロ → 同じ行のプライム付きゼロ、と交互にたどり、
 *    星付きゼロの星を外し、プライム付きゼロに星を付ける (星が見つからなくなるまで)
 * -# newStarMatrix を新しい starMatrix として採用し、プライムをすべて消し、行のカバーをすべて外す
 * -# step2a() へ進む
 */
void Hungarian::step4(int *assignment, double *distMatrix, bool *starMatrix, bool *newStarMatrix, bool *primeMatrix, bool *coveredColumns, bool *coveredRows, int nOfRows, int nOfColumns, int minDim, int row, int col)
{
    int n, starRow, starCol, primeRow, primeCol;
    int nOfElements = nOfRows * nOfColumns;

    /* generate temporary copy of starMatrix */
    for (n = 0; n < nOfElements; n++)
        newStarMatrix[n] = starMatrix[n];

    /* star current zero */
    newStarMatrix[row + nOfRows * col] = true;

    /* find starred zero in current column */
    starCol = col;
    for (starRow = 0; starRow < nOfRows; starRow++)
        if (starMatrix[starRow + nOfRows * starCol])
            break;

    while (starRow < nOfRows)
    {
        /* unstar the starred zero */
        newStarMatrix[starRow + nOfRows * starCol] = false;

        /* find primed zero in current row */
        primeRow = starRow;
        for (primeCol = 0; primeCol < nOfColumns; primeCol++)
            if (primeMatrix[primeRow + nOfRows * primeCol])
                break;

        /* star the primed zero */
        newStarMatrix[primeRow + nOfRows * primeCol] = true;

        /* find starred zero in current column */
        starCol = primeCol;
        for (starRow = 0; starRow < nOfRows; starRow++)
            if (starMatrix[starRow + nOfRows * starCol])
                break;
    }

    /* use temporary copy as new starMatrix */
    /* delete all primes, uncover all rows */
    for (n = 0; n < nOfElements; n++)
    {
        primeMatrix[n] = false;
        starMatrix[n] = newStarMatrix[n];
    }
    for (n = 0; n < nOfRows; n++)
        coveredRows[n] = false;

    /* move to step 2a */
    Hungarian::step2a(assignment, distMatrix, starMatrix, newStarMatrix, primeMatrix, coveredColumns, coveredRows, nOfRows, nOfColumns, minDim);
}

/********************************************************/
/**
 * @details
 * 処理の流れ:
 * -# カバーされていない要素の最小値 h を求める
 * -# カバーされている各行の全要素に h を加える
 * -# カバーされていない各列の全要素から h を引く
 * -# step3() へ戻る
 *
 * この操作により、カバーされていない領域に新しいゼロが生まれ、
 * 最適解を変えずに探索を進められる。
 */
void Hungarian::step5(int *assignment, double *distMatrix, bool *starMatrix, bool *newStarMatrix, bool *primeMatrix, bool *coveredColumns, bool *coveredRows, int nOfRows, int nOfColumns, int minDim)
{
    double h, value;
    int row, col;

    /* find smallest uncovered element h */
    h = DBL_MAX;
    for (row = 0; row < nOfRows; row++)
        if (!coveredRows[row])
            for (col = 0; col < nOfColumns; col++)
                if (!coveredColumns[col])
                {
                    value = distMatrix[row + nOfRows * col];
                    if (value < h)
                        h = value;
                }

    /* add h to each covered row */
    for (row = 0; row < nOfRows; row++)
        if (coveredRows[row])
            for (col = 0; col < nOfColumns; col++)
                distMatrix[row + nOfRows * col] += h;

    /* subtract h from each uncovered column */
    for (col = 0; col < nOfColumns; col++)
        if (!coveredColumns[col])
            for (row = 0; row < nOfRows; row++)
                distMatrix[row + nOfRows * col] -= h;

    /* move to step 3 */
    Hungarian::step3(assignment, distMatrix, starMatrix, newStarMatrix, primeMatrix, coveredColumns, coveredRows, nOfRows, nOfColumns, minDim);
}

/**
 * @details 標準出力に「Hungarian.cpp とのリンク成功」と表示し、true を返す。
 */
bool Hungarian::isLink()
{
    std::cout << "Hungarian.cpp とのリンク成功" << std::endl;
    return true;
}
