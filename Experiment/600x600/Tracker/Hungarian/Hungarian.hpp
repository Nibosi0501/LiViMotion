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
// modified by Maito Yasui on 2026-04-02
//
// Copyright (c) 2026 Maito Yasui
// This file incorporates code licensed under the BSD and MIT Licenses (see above).
// Modifications and additions by Maito Yasui are released under the GNU GPL-3.0.
// https://github.com/Nibosi0501/LiViMotion/blob/main/LICENSE

#pragma once

#include <iostream>
#include <vector>
#include <stdlib.h>
#include <cfloat>
#include <cmath>

namespace Hungarian
{
    double Solve(std::vector<std::vector<double>> &DistMatrix, std::vector<int> &Assignment);
    void assignmentoptimal(int *assignment, double *cost, double *distMatrixIn, int nOfRows, int nOfColumns);
    void buildassignmentvector(int *assignment, bool *starMatrix, int nOfRows, int nOfColumns);
    void computeassignmentcost(int *assignment, double *cost, double *distMatrix, int nOfRows);
    void step2a(int *assignment, double *distMatrix, bool *starMatrix, bool *newStarMatrix, bool *primeMatrix, bool *coveredColumns, bool *coveredRows, int nOfRows, int nOfColumns, int minDim);
    void step2b(int *assignment, double *distMatrix, bool *starMatrix, bool *newStarMatrix, bool *primeMatrix, bool *coveredColumns, bool *coveredRows, int nOfRows, int nOfColumns, int minDim);
    void step3(int *assignment, double *distMatrix, bool *starMatrix, bool *newStarMatrix, bool *primeMatrix, bool *coveredColumns, bool *coveredRows, int nOfRows, int nOfColumns, int minDim);
    void step4(int *assignment, double *distMatrix, bool *starMatrix, bool *newStarMatrix, bool *primeMatrix, bool *coveredColumns, bool *coveredRows, int nOfRows, int nOfColumns, int minDim, int row, int col);
    void step5(int *assignment, double *distMatrix, bool *starMatrix, bool *newStarMatrix, bool *primeMatrix, bool *coveredColumns, bool *coveredRows, int nOfRows, int nOfColumns, int minDim);

    bool isLink();
}