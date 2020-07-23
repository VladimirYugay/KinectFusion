// Copyright 2020 Vladimir
// Author: Vladimir
#pragma once

#include "../models/Frame.h"
#include "../helpers/Eigen.h"

namespace CUDA {

    int* findCorrespondences(
        const Frame &prevFrame,
        const Frame &curFrame,
        const Matrix4f &extrinsics,
        const Matrix4f &pose);

    int* testFindCorrespondences(
        const Vector3f* prevV, const Vector3f* prevN,
        int prevSize, int prevWidth, int prevHeight,
        const Vector3f* curV, const Vector3f* curN,
        int curSize, int curWidth, int curHeight,
        const Matrix4f extrinsics, const Matrix4f pose);

    void createEquations(
        const float* sourcePoints,
        const float* targetPoints,
        const float* targetNormals,
        int pointsNum,
        float* A,
        float* b);
}
