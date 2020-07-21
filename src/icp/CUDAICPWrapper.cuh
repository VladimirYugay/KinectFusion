// Copyright 2020 Vladimir
// Author: Vladimir
#pragma once

#include "../models/Frame.h"
#include "../helpers/Eigen.h"

namespace CUDA {

    int* findCorrespondences(
        const Frame &prevFrame,
        const Frame &curFrame,
        const Matrix4f &estimatedPose
    );

    void createEquations(
        const float* sourcePoints,
        const float* targetPoints,
        const float* targetNormals,
        int pointsNum,
        float* A,
        float* b);
}
