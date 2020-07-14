// Copyright 2020 Vladimir
// Author: Vladimir
#pragma once

namespace CUDA {
    void createEquations(
        const float* sourcePoints,
        const float* targetPoints,
        const float* targetNormals,
        int pointsNum,
        float* A,
        float* b);
}
