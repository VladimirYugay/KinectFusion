// Copyright 2020 Vladimir
// Author: Vladimir
#pragma once

#include <iostream>
#include "../helpers/Eigen.h"

namespace CUDA {

    void integrate(
        Vector3f bottomLeft, Vector3f topRight,
        uint dx, uint dy, uint dz,
        Matrix4f extrinsic, Matrix3f intrinsic,
        int frameWidth, int frameHeight,
        const float* depthMap,
        float* values, float* weights);
}  // namespace CUDA