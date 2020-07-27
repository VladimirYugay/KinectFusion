// Copyright 2020 Vladimir
// Author: Vladimir
#pragma once

#include <iostream>
#include "../helpers/Eigen.h"

namespace CUDA {

    void findCorrespondences(
        const Vector3f* prevV, const Vector3f* prevN,
        int prevWidth, int prevHeight,
        const Matrix4f prevExtrinsic, const Matrix3f prevIntrinsic,
        const Vector3f* curV, const Vector3f* curN,
        int curWidth, int curHeight,
        const Matrix4f curExtrinsic, const Matrix3f curIntrinsic,
        const Matrix4f estPose,
        int* corrIds);
}  // namespace CUDA