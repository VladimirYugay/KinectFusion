// Copyright 2020 Vladimir
// Author: Vladimir
#pragma once

#include <vector>

#include "../helpers/Eigen.h"
#include "../models/Frame.h"

class ICP {
 public:
    ICP();
/**
 * Estimates the pose of the camera
 * Expects points in global coordinate frame
*/
    Matrix4f estimatePose(
        const std::vector<Vector3f>& sourcePoints,
        const std::vector<Vector3f>& targetPoints,
        const std::vector<Vector3f>& targetNormals,
        int iterationsNum = 1);
};
