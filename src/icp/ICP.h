// Copyright 2020 Vladimir
// Author: Vladimir
#pragma once

#include <utility>
#include <vector>

#include "../helpers/Eigen.h"
#include "../models/Frame.h"

class ICP {
 public:
    ICP(const Frame& _prevFrame, const Frame& _curFrame);
    Matrix4f estimatePose(
        const std::vector<std::pair<size_t, size_t>>& correspondenceIds,
        int iterationsNum = 1);
    
 private:
    const Frame& prevFrame;
    const Frame& curFrame;
};
