// Copyright 2020 Vladimir
// Author: Vladimir
#pragma once

#include <memory>
#include <utility>
#include <vector>

#include "Eigen.h"
#include "Frame.h"

class ICP {
 public:
  ICP(Frame &_prevFrame, Frame &_curFrame, const double distanceThreshold,
      const double normalThreshold);

  Matrix4f estimatePose(
      const std::vector<std::pair<size_t, size_t>> &correspondenceIds,
      int iterationsNum = 1);

  std::vector<std::pair<size_t, size_t>> findIndicesOfCorrespondingPoints(
      const Eigen::Matrix4f &estimatedPose);

 private:
  Frame &prevFrame;
  Frame &curFrame;
  const double distanceThreshold;
  const double normalThreshold;
};
