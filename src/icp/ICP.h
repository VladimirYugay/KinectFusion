// Copyright 2020 Vladimir
// Author: Vladimir
#pragma once

#ifndef ICP_H
#define ICP_H

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
      Eigen::Matrix4f& estimatedPose,
      int iterationsNum = 10 );

  std::vector<std::pair<size_t, size_t>> findIndicesOfCorrespondingPoints(
      const Eigen::Matrix4f &estimatedPose);

 private:
  Frame &prevFrame;
  Frame &curFrame;
  const double distanceThreshold;
  const double normalThreshold;
};

#endif // !ICP_H