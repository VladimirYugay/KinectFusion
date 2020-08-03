// Copyright 2020 Vladimir
// Author: Vladimir
#include "ICP.h"

#include <iostream>
#include <memory>
#include <utility>

ICP::ICP(Frame &_prevFrame, Frame &_curFrame, const double distanceThreshold,
         const double normalThreshold)
    : prevFrame(_prevFrame),
      curFrame(_curFrame),
      distanceThreshold(distanceThreshold),
      normalThreshold(normalThreshold) {}

Matrix4f ICP::estimatePose(
    Eigen::Matrix4f& estimatedPose,
    int iterationsNum
) {

  for (size_t iteration = 0; iteration < iterationsNum; iteration++) {
    const std::vector<std::pair<size_t, size_t>> correspondenceIds = findIndicesOfCorrespondingPoints(estimatedPose);

    std::cout << "# corresponding points: " << correspondenceIds.size()
        << std::endl;
    std::cout << "# total number of points: "
        << curFrame.getVertexMap().size() << std::endl;

    int nPoints = correspondenceIds.size();
    Eigen::Matrix3f rotationEP = estimatedPose.block(0, 0, 3, 3);
    Eigen::Vector3f translationEP = estimatedPose.block(0, 3, 3, 1);

    MatrixXf A = MatrixXf::Zero(nPoints, 6);
    VectorXf b = VectorXf::Zero(nPoints);

    for (size_t i = 0; i < nPoints; i++) {
      auto pair = correspondenceIds[i];
      const auto &x = rotationEP * curFrame.getVertexGlobal(pair.second) + translationEP;
      const auto &y = prevFrame.getVertexGlobal(pair.first);
      const auto &n = prevFrame.getNormalGlobal(pair.first);

      A(i, 0) = n(2) * x(1) - n(1) * x(2);
      A(i, 1) = n(0) * x(2) - n(2) * x(0);
      A(i, 2) = n(1) * x(0) - n(0) * x(1);
      A(i, 3) = n(0);
      A(i, 4) = n(1);
      A(i, 5) = n(2);
      b(i) = n(0) * y(0) + n(1) * y(1) + n(2) * y(2) - n(0) * x(0) -
             n(1) * x(1) - n(2) * x(2);
    }

    VectorXf x(6);
    x = A.bdcSvd(ComputeThinU | ComputeThinV).solve(b);

    const float alpha = x(0), beta = x(1), gamma = x(2);
    Matrix3f rotation =
        AngleAxisf(alpha, Vector3f::UnitX()).toRotationMatrix() *
        AngleAxisf(beta, Vector3f::UnitY()).toRotationMatrix() *
        AngleAxisf(gamma, Vector3f::UnitZ()).toRotationMatrix();
    Vector3f translation = x.tail(3);

    Matrix4f curentPose = Matrix4f::Identity();
    curentPose.block<3, 3>(0, 0) = rotation;
    curentPose.block<3, 1>(0, 3) = translation;
    estimatedPose = curentPose * estimatedPose;
  }
  return estimatedPose;
}

// Helper method to find corresponding points between curent frame and
// previous frame Reference Paper:
// https://www.cvl.iis.u-tokyo.ac.jp/~oishi/Papers/Alignment/Blais_Registering_multiview_PAMI1995.pdf
// Input: curent frame, previous frame, estimated pose of previous
// frame Output: indices of corresponding vertices in curent and
// previous frame Simple version: only take euclidean distance between
// points into consideration without normals Advanced version: Euclidean
// distance between points + difference in normal angles
std::vector<std::pair<size_t, size_t>> ICP::findIndicesOfCorrespondingPoints(
    const Eigen::Matrix4f &estPose) {
  Eigen::Matrix4f estimatedPose = estPose;
  std::vector<std::pair<size_t, size_t>> indicesOfCorrespondingPoints;

  std::vector<Eigen::Vector3f> prevFrameVertexMapGlobal = prevFrame.getVertexMapGlobal();
  std::vector<Eigen::Vector3f> prevFrameNormalMapGlobal = prevFrame.getNormalMapGlobal();

  std::vector<Eigen::Vector3f> curFrameVertexMapGlobal =
      curFrame.getVertexMapGlobal();
  std::vector<Eigen::Vector3f> curFrameNormalMapGlobal =
      curFrame.getNormalMapGlobal();

  const auto rotation = estimatedPose.block(0, 0, 3, 3);
  const auto translation = estimatedPose.block(0, 3, 3, 1);

  const auto estimatedPoseInv = estimatedPose.inverse();

  const auto rotationInv = estimatedPoseInv.block(0, 0, 3, 3);
  const auto translationInv = estimatedPoseInv.block(0, 3, 3, 1);

  // GPU implementation: use a separate thread for every run of the for
  // loop
  for (size_t idx = 0; idx < prevFrameVertexMapGlobal.size(); idx++) {
    Eigen::Vector3f prevPointGlobal = prevFrameVertexMapGlobal[idx];
    Eigen::Vector3f prevNormalGlobal = prevFrameNormalMapGlobal[idx];
    // std::cout << "Curent Point (Camera): " << curPoint[0] << " " <<
    // curPoint[1] << " " << curPoint[2] << std::endl;
    if (prevPointGlobal.allFinite() && prevNormalGlobal.allFinite()) {

        Eigen::Vector3f prevPointCurCamera = rotationInv * prevPointGlobal + translationInv;
        Eigen::Vector3f prevNormalCurCamera = rotationInv * prevFrameNormalMapGlobal[idx];

      // project point from global camera system into camera system of
      // the current frame
      const Eigen::Vector3f prevPointCurFrame =
          curFrame.projectPointIntoFrame(prevPointCurCamera);
      // project point from camera system of the previous frame onto the
      // image plane of the current frame
      const Eigen::Vector2i prevPointImgCoordCurFrame =
          curFrame.projectOntoImgPlane(prevPointCurFrame);

      if (curFrame.containsImgPoint(prevPointImgCoordCurFrame)) {
        size_t curIdx =
            prevPointImgCoordCurFrame[1] * curFrame.getFrameWidth() +
            prevPointImgCoordCurFrame[0];

        Eigen::Vector3f curFramePointGlobal = rotation * curFrameVertexMapGlobal[curIdx] + translation;
        Eigen::Vector3f curFrameNormalGlobal = rotation * curFrameNormalMapGlobal[curIdx];

        if (curFramePointGlobal.allFinite() &&
            (curFramePointGlobal - prevPointGlobal).norm() <
                distanceThreshold &&
            curFrameNormalGlobal.allFinite() &&
            (std::abs(curFrameNormalGlobal.dot(prevNormalGlobal)) / curFrameNormalGlobal.norm() / prevNormalGlobal.norm() <
             normalThreshold)) {
          indicesOfCorrespondingPoints.push_back(std::make_pair(idx, curIdx));
        }
      }
    }
  }
  return indicesOfCorrespondingPoints;
}