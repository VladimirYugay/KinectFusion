// Copyright 2020 Vladimir
// Author: Vladimir
#include <iostream>

#include "ICP.h"
#include "CUDAICPWrapper.cuh"


ICP::ICP(const Frame& _prevFrame, const Frame& _curFrame)
    : prevFrame(_prevFrame), curFrame(_curFrame) {}

Matrix4f ICP::estimatePose(
    const std::vector<std::pair<size_t, size_t>>& correspondenceIds,
    int iterationsNum) {

    int nPoints = correspondenceIds.size();
    Matrix4f estimatedPose = Matrix4f::Identity();
    MatrixXf A = MatrixXf::Zero(nPoints, 6);
    VectorXf b = VectorXf::Zero(nPoints);


    // TEST RUN ON TEST DATA
    float sourcePoints[9] = {0, 0, 1, 0, 1, 0, 1, 0, 0};
    float targetPoints[9] = {0, 0, 1, 0, 1, 0, 1, 0, 0};
    float targetNormals[9] = {0, 0, 1, 0, 1, 0, 1, 0, 0};
    float A_arr[9] = {0, 0, 1, 0, 1, 0, 1, 0, 0};
    float b_arr[9] = {0, 0, 1, 0, 1, 0, 1, 0, 0};

    CUDA::createEquations(sourcePoints, targetPoints, targetNormals,
        3, A_arr, b_arr);


    for (size_t i = 0; i < nPoints; i++) {
        auto pair = correspondenceIds[i];
        const auto& x = prevFrame.getVertex(pair.first);
        const auto& y = curFrame.getVertex(pair.second);
        const auto& n = curFrame.getNormal(pair.second);

        A.block<1, 3>(i, 0) = x.cross(n);
        A.block<1, 3>(i, 3) = n;
        b(i) = n.dot(y) - n.dot(x);
    }

    VectorXf x(6);
    x = A.bdcSvd(ComputeThinU | ComputeThinV).solve(b);

    const float alpha = x(0), beta = x(1), gamma = x(2);
    Matrix3f rotation =
                AngleAxisf(alpha, Vector3f::UnitX()).toRotationMatrix() *
                AngleAxisf(beta, Vector3f::UnitY()).toRotationMatrix() *
                AngleAxisf(gamma, Vector3f::UnitZ()).toRotationMatrix();
    Vector3f translation = x.tail(3);

    Matrix4f currentPose = Matrix4f::Identity();
    currentPose.block<3, 3>(0, 0) = rotation;
    currentPose.block<3, 1>(0, 3) = translation;
    estimatedPose = currentPose * estimatedPose;


    return estimatedPose;
}
