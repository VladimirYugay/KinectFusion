// Copyright 2020 Vladimir
// Author: Vladimir
#include <iostream>

#include "ICP.h"

ICP::ICP(const Frame& _prevFrame, const Frame& _curFrame)
    : prevFrame(_prevFrame), curFrame(_curFrame) {}

Matrix4f ICP::estimatePose(
    const std::vector<std::pair<size_t, size_t>>& correspondenceIds,
    int iterationsNum) {

    int nPoints = correspondenceIds.size();
    Matrix4f estimatedPose = Matrix4f::Identity();

    for (size_t iteration = 0; iteration < iterationsNum; iteration++) {
        MatrixXf A = MatrixXf::Zero(nPoints, 6);
        VectorXf b = VectorXf::Zero(nPoints);

        for (size_t i = 0; i < nPoints; i++) {
            auto pair = correspondenceIds[i];
            const auto& x = prevFrame.getVertex(pair.first);
            const auto& y = curFrame.getVertex(pair.second);
            const auto& n = curFrame.getNormal(pair.second);

            A(i , 0) = n(2) * x(1) - n(1) * x(2);
            A(i, 1) = n(0) * x(2) - n(2) * x(0);
            A(i, 2) = n(1) * x(0) - n(0) * x(1);
            A(i, 3) = n(0);
            A(i, 4) = n(1);
            A(i, 5) = n(2);
            b(i) = n(0) * y(0) + n(1) * y(1) + n(2) * y(2) -
                n(0) * x(0) - n(1) * x(1) - n(2) * x(2);
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
    }
    return estimatedPose;
}
