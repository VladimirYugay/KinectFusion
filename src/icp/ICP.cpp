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

    // TEST RUN ON TEST DATA
    float sourcePoints[3 * nPoints];
    float targetPoints[3 * nPoints];
    float targetNormals[3 * nPoints];
    float cudaA[6 * nPoints] = { 0.0 };
    float cudab[nPoints] = { 0.0 };

    for (int i = 0; i < nPoints; i++) {
        auto pair = correspondenceIds[i];
        const auto& x = prevFrame.getVertex(pair.first);
        const auto& y = curFrame.getVertex(pair.second);
        const auto& n = curFrame.getNormal(pair.second);

        sourcePoints[3 * i] = x(0);
        sourcePoints[3 * i + 1] = x(1);
        sourcePoints[3 * i + 2] = x(2);

        targetPoints[3 * i] = y(0);
        targetPoints[3 * i + 1] = y(1);
        targetPoints[3 * i + 2] = y(2);

        targetNormals[3 * i] = n(0);
        targetNormals[3 * i + 1] = n(1);
        targetNormals[3 * i + 2] = n(2);
    }

    // Creates a system of linear equations
    // CUDA::createEquations(
    //     sourcePoints, targetPoints, targetNormals,
    //     nPoints, cudaA, cudab);

    auto mat = Matrix4f::Identity(4, 4);

    Vector3f* pointer = prevFrame.getVerticesPtr();
    std::cout << "Before CUDA " << pointer[302990] << std::endl;

    CUDA::testFindCorrespondences(
        prevFrame.getVerticesPtr(), prevFrame.getNormalsPtr(),
        prevFrame.getVertexCount(), prevFrame.getWidth(), prevFrame.getHeight(),
        curFrame.getVerticesPtr(), curFrame.getNormalsPtr(),
        curFrame.getVertexCount(), curFrame.getWidth(), curFrame.getHeight(),
        mat, mat);

    // CUDA::findCorrespondences(
    //     prevFrame, curFrame, mat, mat);

    // Wrap back all the arrays to the eigen format to use the solver

    // std::cout << "Constructed system on CUDA" << std::endl;
    // for (int i = 0; i < nPoints; i++) {
    //     std::cout << cudaA[i] << " " << cudaA[i + 1] << " "
    //     << cudaA[i + 2] << " " << cudaA[i + 3] << " "
    //     << cudaA[i + 4] << " " << cudaA[i + 5] << std::endl;
    // }

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
        b(i) = n.dot(y) - n.dot(x);
    }
    // std::cout << "Constructed system without CUDA" << std::endl;
    // std::cout << A << std::endl;

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
