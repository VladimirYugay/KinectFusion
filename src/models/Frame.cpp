// Copyright 2020 Vladimir
// Author: Vladimir
#include "Frame.h"

#include <iostream>


Frame::Frame(const float* depthMap,
            const BYTE* colorMap,
            const Eigen::Matrix3f &depthIntrinsics,
            const Eigen::Matrix4f &depthExtrinsicsInv,
            const Eigen::Matrix4f &trajectoryInv,
            int depthWidth, int depthHeight) {
    computeVertices(depthMap, depthIntrinsics, depthWidth, depthHeight);
    std::cout << "Frame created!" << std::endl;
}

void Frame::computeVertices(const float* depthMap,
    const Eigen::Matrix3f &depthIntrinsics,
    int depthWidth, int depthHeight) {

    float fX = depthIntrinsics(0, 0);
    float fY = depthIntrinsics(1, 1);
    float cX = depthIntrinsics(0, 2);
    float cY = depthIntrinsics(1, 2);

    mVertices = std::vector<Eigen::Vector3f>(depthHeight * depthWidth);
    for (size_t i = 0; i < depthHeight; i++) {
        for (size_t j = 0; j < depthWidth; j++) {
            size_t idx = i * depthWidth + j;
            float depth = depthMap[idx];
            if (depth == MINF) {
                mVertices[idx] = Vector3f(MINF, MINF, MINF);
            } else {
                mVertices[idx] = Vector3f(
                    (j - cX) / fX * depth, (i - cY) / fY * depth, depth);
            }
        }
    }
}
