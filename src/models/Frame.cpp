// Copyright 2020 Vladimir
// Author: Vladimir
#include "Frame.h"

#include <iostream>


Frame::Frame() {}

Frame::Frame(const float* depthMap,
            const BYTE* colorMap,
            const Eigen::Matrix3f &depthIntrinsics,
            const Eigen::Matrix4f &depthExtrinsics,
            const Eigen::Matrix4f &trajectoryInv,
            int depthWidth, int depthHeight) {
    computeVertexMap(depthMap, depthIntrinsics, depthWidth, depthHeight);
    computeNormalMap(depthWidth, depthHeight);
    mExtrinsics = depthExtrinsics;
    mWidth = depthWidth;
    mHeight = depthHeight;
    std::cout << "Frame created!" << std::endl;
}

Vector3f* Frame::getVerticesPtr() const {
    return (Vector3f*)&mVertices[0];
}

Vector3f* Frame::getNormalsPtr() const {
    return (Vector3f*)&mNormals[0];
}

Eigen::Vector3f Frame::getVertex(size_t idx) const {
    return mVertices[idx];
}

Eigen::Vector3f Frame::getNormal(size_t idx) const {
    return mNormals[idx];
}

int Frame::getVertexCount() const {
    return mVertices.size();
}

int Frame::getWidth() const {
    return mWidth;
}

int Frame::getHeight() const {
    return mHeight;
}


void Frame::computeVertexMap(const float* depthMap,
    const Eigen::Matrix3f &depthIntrinsics,
    int depthWidth, int depthHeight) {

    float fX = depthIntrinsics(0, 0);
    float fY = depthIntrinsics(1, 1);
    float cX = depthIntrinsics(0, 2);
    float cY = depthIntrinsics(1, 2);

    mVertices = std::vector<Eigen::Vector3f>(
        depthHeight * depthWidth, Vector3f(MINF, MINF, MINF));
    for (size_t i = 0; i < depthHeight; i++) {
        for (size_t j = 0; j < depthWidth; j++) {
            size_t idx = i * depthWidth + j;
            float depth = depthMap[idx];
            if (depth != MINF) {
                mVertices[idx] = Vector3f(
                    (j - cX) / fX * depth, (i - cY) / fY * depth, depth);
            }
        }
    }
}

void Frame::computeNormalMap(int depthWidth, int depthHeight) {
    mNormals = std::vector<Eigen::Vector3f>(
        depthHeight * depthWidth, Vector3f(MINF, MINF, MINF));

    for (size_t i = 1; i < depthHeight - 1; i++) {
        for (size_t j = 1; j < depthWidth - 1; j++) {
            size_t idx = i * depthWidth + j;
            Eigen::Vector3f du = mVertices[idx + 1] - mVertices[idx - 1];
            Eigen::Vector3f dv = mVertices[
                idx + depthWidth] - mVertices[idx - depthWidth];
            if (du.allFinite() && dv.allFinite()) {
                mNormals[idx] = du.cross(dv);
                mNormals[idx].normalize();
            }
        }
    }
}
