// Copyright 2020 Vladimir
// Author: Vladimir
#pragma once

#include <vector>

#include "../helpers/Eigen.h"
#include "../helpers/VirtualSensor.h"


class Frame {
 public:
    Frame();
    Frame(const float* depthMap,
        const BYTE* colorMap,
        const Eigen::Matrix3f &depthIntrinsicsInverse,
        const Eigen::Matrix4f &depthExtrinsicsInv,
        const Eigen::Matrix4f &trajectoryInv,
        int depthWidth, int depthHeight);

    Eigen::Vector3f getVertex(size_t idx) const;
    Eigen::Vector3f getNormal(size_t idx) const;
    Vector3f* getVerticesPtr() const;
    Vector3f* getNormalsPtr() const;
    int getVertexCount() const;
    int getWidth() const;
    int getHeight() const;

 private:
    void computeVertexMap(const float* depthMap,
            const Eigen::Matrix3f &depthIntrinsics,
            int depthWidth, int depthHeight);
    void computeNormalMap(int depthWidth, int depthHeight);
    std::vector<Eigen::Vector3f> mVertices;
    std::vector<Eigen::Vector3f> mNormals;
    Eigen::Matrix4f mExtrinsics;
    int mWidth;
    int mHeight;
};
