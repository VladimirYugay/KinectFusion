// Copyright 2020 Vladimir
// Author: Vladimir
#pragma once

#include <vector>

#include "../helpers/Eigen.h"
#include "../helpers/VirtualSensor.h"


class Frame {
 public:
    Frame(const float* depthMap,
        const BYTE* colorMap,
        const Eigen::Matrix3f &depthIntrinsicsInverse,
        const Eigen::Matrix4f &depthExtrinsicsInv,
        const Eigen::Matrix4f &trajectoryInv,
        int depthWidth, int depthHeight);
 private:
/**
 * Computes vertices in camera space
*/
void computeVertexMap(const float* depthMap,
        const Eigen::Matrix3f &depthIntrinsics,
        int depthWidth, int depthHeight);
/**
 * Computes normals
*/        
void computeNormalMap(int depthWidth, int depthHeight);

std::vector<Eigen::Vector3f> mVertices;
std::vector<Eigen::Vector3f> mNormals;
void computePixel2Camera();
};
