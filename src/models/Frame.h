#pragma once

#include <vector>

#include "../helpers/Eigen.h"
#include "../helpers/VirtualSensor.h"

#include "Vertex.h"

class Frame {
public:
    Frame(const float* depthMap, 
        const BYTE* colorMap, 
        const Eigen::Matrix3f &depthIntrinsicsInverse, 
        const Eigen::Matrix4f &depthExtrinsicsInv, 
        const Eigen::Matrix4f &trajectoryInv,
        int depthWidth, int depthHeight);
private: 
Vertex* mVertices;
std::vector<Eigen::Vector3d> mNormals;
void computePixel2Camera();
};