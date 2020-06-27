#include "Frame.h"

#include <iostream>


Frame::Frame(const float* depthMap, 
            const BYTE* colorMap, 
            const Eigen::Matrix3f &depthIntrinsicsInverse, 
            const Eigen::Matrix4f &depthExtrinsicsInv, 
            const Eigen::Matrix4f &trajectoryInv,
            int depthWidth, int depthHeight){
    
    mVertices = new Vertex[depthWidth * depthHeight];
    MatrixXf identity = MatrixXf::Identity(4, 3);
    for (size_t i = 0; i < depthHeight; i++) {
        for (size_t j = 0; j < depthWidth; j++){
            size_t idx = i * depthWidth + j;
            if (depthMap[idx] == MINF){
                mVertices[idx].position = Vector4f(MINF, MINF, MINF, MINF);
                mVertices[idx].color = Vector4uc(0, 0, 0, 0);
            }else {
                Vector3f pImage(j * depthMap[idx], i * depthMap[idx], depthMap[idx]);
                Vector4f pCamera = identity * depthIntrinsicsInverse * pImage;
                Vector4f pWorld = trajectoryInv * depthExtrinsicsInv * pCamera;
                mVertices[idx].position = pWorld;
                mVertices[idx].color = Vector4uc(
                    colorMap[4 * idx + 0], colorMap[4 * idx + 1],
                    colorMap[4 * idx + 2], colorMap[4 * idx + 3]);	
            }
        }
    }



    std::cout << "Frame created!" << std::endl;
}