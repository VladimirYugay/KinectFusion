// Copyright 2020 Vladimir
// Author: Vladimir
#include <stdio.h>

#include "VolumeCUDA.cuh"

__global__
void integrateKernel(
    Vector3f bottomLeft, Vector3f topRight,
    uint dx, uint dy, uint dz,
    Matrix4f extrinsic, Matrix3f intrinsic,
    int frameWidth, int frameHeight,
    float* depthMap,
    float* values, float* weights) {

    printf("Inside CUDA babe \n");
}


namespace CUDA {

    void integrate(
        Vector3f bottomLeft, Vector3f topRight,
        uint dx, uint dy, uint dz,
        Matrix4f extrinsic, Matrix3f intrinsic,
        int frameWidth, int frameHeight,
        const float* depthMap,
        float* values, float* weights) {

        std::cout << "Almost there! " << std::endl;

        // setup device variables and allocate memory for them
        float *dDepthMap, *dValues, *dWeights;
        cudaMalloc(
            (void**)&dDepthMap, frameWidth * frameHeight * sizeof(float));
        cudaMalloc((void**)&dValues, dx * dy * dz * sizeof(float));
        cudaMalloc((void**)&dWeights, dx * dy * dz * sizeof(float));

        // copy data to device
        cudaMemcpy(
            dDepthMap, depthMap, frameWidth * frameHeight * sizeof(float),
            cudaMemcpyHostToDevice);
        cudaMemcpy(
            dValues, values, dx * dy * dz * sizeof(float),
            cudaMemcpyHostToDevice);
        cudaMemcpy(
            dWeights, weights, dx * dy * dz * sizeof(float),
            cudaMemcpyHostToDevice);

        cudaFree(dDepthMap);
        cudaFree(dValues);
        cudaFree(dWeights);

    }
}  // namespace CUDA