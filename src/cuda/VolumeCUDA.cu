// Copyright 2020 Vladimir
// Author: Vladimir
#include <stdio.h>

#include "VolumeCUDA.cuh"

__global__
void integrateKernel(
    Vector3f min, Vector3f max,
    uint dx, uint dy, uint dz,
    Matrix4f extrinsic, Matrix3f intrinsic,
    int frameWidth, int frameHeight,
    float* depthMap,
    float* values, float* weights) {

    int x = blockIdx.x;
    int y = blockIdx.y;
    int z = blockIdx.z;

    if (x >= dx || y >= dy || z >= dz)
        return;


    // grid point into image space
    Vector3f pG = Vector3f(
        min.x() + (max.x() - min.x()) * (float(x) * (1.0f / (dx - 1))),
        min.y() + (max.y() - min.y()) * (float(y) * (1.0f / (dy - 1))),
        min.z() + (max.z() - min.z()) * (float(z) * (1.0f / (dz - 1))));

    // project into camera space
    Vector3f pC = extrinsic.block(0, 0, 3, 3) * pG +
        extrinsic.block(0, 3, 3, 1);

    // project into image space
    Vector2i pI = Vector2i(MINF, MINF);
    Vector3f projectedPoint = intrinsic * pC;
    if (projectedPoint.z() != 0) {
        pI.x() = (int)round(projectedPoint.x() / projectedPoint.z());
        pI.y() = (int)round(projectedPoint.y() / projectedPoint.z());
    }

    // printf("Pixels on GPU %d, %d \n", pI.x(), pI.y());
    if (0 <= pI.x() && pI.x() < frameWidth &&
        0 <= pI.y() && pI.y() < frameHeight) {
        uint idx = pI.y() * frameWidth + pI.x();
        printf("Index inside: %d", idx);
        float depth = depthMap[idx];
        // if (depth == MINF)
        //     printf("Depth is MINF in CUDA");
        //     return;
        // float lambda = (pC / pC.z()).norm();
    }

    // printf("CUDA CALL COMPLETED\n");
}


namespace CUDA {

    void integrate(
        Vector3f min, Vector3f max,
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

        integrateKernel<<<dx, dy, 1>>>(
            min, max,
            dx, dy, dz,
            extrinsic, intrinsic,
            frameWidth, frameHeight,
            dDepthMap,
            dValues, dWeights);

        cudaFree(dDepthMap);
        cudaFree(dValues);
        cudaFree(dWeights);

    }
}  // namespace CUDA