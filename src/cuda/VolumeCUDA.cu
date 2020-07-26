// Copyright 2020 Vladimir
// Author: Vladimir
#include <stdio.h>
#include <float.h>

#include "VolumeCUDA.cuh"

__global__
void integrateKernel(
    Vector3f min, Vector3f max,
    uint dx, uint dy, uint dz,
    Matrix4f extrinsic, Matrix3f intrinsic,
    int frameWidth, int frameHeight,
    float* depthMap, Vector3f* normals,
    float* values, float* weights) {

    const int x = blockIdx.x * blockDim.x + threadIdx.x;
    const int y = blockIdx.y * blockDim.y + threadIdx.y;

    if (x >= dx || y >= dy)
        return;

    // grid point into image space
    for (int z = 0; z < dz; z++) {

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
        if (projectedPoint.z() != 0.0f) {
            pI.x() = (int)round(projectedPoint.x() / projectedPoint.z());
            pI.y() = (int)round(projectedPoint.y() / projectedPoint.z());
        }

        // if (x == 246 && y == 244 && z == 0)
        //     printf("Pixels on GPU %d, %d \n", pI.x(), pI.y());

        if (0 <= pI.x() && pI.x() < frameWidth &&
            0 <= pI.y() && pI.y() < frameHeight) {
            uint idx = pI.y() * frameWidth + pI.x();
            float depth = depthMap[idx];
            if (depth == MINF)
                continue;

            // if (x == 247 && y == 244 && z == 0)
            //     printf("Depth on GPU %f \n", depth);

            float lambda = (pC / pC.z()).norm();
            float sdf = depth - (
                (pG - extrinsic.block(0, 3, 3, 1)) / lambda).norm();

            Vector3f ray = pG.normalized();
            Vector3f normal = normals[idx];
            float cosAngle = ray.dot(normal) / ray.norm() / normal.norm();
            float tsdfWeight = 1.0f;

            uint prevIdx = x * dy * dz + y * dz + z;
            float prevVal = values[prevIdx];
            float prevWeight = weights[prevIdx];

            if (prevVal == FLT_MAX) {
                prevVal = 0;
                prevWeight = 0;
            }

            float maxTruncation = 1.0f;
            float minTruncation = 1.0f;
            float tsdf =
                (sdf / minTruncation > -1.0f) ? sdf / minTruncation : -1.0f;
            if (sdf > 0) {
                tsdf =
                    (sdf / maxTruncation < 1.0f) ? sdf / maxTruncation: 1.0f;
            }

            values[prevIdx] = prevVal * prevWeight + tsdf * tsdfWeight;
            weights[prevIdx] = prevWeight + tsdfWeight;

            // if (x == 247 && y == 244 && z == 0)
            //     printf("Val on GPU %f \n", values[prevIdx]);
        }
    }

    // printf("CUDA CALL COMPLETED\n");
}


namespace CUDA {

    void integrate(
        Vector3f min, Vector3f max,
        uint dx, uint dy, uint dz,
        Matrix4f extrinsic, Matrix3f intrinsic,
        int frameWidth, int frameHeight,
        const float* depthMap, const Vector3f* normals,
        float* values, float* weights) {

        std::cout << "Almost there! " << std::endl;

        // setup device variables and allocate memory for them
        float *dDepthMap, *dValues, *dWeights;
        Vector3f* dNormals;
        cudaMalloc(
            (void**)&dDepthMap, frameWidth * frameHeight * sizeof(float));
        cudaMalloc(
            (void**)&dNormals, frameWidth * frameHeight * sizeof(Vector3f));
        cudaMalloc((void**)&dValues, dx * dy * dz * sizeof(float));
        cudaMalloc((void**)&dWeights, dx * dy * dz * sizeof(float));

        // copy data to device
        cudaMemcpy(
            dDepthMap, depthMap, frameWidth * frameHeight * sizeof(float),
            cudaMemcpyHostToDevice);
        cudaMemcpy(
            dNormals, normals, frameWidth * frameHeight * sizeof(Vector3f),
            cudaMemcpyHostToDevice);
        cudaMemcpy(
            dValues, values, dx * dy * dz * sizeof(float),
            cudaMemcpyHostToDevice);
        cudaMemcpy(
            dWeights, weights, dx * dy * dz * sizeof(float),
            cudaMemcpyHostToDevice);


        const dim3 threads(32, 32);
        const dim3 blocks(
            (dx + threads.x - 1) / threads.x,
            (dy + threads.y - 1) / threads.y);

        integrateKernel<<<blocks, threads>>>(
            min, max,
            dx, dy, dz,
            extrinsic, intrinsic,
            frameWidth, frameHeight,
            dDepthMap, dNormals,
            dValues, dWeights);

        cudaDeviceSynchronize();

        cudaMemcpy(
            values, dValues, dx * dy * dz * sizeof(float),
            cudaMemcpyDeviceToHost);
        cudaMemcpy(
            weights, dWeights, dx * dy * dz * sizeof(float),
            cudaMemcpyDeviceToHost);

        // printf("Val after GP:  %f \n", values[64874496]);

        cudaFree(dDepthMap);
        cudaFree(dValues);
        cudaFree(dWeights);
        cudaFree(dNormals);

    }
}  // namespace CUDA