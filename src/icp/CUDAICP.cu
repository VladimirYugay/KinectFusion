// Copyright 2020 Vladimir
// Author: Vladimir
#include <stdio.h>

#include "CUDAICPWrapper.cuh"

__global__
void fillSystem(
    float *x, float* y, float *n, int ptsNum, float *A, float*b) {
    int i = blockIdx.x * blockDim.x + threadIdx.x;
    if (i < ptsNum) {
        int id = 3 * i;
        A[6 * i] = n[id + 2] * x[id + 1] - n[id + 1] * x[id + 2];
        A[6 * i + 1] = n[id] * x[id + 2] - n[id + 2] * x[id];
        A[6 * i + 2] = n[id + 1] * x[id] - n[id] * x[id + 1];
        A[6 * i + 3] = n[id];
        A[6 * i + 4] = n[id + 1];
        A[6 * i + 5] = n[id + 2];
        b[i] = n[id] * y[id] + n[id + 1] * y[id + 1] +
            n[id + 2] * y[id + 2] - n[id] * x[id] -
            n[id + 1] * x[id + 1] - n[id + 2] * x[id + 2];
    }
}

__global__
void findCorr(Vector3f* prevV, int prevSize) {

    printf("VERTICES FROM CUDA");
}

namespace CUDA {

    int* findCorrespondences(
        const Frame &prevFrame,
        const Frame &curFrame,
        const Matrix4f &estimatedPose) {

        Vector3f* dPrevV;
        Vector3f* dPrevN;
        Vector3f* dCurV;
        Vector3f* dCurN;
        Matrix4f* dExtrinsics;
        Matrix3f* dRotation;
        size_t vSize = sizeof(Vector3f);

        cudaMalloc(&dPrevV, vSize * prevFrame.getVertexCount());
        cudaMalloc(&dPrevN, vSize * prevFrame.getVertexCount());
        cudaMalloc(&dCurN, vSize * curFrame.getVertexCount());
        cudaMalloc(&dCurV, vSize * curFrame.getVertexCount());
        cudaMalloc(&dExtrinsics, sizeof(Matrix4f));
        cudaMalloc(&dRotation, sizeof(Matrix3f));

        cudaMemcpy(
            dPrevV,
            prevFrame.getVertices().data(),
            vSize * prevFrame.getVertexCount(), cudaMemcpyHostToDevice);

        cudaMemcpy(
            dPrevN,
            prevFrame.getNormals().data(),
            vSize * prevFrame.getVertexCount(), cudaMemcpyHostToDevice);

        cudaMemcpy(
            dCurV,
            prevFrame.getVertices().data(),
            vSize * curFrame.getVertexCount(), cudaMemcpyHostToDevice);

        cudaMemcpy(
            dCurN,
            prevFrame.getVertices().data(),
            vSize * curFrame.getVertexCount(), cudaMemcpyHostToDevice);

        findCorr<<<1, 256>>>(
            dPrevV, prevFrame.getVertexCount());


        cudaFree(dPrevV);
        cudaFree(dPrevN);
        cudaFree(dCurV);
        cudaFree(dCurN);
        cudaFree(dExtrinsics);
        cudaFree(dRotation);
        return 0;

    }

    void createEquations(
        const float *sourcePts,
        const float *targetPts,
        const float *targetNrmls,
        int n, float *A, float *b) {

        float *dSourcePts, *dTargetPts, *dTargetNrmls;
        float *dA, *db;
        size_t size = sizeof(float);

        // allocate memory on the device
        cudaMalloc(&dSourcePts, 3 * size * n);
        cudaMalloc(&dTargetPts, 3 * size * n);
        cudaMalloc(&dTargetNrmls, 3 * size * n);
        cudaMalloc(&dA, 6 * size * n);
        cudaMalloc(&db, size * n);

        // transfer data to the device
        cudaMemcpy(
            dSourcePts, sourcePts, 3 * size * n, cudaMemcpyHostToDevice);
        cudaMemcpy(
            dTargetPts, targetPts, 3 * size * n, cudaMemcpyHostToDevice);
        cudaMemcpy(
            dTargetNrmls, targetNrmls, 3 * size * n, cudaMemcpyHostToDevice);

        // execute the kernel
        int block_size = 256;
        int grid_size = ((n + block_size) / block_size);
        fillSystem<<<block_size, grid_size>>>(
            dSourcePts, dTargetPts, dTargetNrmls, n, dA, db);

        // transfer data back
        cudaMemcpy(A, dA, size * n * 6, cudaMemcpyDeviceToHost);
        cudaMemcpy(b, db, size * n, cudaMemcpyDeviceToHost);

        // deallocate the device
        cudaFree(dSourcePts);
        cudaFree(dTargetPts);
        cudaFree(dTargetNrmls);
        cudaFree(dA);
        cudaFree(db);
    }
}  // namespace CUDA
