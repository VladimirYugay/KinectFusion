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
void testFindCorr(
    Vector3f* prevV, Vector3f* prevN,
    int prevSize, int prevWidth, int prevHeight,
    Vector3f* curV, Vector3f* curN,
    int curSize, int curWidth, int curHeight,
    Matrix4f extrinsics, Matrix4f pose,
    int* corr) {

        printf("CUDA \n");
    }

__global__
void cu_dot(Eigen::Vector3f *v1, Eigen::Vector3f *v2, double *out, size_t N) {

    for (int i = 0; i < N; i++) {
        out[i] = v1[i].dot(v2[i]);
    }
    return;
}

namespace CUDA {

    int* testFindCorrespondences(
        const Vector3f* prevV, const Vector3f* prevN,
        int prevSize, int prevWidth, int prevHeight,
        const Vector3f* curV, const Vector3f* curN,
        int curSize, int curWidth, int curHeight,
        const Matrix4f extrinsics, const Matrix4f pose) {

        Vector3f* dPrevV;
        Vector3f* dPrevN;
        Vector3f* dCurN;
        Vector3f* dCurV;
        size_t vSize = sizeof(Vector3f);

        cudaMalloc((void**)&dPrevV, vSize * prevSize);
        cudaMalloc((void**)&dPrevN, vSize * prevSize);
        cudaMalloc((void**)&dCurV, vSize * curSize);
        cudaMalloc((void**)&dCurN, vSize * curSize);

        cudaMemcpy(dPrevV, prevV, vSize * prevSize, cudaMemcpyHostToDevice);
        cudaMemcpy(dPrevN, prevN, vSize * prevSize, cudaMemcpyHostToDevice);
        cudaMemcpy(dCurV, curV, vSize * curSize, cudaMemcpyHostToDevice);
        cudaMemcpy(dCurN, curN, vSize * curSize, cudaMemcpyHostToDevice);

        int corrSize = 2 * std::max(prevSize, curSize);
        int* corr = new int[corrSize];
        int* dCorr;
        cudaMalloc((void**)&dCorr, corrSize * sizeof(int));

        testFindCorr<<<1, 1>>>(
            dPrevV, dPrevN,
            prevSize, prevWidth, prevHeight,
            dCurV, dCurN,
            curSize, curWidth, curHeight,
            extrinsics, pose,
            dCorr);

        cudaMemcpy(corr, dCorr, corrSize * sizeof(int), cudaMemcpyDeviceToHost);

        // int N = 1;
        // Eigen::Vector3f *a = new Eigen::Vector3f[N];
        // Eigen::Vector3f *b = new Eigen::Vector3f[N];
        // double *c = new double[N];

        // for (int i = 0; i < N; i++) {
        //     a[i] = Vector3f(1, 1, 1);
        //     b[i] = Vector3f(2, 2, 2);
        //     c[i] = 0.0;
        // }

        // Eigen::Vector3f *da;
        // Eigen::Vector3f *db;
        // double *dc;

        // cudaMalloc((void**)&da, sizeof(Eigen::Vector3f)*N);
        // cudaMalloc((void**)&db, sizeof(Eigen::Vector3f)*N);
        // cudaMalloc((void**)&dc, sizeof(double)*N);

        // cudaMemcpy(da, a, sizeof(Eigen::Vector3f)*N, cudaMemcpyHostToDevice);
        // cudaMemcpy(db, b, sizeof(Eigen::Vector3f)*N, cudaMemcpyHostToDevice);
        // cudaMemcpy(dc, c, sizeof(double)*N, cudaMemcpyHostToDevice);

        // cu_dot<<<1, 1>>>(da, db, dc, N);

        // cudaMemcpy(c, dc, sizeof(double)*N, cudaMemcpyDeviceToHost);
        // std::cout << "After CUDA: " << c[0] << std::endl;


        return 0;
    }

    // int* findCorrespondences(
    //     const Frame &prevFrame,
    //     const Frame &curFrame,
    //     const Matrix4f &extrinsics,
    //     const Matrix4f &pose) {

    //     Vector3f* dPrevV;
    //     Vector3f* dPrevN;
    //     Vector3f* dCurV;
    //     Vector3f* dCurN;
    //     Matrix4f* dExtrinsics;
    //     Matrix4f* dPose;
    //     size_t vSize = sizeof(Vector3f);

    //     Vector3f* rawN = prevFrame.getVertices().data();
    //     // for (int i = 0; i < prevFrame.getVertexCount(); i++) {
    //     //     printf("Almost there: %d \n", rawN[i](0));
    //     // }
    //     // printf("Almost Inside CUDA: %d\n", rawN[302990](0));
    //     // printf("Almost Inside CUDA: %d\n", rawN[302990]);
    //     // printf("Almost Inside CUDA: %d\n", rawN[302990]);

    //     cudaMalloc(&dPrevN, vSize * prevFrame.getVertexCount());
    //     cudaMalloc(&dCurN, vSize * curFrame.getVertexCount());
    //     cudaMalloc(&dCurV, vSize * curFrame.getVertexCount());
    //     cudaMalloc(&dExtrinsics, sizeof(Matrix4f));
    //     cudaMalloc(&dPose, sizeof(Matrix4f));

    //     cudaMemcpy(
    //         dPrevV,
    //         prevFrame.getVertices().data(),
    //         vSize * prevFrame.getVertexCount(), cudaMemcpyHostToDevice);

    //     cudaMemcpy(
    //         dPrevN,
    //         prevFrame.getNormals().data(),
    //         vSize * prevFrame.getVertexCount(), cudaMemcpyHostToDevice);

    //     cudaMemcpy(
    //         dCurV,
    //         prevFrame.getVertices().data(),
    //         vSize * curFrame.getVertexCount(), cudaMemcpyHostToDevice);

    //     cudaMemcpy(
    //         dCurN,
    //         prevFrame.getVertices().data(),
    //         vSize * curFrame.getVertexCount(), cudaMemcpyHostToDevice);

    //     cudaMemcpy(
    //         dExtrinsics,
    //         extrinsics.data(),
    //         sizeof(Matrix4f),
    //         cudaMemcpyHostToDevice);

    //     cudaMemcpy(
    //         dPose,
    //         pose.data(),
    //         sizeof(Matrix4f),
    //         cudaMemcpyHostToDevice);

    //     // findCorr<<<1, 1>>>(
    //     //     dPrevV, dPrevN, prevFrame.getVertexCount(),
    //     //     dCurV, dCurN, curFrame.getVertexCount(),
    //     //     dExtrinsics, dPose);


    //     cudaFree(dPrevV);
    //     cudaFree(dPrevN);
    //     cudaFree(dCurV);
    //     cudaFree(dCurN);
    //     cudaFree(dExtrinsics);
    //     return 0;

    // }

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
