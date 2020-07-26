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
    Matrix4f curExtrinsics, Matrix3f curIntrinsics,
    Matrix4f pose, int* corr) {

        size_t counter = 0;
        int tmp = 0;
        Matrix3f curR = curExtrinsics.block(0, 0, 3, 3);
        Vector3f curT = curExtrinsics.block(0, 3, 3, 1);
        Matrix3f poseR = pose.block(0, 0, 3, 3);
        for (int i = 0; i < prevSize; i++) {
            Vector3f pV = prevV[i];
            Vector3f pN = prevN[i];
            if (pV.x() && pV.y() != MINF && pV.z() != MINF &&
                pN.x() != MINF && pN.y() != MINF && pN.z() != MINF) {
                Vector3f pVGlobal = curR * pV + curT;
                Vector3f pNGlobal = poseR * pN;
                Vector3f pVCurrent = curR * pVGlobal + curT;
                Vector3f pVCurrentImg = curIntrinsics * pVCurrent;
                Vector2i pVCurrentPx = Vector2i(MINF, MINF);
                if (pVCurrentImg.z() != 0) {
                    pVCurrentPx.x() = pVCurrentImg.x() / pVCurrentImg.z();
                    pVCurrentPx.y() = pVCurrentImg.y() / pVCurrentImg.z();
                }
                if (0 <= pVCurrentPx.x() < curWidth &&
                    0 <= pVCurrentPx.y() < curHeight &&
                    pVCurrent.x() != MINF && pVCurrent.y() != MINF &&
                    pVCurrent.z() != MINF) {
                    size_t idx = pVCurrentPx.x() * curWidth + pVCurrentPx.y();
                    // tmp++;
                    // Move pts of the current fram to the global coord sys
                    if (idx >= curWidth * curHeight) {
                        continue;
                    }
                    Vector3f cV = curV[idx];
                    Vector3f cN = curN[idx];

                    tmp++;
                    // if (cV.x() != MINF && cV.y() != MINF && cV.z() != MINF &&
                    //     cN.x() != MINF && cN.y() != MINF && cN.z() != MINF) {
                    //     // cV = curR * cV + curT;
                    //     // cN = curR * cN;
                    //     tmp++;
                    // }
                    // else {
                    //     cV = Vector3f(MINF, MINF, MINF);
                    //     cN = Vector3f(MINF, MINF, MINF);
                    // }
                }
            }
        }
        printf("Transposed points: %d", tmp);

}

__global__
void cu_dot(Eigen::Vector3f *v1, Eigen::Vector3f *v2, double *out, size_t N) {

    int counter = 0;
    for (int i = 0; i < N; i++) {
        Vector3f v = v1[i];
        Vector3f u = v2[i];
        if (v.x() != MINF && v.y() != MINF && v.z() != MINF &&
            u.x() != MINF && u.y() != MINF && u.z() != MINF) {
            counter++;
        }
    }
    printf("Non zero pts: %d", counter);
    return;
}

namespace CUDA {

    int* testFindCorrespondences(
        const Vector3f* prevV, const Vector3f* prevN,
        int prevSize, int prevWidth, int prevHeight,
        const Vector3f* curV, const Vector3f* curN,
        int curSize, int curWidth, int curHeight,
        const Matrix4f extrinsics, const Matrix3f intrinsics,
        const Matrix4f pose) {

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
            extrinsics, intrinsics,
            pose, dCorr);

        cudaMemcpy(corr, dCorr, corrSize * sizeof(int), cudaMemcpyDeviceToHost);

        for (int i = 0; i < 10; i++) {
            std::cout << corr[i] << " " << std::endl;
        }
        std::cout << curWidth << std::endl;

        cudaFree(dPrevV);
        cudaFree(dPrevN);
        cudaFree(dCurV);
        cudaFree(dCurN);

        // int N = 1;
        // Eigen::Vector3f *a = new Eigen::Vector3f[N];
        // Eigen::Vector3f *b = new Eigen::Vector3f[N];
        // double *c = new double[N];

        // for (int i = 0; i < N; i++) {
        //     a[i] = Vector3f(MINF, MINF, 1);
        //     b[i] = Vector3f(MINF, 2, 2);
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


        // cu_dot<<<1, 1>>>(dPrevV, dCurV, dc, prevSize);

        // cudaMemcpy(c, dc, sizeof(double)*N, cudaMemcpyDeviceToHost);
        // std::cout << "After CUDA: " << c[0] << std::endl;


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
