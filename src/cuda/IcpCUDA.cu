// Copyright 2020 Vladimir
// Author: Vladimir
#include <stdio.h>
#include <float.h>

#include "IcpCUDA.cuh"

__global__
void findCorrespondencesKernel(
    const Vector3f* prevV, const Vector3f* prevN,
    int prevWidth, int prevHeight,
    const Matrix4f prevExtrinsic, const Matrix3f prevIntrinsic,
    const Vector3f* curV, const Vector3f* curN,
    int curWidth, int curHeight,
    const Matrix4f curExtrinsic, const Matrix3f curIntrinsic,
    const Matrix4f estPose,
    int* corrIds) {


    int i = blockIdx.x * blockDim.x + threadIdx.x;

    if (i >= prevWidth * prevHeight)
        return;

    Matrix4f prevExtrinsicInv = prevExtrinsic.inverse();
    Matrix4f curExtrinsicInv = curExtrinsic.inverse();
    Matrix4f estPoseInv = estPose.inverse();

    // transform on the fly vertex and the normal
    Vector3f prevVG = Vector3f(MINF, MINF, MINF);
    if (prevV[i].x() != MINF && prevV[i].y() != MINF &&
        prevV[i].z() != MINF) {
        prevVG = prevExtrinsicInv.block(0, 0, 3, 3) * prevV[i] +
            prevExtrinsicInv.block(0, 3, 3, 1);
    }
    Vector3f prevNG = Vector3f(MINF, MINF, MINF);
    if (prevN[i].x() != MINF && prevN[i].y() != MINF &&
        prevN[i].z() != MINF) {
        prevNG = prevExtrinsicInv.block(0, 0, 3, 3) * prevN[i];
    }

    if (prevVG.x() != MINF && prevVG.y() != MINF && prevVG.z() != MINF &&
        prevNG.x() != MINF && prevNG.y() != MINF && prevNG.z() != MINF) {


        // if (i == 22415)
        //     printf("GPU prev v: %f  %f %f\n",
        //         prevVG.x(), prevVG.y(), prevVG.z());

        Vector3f prevVCurCam = estPoseInv.block(0, 0, 3, 3) * prevVG +
            estPoseInv.block(0, 3, 3, 1);
        Vector3f prevNCurCam = estPoseInv.block(0, 0, 3, 3) * prevNG;

        Vector3f prevVCurFrame = curExtrinsic.block(0, 0, 3, 3) * prevVCurCam +
            curExtrinsic.block(0, 3, 3, 1);

        Vector2i prevVCurImg = Vector2i(MINF, MINF);
        Vector3f projected = curIntrinsic * prevVCurFrame;
        if (projected.z() != 0) {
            prevVCurImg.x() = int(round(projected.x() / projected.z()));
            prevVCurImg.y() = int(round(projected.y() / projected.z()));
        }

        if (0 <= prevVCurImg.x() && prevVCurImg.x() < curWidth &&
            0 <= prevVCurImg.y() && prevVCurImg.y() < curHeight) {

            int idx = prevVCurImg.y() * curWidth + prevVCurImg.x();

            // if (i == 22415)
            //     printf("Index GPU: %d \n", idx);

            // transform on the fly vertex and the normal
            Vector3f curVG = Vector3f(MINF, MINF, MINF);
            if (curV[i].x() != MINF && curV[i].y() != MINF &&
                curV[i].z() != MINF) {
                curVG = curExtrinsicInv.block(0, 0, 3, 3) * curV[i] +
                    curExtrinsicInv.block(0, 3, 3, 1);
            }
            Vector3f curNG = Vector3f(MINF, MINF, MINF);
            if (curN[i].x() != MINF && curN[i].y() != MINF &&
                curN[i].z() != MINF) {
                curNG = curExtrinsicInv.block(0, 0, 3, 3) * curN[i];
            }

            // if (i == 22416)
            //     printf("GPU global: %f %f %f \n",
            //         curVG.x(), curVG.y(), curVG.z());

            Vector3f curVCurCam = estPoseInv.block(0, 0, 3, 3) * curVG +
                estPoseInv.block(0, 3, 3, 1);
            Vector3f curNCurCam = estPoseInv.block(0, 0, 3, 3) * curNG;

            // if (i == 305038) {
            //     printf("GPU cam: %f %f %f \n",
            //     curVCurCam.x(), curVCurCam.y(), curVCurCam.z());
            //     printf("GPU norm: %f \n", (curVCurCam - prevVG).norm());
            //     printf("GPU angle: %f \n", abs(curNCurCam.dot(curNG)));
            //     printf("GPU prev norm: %f %f %f \n",
            //         prevNG.x(), prevNG.y(), prevNG.z());
            //     printf("GPU cur norm: %f %f %f \n",
            //         curNCurCam.x(), curNCurCam.y(), curNCurCam.z());
            // }

            if (curVCurCam.x() != MINF && curVCurCam.y() != MINF &&
                curVCurCam.z() != MINF &&
                (curVCurCam - prevVG).norm() < 0.1f &&
                curNCurCam.x() != MINF && curNCurCam.y() != MINF &&
                curNCurCam.z() != MINF &&
                (abs(curNCurCam.dot(prevNG)) > 0.5f)) {

                // if (i == 22416) {
                //     printf("GPU pair: %d %d\n", i, idx);
                // }
                corrIds[i] = idx;
            } else {
                corrIds[i] = -1.0f;
            }
        }
    }
}


namespace CUDA {

    void findCorrespondences(
        const Vector3f* prevV, const Vector3f* prevN,
        int prevWidth, int prevHeight,
        const Matrix4f prevExtrinsic, const Matrix3f prevIntrinsic,
        const Vector3f* curV, const Vector3f* curN,
        int curWidth, int curHeight,
        const Matrix4f curExtrinsic, const Matrix3f curIntrinsic,
        const Matrix4f estPose,
        int* corrIds) {

        std::cout << "Almost there! " << std::endl;

        Vector3f *dPrevV, *dPrevN, *dCurV, *dCurN;
        int *dCorrIds;
        size_t vSize = sizeof(Vector3f);

        cudaMalloc((void**)&dPrevV, prevWidth * prevHeight * vSize);
        cudaMalloc((void**)&dPrevN, prevWidth * prevHeight * vSize);
        cudaMalloc((void**)&dCurV, curWidth * curHeight * vSize);
        cudaMalloc((void**)&dCurN, curWidth * curHeight * vSize);

        cudaMalloc((void**)&dCorrIds, prevWidth * prevHeight * vSize);

        // copy data to device
        cudaMemcpy(
            dPrevV, prevV, prevWidth * prevHeight * vSize,
            cudaMemcpyHostToDevice);
        cudaMemcpy(
            dPrevN, prevN, prevWidth * prevHeight * vSize,
            cudaMemcpyHostToDevice);
        cudaMemcpy(
            dCurV, curV, curWidth * curHeight * vSize,
            cudaMemcpyHostToDevice);
        cudaMemcpy(
            dCurN, curN, curWidth * curHeight * vSize,
            cudaMemcpyHostToDevice);

        int block_size = 256;
        int grid_size = ((
            prevWidth * prevHeight + block_size) / block_size);

        findCorrespondencesKernel<<<grid_size, block_size>>>(
            dPrevV, dPrevN,
            prevWidth, prevHeight,
            prevExtrinsic, prevIntrinsic,
            dCurV, dCurN,
            curWidth, curHeight,
            curExtrinsic, curIntrinsic,
            estPose,
            dCorrIds);

        cudaMemcpy(
            corrIds, dCorrIds, prevWidth * prevHeight * sizeof(int),
            cudaMemcpyDeviceToHost);

        // printf("After CUDA: %d \n", corrIds[22416]);

        cudaFree(dPrevV);
        cudaFree(dPrevN);
        cudaFree(dCurV);
        cudaFree(dCurN);
        cudaFree(dCorrIds);
    }
}  // namespace CUDA