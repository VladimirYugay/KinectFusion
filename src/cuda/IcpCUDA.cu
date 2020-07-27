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
    int* leftIds, int* rightIds) {


    int i = blockIdx.x * blockDim.x + threadIdx.x;

    if (i >= prevWidth * prevHeight)
        return;

    Matrix4f prevExtrinsicInv = prevExtrinsic.inv();
    Matrix4f curExtrinsicInv = curExtrinsic.inv();
    Matrix4f estPoseInv = estPose.inv();

    // transform on the fly vertex and the normal
    Vector3f prevVG = Vector3f(MINF, MINF, MINF);
    if (prevV[i].x() != MINF && prevV[i].y() != MINF &&
        prevV[i].z() != MINF) {
        prevVG = prevExtrinsicInv.block(0, 0, 3, 3) * prevV[i] +
            prevExtrinsicInv(0, 3, 3, 1);
    }
    Vector3f prevNG = Vector3f(MINF, MINF, MINF);
    if (prevN[i].x() != MINF && prevN[i].y() != MINF &&
        prevN[i].z() != MINF) {
        prevNG = prevExtrinsicInv.block(0, 0, 3, 3) * prevN[i];
    }

    if (prevVG.x() != MINF && prevVG.y() != MINF && prevVG.z() != MINF &&
        prevNG.x() != MINF && prevNG.y() != MINF && prevNG.z() != MINF) {

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

            // transform on the fly vertex and the normal
            Vector3f curVG = Vector3f(MINF, MINF, MINF);
            if (curVG[i].x() != MINF && curVG[i].y() != MINF &&
                curVG[i].z() != MINF) {
                curVG = curExtrinsicInv.block(0, 0, 3, 3) * prevV[i] +
                    curExtrinsicInv(0, 3, 3, 1);
            }
            Vector3f curNG = Vector3f(MINF, MINF, MINF);
            if (curNG[i].x() != MINF && curNG[i].y() != MINF &&
                curNG[i].z() != MINF) {
                curNG = curExtrinsicInv.block(0, 0, 3, 3) * prevN[i];
            }

            Vector3f curVCurCam = estPoseInv.block(0, 0, 3, 3) * curVG +
                estPoseInv.block(0, 3, 3, 1);
            Vector3f curNCurCam = estPoseInv.block(0, 0, 3, 3) * curNG;

            if (curVCurCam.x() != MINF && curVCurCam.y() != MINF &&
                curVCurCam.z() != MINF &&
                (curVCurCam - prevVG).norm() < 0.1f &&
                curNCurCam.x() != MINF && curNCurCam.y() != MINF &&
                curNCurCam.z() != MINF &&
                (abs(curNCurCam.dot(prevNG)) < 0.1f)) {

                leftIds[i] = i;
                rightIds[i] = idx;
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
        size_t vSize = sizeof(Vector3f);

        cudaMalloc((void**)&dPrevV, prevWidth * prevHeight * vSize);
        cudaMalloc((void**)&dPrevN, prevWidth * prevHeight * vSize);
        cudaMalloc((void**)&dCurV, curWidth * curHeight * vSize);
        cudaMalloc((void**)&dCurN, curWidth * curHeight * vSize);


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

        cudaFree(dPrevV);
        cudaFree(dPrevN);
        cudaFree(dCurV);
        cudaFree(dCurN);
    }
}  // namespace CUDA