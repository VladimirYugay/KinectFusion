#include "CUDAICPWrapper.cuh"

__global__ void fillSystem(
    float *sourcePts, float* targetPts, float *targetNormals,
    int n, float *A, float*b) {

    
    printf("AXAXAXAXAXA");
}

namespace CUDA {
	void createEquations(
        const float *sourcePts,
        const float *targetPts,
        const float *targetNrmls,
        int n,
        float *A,
        float *b) {


        float *dSourcePts, *dTargetPts, *dTargetNrmls;
        float *dA, *db;
        size_t size = sizeof(float);

        // allocate memory on the device
        cudaMalloc((void**)&dSourcePts, size * n);
        cudaMalloc((void**)&dTargetPts, size * n);
        cudaMalloc((void**)&dTargetNrmls, size * n);
        cudaMalloc((void**)&dA, size * n * 6);
        cudaMalloc((void**)&db, size * n);

        // transfer data to the device
        cudaMemcpy(
            dSourcePts, sourcePts, size * n, cudaMemcpyHostToDevice);
        cudaMemcpy(
            dTargetPts, targetPts, size * n, cudaMemcpyHostToDevice);
        cudaMemcpy(
            dTargetNrmls, targetNrmls, size * n, cudaMemcpyHostToDevice);
        cudaMemcpy(dA, A, size * n * 6, cudaMemcpyHostToDevice);
        cudaMemcpy(db, b, size * n, cudaMemcpyHostToDevice);

        // execute the kernel
        fillSystem<<<1, 1>>>(
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
}
