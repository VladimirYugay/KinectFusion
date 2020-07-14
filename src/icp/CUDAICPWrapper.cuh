#pragma once
#include "cuda_runtime.h"
#include "device_launch_parameters.h"

#include <stdio.h>


namespace CUDA {
	void createEquations(
        const float* sourcePoints,
        const float* targetPoints,
        const float* targetNormals,
        int pointsNum,
        float* A,
        float* b);
}