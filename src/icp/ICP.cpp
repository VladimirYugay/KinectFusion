// Copyright 2020 Vladimir
// Author: Vladimir
#include <iostream>

#include "ICP.h"


Matrix4f ICP::estimatePose(
    const std::vector<Vector3f>& sourcePoints,
    const std::vector<Vector3f>& targetPoints,
    const std::vector<Vector3f>& targetNormals,
    int iterationsNums) {
    std::cout << "Estimating the pose!" << std::endl;
}
