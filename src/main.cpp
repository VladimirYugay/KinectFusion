// Copyright 2020 Vladimir
// Author: Vladimir
#include <iostream>
#include <fstream>
#include <array>

#include "models/Frame.h"
#include "icp/ICP.h"

int main() {
    // Make sure this path points to the data folder
    std::string filenameIn = "../data/";

    // load video
    std::cout << "Initialize virtual sensor..." << std::endl;
    VirtualSensor sensor;
    if (!sensor.Init(filenameIn)) {
        std::cout << "Failed to initialize the sensor!" << std::endl;
        return -1;
    }

    int maxFrameNum = 2;
    int frameCount = 0;
    while (frameCount < maxFrameNum && sensor.ProcessNextFrame()) {
        float* depthMap = sensor.GetDepth();
        BYTE* colorMap = sensor.GetColorRGBX();
        Matrix3f depthIntrinsics = sensor.GetDepthIntrinsics();
        Matrix4f depthExtrinsicsInv = sensor.GetDepthExtrinsics().inverse();
        Matrix4f trajectory = sensor.GetTrajectory();
        Matrix4f trajectoryInv = sensor.GetTrajectory().inverse();
        Matrix3f depthIntrinsicsInv = depthIntrinsics.inverse();
        int depthHeight = sensor.GetDepthImageHeight();
        int depthWidth = sensor.GetDepthImageWidth();

        Frame frame = Frame(depthMap,
                            colorMap,
                            depthIntrinsicsInv,
                            depthExtrinsicsInv,
                            trajectoryInv,
                            depthWidth,
                            depthHeight);
        std::cout << "Processing frame!" << std::endl;

        frameCount++;
    }
    return 0;
}
