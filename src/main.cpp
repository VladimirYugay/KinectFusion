// Copyright 2020 Vladimir
// Author: Vladimir
#include <iostream>
#include <fstream>
#include <array>

#include "models/Frame.h"
#include "icp/ICP.h"
#include "icp/SVD.cuh"

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
    Frame prevFrame;

    Wrapper::wrapper();

    // while (frameCount < maxFrameNum && sensor.ProcessNextFrame()) {
    //     float* depthMap = sensor.GetDepth();
    //     BYTE* colorMap = sensor.GetColorRGBX();
    //     Matrix3f depthIntrinsics = sensor.GetDepthIntrinsics();
    //     Matrix4f depthExtrinsicsInv = sensor.GetDepthExtrinsics().inverse();
    //     Matrix4f trajectory = sensor.GetTrajectory();
    //     Matrix4f trajectoryInv = sensor.GetTrajectory().inverse();
    //     Matrix3f depthIntrinsicsInv = depthIntrinsics.inverse();
    //     int depthHeight = sensor.GetDepthImageHeight();
    //     int depthWidth = sensor.GetDepthImageWidth();

    //     Frame curFrame = Frame(depthMap,
    //                         colorMap,
    //                         depthIntrinsicsInv,
    //                         depthExtrinsicsInv,
    //                         trajectoryInv,
    //                         depthWidth,
    //                         depthHeight);
    //     // Do the job
    //     // test run for pose estimation, uncomment to run
    //     if (frameCount > 0) {
    //         std::cout << prevFrame.getVertex(302992) << std::endl;
    //         std::cout << curFrame.getVertex(302992) << std::endl;
    //         ICP icp(prevFrame, curFrame);
    //         std::vector<std::pair<size_t, size_t>> correspondenceIds(
    //             {{302990, 302990}});
    //         Matrix4f pose = icp.estimatePose(correspondenceIds, 1);
    //         std::cout << pose << std::endl;
    //     }
    //     prevFrame = curFrame;
    //     frameCount++;
    // }
    return 0;
}
