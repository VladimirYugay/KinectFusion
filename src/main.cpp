// Copyright 2020 Vladimir
// Author: Vladimir
#include <array>
#include <fstream>
#include <iostream>

#include "icp/ICP.h"
#include "models/Frame.h"

int main() {
  // Make sure this path points to the data folder
  std::string filenameIn = "../data/";
  const double distanceThreshold = 0.1;
  const double angleThreshold = 0.5;

  // load video
  std::cout << "Initialize virtual sensor..." << std::endl;
  VirtualSensor sensor;
  if (!sensor.Init(filenameIn)) {
    std::cout << "Failed to initialize the sensor!" << std::endl;
    return -1;
  }

  int maxFrameNum = 10;
  int frameCount = 0;
  Frame prevFrame;

  while (frameCount < maxFrameNum && sensor.ProcessNextFrame()) {
    float *depthMap = sensor.GetDepth();
    BYTE *colorMap = sensor.GetColorRGBX();
    Matrix3f depthIntrinsics = sensor.GetDepthIntrinsics();
    Matrix4f depthExtrinsicsInv = sensor.GetDepthExtrinsics().inverse();
    Matrix4f trajectory = sensor.GetTrajectory();
    Matrix4f trajectoryInv = sensor.GetTrajectory().inverse();
    Matrix3f depthIntrinsicsInv = depthIntrinsics.inverse();
    int depthHeight = sensor.GetDepthImageHeight();
    int depthWidth = sensor.GetDepthImageWidth();
    Matrix4f identity = Matrix4f::Identity(4, 4);  // initial estimate

    Frame curFrame =
        Frame(depthMap, colorMap, depthIntrinsicsInv, depthExtrinsicsInv,
              trajectoryInv, depthWidth, depthHeight);
    // Do the job
    // test run for pose estimation, uncomment to run

    if (frameCount > 0) {
      std::cout << prevFrame.getVertex(302992) << std::endl;
      std::cout << curFrame.getVertex(302992) << std::endl;
      ICP icp(prevFrame, curFrame, distanceThreshold, angleThreshold);
      // std::vector<std::pair<size_t, size_t>> correspondenceIds(
      //     {{302990, 302990}});
      auto mat = Matrix4f::Identity(4, 4);
      std::vector<std::pair<size_t, size_t>> correspondenceIds =
          icp.findIndicesOfCorrespondingPoints(mat);
      std::cout << "# corresponding points: " << correspondenceIds.size()
                << std::endl;
      std::cout << "# total number of points: "
                << curFrame.getVertexMap().size() << std::endl;
      Matrix4f pose = icp.estimatePose(correspondenceIds, 1);
      std::cout << pose << std::endl;
    }
    prevFrame = curFrame;
    frameCount++;
  }

  return 0;
}
