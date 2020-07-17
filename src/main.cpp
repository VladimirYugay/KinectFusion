// Copyright 2020 Vladimir
// Author: Vladimir
#include <array>
#include <fstream>
#include <iostream>

#include "icp/ICP.h"
#include "raycaster/RayCaster.h"
#include "raycaster/Ray.h"
#include "models/Frame.h"
#include "models/Volume.h"
#include "helpers/VirtualSensor.h"
#include "helpers/Eigen.h"

#define DISTANCE_THRESHOLD 0.1
#define EDGE_THRESHOLD 0.01
#define ANGLE_THRESHOLD 0.5
#define MAX_FRAME_NUM 10
#define MIN_POINT -1.5f, -1.0f, 0.0f
#define MAX_POINT 1.5f, 1.0f, 3.5f
#define RESOLUTION 512, 512, 512

int main() {
  // Make sure this path points to the data folder
    std::string filenameIn = "../../data/rgbd_dataset_freiburg1_xyz/";
    std::string filenameBaseOut = std::string("../../output/mesh_");

  // load video
  std::cout << "Initialize virtual sensor..." << std::endl;
  VirtualSensor sensor;
  if (!sensor.Init(filenameIn)) {
    std::cout << "Failed to initialize the sensor!" << std::endl;
    return -1;
  }

  int frameCount = 0;
  Frame prevFrame;
  Volume volume = Volume(Vector3f{ MIN_POINT }, Vector3f{ MAX_POINT }, RESOLUTION, 3);
  RayCaster rc = RayCaster(volume);

  while (frameCount < MAX_FRAME_NUM && sensor.ProcessNextFrame()) {
    float *depthMap = sensor.GetDepth();
    BYTE *colorMap = sensor.GetColorRGBX();
    Matrix3f depthIntrinsics = sensor.GetDepthIntrinsics();
    Matrix3f depthIntrinsicsInv = depthIntrinsics.inverse();
    Matrix4f depthExtrinsicsInv = sensor.GetDepthExtrinsics().inverse();
    Matrix4f trajectory = sensor.GetTrajectory();
    Matrix4f trajectoryInv = sensor.GetTrajectory().inverse();
    int depthHeight = sensor.GetDepthImageHeight();
    int depthWidth = sensor.GetDepthImageWidth();
    Matrix4f identity = Matrix4f::Identity(4, 4);  // initial estimate
    Matrix4f pose, mat;
    std::stringstream ss;

    Frame curFrame =
        Frame(depthMap, colorMap, depthIntrinsicsInv, depthExtrinsicsInv,
              trajectoryInv, depthWidth, depthHeight);
    // Do the job
    // test run for pose estimation, uncomment to run

    if (frameCount == 0) {
        mat = identity;
    }
    else {
      //std::cout << prevFrame.getVertex(302992) << std::endl;
      //std::cout << curFrame.getVertex(302992) << std::endl;
      ICP icp(prevFrame, curFrame, DISTANCE_THRESHOLD, ANGLE_THRESHOLD);
      // std::vector<std::pair<size_t, size_t>> correspondenceIds(
      //     {{302990, 302990}});
      mat = pose;
      std::vector<std::pair<size_t, size_t>> correspondenceIds =
          icp.findIndicesOfCorrespondingPoints(mat);
      std::cout << "# corresponding points: " << correspondenceIds.size()
                << std::endl;
      std::cout << "# total number of points: "
                << curFrame.getVertexMap().size() << std::endl;
      pose = icp.estimatePose(correspondenceIds, 1);
      std::cout << pose << std::endl;

      volume.integrate(curFrame);

      rc.changeFrame(curFrame);
      rc.rayCast();
      prevFrame = curFrame;

      if (frameCount % 5 == 0) {
          ss << filenameBaseOut << frameCount<< ".off";
          
          if (!curFrame.writeMesh(ss.str(), EDGE_THRESHOLD)) {
            std::cout << "Failed to write mesh!\nCheck file path!" << std::endl;
            return -1;
      }
    }

    frameCount++;
  }

  return 0;
}
