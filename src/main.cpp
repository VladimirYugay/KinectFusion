// Copyright 2020 Vladimir
// Author: Vladimir
#include <array>
#include <fstream>
#include <iostream>

#include "ICP.h"
#include "RayCaster.h"
#include "Ray.h"
#include "Frame.h"
#include "Volume.h"
#include "VirtualSensor.h"
#include "Eigen.h"

#define DISTANCE_THRESHOLD 0.1
#define EDGE_THRESHOLD 0.01
#define ANGLE_THRESHOLD 0.5
#define MAX_FRAME_NUM 2
#define MIN_POINT -1.5f, -1.0f, -0.1f
#define MAX_POINT 1.5f, 1.0f, 3.5f
#define RESOLUTION 512, 512, 512

int main() {
  // Make sure this path points to the data folder
    std::string filenameIn = "../data/";
    std::string filenameBaseOut = std::string("../output/mesh_");

  // load video
  std::cout << "Initialize virtual sensor..." << std::endl;
  VirtualSensor sensor;
  if (!sensor.Init(filenameIn)) {
    std::cout << "Failed to initialize the sensor!" << std::endl;
    return -1;
  }

  int frameCount = 0;
  Frame curFrame, prevFrame;
  Vector3f min_point = Vector3f{ MIN_POINT };
  Vector3f max_point = Vector3f{ MAX_POINT };

  Volume volume = Volume(min_point, max_point, RESOLUTION, 3);
  RayCaster rc = RayCaster(volume);
  Matrix4f identity = Matrix4f::Identity(4, 4);  // initial estimate
  Matrix4f pose = identity;

  while (frameCount < MAX_FRAME_NUM && sensor.ProcessNextFrame()) {
      float* depthMap = sensor.GetDepth();
      BYTE* colorMap = sensor.GetColorRGBX();
      Matrix3f depthIntrinsics = sensor.GetDepthIntrinsics();
      Matrix4f depthExtrinsics = sensor.GetDepthExtrinsics();
      Matrix4f trajectory = sensor.GetTrajectory();
      Matrix4f trajectoryInv = sensor.GetTrajectory().inverse();
      int depthHeight = sensor.GetDepthImageHeight();
      int depthWidth = sensor.GetDepthImageWidth();
      std::stringstream ss;

      //std::cout << trajectory;

      curFrame =
          Frame(depthMap, colorMap, depthIntrinsics, depthExtrinsics,
              trajectoryInv, depthWidth, depthHeight);

      // Do the job
      // test run for pose estimation, uncomment to run


      if (frameCount == 0) {
          volume.integrate(curFrame);
      }
      else {
          //std::cout << prevFrame.getVertex(302992) << std::endl;
          //std::cout << curFrame.getVertex(302992) << std::endl;

          ICP icp(prevFrame, curFrame, DISTANCE_THRESHOLD, ANGLE_THRESHOLD);
          // std::vector<std::pair<size_t, size_t>> correspondenceIds(
          //     {{302990, 302990}});

          pose = icp.estimatePose(pose, 10);
          std::cout << pose << std::endl;

          curFrame.setExtrinsicMatrix(curFrame.getExtrinsicMatrix() * pose.inverse());

          volume.integrate(curFrame);

          rc.changeFrame(curFrame);
          curFrame = rc.rayCast();

          if (frameCount % 5 == 1) {
              ss << filenameBaseOut << frameCount << ".off";

              if (!curFrame.writeMesh(ss.str(), EDGE_THRESHOLD)) {
                  std::cout << "Failed to write mesh!\nCheck file path!" << std::endl;
                  return -1;
              }
          }
      }

      prevFrame = curFrame;
      frameCount++;
  }

  return 0;
}
