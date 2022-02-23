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
#include "SimpleMesh.h"
#include "MarchingCubes.h"

#define DISTANCE_THRESHOLD 0.05
#define EDGE_THRESHOLD 0.02
#define ANGLE_THRESHOLD 1.05
#define MAX_FRAME_NUM 800
#define MIN_POINT -1.5f, -1.0f, -0.1f
#define MAX_POINT 1.5f, 1.0f, 3.5f
#define RESOLUTION 512, 512, 512
#define ICP_ITERATIONS 20

int main() {
  // Make sure this path points to the data folder
    std::string filenameIn = "../data/rgbd_dataset_freiburg1_xyz/";
    std::string filenameBaseOut = std::string("../output/mesh_");
    std::string filenameBaseOutMC = std::string("../output/MCmesh_");

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

          pose = icp.estimatePose(pose, ICP_ITERATIONS);
          std::cout << pose << std::endl;

          curFrame.setExtrinsicMatrix(curFrame.getExtrinsicMatrix() * pose.inverse());

          volume.integrate(curFrame);

          rc.changeFrame(curFrame);
          curFrame = rc.rayCast();

          if (frameCount % 5 == 1) {
              std::stringstream ss;
              ss << filenameBaseOut << frameCount << ".off";

              if (!curFrame.writeMesh(ss.str(), EDGE_THRESHOLD)) {
                  std::cout << "Failed to write mesh!\nCheck file path!" << std::endl;
                  return -1;
              }
          }

          if (frameCount % 20 == 1) {
              //TODO
              std::stringstream ss;
              ss << filenameBaseOutMC << frameCount << ".off";

              std::cout << "Marching Cubes started..." << std::endl;
               // extract the zero iso-surface using marching cubes
              SimpleMesh mesh;
              
              std::unordered_map<Vector3i, bool, matrix_hash<Vector3i>> visitedVoxels = volume.getVisitedVoxels();

              for (auto it = visitedVoxels.begin(); it != visitedVoxels.end(); it++)
              {
                  //std::cout << it->first << std::endl;
                  Vector3i voxelCoords = it->first;
                  ProcessVolumeCell(&volume, voxelCoords[0], voxelCoords[1], voxelCoords[2], 0.00f, &mesh);
              }
              /*
              for (unsigned int x = 0; x < volume.getDimX() - 1; x++)
              {
                  //std::cerr << "Marching Cubes on slice " << x << " of " << volume.getDimX() << std::endl;

                  for (unsigned int y = 0; y < volume.getDimY() - 1; y++)
                  {
                      for (unsigned int z = 0; z < volume.getDimZ() - 1; z++)
                      {
                          ProcessVolumeCell(&volume, x, y, z, 0.00f, &mesh);
                      }
                  }
              }
              */
              std::cout << "Marching Cubes done! " << mesh.getVertices().size() << " " << mesh.getTriangles().size() << std::endl;

              // write mesh to file
              if (!mesh.writeMesh(ss.str()))
              {
                  std::cout << "ERROR: unable to write output file!" << std::endl;
                  return -1;
              }
          }
      }

      prevFrame = curFrame;
      frameCount++;
  }

  return 0;
}
