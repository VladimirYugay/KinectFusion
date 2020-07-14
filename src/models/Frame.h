// Copyright 2020 Vladimir
// Author: Vladimir
#pragma once

#include <vector>

#include "../helpers/Eigen.h"
#include "../helpers/VirtualSensor.h"

class Frame {
 public:
  Frame();
  Frame(const float *depthMap, const BYTE *colorMap,
        const Eigen::Matrix3f &depthIntrinsicsInverse,
        const Eigen::Matrix4f &depthExtrinsicsInv,
        const Eigen::Matrix4f &trajectoryInv, int depthWidth, int depthHeight);
  Eigen::Vector3f getVertex(size_t idx) const;
  Eigen::Vector3f getNormal(size_t idx) const;
  int getVertexCount() const;
  std::vector<Eigen::Vector3f> &getVertexMapGlobal();
  std::vector<Eigen::Vector3f> &getNormalMapGlobal();
  std::vector<Eigen::Vector3f> &getVertexMap();
  std::vector<Eigen::Vector3f> &getNormalMap();
  int getFrameHeight();
  int getFrameWidth();
  void setExtrinsicMatrix(const Eigen::Matrix4f &extrensicMatrix);
  bool containsImgPoint(Eigen::Vector2i);
  Eigen::Vector3f projectPointIntoFrame(const Eigen::Vector3f &point);
  Eigen::Vector2i projectOntoImgPlane(const Eigen::Vector3f &point);

 private:
  void computeVertexMap(const float *depthMap,
                        const Eigen::Matrix3f &depthIntrinsics, int depthWidth,
                        int depthHeight);
  void computeNormalMap(int depthWidth, int depthHeight);
  std::vector<Eigen::Vector3f> transformPoints(
      const std::vector<Eigen::Vector3f> &points,
      const Eigen::Matrix4f &transformation);

  std::vector<Eigen::Vector3f> rotatePoints(
      const std::vector<Eigen::Vector3f> &points,
      const Eigen::Matrix3f &rotation);

  int depthWidth;
  int depthHeight;
  std::vector<Eigen::Vector3f> mVertices;
  std::vector<Eigen::Vector3f> mNormals;
  std::vector<Eigen::Vector3f> mVerticesGlobal;
  std::vector<Eigen::Vector3f> mNormalsGlobal;
  Eigen::Matrix4f extrinsicMatrix;
  Eigen::Matrix3f intrinsicMatrix;
};
