// Copyright 2020 Vladimir
// Author: Vladimir
#include "Frame.h"

#include <iostream>

Frame::Frame() {}

Frame::Frame(const float *depthMap, const BYTE *colorMap,
             const Eigen::Matrix3f &depthIntrinsics,
             const Eigen::Matrix4f &depthExtrinsicsInv,
             const Eigen::Matrix4f &trajectoryInv, int depthWidth,
             int depthHeight)
    : depthWidth(depthWidth), depthHeight(depthHeight) {
  computeVertexMap(depthMap, depthIntrinsics, depthWidth, depthHeight);
  computeNormalMap(depthWidth, depthHeight);
  setExtrinsicMatrix(depthExtrinsicsInv.inverse());
  intrinsicMatrix = depthIntrinsics;
  std::cout << "Frame created!" << std::endl;
}

Eigen::Vector3f Frame::getVertex(size_t idx) const { return mVertices[idx]; }

Eigen::Vector3f Frame::getNormal(size_t idx) const { return mNormals[idx]; }

int Frame::getVertexCount() const { return mVertices.size(); }

bool Frame::containsImgPoint(Eigen::Vector2i imgPoint) {
  return imgPoint[0] >= 0 && imgPoint[0] < depthWidth && imgPoint[1] >= 0 &&
         imgPoint[1] < depthHeight;
}

int Frame::getFrameHeight() { return depthHeight; }

int Frame::getFrameWidth() { return depthWidth; }

std::vector<Eigen::Vector3f> &Frame::getVertexMap() { return mVertices; }
std::vector<Eigen::Vector3f> &Frame::getNormalMap() { return mNormals; }

std::vector<Eigen::Vector3f> &Frame::getVertexMapGlobal() {
  return mVerticesGlobal;
}

std::vector<Eigen::Vector3f> &Frame::getNormalMapGlobal() { return mNormals; }

Eigen::Vector3f Frame::projectPointIntoFrame(const Eigen::Vector3f &point) {
  const auto rotation = extrinsicMatrix.block(0, 0, 3, 3);
  const auto translation = extrinsicMatrix.block(0, 3, 3, 1);
  return rotation * point + translation;
}

void Frame::setExtrinsicMatrix(const Eigen::Matrix4f &extMatrix) {
  extrinsicMatrix = extMatrix;
  const auto rotation = extMatrix.block(0, 0, 3, 3);
  mVerticesGlobal = transformPoints(mVertices, extrinsicMatrix);
  mNormalsGlobal = rotatePoints(mNormals, rotation);
}

Eigen::Vector2i Frame::projectOntoImgPlane(const Eigen::Vector3f &point) {
  Eigen::Vector3f projected = intrinsicMatrix * point;
  if (projected[2] == 0) {
    return Eigen::Vector2i(MINF, MINF);
  }
  projected /= projected[2];
  return (
      Eigen::Vector2i((int)round(projected.x()), (int)round(projected.y())));
}

std::vector<Eigen::Vector3f> Frame::transformPoints(
    const std::vector<Eigen::Vector3f> &points,
    const Eigen::Matrix4f &transformation) {
  const Eigen::Matrix3f rotation = transformation.block(0, 0, 3, 3);
  const Eigen::Vector3f translation = transformation.block(0, 3, 3, 1);
  std::vector<Eigen::Vector3f> transformed(points.size());

  for (size_t idx = 0; idx < points.size(); ++idx) {
    if (points[idx].allFinite())
      transformed[idx] = rotation * points[idx] + translation;
    else
      transformed[idx] = (Eigen::Vector3f(MINF, MINF, MINF));
  }
  return transformed;
}

std::vector<Eigen::Vector3f> Frame::rotatePoints(
    const std::vector<Eigen::Vector3f> &points,
    const Eigen::Matrix3f &rotation) {
  std::vector<Eigen::Vector3f> transformed(points.size());

  for (size_t idx = 0; idx < points.size(); ++idx) {
    if (points[idx].allFinite())
      transformed[idx] = rotation * points[idx];
    else
      transformed[idx] = (Eigen::Vector3f(MINF, MINF, MINF));
  }
  return transformed;
}

void Frame::computeVertexMap(const float *depthMap,
                             const Eigen::Matrix3f &depthIntrinsics,
                             int depthWidth, int depthHeight) {
  float fX = depthIntrinsics(0, 0);
  float fY = depthIntrinsics(1, 1);
  float cX = depthIntrinsics(0, 2);
  float cY = depthIntrinsics(1, 2);

  mVertices = std::vector<Eigen::Vector3f>(depthHeight * depthWidth,
                                           Vector3f(MINF, MINF, MINF));
  for (size_t i = 0; i < depthHeight; i++) {
    for (size_t j = 0; j < depthWidth; j++) {
      size_t idx = i * depthWidth + j;
      float depth = depthMap[idx];
      if (depth != MINF) {
        mVertices[idx] =
            Vector3f((j - cX) / fX * depth, (i - cY) / fY * depth, depth);
      }
    }
  }
}

void Frame::computeNormalMap(int depthWidth, int depthHeight) {
  mNormals = std::vector<Eigen::Vector3f>(depthHeight * depthWidth,
                                          Vector3f(MINF, MINF, MINF));

  for (size_t i = 1; i < depthHeight - 1; i++) {
    for (size_t j = 1; j < depthWidth - 1; j++) {
      size_t idx = i * depthWidth + j;
      Eigen::Vector3f du = mVertices[idx + 1] - mVertices[idx - 1];
      Eigen::Vector3f dv =
          mVertices[idx + depthWidth] - mVertices[idx - depthWidth];
      if (du.allFinite() && dv.allFinite()) {
        mNormals[idx] = du.cross(dv);
        mNormals[idx].normalize();
      }
    }
  }
}
