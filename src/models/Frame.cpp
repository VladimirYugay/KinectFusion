// Copyright 2020 Vladimir
// Author: Vladimir
#include "Frame.h"

#include <iostream>
#include <fstream>

Frame::Frame() {}

Frame::Frame(const Frame& other) {
    depthWidth = other.depthWidth;
    depthHeight = other.depthHeight;
    depthMap = other.depthMap;
    colorMap = other.colorMap;
    mVertices = other.mVertices;
    mNormals = other.mNormals;
    mVerticesGlobal = other.mVerticesGlobal;
    mNormalsGlobal = other.mNormalsGlobal;
    extrinsicMatrix = other.extrinsicMatrix;
    intrinsicMatrix = other.intrinsicMatrix;
}

Frame::Frame(const float* depthMap, const BYTE* colorMap,
    const Eigen::Matrix3f& depthIntrinsics,
    const Eigen::Matrix4f& depthExtrinsics,
    const Eigen::Matrix4f& trajectoryInv,
    int depthWidth,
    int depthHeight)
    : depthWidth(depthWidth), depthHeight(depthHeight), depthMap(depthMap), colorMap(colorMap) {

    computeVertexMap(depthMap, depthIntrinsics, depthWidth, depthHeight);
    computeNormalMap(depthWidth, depthHeight);
    setExtrinsicMatrix(depthExtrinsics);

    intrinsicMatrix = depthIntrinsics;
    std::cout << "Frame created!" << std::endl;
}

Eigen::Vector3f Frame::getVertexGlobal(size_t idx) const { return mVerticesGlobal->at(idx); }

Eigen::Vector3f Frame::getNormalGlobal(size_t idx) const { return mNormalsGlobal->at(idx); }

Eigen::Vector3f Frame::getVertex(size_t idx) const { return mVertices->at(idx); }

Eigen::Vector3f Frame::getNormal(size_t idx) const { return mNormals->at(idx); }

int Frame::getVertexCount() const { return mVertices->size(); }

bool Frame::containsImgPoint(Eigen::Vector2i imgPoint) {
    return imgPoint[0] >= 0 && imgPoint[0] < depthWidth && imgPoint[1] >= 0 &&
        imgPoint[1] < depthHeight;
}

int Frame::getFrameHeight() { return depthHeight; }

int Frame::getFrameWidth() { return depthWidth; }

std::vector<Eigen::Vector3f>& Frame::getVertexMap() { return *mVertices; }
std::vector<Eigen::Vector3f>& Frame::getNormalMap() { return *mNormals; }

std::vector<Eigen::Vector3f>& Frame::getVertexMapGlobal() {
    return *mVerticesGlobal;
}

std::vector<Eigen::Vector3f>& Frame::getNormalMapGlobal() { return *mNormalsGlobal; }

Eigen::Vector3f Frame::projectPointIntoFrame(const Eigen::Vector3f& point) {
    const auto rotation = extrinsicMatrix.block(0, 0, 3, 3);
    const auto translation = extrinsicMatrix.block(0, 3, 3, 1);
    return rotation * point + translation;
}

void Frame::setExtrinsicMatrix(const Eigen::Matrix4f& extMatrix) {
    extrinsicMatrix = extMatrix;
    Eigen::Matrix4f extrinsicMatrixInv = extrinsicMatrix.inverse();
    const auto rotation = extrinsicMatrixInv.block(0, 0, 3, 3);
    mVerticesGlobal = std::make_shared<std::vector<Eigen::Vector3f>>(transformPoints(*mVertices, extrinsicMatrixInv));
    mNormalsGlobal = std::make_shared<std::vector<Eigen::Vector3f>>(rotatePoints(*mNormals, rotation));
}

Eigen::Vector2i Frame::projectOntoImgPlane(const Eigen::Vector3f& point) {
    Eigen::Vector3f projected = intrinsicMatrix * point;
    if (projected[2] == 0) {
        return Eigen::Vector2i(MINF, MINF);
    }
    projected /= projected[2];
    return Eigen::Vector2i((int)round(projected.x()), (int)round(projected.y()));
}

std::vector<Eigen::Vector3f> Frame::transformPoints(
    const std::vector<Eigen::Vector3f>& points,
    const Eigen::Matrix4f& transformation) {
    const Eigen::Matrix3f rotation = transformation.block(0, 0, 3, 3);
    const Eigen::Vector3f translation = transformation.block(0, 3, 3, 1);
    std::vector<Eigen::Vector3f> transformed(points.size());

    for (size_t idx = 0; idx < points.size(); ++idx) {
        if (points[idx].allFinite())
            transformed[idx] = rotation * points[idx] + translation;
        else
            transformed[idx] = Eigen::Vector3f(MINF, MINF, MINF);
    }
    return transformed;
}

std::vector<Eigen::Vector3f> Frame::rotatePoints(
    const std::vector<Eigen::Vector3f>& points,
    const Eigen::Matrix3f& rotation) {
    std::vector<Eigen::Vector3f> transformed(points.size());

    for (size_t idx = 0; idx < points.size(); ++idx) {
        if (points[idx].allFinite())
            transformed[idx] = rotation * points[idx];
        else
            transformed[idx] = Eigen::Vector3f(MINF, MINF, MINF);
    }
    return transformed;
}

void Frame::computeVertexMap(const float* depthMap,
    const Eigen::Matrix3f& depthIntrinsics,
    int depthWidth, int depthHeight) {
    float fX = depthIntrinsics(0, 0);
    float fY = depthIntrinsics(1, 1);
    float cX = depthIntrinsics(0, 2);
    float cY = depthIntrinsics(1, 2);

    mVertices = std::make_shared<std::vector<Eigen::Vector3f>> 
        (
            std::vector<Eigen::Vector3f>()
        );
    mVertices->reserve(depthHeight * depthWidth);

    for (size_t i = 0; i < depthHeight; i++) {
        for (size_t j = 0; j < depthWidth; j++) {
            size_t idx = i * depthWidth + j;
            float depth = depthMap[idx];
            if (depth != MINF) {
                mVertices->emplace_back(
                    Vector3f((j - cX) / fX * depth, (i - cY) / fY * depth, depth));
            }
            else {
                mVertices->emplace_back(Vector3f(MINF, MINF, MINF));
            }
        }
    }
}

void Frame::computeNormalMap(int depthWidth, int depthHeight) {

    mNormals = std::make_shared<std::vector<Eigen::Vector3f>>
        (
            std::vector<Eigen::Vector3f> ()
        );
    mNormals->reserve(depthHeight * depthWidth);

    for (size_t i = 0; i < depthHeight; i++) {
        for (size_t j = 0; j < depthWidth; j++) {
            size_t idx = i * depthWidth + j;
            if (i == 0 || j == 0 || i == depthHeight - 1 || j == depthWidth - 1)
                mNormals->emplace_back(Vector3f(MINF, MINF, MINF));
            else {
                Eigen::Vector3f du = mVertices->at(idx + 1) - mVertices->at(idx - 1);
                Eigen::Vector3f dv =
                    mVertices->at(idx + depthWidth) - mVertices->at(idx - depthWidth);
                if (du.allFinite() && dv.allFinite()) {
                    mNormals->emplace_back(du.cross(dv).normalized());
                }
                else {
                    mNormals->emplace_back(Vector3f(MINF, MINF, MINF));
                }
            }
        }
    }
}

const Eigen::Matrix4f Frame::getExtrinsicMatrix() {
    return extrinsicMatrix;
}
const Eigen::Matrix3f Frame::getIntrinsicMatrix() {
    return intrinsicMatrix;
}

const float* Frame::getDepthMap() {
    return depthMap;
}
const BYTE* Frame::getColorMap() {
    return colorMap;
}

bool Frame::writeMesh(const std::string& filename, float edgeThreshold) {
    // Write off file.
    std::ofstream outFile(filename);
    if (!outFile.is_open()) return false;

    std::cout << "Writing mesh: '" << filename << "'!" << std::endl;

    // Create triangles
    std::vector<Vector3i> mTriangles;
    mTriangles.reserve((depthHeight - 1) * (depthWidth - 1) * 2);
    for (unsigned int i = 0; i < depthHeight - 1; i++) {
        for (unsigned int j = 0; j < depthWidth - 1; j++) {
            unsigned int i0 = i * depthWidth + j;
            unsigned int i1 = (i + 1) * depthWidth + j;
            unsigned int i2 = i * depthWidth + j + 1;
            unsigned int i3 = (i + 1) * depthWidth + j + 1;

            bool valid0 = mVerticesGlobal->at(i0).allFinite();
            bool valid1 = mVerticesGlobal->at(i1).allFinite();
            bool valid2 = mVerticesGlobal->at(i2).allFinite();
            bool valid3 = mVerticesGlobal->at(i3).allFinite();

            if (valid0 && valid1 && valid2) {
                float d0 = (mVerticesGlobal->at(i0) - mVerticesGlobal->at(i1)).norm();
                //std::cout << d0 << std::endl;
                float d1 = (mVerticesGlobal->at(i0) - mVerticesGlobal->at(i2)).norm();
                //std::cout << d1 << std::endl;
                float d2 = (mVerticesGlobal->at(i1) - mVerticesGlobal->at(i2)).norm();
                //std::cout << d2 << std::endl;
                if (edgeThreshold > d0 && edgeThreshold > d1 && edgeThreshold > d2)
                    mTriangles.emplace_back(Vector3i(i0, i1, i2));
            }
            if (valid1 && valid2 && valid3) {
                float d0 = (mVerticesGlobal->at(i3) - mVerticesGlobal->at(i1)).norm();
                float d1 = (mVerticesGlobal->at(i3) - mVerticesGlobal->at(i2)).norm();
                float d2 = (mVerticesGlobal->at(i1) - mVerticesGlobal->at(i2)).norm();
                if (edgeThreshold > d0 && edgeThreshold > d1 && edgeThreshold > d2)
                    mTriangles.emplace_back(Vector3i(i1, i3, i2));
            }
        }
    }

    std::cout << mTriangles.size() << std::endl;
    // Write header.
    outFile << "COFF" << std::endl;
    outFile << mVerticesGlobal->size() << " " << mTriangles.size() << " 0" << std::endl;

    // Save vertices.
    for (unsigned int i = 0; i < mVerticesGlobal->size(); i++) {
        const auto& vertex = mVerticesGlobal->at(i);
        if (vertex.allFinite())
            outFile << vertex.x() << " " << vertex.y() << " " << vertex.z() << " "
            << int(colorMap[4 * i]) << " " << int(colorMap[4 * i + 1]) << " " << int(colorMap[4 * i + 2]) << " " << int(colorMap[4 * i + 3]) << std::endl;
        else
            outFile << "0.0 0.0 0.0 0 0 0 0" << std::endl;
    }

    // Save faces.
    for (unsigned int i = 0; i < mTriangles.size(); i++) {
        outFile << "3 " << mTriangles[i][0] << " " << mTriangles[i][1] << " " << mTriangles[i][2] << std::endl;
    }

    // Close file.
    outFile.close();

    std::cout << "Mesh written!" << std::endl;

    return true;
}

Eigen::Vector3f Frame::transformPoint(
    Eigen::Vector3f& point,
    const Eigen::Matrix4f& transformation) {
    const Eigen::Matrix3f rotation = transformation.block(0, 0, 3, 3);
    const Eigen::Vector3f translation = transformation.block(0, 3, 3, 1);
    Eigen::Vector3f transformed;

    if (point.allFinite())
        transformed = rotation * point + translation;
    else
        transformed = Eigen::Vector3f(MINF, MINF, MINF);

    return transformed;
}

Eigen::Vector2i Frame::perspectiveProjection(
    Eigen::Vector3f& point,
    const Eigen::Matrix3f& intrinsic) {
    Eigen::Vector3f projected;

    projected = intrinsic * point;

    projected /= projected[2];

    return Eigen::Vector2i((int)round(projected.x()), (int)round(projected.y()));
}