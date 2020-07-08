// Copyright 2020 Group11
// Author: Group11
#pragma once

#ifndef VOLUME_H
#define VOLUME_H
#define MU 1.0

#include <limits>
#include <vector>
#include <algorithm>
#include "../helpers/Eigen.h"
typedef unsigned int uint;

class Voxel {
 private:
    double value;
    double weight;

 public:
    Voxel() {}

    Voxel(double value_, double weight_)
        : value { value_ }, weight { weight_ } {}

    double getValue() {
        return value;
    }

    double getWeight() {
        return weight;
    }

    void setValue(double v) {
        value = v;
    }

    void setWeight(double w) {
        weight = w;
    }
};

// ! A regular volume dataset
class Volume {
 public:
    // ! Initializes an empty volume dataset.
    Volume(Eigen::Vector3d min_,
        Eigen::Vector3d max_,
        uint dx_ = 10,
        uint dy_ = 10,
        uint dz_ = 10,
        uint dim = 1);

    ~Volume();

    inline void setVoxelAtPoint(const Eigen::Vector3d& p, const Voxel& v) {}
    inline Voxel& getVoxelAtPoint(const Eigen::Vector3d& p) const {}

    double getTruncation(double sdf) {
        if (sdf >= -MU)
            return min(1, sdf / MU);
        else
            return std::numeric_limits<double>::min();
    }

    void integrate(const Eigen::Matrix3f intrinsic,
        const Eigen::Matrix4f cameraToWorld,
        const float* depthmap,
        double depthMapWidth,
        double depthMapHeight,
        const std::vector<Eigen::Vector3f> normals) {
            const Eigen::Matrix4f worldToCamera = cameraToWorld.inverse();
            Eigen::Vector4d translation = cameraToWorld.block(3, 3, 4, 1);

            for (int k = 0; k < dz; k++) {
                for (int j = 0; j < dy; j++) {
                    for (int i = 0; i < dx; i++) {
                        Eigen::Matrix3f I = Matrix3f::Identity();
                        Eigen::Vector4f p = worldToCamera * pos(i, j, k);
                        Eigen::Vector3f u = intrinsic * I * p;
                        u[0] = round(u[0] / u[2]);
                        u[1] = round(u[1] / u[2]);

                        if (u[0] >= 0 && u[1] >= 0
                            && u[0] <= depthMapHeight
                            && u[1] <= depthMapWidth) {
                                float depth = depthmap[static_cast<int>(u[0] *
                                    depthMapWidth + u[1])];
                                Eigen::Vector3d KInv_u = I * p / u[2];

                                double sdf = (1 / KInv_u.norm() * I *
                                    (pos(i, j, k) - translation)).norm() -depth;
                                double truncation = getTruncation(sdf);

                                if (sdf >= -truncation
                                    && vol[getPosFromTuple(i, j, k)].getValue()
                                    != std::numeric_limits<double>::max()) {
                                    Eigen::Vector4d zero;
                                    zero << 0, 0, 0, 1;
                                    zero = cameraToWorld * zero;
                                    Eigen::Vector3f ray = I *
                                        (pos(i, j, k) - zero);

                                    double dot = ray.dot(
                                        normals[static_cast<int>(
                                            u[0] * depthMapWidth + u[1])]);
                                    double lenRayVector = ray.norm();
                                    double lenNormal = normals[
                                        static_cast<int>(u[0] *
                                            depthMapWidth + u[1])].norm();
                                    double cos_angle = dot /
                                        (lenRayVector * lenNormal);

                                    double sdf_weight = cos_angle / depth;

                                    double value = vol[
                                        getPosFromTuple(i, j, k)].getValue();
                                    double weight = vol[
                                        getPosFromTuple(i, j, k)].getWeight();

                                    vol[getPosFromTuple(i, j, k)].setValue(
                                        (value * weight + truncation
                                            *sdf_weight)/(weight+sdf_weight));
                                    vol[getPosFromTuple(i, j, k)].setWeight(
                                        weight + sdf_weight);
                                }
                        }
                    }
                }
            }
        }

    inline void computeMinMaxValues(double* minVal, double* maxVal) const {
        *minVal = std::numeric_limits<double>::max();
        *maxVal = -*minVal;
        for (uint i1 = 0; i1 < dx * dy * dz; i1++) {
            if (*minVal > vol[i1].getValue()) *minVal = vol[i1].getValue();
            if (*maxVal < vol[i1].getValue()) *maxVal = vol[i1].getValue();
        }
    }

    // ! Computes spacing in x,y,z-directions.
    void compute_ddx_dddx();

    // ! Zeros out the memory
    void zeroOutMemory();

    // ! Set the value at i.
    inline void set(uint i, Voxel v) {
        if (v.getValue() > maxValue)
            maxValue = v.getValue();

        if (v.getValue() < minValue)
            minValue = v.getValue();

        vol[i] = v;
    }

    // ! Set the value at (x_, y_, z_).
    inline void set(uint x_, uint y_, uint z_, const Voxel& v) {
        vol[getPosFromTuple(x_, y_, z_)] = v;
    }

    // ! Get the value at (x_, y_, z_).
    inline Voxel& get(uint i) const {
        return vol[i];
    }

    // ! Get the value at (x_, y_, z_).
    inline Voxel& get(uint x_, uint y_, uint z_) const {
        return vol[getPosFromTuple(x_, y_, z_)];
    }

    // ! Get the value at (pos.x, pos.y, pos.z).
    inline Voxel& get(const Eigen::Vector3i& pos_) const {
        return(get(pos_[0], pos_[1], pos_[2]));
    }

    // ! Returns the cartesian x-coordinates of node (i,..).
    inline double posX(int i) const {
        return min[0] + diag[0] * (static_cast<double>(i) * ddx);
    }

    // ! Returns the cartesian y-coordinates of node (..,i,..).
    inline double posY(int i) const {
        return min[1] + diag[1] * (static_cast<double>(i) * ddy);
    }

    // ! Returns the cartesian z-coordinates of node (..,i).
    inline double posZ(int i) const {
        return min[2] + diag[2] * (static_cast<double>(i) * ddz);
    }

    // ! Returns the cartesian coordinates of node (i,j,k).
    inline Eigen::Vector4d pos(int i, int j, int k) const {
        Eigen::Vector4d coord(0, 0, 0, 1);

        coord[0] = min[0] + (max[0] - min[0]) * (static_cast<double>(i) * ddx);
        coord[1] = min[1] + (max[1] - min[1]) * (static_cast<double>(j) * ddy);
        coord[2] = min[2] + (max[2] - min[2]) * (static_cast<double>(k) * ddz);

        return coord;
    }

    // ! Returns the Data.
    Voxel* getData();

    // ! Sets all entries in the volume to '0'
    void clean();

    // ! Returns number of cells in x-dir.
    inline uint getDimX() const { return dx; }

    // ! Returns number of cells in y-dir.
    inline uint getDimY() const { return dy; }

    // ! Returns number of cells in z-dir.
    inline uint getDimZ() const { return dz; }

    inline Eigen::Vector3d getMin() { return min; }
    inline Eigen::Vector3d getMax() { return max; }

    // ! Sets minimum extension
    void SetMin(Eigen::Vector3d min_);

    // ! Sets maximum extension
    void SetMax(Eigen::Vector3d max_);

    inline uint getPosFromTuple(int x, int y, int z) const {
        return x * dy * dz + y * dz + z;
    }


    // ! Lower left and Upper right corner.
    Eigen::Vector3d min, max;

    // ! max-min
    Eigen::Vector3d diag;

    double ddx, ddy, ddz;
    double dddx, dddy, dddz;

    // ! Number of cells in x, y and z-direction.
    uint dx, dy, dz;

    Voxel* vol;

    double maxValue, minValue;

    uint m_dim;

 private:
    // ! x,y,z access to vol*
    inline Voxel& vol_access(int x, int y, int z) const {
        return vol[getPosFromTuple(x, y, z)];
    }
};

#endif  // VOLUME_H
