// Copyright 2020 Group11
// Author: Group11

#include "Volume.h"

//! Initializes an empty volume dataset.
Volume::Volume(Vector3d min_,
    Vector3d max_,
    uint dx_,
    uint dy_,
    uint dz_,
    uint dim) {
    min = min_;
    max = max_;
    diag = max - min;
    dx = dx_;
    dy = dy_;
    dz = dz_;
    m_dim = dim;
    vol = NULL;

    vol = new Voxel[dx * dy * dz];

    compute_ddx_dddx();

    for (int k = 0; k < dz; k++) {
        for (int j = 0; j < dy; j++) {
            for (int i = 0; i < dx; i++) {
                vol[i * dy * dz + j * dz + k].setValue(
                    std::numeric_limits<double>::max());
            }
        }
    }
}

Volume::~Volume() {
    delete[] vol;
}

inline void Volume::setVoxelAtPoint(const Vector3d& p, const Voxel& v) {
    vol[Volume::getPosFromTuple(static_cast<int>(p[0]),
        static_cast<int>(p[1]),
        static_cast<int>(p[2]))] = v;
}

inline Voxel& Volume::getVoxelAtPoint(const Vector3d& p) const {
    return vol_access(static_cast<int>(p[0]),
        static_cast<int>(p[1]),
        static_cast<int>(p[2]));
}


//! Computes spacing in x,y,z-directions.
void Volume::compute_ddx_dddx() {
    ddx = 1.0f / (dx - 1);
    ddy = 1.0f / (dy - 1);
    ddz = 1.0f / (dz - 1);

    dddx = (max[0] - min[0]) / (dx - 1);
    dddy = (max[1] - min[1]) / (dy - 1);
    dddz = (max[2] - min[2]) / (dz - 1);

    if (dz == 1) {
        ddz = 0;
        dddz = 0;
    }

    diag = max - min;
}

//! Zeros out the memory
void Volume::zeroOutMemory() {
    for (uint i1 = 0; i1 < dx * dy * dz; i1++)
        vol[i1] = Voxel(static_cast<double>(0.0),
            static_cast<double>(0.0));
}

//! Returns the Data.
Voxel* Volume::getData() {
    return vol;
}

//! Sets all entries in the volume to '0'
void Volume::clean() {
    for (uint i1 = 0; i1 < dx * dy * dz; i1++) {
        vol[i1] = Voxel(static_cast<double>(0.0),
            static_cast<double>(0.0));
    }
}

//! Sets minimum extension
void Volume::SetMin(Vector3d min_) {
    min = min_;
    diag = max - min;
}

//! Sets maximum extension
void Volume::SetMax(Vector3d max_) {
    max = max_;
    diag = max - min;
}
