#pragma once

#ifndef VOLUME_H
#define VOLUME_H

#include <limits>
#include "Eigen.h"
#include "Frame.h"
#include <unordered_map>
#include <vector>

typedef unsigned int uint;

using namespace Eigen;

class Voxel
{
private:
	float value;
	float weight;
	Vector4uc color;

public:
	Voxel() {}

	Voxel(float value_, float weight_, Vector4uc color_) : value{ value_ }, weight{ weight_ }, color{ color_ } {}

	float getValue() {
		return value;
	}

	float getWeight() {
		return weight;
	}

	Vector4uc getColor() {
		return color;
	}

	void setValue(float v) {
		value = v;
	}

	void setWeight(float w) {
		weight = w;
	}

	void setColor(Vector4uc c) {
		color = c;
	}
};

// Hash function for Eigen matrix and vector.
// The code is from `hash_combine` function of the Boost library. See
// http://www.boost.org/doc/libs/1_55_0/doc/html/hash/reference.html#boost.hash_combine .
template<typename T>
struct matrix_hash : std::unary_function<T, size_t> {
	std::size_t operator()(T const& matrix) const {
		// Note that it is oblivious to the storage order of Eigen matrix (column- or
		// row-major). It will give you the same hash value for two different matrices if they
		// are the transpose of each other in different storage order.
		size_t seed = 0;
		for (size_t i = 0; i < matrix.size(); ++i) {
			auto elem = *(matrix.data() + i);
			seed ^= std::hash<typename T::Scalar>()(elem) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
		}
		return seed;
	}
};

//! A regular volume dataset
class Volume
{
private:
	//! Lower left and Upper right corner.
	Vector3f min, max;

	//! max-min
	Vector3f diag;

	float ddx, ddy, ddz;
	float dddx, dddy, dddz;

	//! Number of cells in x, y and z-direction.
	uint dx, dy, dz;

	Voxel* vol;

	uint m_dim;

	//map that tracks raycasted voxels
	std::unordered_map<Vector3i, bool, matrix_hash<Vector3i>> visitedVoxels;

public:
	
	Volume();
	//! Initializes an empty volume dataset.
	Volume(Vector3f& min_, Vector3f& max_, uint dx_ = 10, uint dy_ = 10, uint dz_ = 10, uint dim = 1);

	~Volume();

	inline static Vector3i intCoords(const Vector3f& p) {
		Vector3i coord{ 0, 0, 0 };

		coord[0] = int(p[0]);
		coord[1] = int(p[1]);
		coord[2] = int(p[2]);

		return coord;
	}

	inline static Vector3f roundCoords(const Vector3f& p) {
		Vector3f coord{ 0.0f, 0.0f, 0.0f };

		coord[0] = round(p[0]);
		coord[1] = round(p[1]);
		coord[2] = round(p[2]);

		return coord;
	}


	// estimate the normal for a point in voxel grid coordinates using voxel grid by calculating the numerical derivative of TSDF
	Vector3f calculateNormal(const Vector3f& point);

	// trilinear interpolation of a point in voxel grid coordinates to get SDF at the point
	float trilinearInterpolation(const Vector3f& p);
		
	// using given frame calculate TSDF values for all voxels in the grid
	void integrate(Frame frame);

	//! Zeros out the memory
	void zeroOutMemory();

	//! Get index of voxel at (x, y, z)
	inline uint getPosFromTuple(int x, int y, int z) const
	{
		return x * dy * dz + y * dz + z;
	}

	//! Set the value at (x_, y_, z_).
	inline void set(uint x_, uint y_, uint z_, Voxel& v)
	{
		vol[getPosFromTuple(x_, y_, z_)] = v;
	};

	//! Set the value at (pos.x, pos.y, pos.z).
	inline void set(const Vector3i& pos_, Voxel& v)
	{
		set(pos_[0], pos_[1], pos_[2], v);
	};

	//! Get the value at (x_, y_, z_).
	inline Voxel& get(uint x_, uint y_, uint z_) const
	{
		return vol[getPosFromTuple(x_, y_, z_)];
	};

	//! Get the value at (pos.x, pos.y, pos.z).
	inline Voxel& get(const Vector3i& pos_) const
	{
		return(get(pos_[0], pos_[1], pos_[2]));
	}

	//! Returns the cartesian coordinates of node (i,j,k).
	inline Vector3f gridToWorld(int i, int j, int k) const
	{
		Vector3f coord(0.0f, 0.0f, 0.0f);

		coord[0] = min[0] + (max[0] - min[0]) * (float(i) * ddx);
		coord[1] = min[1] + (max[1] - min[1]) * (float(j) * ddy);
		coord[2] = min[2] + (max[2] - min[2]) * (float(k) * ddz);

		return coord;
	}

	//! Returns the cartesian coordinates of a vector in grid coordinates (p.x, p.y, p.z).
	inline Vector3f gridToWorld(Vector3f& p) const
	{
		Vector3f coord(0.0f, 0.0f, 0.0f);

		coord[0] = min[0] + (max[0] - min[0]) * (p[0] * ddx);
		coord[1] = min[1] + (max[1] - min[1]) * (p[1] * ddy);
		coord[2] = min[2] + (max[2] - min[2]) * (p[2] * ddz);

		return coord;
	}

	inline Vector3f worldToGrid(Vector3f& p) {
		Vector3f coord(0.0f, 0.0f, 0.0f);

		coord[0] = (p[0] - min[0]) / (max[0] - min[0]) / ddx;
		coord[1] = (p[1] - min[1]) / (max[1] - min[1]) / ddy;
		coord[2] = (p[2] - min[2]) / (max[2] - min[2]) / ddz;

		return coord;
	}

	//! Returns the Data.
	Voxel* getData();

	//! Returns number of cells in x-dir.
	inline uint getDimX() const { return dx; }

	//! Returns number of cells in y-dir.
	inline uint getDimY() const { return dy; }

	//! Returns number of cells in z-dir.
	inline uint getDimZ() const { return dz; }

	//! Sets new min and max points
	void setNewBoundingPoints(Vector3f& min_, Vector3f& max_);

	//! Checks if a voxel at coords (x, y, z) was raycasted
	bool voxelVisited(int x, int y, int z) {
		Vector3i pos = Vector3i{ x, y, z };
		if (visitedVoxels.find(pos) != visitedVoxels.end())
			return true;
		else
			return false;
	}

	//! Checks if a voxel at point p in grid coords was raycasted
	bool voxelVisited(Vector3f& p) {
		Vector3i pi = Volume::intCoords(p);

		return voxelVisited(pi[0], pi[1], pi[2]);
	}

	//! Adds voxel to visited voxels
	void setVisited(Vector3i& voxCoords) {
		std::vector<Vector3i> starting_points;
		Vector3i p_int = voxCoords;
		
		starting_points.emplace_back(Vector3i{ p_int[0] + 0, p_int[1] + 0, p_int[2] + 0 });
		starting_points.emplace_back(Vector3i{ p_int[0] - 1, p_int[1] + 0, p_int[2] + 0 });
		starting_points.emplace_back(Vector3i{ p_int[0] + 0, p_int[1] - 1, p_int[2] + 0 });
		starting_points.emplace_back(Vector3i{ p_int[0] - 1, p_int[1] - 1, p_int[2] + 0 });
		starting_points.emplace_back(Vector3i{ p_int[0] + 0, p_int[1] + 0, p_int[2] - 1 });
		starting_points.emplace_back(Vector3i{ p_int[0] - 1, p_int[1] + 0, p_int[2] - 1 });
		starting_points.emplace_back(Vector3i{ p_int[0] + 0, p_int[1] - 1, p_int[2] - 1 });
		starting_points.emplace_back(Vector3i{ p_int[0] - 1, p_int[1] - 1, p_int[2] - 1 });

		for (auto p_int : starting_points) {
			visitedVoxels[Vector3i{ p_int[0] + 0, p_int[1] + 0, p_int[2] + 0 }] = true;
			visitedVoxels[Vector3i{ p_int[0] + 1, p_int[1] + 0, p_int[2] + 0 }] = true;
			visitedVoxels[Vector3i{ p_int[0] + 0, p_int[1] + 1, p_int[2] + 0 }] = true;
			visitedVoxels[Vector3i{ p_int[0] + 1, p_int[1] + 1, p_int[2] + 0 }] = true;
			visitedVoxels[Vector3i{ p_int[0] + 0, p_int[1] + 0, p_int[2] + 1 }] = true;
			visitedVoxels[Vector3i{ p_int[0] + 1, p_int[1] + 0, p_int[2] + 1 }] = true;
			visitedVoxels[Vector3i{ p_int[0] + 0, p_int[1] + 1, p_int[2] + 1 }] = true;
			visitedVoxels[Vector3i{ p_int[0] + 1, p_int[1] + 1, p_int[2] + 1 }] = true;
		}



		//std::cout << voxCoords << std::endl;
		//std::cout << visitedVoxels[voxCoords] << std::endl;
		//if (visitedVoxels.empty())
		//	std::cout << "Bok!\n";
	}

	//! Removes vocel from visited voxels
	void removeVisited(Vector3i& voxCoords) {
		visitedVoxels.erase(voxCoords);
	}

	//! Get visited voxels map
	std::unordered_map<Vector3i, bool, matrix_hash<Vector3i>>& getVisitedVoxels() {
		return visitedVoxels;
	}

	//! Updates the color of a voxel
	void updateColor(Vector3i voxelCoords, Vector4uc& color, bool notVisited);

	//! Updates the color of a voxel for a point p in grid coordinates
	void updateColor(Vector3f point, Vector4uc& color, bool notVisited);

	//! Checks if the point in grid coordinates is in the volume
	bool isPointInVolume(Vector3f& point) {
		return
			!(
				point[0] > dx - 1 ||
				point[1] > dy - 1 ||
				point[2] > dz - 1 ||
				point[0] < 0 ||
				point[1] < 0 ||
				point[2] < 0
				);
	}

	//! Checks if the trilinear interpolation possible for a given point (we have to have 8 surrounding points)
	bool isInterpolationPossible(Vector3f& point) {
		return
			!(
				point[0] > dx - 3 ||
				point[1] > dy - 3 ||
				point[2] > dz - 3 ||
				point[0] < 2 ||
				point[1] < 2 ||
				point[2] < 2
				);
	}

private:

	//! Computes spacing in x,y,z-directions.
	void compute_ddx_dddx();

};

#endif // VOLUME_H