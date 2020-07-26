#include "Volume.h"

#define MAX_TRUNCATION 1.0f
#define MIN_TRUNCATION 1.0f

Volume::Volume() {}

//! Initializes an empty volume dataset.
Volume::Volume(Vector3f& min_, Vector3f& max_, uint dx_, uint dy_, uint dz_, uint dim)
{
	min = min_;
	max = max_;
	diag = max - min;
	dx = dx_;
	dy = dy_;
	dz = dz_;
	m_dim = dim;
	vol = NULL;

	vol = new Voxel[dx * dy * dz];

	zeroOutMemory();
	compute_ddx_dddx();
}

Volume::~Volume()
{
	delete[] vol;
};

//! Computes spacing in x,y,z-directions.
void Volume::compute_ddx_dddx()
{
	ddx = 1.0f / (dx - 1);
	ddy = 1.0f / (dy - 1);
	ddz = 1.0f / (dz - 1);

	dddx = (max[0] - min[0]) / (dx - 1);
	dddy = (max[1] - min[1]) / (dy - 1);
	dddz = (max[2] - min[2]) / (dz - 1);

	if (dz == 1)
	{
		ddz = 0;
		dddz = 0;
	}

	diag = max - min;
}

//! Zeros out the memory
void Volume::zeroOutMemory()
{
	for (uint i1 = 0; i1 < dx * dy * dz; i1++)
		vol[i1] = Voxel(std::numeric_limits<float>::max(), 0.0f);
}

//! Returns the Data.
Voxel* Volume::getData()
{
	return vol;
};

//! Sets new min and max points
void Volume::setNewBoundingPoints(Vector3f& min_, Vector3f& max_)
{
	min = min_;
	max = max_;
	zeroOutMemory();
	compute_ddx_dddx();
}

// estimate the normal for a point in voxel grid coordinates using voxel grid by calculating the numerical derivative of TSDF
Vector3f Volume::calculateNormal(const Vector3f& point) {
	Vector3f shiftedXup, shiftedXdown, shiftedYup, shiftedYdown, shiftedZup, shiftedZdown;
	float x_dir, y_dir, z_dir;
	Vector3f normal;

	shiftedXup = point;
	shiftedXup[0] += 1;
	shiftedXdown = point;
	shiftedXdown[0] -= 1;

	shiftedYup = point;
	shiftedYup[0] += 1;
	shiftedYdown = point;
	shiftedYdown[0] -= 1;

	shiftedZup = point;
	shiftedZup[0] += 1;
	shiftedZdown = point;
	shiftedZdown[0] -= 1;

	float sdfXup = trilinearInterpolation(shiftedXup);
	float sdfXdown = trilinearInterpolation(shiftedXdown);

	float sdfYup = trilinearInterpolation(shiftedYup);
	float sdfYdown = trilinearInterpolation(shiftedYdown);

	float sdfZup = trilinearInterpolation(shiftedYup);
	float sdfZdown = trilinearInterpolation(shiftedYdown);

	x_dir = (sdfXup - sdfXdown) / (2 *ddx);
	y_dir = (sdfYup - sdfYdown) / (2 *ddy);
	z_dir = (sdfZup - sdfZdown) / (2 *ddz);

	normal = Vector3f{ x_dir, y_dir, z_dir };
	normal.normalize();

	return normal;
}

// trilinear interpolation of a point in voxel grid coordinates to get SDF at the point
float Volume::trilinearInterpolation(const Vector3f& p) {
	Vector3i start = intCoords(p);
	float c000, c001, c010, c011, c100, c101, c110, c111;

	c000 = get(start[0] + 0, start[1] + 0, start[2] + 0).getValue();
	c100 = get(start[0] + 1, start[1] + 0, start[2] + 0).getValue();
	c001 = get(start[0] + 0, start[1] + 0, start[2] + 1).getValue();
	c101 = get(start[0] + 1, start[1] + 0, start[2] + 1).getValue();
	c010 = get(start[0] + 0, start[1] + 1, start[2] + 0).getValue();
	c110 = get(start[0] + 1, start[1] + 1, start[2] + 0).getValue();
	c011 = get(start[0] + 0, start[1] + 1, start[2] + 1).getValue();
	c111 = get(start[0] + 1, start[1] + 1, start[2] + 1).getValue();

	if (
		c000 == std::numeric_limits<float>::max() ||
		c001 == std::numeric_limits<float>::max() ||
		c010 == std::numeric_limits<float>::max() ||
		c011 == std::numeric_limits<float>::max() ||
		c100 == std::numeric_limits<float>::max() ||
		c101 == std::numeric_limits<float>::max() ||
		c110 == std::numeric_limits<float>::max() ||
		c111 == std::numeric_limits<float>::max()
	)
		return std::numeric_limits<float>::max();

	float xd, yd, zd;

	xd = p[0] - start[0]; //(p[0] - start[0]) / (start[0] + 1 - start[0]);
	yd = p[1] - start[1]; //(p[1] - start[1]) / (start[1] + 1 - start[1]);
	zd = p[2] - start[2]; //(p[1] - start[2]) / (start[2] + 1 - start[2]);

	float c00, c01, c10, c11;

	c00 = c000 * (1 - xd) + c100 * xd;
	c01 = c001 * (1 - xd) + c101 * xd;
	c10 = c010 * (1 - xd) + c110 * xd;
	c11 = c011 * (1 - xd) + c111 * xd;

	float c0, c1;

	c0 = c00 * (1 - yd) + c10 * yd;
	c1 = c01 * (1 - yd) + c11 * yd;

	float c;

	c = c0 * (1 - zd) + c1 * zd;

	return c;
}

// using given frame calculate TSDF values for all voxels in the grid
void Volume::integrate(Frame frame) {
	const Matrix4f worldToCamera = frame.getExtrinsicMatrix();
	const Matrix4f cameraToWorld = worldToCamera.inverse();
	const Matrix3f intrinsic = frame.getIntrinsicMatrix();
	Vector3f translation = cameraToWorld.block(0, 3, 3, 1);
	const float* depthMap = frame.getDepthMap();
	const BYTE* colorMap = frame.getColorMap();
	int width = frame.getFrameWidth();
	int height = frame.getFrameHeight();

	//std::cout << intrinsic << std::endl;

	// subscripts: g - global coordinate system | c - camera coordinate system | i - image space
	// short notations: V - vector | P - point | sdf - signed distance field value | tsdf - truncated sdf
	Vector3f Pg, Pc, ray, normal;
	Vector2i Pi;
	float depth, lambda, sdf, tsdf, tsdf_weight, value, weight, cos_angle;
	uint index;

    std::cout << "CUDA CALL: " << std::endl;
    // these values and weights are equival of Voxel* vol
    // This is a temporary fix since I don't want to break the interface
    // of all other components by modifying Voxel class for CUDA
    float* values = (float*)malloc(sizeof(float) * dx * dy * dz);
    float* weights = (float*)malloc(sizeof(float) * dx * dy * dz);
	for (int k = 0; k < dz; k++) {
		for (int j = 0; j < dy; j++) {
			for (int i = 0; i < dx; i++) {
                uint idx = i * dy * dz + j * dz + k;
                values[idx] = vol[idx].getValue();
                weights[idx] = vol[idx].getWeight();
            }
        }
    }

    CUDA::integrate(
        min, max, dx, dy, dz,
        worldToCamera, intrinsic,
        width, height,
        depthMap,
        values, weights);

	std::cout << "Integrate starting..." << std::endl;

	for (int k = 0; k < dz; k++) {
		for (int j = 0; j < dy; j++) {
			for (int i = 0; i < dx; i++) {

				// project the grid point into image space
				Pg = gridToWorld(i, j, k);
				Pc = frame.projectPointIntoFrame(Pg);
				Pi = frame.projectOntoImgPlane(Pc);

				//std::cout << Pg << std::endl << Pc << std::endl << Pi << std::endl;

				// Pg = gridToWorld(i, j, k);
				// Pc = Frame::transformPoint(Pg, worldToCamera);
				// Pi = Frame::perspectiveProjection(Pc, intrinsic);

				//std::cout << Pg << std::endl << Pc << std::endl << Pi << std::endl;

				if (frame.containsImgPoint(Pi)) {
                    printf("Pixels on CPU %d, %d \n", Pi[0], Pi[1]);
					// get the depth of the point
					index = Pi[1] * width + Pi[0];
					depth = depthMap[index];

					if (depth == MINF)
						continue;

					//std::cout << "Odbok!!\n";

					// calculate the sdf value
					lambda = (Pc / Pc[2]).norm();

					sdf = depth - ((Pg - translation) / lambda).norm();

					// compute the weight as the angle between the ray f qrom the voxel point and normal of the associated frame point devided by depth
					ray = Pg.normalized();
					normal = frame.getNormal(index);

					cos_angle = ray.dot(normal) / ray.norm() / normal.norm();

					tsdf_weight = 1; //cos_angle / depth; // 1; // 1 / depth;

					// get the previous value and weight
					value = vol[getPosFromTuple(i, j, k)].getValue();
					weight = vol[getPosFromTuple(i, j, k)].getWeight();

					// if we are doing the integration for the first time
					if (value == std::numeric_limits<float>::max()) {
						value = 0;
						weight = 0;
					}

					// truncation of the sdf
					if (sdf > 0) {
						tsdf = std::min(1.0f, sdf / MAX_TRUNCATION);
					}
					else {
						tsdf = std::max(-1.0f, sdf / MIN_TRUNCATION);
					}

					// the new value and weight is the running average
					vol[getPosFromTuple(i, j, k)].setValue((value * weight + tsdf * tsdf_weight) / (weight + tsdf_weight));
					vol[getPosFromTuple(i, j, k)].setWeight(weight + tsdf_weight);

					//std::cout << vol[getPosFromTuple(i, j, k)].getValue() << std::endl;
				}

			}
		}
	}

	std::cout << "Integrate done!" << std::endl;


}
