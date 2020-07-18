#include "RayCaster.h"
#include "Volume.h"
#include "Frame.h"
#include "Ray.h"

RayCaster::RayCaster() {}

RayCaster::RayCaster(Volume& vol_) : vol(vol_) {}

RayCaster::RayCaster(Volume& vol_, Frame& frame_) : vol(vol_), frame(frame_) {}

void RayCaster::changeFrame(Frame& frame_) {
	frame = frame_;
}

void RayCaster::changeVolume(Volume& vol_) {
	vol = vol_;
}

// a function that writes down the invalid results
void mistake(
	std::vector<Vector3f>& ovg, std::vector<Vector3f>& ong
) {
	ovg.emplace_back(Vector3f(MINF, MINF, MINF));
	ong.emplace_back(Vector3f(MINF, MINF, MINF));
}

Frame& RayCaster::rayCast() {
	const Matrix4f worldToCamera = frame.getExtrinsicMatrix();
	const Matrix4f cameraToWorld = worldToCamera.inverse();
	const Matrix3f intrinsic_inverse = frame.getIntrinsicMatrix().inverse();
	Vector3f translation = cameraToWorld.block(0, 3, 3, 1);
	Matrix3f rotationMatrix = cameraToWorld.block(0, 0, 3, 3);

	int width = frame.getFrameWidth();
	int height = frame.getFrameHeight();
;
	Vector3f ray_start, ray_dir, ray_current, ray_previous;
	Vector3i ray_current_int, ray_previous_int;
	std::vector<Vector3f> output_vertices_global, output_normals_global;

	float sdf_1, sdf_2;
	Vector3f p, v, n;
	uint index;

	std::cout << "Ray Cast starting..." << std::endl;

	for (int i = 0; i < height; i++) {
		for (int j = 0; j < width; j++) {

			// starting point is the position of the camera (translation) in grid coordinates
			ray_start = vol.worldToGrid(translation);

			// calculate the direction vector as vector from camera position to the pixel(i, j)s world coordinates
			index = i * width + j;

			ray_dir = frame.getVertexMapGlobal()[index];
			ray_dir = vol.worldToGrid(ray_dir);

			ray_dir -= ray_start;
			ray_dir.normalize();

			Ray ray = Ray(ray_start, ray_dir);

			ray_current = ray_start;
			ray_current_int = Volume::intCoords(ray_current);

			if (!vol.isPointInVolume(ray_current)) {
				mistake(output_vertices_global, output_normals_global);
				continue;
			}

			while (vol.isPointInVolume(ray_current)) {
				do {
					ray_previous = ray_current;
					ray_previous_int == Volume::intCoords(ray_previous);

					ray_current = ray.next();
					ray_current_int = Volume::intCoords(ray_current);

				} while (ray_previous_int == ray_current_int);
					
				if (!vol.isInterpolationPossible(ray_current)) {
					mistake(output_vertices_global, output_normals_global);
					break;
				}

				if (vol.get(ray_previous_int).getValue() >= 0 && vol.get(ray_current_int).getValue() <= 0) {
					sdf_1 = vol.trilinearInterpolation(ray_previous);
					sdf_2 = vol.trilinearInterpolation(ray_current);

					if (sdf_1 == std::numeric_limits<float>::max() || sdf_2 == std::numeric_limits<float>::max()) {
						mistake(output_vertices_global, output_normals_global);
						break;
					}

					p = ray_previous - (ray_dir * sdf_1) / (sdf_2 - sdf_1);
					v = vol.gridToWorld(p);
					n = vol.calculateNormal(v);

					output_vertices_global.emplace_back(v);
					output_normals_global.emplace_back(n);
					break;
				}
			}
		}			
	}

	Frame result = Frame(frame);
	//result.mVerticesGlobal = output_vertices_global;
	//result.mNormalsGlobal = &output_normals_global;
	//result.mVertices = frame.transformPoints(output_vertices_global, worldToCamera);
	//result.mNormals = frame.rotatePoints(output_normals_global, rotationMatrix);

	std::cout << "Ray Cast done!" << std::endl;

	return result;
}