#include "RayCaster.h"

//RayCaster::RayCaster() {}

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
	Vector4uc color;

	int width = frame.getFrameWidth();
	int height = frame.getFrameHeight();
;
	Vector3f ray_start, ray_dir, ray_current, ray_previous, ray_next;
	Vector3i ray_current_int, ray_previous_int;

	std::shared_ptr<std::vector<Vector3f>> output_vertices_global = std::make_shared<std::vector<Vector3f>>(std::vector<Vector3f>());
	output_vertices_global->reserve(width * height);

	std::shared_ptr<std::vector<Vector3f>> output_normals_global = std::make_shared<std::vector<Vector3f>>(std::vector<Vector3f>());
	output_normals_global->reserve(width * height);

	std::shared_ptr<std::vector<Vector3f>> output_vertices, output_normals;

	float sdf_1, sdf_2;
	Vector3f p, v, n;
	uint index;

	std::cout << "RayCast starting..." << std::endl;

	for (int i = 0; i < height; i++) {
		for (int j = 0; j < width; j++) {
			//std::cout << i << " " << j << std::endl;

			// starting point is the position of the camera (translation) in grid coordinates
			ray_start = vol.worldToGrid(translation);

			// calculate the direction vector as vector from camera position to the pixel(i, j)s world coordinates
			index = i * width + j;
			color = Vector4uc{
				frame.colorMap[4 * index + 0],
				frame.colorMap[4 * index + 1],
				frame.colorMap[4 * index + 2],
				frame.colorMap[4 * index + 3] 
			};
			/*
			ray_dir = Vector3f{ float(j), float(i), 1.0f };
			ray_dir = intrinsic_inverse * ray_dir;
			ray_dir = rotationMatrix * ray_dir;
			ray_dir = ray_dir.normalized();
			*/
			//ray_next = vol.worldToGrid(frame.getVertexGlobal(index));

			ray_next = Vector3f{ float(j), float(i), 1.0f };
			ray_next = intrinsic_inverse * ray_next;
			ray_next = rotationMatrix * ray_next + translation;
			ray_next = vol.worldToGrid(ray_next);
			
			ray_dir = ray_next - ray_start;
			ray_dir = ray_dir.normalized();
			
			if (!ray_dir.allFinite() || ray_dir == Vector3f{ 0.0f, 0.0f, 0.0f }) {
				mistake(*output_vertices_global, *output_normals_global);
				continue;
			}

			Ray ray = Ray(ray_start, ray_dir);

			ray_current = ray_start;
			ray_current_int = Volume::intCoords(ray_current);

			if (!vol.isPointInVolume(ray_current)) {
				mistake(*output_vertices_global, *output_normals_global);
				continue;
			}

			while (true) {//vol.isPointInVolume(ray_current)) {
				ray_previous = ray_current;
				ray_previous_int = ray_current_int;

				do {
					//std::cout << ray_current << std::endl;

					ray_current = ray.next();
					ray_current_int = Volume::intCoords(ray_current);

				} while (ray_previous_int == ray_current_int);

					
				if (!vol.isInterpolationPossible(ray_previous) || !vol.isInterpolationPossible(ray_current)) {
					mistake(*output_vertices_global, *output_normals_global);
					break;
				} 
				
				else if (vol.get(ray_previous_int).getValue() == 0) {
					v = vol.gridToWorld(ray_previous);
					//n = vol.calculateNormal(ray_previous);

					/*
					if (n == Vector3f{ MINF, MINF, MINF }) {
						mistake(*output_vertices_global, *output_normals_global);
						break;
					}
					*/
					output_vertices_global->emplace_back(v);
					//output_normals_global->emplace_back(n);

					if (!vol.voxelVisited(ray_previous)) {
						//vol.updateColor(ray_previous_int, color, true);
						vol.setVisited(ray_previous_int);
					}
						
					break;
				}

				else if (vol.get(ray_current_int).getValue() == 0){
					v = vol.gridToWorld(ray_current);
					//n = vol.calculateNormal(ray_current);
					/*
					if (n == Vector3f{ MINF, MINF, MINF }) {
						mistake(*output_vertices_global, *output_normals_global);
						break;
					}
					*/
					output_vertices_global->emplace_back(v);
					//output_normals_global->emplace_back(n);

					if (!vol.voxelVisited(ray_current)) {
						//vol.updateColor(ray_previous_int, color, true);
						vol.setVisited(ray_current_int);
					}

					break;
				}
				
				else if (
					vol.get(ray_previous_int).getValue() != std::numeric_limits<float>::max() && 
					vol.get(ray_previous_int).getValue() > 0 &&  
					vol.get(ray_current_int).getValue() != std::numeric_limits<float>::max() &&
					vol.get(ray_current_int).getValue() < 0
				) {
					sdf_1 = vol.trilinearInterpolation(ray_previous);
					sdf_2 = vol.trilinearInterpolation(ray_current);

					if (sdf_1 == std::numeric_limits<float>::max() || sdf_2 == std::numeric_limits<float>::max() || sdf_2 == sdf_1) {
						mistake(*output_vertices_global, *output_normals_global);
						break;
					}

					p = ray_previous - (ray_dir * sdf_1) / (sdf_2 - sdf_1);

					if (!vol.isInterpolationPossible(p)) {
						mistake(*output_vertices_global, *output_normals_global);
						break;
					}

					//std::cout << ray_previous << std::endl << ray_current << std::endl << ray_dir << std::endl << sdf_1 << " " << sdf_2 << std::endl << p << std::endl;
					v = vol.gridToWorld(p);
					//n = vol.calculateNormal(p);
					/*
					if (n == Vector3f{ MINF, MINF, MINF }) {
						mistake(*output_vertices_global, *output_normals_global);
						break;
					}
					*/
					output_vertices_global->emplace_back(v);
					//output_normals_global->emplace_back(n);

					if (!vol.voxelVisited(ray_previous)) {
						vol.setVisited(ray_previous_int);
					}

					if (!vol.voxelVisited(ray_current))
						vol.setVisited(ray_current_int);

					//std::cout << p << std::endl;
					//if(!vol.voxelVisited(p))
					//	vol.updateColor(p, color, true);
					//else 
					//	vol.updateColor(p, color, false);

					break;
				}
			}
		}			
	}

	frame.mVerticesGlobal = output_vertices_global;
	//frame.mNormalsGlobal = output_normals_global;
	frame.mVertices = std::make_shared<std::vector<Vector3f>>(frame.transformPoints(*output_vertices_global, worldToCamera));
	frame.computeNormalMap(width, height);
	frame.mNormalsGlobal = std::make_shared<std::vector<Vector3f>>(frame.rotatePoints(frame.getNormalMap(), rotationMatrix));

	std::cout << "RayCast done!" << std::endl;

	return frame;
}