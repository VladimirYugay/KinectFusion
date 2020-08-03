#pragma once

#ifndef RAY_H
#define RAY_H

#include "Eigen.h"

class Ray {
private:
	Vector3f start;
	Vector3f direction;
	Vector3f currentPoint;
	Vector3f previousPoint;

public:
	Ray();
	Ray(Vector3f& start_, Vector3f& direction_);

	Vector3f& next();

	Vector3f& getStartingPoint();
	void setStartingPoint(Vector3f& start_);

	Vector3f& getDirection();
	void setDirection(Vector3f& direction_);

	Vector3f& getCurrentPosition();
	Vector3f& getPreviousPosition();
};

#endif //RAY_H