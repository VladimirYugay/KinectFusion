#include "Ray.h"
#include "Eigen.h"

Ray::Ray() {}

Ray::Ray(Vector3f& start_, Vector3f& direction_) : start(start_), direction(direction_) {
	currentPoint = start_;
	direction.normalize();
}

Vector3f& Ray::next() {
	previousPoint = currentPoint;
	currentPoint += direction;

	return currentPoint;
}

Vector3f& Ray::getStartingPoint() {
	return start;
}
void Ray::setStartingPoint(Vector3f& start_) {
	start = start_;
	currentPoint = start_;
}

Vector3f& Ray::getDirection() {
	return direction;
}
void Ray::setDirection(Vector3f& direction_) {
	direction = direction_;
}

Vector3f& Ray::getCurrentPosition() {
	return currentPoint;
}
Vector3f& Ray::getPreviousPosition() {
	return previousPoint;
}