#pragma once

#ifndef RAYCASTER_H
#define RAYCASTER_H

#include "Volume.h"
#include "Frame.h"
#include "Ray.h"

class RayCaster {
private:
	Volume& vol;
	Frame frame;

public:

	RayCaster();
	RayCaster(Volume& vol);
	RayCaster(Volume& vol, Frame& frame);

	void changeFrame(Frame& frame);
	void changeVolume(Volume& vol);
	Frame& rayCast();
};

#endif // !RAYCASTER_H