#pragma once

#ifndef RAYCASTER_H
#define RAYCASTER_H

#include "Volume.h"
#include "Frame.h"

class RayCaster {
private:
	Volume vol;
	Frame frame;

public:
	RayCaster();
	RayCaster(Volume& vol_);
	RayCaster(Volume& vol_, Frame& frame_);

	void changeFrame(Frame& frame_);
	void changeVolume(Volume& vol_);
	Frame& rayCast();
};

#endif // RAYCASTER_H