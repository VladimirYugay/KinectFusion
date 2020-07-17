#pragma once

#include "../models/Volume.h"
#include "../models/Frame.h"

class RayCaster {
private:
	Volume vol;
	Frame frame;

public:

	RayCaster();
	RayCaster(Volume& vol);
	RayCaster(Volume& vol, Frame& frame);

	void changeFrame(Frame& frame);
	void changeVolume(Volume& vol);
	Frame& rayCast();
};