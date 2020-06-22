#include <iostream>
#include <fstream>
#include <array>
#include <vector> 

#include "helpers/Eigen.h"

#include "helpers/VirtualSensor.h"


int main(){
	// Make sure this path points to the data folder
	std::string filenameIn = "../data/";

	// load video
	std::cout << "Initialize virtual sensor..." << std::endl;
	VirtualSensor sensor;
	if (!sensor.Init(filenameIn)){
		std::cout << "Failed to initialize the sensor!\nCheck file path!" << std::endl;
		return -1;
	}

	while (sensor.ProcessNextFrame()){
		float* depthMap = sensor.GetDepth();
		BYTE* colorMap = sensor.GetColorRGBX();

		Matrix3f depthIntrinsics = sensor.GetDepthIntrinsics();
		float fX = depthIntrinsics(0, 0);
		float fY = depthIntrinsics(1, 1);
		float cX = depthIntrinsics(0, 2);
		float cY = depthIntrinsics(1, 2);

		Matrix4f depthExtrinsicsInv = sensor.GetDepthExtrinsics().inverse();

		Matrix4f trajectory = sensor.GetTrajectory();
		Matrix4f trajectoryInv = sensor.GetTrajectory().inverse();

		MatrixXf identity = MatrixXf::Identity(4, 3);
		Matrix3f depthIntrinsicsInv = depthIntrinsics.inverse();

		int depth_height = sensor.GetDepthImageHeight();
		int depth_width = sensor.GetDepthImageWidth();	

		std::cout << "Processing frame!" << std::endl;	
	}
	return 0;
}
