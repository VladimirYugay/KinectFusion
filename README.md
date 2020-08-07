## Kinect Fusion


### Project overview

The goal of this project is to obtain a 3D reconstruction of a static scene which was recorded with a Kinect camera. The Kinect camera provides RGB as well as depth information as can be seen in the picture below.
![here](readme_images/readme_image.jpg)

### Results

Below you can see a result of a raycast which was performed after ~120 frames.
![here](readme_images/raycasted_result2.png)

### How to run the code?

#### Setup (Linux)

- If you're contributing, please use `cpplint`\
  Install via: `pip install cpplint`\
  Run with: `cpplint [OPTIONS] files`

- Install `cmake` if it's not installed yet\
  `sudo apt-get install cmake`

- Run `sudo apt-get install libfreeimage3 libfreeimage-dev` in order to be able to process images from the dataset by simulating the virtal sensor

- Clone the repo\
  `git clone https://github.com/VladimirYugay/KinectFusion.git`

- Download header only Eigen library and put it into `libs` folder by runnning:\
  `cd KinectFusion/libs`\
  `git clone https://gitlab.com/libeigen/eigen.git`

- Download the dataset from [here](/home/vladimir/university/3d_scanning_and_motion_capture/3DSaMC/data/exercise_1_data) and save it under `KinectFusion/data` folder

- Build and run the project \
  `cd ..`\
  `mkdir build`\
  `cd build`\
  `cmake ..`\
  `make`\
  `./kinect_fusion`

#### Setup (Windows)

- Follow the same file structure as in Linux Setup
- Add pre-compiled libraries of FreeImage in `KinectFusion/libs`

#### Setup (Mac)
- `brew install eigen`
- `brew install freeimage`

#### Download dataset
You can either use your own image sequence which was recorded with a Kinect Camera v2 or you can download the same dataset we used [here](https://vision.in.tum.de/rgbd/dataset/freiburg1/rgbd_dataset_freiburg1_xyz.tgz). 
Extract the folder and move it to `KinectFusion/data/rgbd_dataset_freiburg1_xyz`.
