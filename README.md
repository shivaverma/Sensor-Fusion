## Basic Info

- Library: OpenCV 4.1
- Language: C++
- Plateform: Mac

## Project 1 | Lidar Obstacle Detection

- Created a pipiline for `segmentation` and `clustering` of the real time `PCD` data.
- Real time `PCD` data is streamed to the pipeline.
- Data is downsampled using `Voxel-Grid`.
- Implemented `Ransac Algorithm` for segmentation of road from obstacles.
- Implemented `Euclidean Clustering Algorithm` using `KD-Tree` for clusteration of the obstacles.

This is how the results looks like:

<img src=Lidar-Obstacle-Detection/media/result1.gif width="500">

<img src=Lidar-Obstacle-Detection/media/result2.gif width="500">

## Project 2 | Feature Tracking 2D

- Implemented `Keypoint Detection`, `Description Extraction` and `Keypoints Matching` algorithms.
- First `keypoints` are located in the images.
- Using these keypoints their `descriptors` are extracted.
- Same keypoints are `matched` using their `descriptors` in different images.

<img src="Feature-Tracking-2D/images/keypoints.png" width="820" height="248" />

Following is a detailed analysis of algorithms used for keypoints detection, descriptor extraction and keypoints matching. 

<img src="Feature-Tracking-2D/images/analysis.png" width="820"/>


