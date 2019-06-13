## Circular Buffer

- Implemented circular buffer using vector.
- Value is pushed in vector at first and second index alternatively.
- Previous and current index are switched alternatively in vector.

## Detectors

- Implemented HARRIS, FAST, BRISK, ORB, AKAZE, and SIFT Detectors using OpenCV.
- You can choose your type of detectors by passing it's name in function argument of type string.

## Keypoints Removal

- Removed keypoints outside rectangle area by checking the X and Y coordinates of keypoints.

## Descriptors

- Implemented BRIEF, ORB, FREAK, AKAZE and SIFT descriptors using OpenCV.
- You can choose your type of descriptors by passing it's name in function argument of type string.

## Descriptor Matching

- Implemented Brute Force and FLANN method for keypoints matching.
- Implemented NN and KNN selection methods.

## Top 3 Detector-Discriptor Combination

- FAST + BRIEF (19 ms)
- FAST + BRISK (26 ms)
- FAST + ORB (27 ms)

All other information is stored in the excel sheet.