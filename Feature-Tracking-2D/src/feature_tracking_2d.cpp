/* INCLUDES FOR THIS PROJECT */
#include <iostream>
#include <fstream>
#include <sstream>
#include <iomanip>
#include <vector>
#include <cmath>
#include <limits>
#include <opencv2/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/xfeatures2d.hpp>
#include <opencv2/xfeatures2d/nonfree.hpp>

#include "dataStructures.h"
#include "matching2D.hpp"

using namespace std;

/* MAIN PROGRAM */
int main(int argc, const char *argv[])
{

    /* INIT VARIABLES AND DATA STRUCTURES */

    // data location
    string dataPath = "../../";
    string imgFileType = ".png";
    string imgBasePath = dataPath + "images/";
    string imgPrefix = "KITTI/2011_09_26/image_00/data/000000";
    
    string matcherType = "BF";              // BF, FLANN
    string DataType = "nBINARY";             // BINARY, HOG
    string selectorType = "KNN";            // NN, KNN
    string detectorType = "SIFT";          // SHITOMASI, HARRIS, FAST, BRISK, SIFT, ORB, AKAZE
    string descriptorType = "SIFT";        // BRIEF, ORB, FREAK, AKAZE, SIFT, BRISK

    
    int imgStartIndex = 0;               // first file index to load (assumes Lidar and camera names have identical naming convention)
    int imgEndIndex = 9;                 // last file index to load
    int imgFillWidth = 4;                // no. of digits which make up the file index (e.g. img-0001.png)

    int dataBufferSize = 2;              // no. of images which are held in memory (ring buffer) at the same time
    vector<DataFrame> dataBuffer;        // list of data frames which are held in memory at the same time
    bool vis = false;                    // visualize results

    /* MAIN LOOP OVER ALL IMAGES */

    for (size_t imgIndex = 0; imgIndex <= imgEndIndex - imgStartIndex; imgIndex++)
    {
        /* LOAD IMAGE INTO BUFFER */

        // assemble filenames for current index
        ostringstream imgNumber;
        imgNumber << setfill('0') << setw(imgFillWidth) << imgStartIndex + imgIndex;
        string imgFullFilename = imgBasePath + imgPrefix + imgNumber.str() + imgFileType;

        // load image from file and convert to grayscale
        cv::Mat img, imgGray;
        img = cv::imread(imgFullFilename);
        cv::cvtColor(img, imgGray, cv::COLOR_BGR2GRAY);

        // push image into data frame buffer
        int cur = imgIndex % dataBufferSize;
        int prev = (imgIndex + 1) % dataBufferSize;
    
        DataFrame frame;
        frame.cameraImg = imgGray;
        dataBuffer.insert(dataBuffer.begin() + cur, frame);
        
        cout << "#1 Image loading done" << endl << endl;

        /* DETECT IMAGE KEYPOINTS */

        // extract 2D keypoints from current image
        vector<cv::KeyPoint> keypoints;
        detKeypoints(keypoints, imgGray, detectorType);
 
        // only keep keypoints on the preceding vehicle
        bool focusOnVehicle = true;
        cv::Rect vehicleRect(535, 180, 180, 150);
        
        if (focusOnVehicle)
        {
            vector<int>index;
            for(int i=0;i<keypoints.size();i++)
            {
                if(!(keypoints[i].pt.x > vehicleRect.x && keypoints[i].pt.x < vehicleRect.x+vehicleRect.height && keypoints[i].pt.y > vehicleRect.y && keypoints[i].pt.y < vehicleRect.y+vehicleRect.width))
                {
                    index.push_back(i);
                }
            }
            for(int i=0;i<index.size();i++)
            {
                keypoints.erase(keypoints.begin() + index[i] - i);
            }
            
            cout << "#2 Dropped " << index.size() << " KeyPoints outside boundry." << " Left Keypoints: " << keypoints.size() << endl << endl;
            
        }
        // optional : limit number of keypoints (helpful for debugging and learning)
        bool limitKpts = false;
        if (limitKpts)
        {
            int maxKeypoints = 50;

            if (detectorType.compare("BRISK") == 0)
            { // there is no response info, so keep the first 50 as they are sorted in descending quality order
                keypoints.erase(keypoints.begin() + maxKeypoints, keypoints.end());
            }
            cv::KeyPointsFilter::retainBest(keypoints, maxKeypoints);
            cout << " NOTE: Keypoints have been limited!" << endl << endl;
        }

        // push keypoints and descriptor for current frame to end of data buffer
        (dataBuffer.begin() + cur)->keypoints = keypoints;
        cout << "#3 : Detect keypoints done" << endl << endl;

        /* EXTRACT KEYPOINT DESCRIPTORS */

        cv::Mat descriptors;
        descKeypoints((dataBuffer.begin() + cur)->keypoints, (dataBuffer.begin() + cur)->cameraImg, descriptors, descriptorType);
        //// EOF STUDENT ASSIGNMENT

        // push descriptors for current frame to end of data buffer
        (dataBuffer.begin() + cur)->descriptors = descriptors;

        cout << "#4 : Extract descriptors done" << endl << endl;

        if (dataBuffer.size() > 1) // wait until at least two images have been processed
        {
            /* MATCH KEYPOINT DESCRIPTORS */

            vector<cv::DMatch> matches;

            matchDescriptors((dataBuffer.begin() + prev)->keypoints, (dataBuffer.begin() + cur)->keypoints,
                             (dataBuffer.begin() + prev)->descriptors, (dataBuffer.begin() + cur)->descriptors,
                             matches, DataType, matcherType, selectorType);

            // store matches in current data frame
            (dataBuffer.begin() + cur)->kptMatches = matches;

            cout << "#4 : Match keypoints descriptors done" << endl << endl;

            // visualize matches between current and previous image
            vis = true;
            if (vis)
            {
                cv::Mat matchImg = ((dataBuffer.begin() + cur)->cameraImg).clone();
                cv::drawMatches((dataBuffer.begin() + prev)->cameraImg, (dataBuffer.begin() + prev)->keypoints,
                                (dataBuffer.begin() + cur)->cameraImg, (dataBuffer.begin() + cur)->keypoints,
                                matches, matchImg,
                                cv::Scalar::all(-1), cv::Scalar::all(-1),
                                vector<char>(), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);

                string windowName = "Matching keypoints between two camera images";
                cv::namedWindow(windowName, 7);
                cv::imshow(windowName, matchImg);
                cout << "Press key to continue to next image" << endl;
                cv::waitKey(0); // wait for key to be pressed
            }
            vis = false;
        }
    }
    return 0;
}
