#include <numeric>
#include "matching2D.hpp"

using namespace std;

// Find best matches for keypoints in two camera images based on several matching methods
void matchDescriptors(std::vector<cv::KeyPoint> &kPtsSource, std::vector<cv::KeyPoint> &kPtsRef, cv::Mat &descSource, cv::Mat &descRef,
                      std::vector<cv::DMatch> &matches, std::string descriptorType, std::string matcherType, std::string selectorType)
{
    // configure matcher
    bool crossCheck = false;
    cv::Ptr<cv::DescriptorMatcher> matcher;

    if (matcherType.compare("BF") == 0)
    {
        int normType = descriptorType.compare("BINARY") == 0 ? cv::NORM_HAMMING : cv::NORM_L2;
        matcher = cv::BFMatcher::create(normType, crossCheck);
    }
    else if (matcherType.compare("FLANN") == 0)
    {
        descSource.convertTo(descSource, CV_32F);
        descRef.convertTo(descRef, CV_32F);
        matcher = cv::FlannBasedMatcher::create();
    }
 
    // perform matching task
    double t = (double)cv::getTickCount();

    if (selectorType.compare("NN") == 0)
    {
        matcher->match(descSource, descRef, matches); // Finds the best match for each descriptor in desc1
    }
    else if (selectorType.compare("KNN") == 0)
    {
        vector<vector<cv::DMatch>> knn_matches;
        matcher->knnMatch(descSource, descRef, knn_matches, 2);
        double minDescDistRatio = 0.8;
        for (auto it = knn_matches.begin(); it != knn_matches.end(); ++it)
        {
            if ((*it)[0].distance < minDescDistRatio * (*it)[1].distance)
            {
                matches.push_back((*it)[0]);
            }
        }
        cout << "Matched Keypoints = " << matches.size() << endl << endl;
    }
    t = ((double)cv::getTickCount() - t) / cv::getTickFrequency();
    cout <<  selectorType << " matches in " << 1000 * t / 1.0 << " ms" << endl;
}

// Use one of several types of state-of-art descriptors to uniquely identify keypoints
void descKeypoints(vector<cv::KeyPoint> &keypoints, cv::Mat &img, cv::Mat &descriptors, string descriptorType)
{
    // select appropriate descriptor
    cv::Ptr<cv::DescriptorExtractor> extractor;
    if (descriptorType.compare("BRISK") == 0)
    {

        int threshold = 30;        // FAST/AGAST detection threshold score.
        int octaves = 3;           // detection octaves (use 0 to do single scale)
        float patternScale = 1.0f; // apply this scale to the pattern used for sampling the neighbourhood of a keypoint.
        extractor = cv::BRISK::create(threshold, octaves, patternScale);
    }
    else if(descriptorType.compare("SIFT") == 0)
    {
        extractor = cv::xfeatures2d::SiftDescriptorExtractor::create();
    }
    else if(descriptorType.compare("ORB") == 0)
    {
        extractor = cv::ORB::create();
    }
    else if(descriptorType.compare("AKAZE") == 0)
    {
        extractor = cv::AKAZE::create();
    }
    else if(descriptorType.compare("BRIEF") == 0)
    {
        extractor = cv::xfeatures2d::BriefDescriptorExtractor::create();
    }
    else if(descriptorType.compare("FREAK") == 0)
    {
        extractor = cv::xfeatures2d::FREAK::create();
    }
    
    // perform feature description
    double t = (double)cv::getTickCount();
    extractor->compute(img, keypoints, descriptors);
    t = ((double)cv::getTickCount() - t) / cv::getTickFrequency();
    cout << descriptorType << " descriptor extraction in " << 1000 * t / 1.0 << " ms" << endl << endl;
}

// Detect keypoints in image using the traditional different detector
void detKeypoints(vector<cv::KeyPoint> &keypoints, cv::Mat &img, std::string name)
{
    bool vis = false;
    double t = (double)cv::getTickCount();
    
    if(name.compare("SHITOMASI") == 0)
    {
        // Apply corner detection
        vector<cv::Point2f> corners;
        
        double k = 0.04;
        int blockSize = 4;                                             // size of an average block for computing a derivative covariation matrix over each pixel neighborhood
        double maxOverlap = 0.0;                                       // max. permissible overlap between two features in %
        double qualityLevel = 0.01;                                    // minimal accepted quality of image corners
        double minDistance = (1.0 - maxOverlap) * blockSize;
        int maxCorners = img.rows * img.cols / max(1.0, minDistance);  // max. num. of keypoints
        cv::goodFeaturesToTrack(img, corners, maxCorners, qualityLevel, minDistance, cv::Mat(), blockSize, false, k);
        
        // add corners to result vector
        for (auto it = corners.begin(); it != corners.end(); ++it)
        {
            cv::KeyPoint newKeyPoint;
            newKeyPoint.pt = cv::Point2f((*it).x, (*it).y);
            newKeyPoint.size = blockSize;
            keypoints.push_back(newKeyPoint);
        }
    }
    else if(name.compare("HARRIS") == 0)
    {
        int blockSize = 2;     // for every pixel, a blockSize × blockSize neighborhood is considered
        int apertureSize = 3;  // aperture parameter for Sobel operator (must be odd)
        int minResponse = 100; // minimum value for a corner in the 8bit scaled response matrix
        double k = 0.04;       // Harris parameter (see equation for details)
        
        // Detect Harris corners and normalize output
        cv::Mat dst;
        dst = cv::Mat::zeros(img.size(), CV_32FC1);
        cv::cornerHarris(img, dst, blockSize, apertureSize, k, cv::BORDER_DEFAULT);
        
        float maxOverlap = 0;
        
        for(int i = 0;i<dst.rows;i++)
        {
            for(int j=0;j<dst.cols;j++)
            {
                int response = (int)dst.at<float>(i, j);
                if(response > minResponse)
                {
                    cv::KeyPoint newKeyPoint;
                    newKeyPoint.pt = cv::Point2f(j, i);
                    newKeyPoint.size = 2*apertureSize;
                    newKeyPoint.response = response;
                    
                    bool overlap = false;
                    for(auto iter = keypoints.begin();iter!=keypoints.end();iter++)
                    {
                        double kptOverlap = cv::KeyPoint::overlap(newKeyPoint, *iter);
                        if(kptOverlap > maxOverlap)
                        {
                            overlap = true;
                            if(newKeyPoint.response > (*iter).response)
                            {
                                *iter = newKeyPoint;
                                break;
                            }
                        }
                    }
                    if(!overlap)
                    {
                        keypoints.push_back(newKeyPoint);
                    }
                }
            }
        }
    }
    else if(name.compare("FAST") == 0)
    {
        int threshold = 30;
        bool nms = true;
        
        cv::FastFeatureDetector::DetectorType type = cv::FastFeatureDetector::TYPE_9_16;
        cv::Ptr<cv::FastFeatureDetector> detector = cv::FastFeatureDetector::create(threshold, nms, type);
        detector->detect(img, keypoints);
    }
    else if (name.compare("BRISK") == 0)
    {
        cv::Ptr<cv::FeatureDetector> detector = cv::BRISK::create();
        detector->detect(img, keypoints);
    }
    else if (name.compare("SIFT") == 0)
    {
        cv::Ptr<cv::FeatureDetector> detector = cv::xfeatures2d::SIFT::create();
        detector->detect(img, keypoints);
    }
    else if (name.compare("ORB") == 0)
    {
        cv::Ptr<cv::FeatureDetector> detector = cv::ORB::create();
        detector->detect(img, keypoints);
    }
    else if (name.compare("AKAZE") == 0)
    {
        cv::Ptr<cv::FeatureDetector> detector = cv::AKAZE::create();
        detector->detect(img, keypoints);
    }
    
    t = ((double)cv::getTickCount() - t) / cv::getTickFrequency();
    cout << name << " detection with n = " << keypoints.size() << " keypoints in " << 1000 * t / 1.0 << " ms" << endl << endl;

    // visualize results
    if (vis)
    {
        cv::Mat visImage = img.clone();
        cv::drawKeypoints(img, keypoints, visImage, cv::Scalar::all(-1), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
        string windowName = name + " Corner Detector Results";
        cv::namedWindow(windowName, 6);
        imshow(windowName, visImage);
        cv::waitKey(0);
    }
}
