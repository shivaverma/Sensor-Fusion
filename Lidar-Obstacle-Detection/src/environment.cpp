/* \author Aaron Brown */
// Create simple 3d highway enviroment using PCL
// for exploring self-driving car sensors

#include <cmath>
#include "quiz/cluster/kdtree.h"
#include "sensors/lidar.h"
#include "render/render.h"
#include "processPointClouds.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "processPointClouds.cpp"

// --------------------------------My Implementations of segmentation and clustering-----------------------------

std::tuple<int, int, int> RandomTriple(int size)
{
    int index1 = rand()%size, index2, index3;
    while(1)
    {
        index2 = rand()%size;
        if(index1!=index2)
            break;
    }
    while(1)
    {
        index3 = rand()%size;
        if(index3!=index2 && index3!=index1)
            break;
    }
    std::tuple<int,int,int> triple = {index1, index2, index3};
    return triple;
}

std::pair<pcl::PointCloud<pcl::PointXYZ>::Ptr, pcl::PointCloud<pcl::PointXYZ>::Ptr> Ransac3D(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int maxIterations, float distanceTol)
{
    auto startTime = std::chrono::steady_clock::now();
    
    std::unordered_set<int> inliersResult;
    srand(time(NULL));
    
    // TODO: Fill in this function
    
    int best_a, best_b, best_c, best_d;
    int max_inliners = 0;
    int size = cloud->points.size();
    
    // For max iterations
    for(int i=0;i<maxIterations;i++)
    {
        int inliners = 0;
        // Randomly sample subset and fit line
        std::tuple<int, int, int> ind = RandomTriple(size);
        
        int ind1 = std::get<0>(ind);
        int ind2 = std::get<1>(ind);
        int ind3 = std::get<2>(ind);
        
        float a = (cloud->points[ind2].y-cloud->points[ind1].y)*(cloud->points[ind3].z-cloud->points[ind1].z)
        - (cloud->points[ind2].z-cloud->points[ind1].z)*(cloud->points[ind3].y-cloud->points[ind1].y);
        
        float b = - (cloud->points[ind2].x-cloud->points[ind1].x)*(cloud->points[ind3].z-cloud->points[ind1].z)
        + (cloud->points[ind2].z-cloud->points[ind1].z)*(cloud->points[ind3].x-cloud->points[ind1].x);
        
        float c = (cloud->points[ind2].x-cloud->points[ind1].x)*(cloud->points[ind3].y-cloud->points[ind1].y)
        - (cloud->points[ind2].y-cloud->points[ind1].y)*(cloud->points[ind3].x-cloud->points[ind1].x);
        
        float x = cloud->points[ind1].x;
        float y = cloud->points[ind1].y;
        float z = cloud->points[ind1].z;
        
        float d = - (a*x + b*y + c*z);
        
        // Measure distance between every point and fitted line
        for(int j=0;j<size;j++)
        {
            float div = sqrt(a*a + b*b + c*c);
            float dist = abs(cloud->points[j].x*a + cloud->points[j].y*b + cloud->points[j].z*c + d)/div;
            // If distance is smaller than threshold count it as inlier
            if(dist <= distanceTol)
                inliners++;
        }
        if(inliners > max_inliners)
        {
            best_a = a;
            best_b = b;
            best_c = c;
            best_d = d;
            max_inliners = inliners;
        }
    }
    
    // Return indicies of inliers from fitted line with most inliers
    for(int j=0;j<size;j++)
    {
        float div = sqrt(best_a*best_a + best_b*best_b + best_c*best_c);
        float dist = abs(cloud->points[j].x*best_a + cloud->points[j].y*best_b + cloud->points[j].z*best_c + best_d)/div;
        // If distance is smaller than threshold count it as inlier
        if(dist <= distanceTol)
            inliersResult.insert(j);
    }
    
    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "plane segmentation took " << elapsedTime.count() << " milliseconds" << std::endl;
        
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudInliers(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudOutliers(new pcl::PointCloud<pcl::PointXYZ>());
    
    for(int index = 0; index < cloud->points.size(); index++)
    {
        pcl::PointXYZ point = cloud->points[index];
        if(inliersResult.count(index))
            cloudInliers->points.push_back(point);
        else
            cloudOutliers->points.push_back(point);
    }
    
    std::pair<pcl::PointCloud<pcl::PointXYZ>::Ptr, pcl::PointCloud<pcl::PointXYZ>::Ptr> segCloud = {cloudOutliers, cloudInliers};
    
    return segCloud;
}

void createCluster(const std::vector<std::vector<float>>& points, pcl::PointCloud<pcl::PointXYZ>::Ptr &cluster, int *flag, KdTree* tree, float distanceTol, int i)
{
    if(flag[i] == 1)
        return;
    
    flag[i] = 1;
    
    pcl::PointXYZ z = {points[i][0], points[i][1], points[i][2]};
    cluster->points.push_back(z);
    
    std::vector<int> nearPoint = tree->search(points[i], distanceTol);
    
    for(int i=0;i<nearPoint.size();i++)
    {
        if(flag[nearPoint[i]] == 0)
            createCluster(points, cluster, flag, tree, distanceTol, nearPoint[i]);
    }
}

std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> euclideanCluster(const std::vector<std::vector<float>>& points, KdTree* tree, float distanceTol, int minSize)
{
    // TODO: Fill out this function to return list of indices for each cluster
    
    // Inserting point into KD-Tree
    for (int i=0; i<points.size(); i++)
        tree->insert(points[i],i);
    
   std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> clusters;
    
    int size = points.size();
    int flag[size];
    
    for(int i=0;i<size;i++)
        flag[i] = 0;
    
    for(int i=0;i<size;i++)
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cluster(new pcl::PointCloud<pcl::PointXYZ>);
        
        if(flag[i] == 0)
        {
            createCluster(points, cluster, flag, tree, distanceTol, i);
            if(cluster->points.size() >= minSize)
                clusters.push_back(cluster);
        }
    }
    return clusters;
}

// ----------------------------------------------------------------------------------------------------

std::vector<Car> initHighway(bool renderScene, pcl::visualization::PCLVisualizer::Ptr& viewer)
{

    Car egoCar( Vect3(0,0,0), Vect3(4,2,2), Color(0,1,0), "egoCar");
    Car car1( Vect3(15,0,0), Vect3(4,2,2), Color(0,0,1), "car1");
    Car car2( Vect3(8,-4,0), Vect3(4,2,2), Color(0,0,1), "car2");	
    Car car3( Vect3(-12,4,0), Vect3(4,2,2), Color(0,0,1), "car3");
  
    std::vector<Car> cars;
    cars.push_back(egoCar);
    cars.push_back(car1);
    cars.push_back(car2);
    cars.push_back(car3);

    if(renderScene)
    {
        renderHighway(viewer);
        egoCar.render(viewer);
        car1.render(viewer);
        car2.render(viewer);
        car3.render(viewer);
    }

    return cars;
}

void cityBlock(pcl::visualization::PCLVisualizer::Ptr &viewer, ProcessPointClouds<pcl::PointXYZ> pointProcessor, pcl::PointCloud<pcl::PointXYZ>::Ptr inputCloud)
{
    // first point is x-axis along with road, second y-axis, third z-axis
    Eigen::Vector4f minVec = Eigen::Vector4f(-10, -6.2, -2, 1);
    Eigen::Vector4f maxVec = Eigen::Vector4f(25, 7, 10, 1);
    
    inputCloud = pointProcessor.FilterCloud(inputCloud, .3   , minVec, maxVec);
    
    // renderPointCloud(viewer, filterCloud, "filterCloud");
    
    std::pair<pcl::PointCloud<pcl::PointXYZ>::Ptr, pcl::PointCloud<pcl::PointXYZ>::Ptr> segCloud = Ransac3D(inputCloud, 900, .4);
    
    // renderPointCloud(viewer, segCloud.first, "obsCloud", Color(1,0,0));
    renderPointCloud(viewer, segCloud.second, "planeCloud", Color(0,1,0));
    
    KdTree* tree = new KdTree;
    
    std::vector<std::vector<float>> points;
    
    for(int i=0;i<segCloud.first->points.size();i++)
    {
        std::vector<float> v;
        v.push_back(segCloud.first->points[i].x);
        v.push_back(segCloud.first->points[i].y);
        v.push_back(segCloud.first->points[i].z);
        points.push_back(v);
    }
    
    // Inserting points into KD-Tree
    for(int i=0; i<points.size(); i++)
        tree->insert(points[i], i);
    
    // std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> clusters = pointProcessor.Clustering(segCloud.first, 1.5, 3, 30);
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> clusters = euclideanCluster(points, tree, .4, 8);
    
    int clusterId = 0;
    std::vector<Color> colors = {Color(1,0,0), Color(1,1,0), Color(0,0,1)};
    
    bool render_box = true;
    
    for(pcl::PointCloud<pcl::PointXYZ>::Ptr cluster: clusters)
    {
        std::cout<<"Cluster size: ";
        pointProcessor.numPoints(cluster);
        renderPointCloud(viewer, cluster, "obsCloud"+std::to_string(clusterId), colors[clusterId % colors.size()]);
        
        if(render_box)
        {
            Box box = pointProcessor.BoundingBox(cluster);
            renderBox(viewer, box, clusterId);
        }
        clusterId++;
    }
}


void simpleHighway(pcl::visualization::PCLVisualizer::Ptr& viewer)
{
    // ----------------------------------------------------
    // -----Open 3D viewer and display simple highway -----
    // ----------------------------------------------------
    
    // RENDER OPTIONS
    bool renderScene = false;
    std::vector<Car> cars = initHighway(renderScene, viewer);
    
    // TODO:: Create lidar sensor
    
    Lidar* lidar = new Lidar(cars, 0);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;
    cloud = lidar->scan();
    
    // renderRays(viewer, lidar->position, cloud);
    // renderPointCloud(viewer, cloud, "pcd_data");
    
    // TODO:: Create point processor
    ProcessPointClouds<pcl::PointXYZ> pointProcessor;
    std::pair<pcl::PointCloud<pcl::PointXYZ>::Ptr, pcl::PointCloud<pcl::PointXYZ>::Ptr> segCloud = Ransac3D(cloud, 100, 0.2);
    
    // renderPointCloud(viewer, segCloud.first, "obsCloud", Color(1,0,0));
    // renderPointCloud(viewer, segCloud.second, "planeCloud", Color(0,1,0));
    
    KdTree* tree = new KdTree;

    std::vector<std::vector<float>> points;
    
    for(int i=0;i<segCloud.first->points.size();i++)
    {
        std::vector<float> v;
        v.push_back(segCloud.first->points[i].x);
        v.push_back(segCloud.first->points[i].y);
        v.push_back(segCloud.first->points[i].z);
        points.push_back(v);
    }
    
    // Inserting points into KD-Tree
    for(int i=0; i<points.size(); i++)
        tree->insert(points[i], i);
    
    // std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> clusters = pointProcessor.Clustering(segCloud.first, 1.5, 3, 30);
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> clusters = euclideanCluster(points, tree, 2, 3);
    
    int clusterId = 0;
    std::vector<Color> colors = {Color(1,0,0), Color(1,1,0), Color(0,0,1)};
    
    bool render_box = true;
    
    for(pcl::PointCloud<pcl::PointXYZ>::Ptr cluster: clusters)
    {
        std::cout<<"Cluster size: ";
        pointProcessor.numPoints(cluster);
        renderPointCloud(viewer, cluster, "obsCloud"+std::to_string(clusterId), colors[clusterId % colors.size()]);
        
        if(render_box)
        {
            Box box = pointProcessor.BoundingBox(cluster);
            renderBox(viewer, box, clusterId);
        }
        clusterId++;
    }
}


//setAngle: SWITCH CAMERA ANGLE {XY, TopDown, Side, FPS}
void initCamera(CameraAngle setAngle, pcl::visualization::PCLVisualizer::Ptr& viewer)
{

    viewer->setBackgroundColor (0, 0, 0);
    
    // set camera position and angle
    viewer->initCameraParameters();
    // distance away in meters
    int distance = 16;
    
    switch(setAngle)
    {
        case XY : viewer->setCameraPosition(-distance, -distance, distance, 1, 1, 0); break;
        case TopDown : viewer->setCameraPosition(0, 0, distance, 1, 0, 1); break;
        case Side : viewer->setCameraPosition(0, -distance, 0, 0, 0, 1); break;
        case FPS : viewer->setCameraPosition(-10, 0, 0, 0, 0, 1);
    }

    if(setAngle!=FPS)
        viewer->addCoordinateSystem (1.0);
}


int main (int argc, char** argv)
{
    std::cout << "starting enviroment" << std::endl;

    pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    CameraAngle setAngle = XY;
    initCamera(setAngle, viewer);
    
    // simpleHighway(viewer);
    // cityBlock(viewer);

    pcl::PointCloud<pcl::PointXYZ>::Ptr inputCloud;
    ProcessPointClouds<pcl::PointXYZ> pointProcessor;
    
    std::vector<boost::filesystem::path> stream = pointProcessor.streamPcd("../../src/sensors/data/pcd/data_1");
    auto streamIterator = stream.begin();
    
    while (!viewer->wasStopped ())
    {
        viewer->removeAllPointClouds();
        viewer->removeAllShapes();
        
        inputCloud= pointProcessor.loadPcd((*streamIterator).string());
        cityBlock(viewer, pointProcessor, inputCloud);
        
        streamIterator++;
        if(streamIterator == stream.end())
            streamIterator = stream.begin();
        
        viewer->spinOnce ();
    } 
}
