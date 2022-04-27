#include <iostream>
#include <cmath>
#include <vector>
#include <set>
#include <utility>
#include <algorithm>
#include <string>
#include <time.h>
#include <ros/ros.h>
#include <boost/format.hpp>
#include <std_msgs/String.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/common.h>
#include <pcl/common/centroid.h>
#include <pcl/common/transforms.h>
#include <pcl/console/parse.h>
#include <pcl/console/time.h>
#include <pcl/features/integral_image_normal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/statistical_outlier_removal.h>   //noise filtering
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/features/normal_3d.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/surface/mls.h>
#include <pcl/surface/impl/mls.hpp>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>

#include "DBSCAN_smpg.h"
#include "sampling.h"

using namespace std;

typedef pcl::PointCloud<pcl::PointXYZ> PCXYZ;
typedef pcl::PointCloud<pcl::PointXYZI> PCXYZI;
typedef pcl::PointXYZ PXYZ;
typedef pcl::PointXYZI PXYZI;

ros::Publisher pub; 
ros::Publisher pub_us; 
float ROI_xMin = 0, ROI_xMax = 8, ROI_yMin = -1, ROI_yMax = 1, ROI_zMin = -1, ROI_zMax = 2;
bool switch_ROI = 1;
double US_searchRadius = 0.03, US_upsamplingRadius = 0.03, US_upsamplingStepSize = 0.02;

inline float cal_dist(float x, float y){ return sqrt(x*x+y*y); }
inline float MidPt(float a, float b){ return (a + b) / 2; }
inline void print_coord(PXYZI tmp){ cout << fixed << setprecision(3) <<"dist : " << cal_dist(tmp.x,tmp.y) << "    x : "<<tmp.x << "    y : "<< tmp.y <<endl; }
inline void print_OBJ(vector<PXYZI>& sorted_OBJ){ for(int i = 0; i < sorted_OBJ.size(); i++) print_coord(sorted_OBJ[i]); }

void ROI(PCXYZI::Ptr rawData, PCXYZI::Ptr outputData){
    if(!switch_ROI) return;
    for(unsigned int j = 0; j<rawData->points.size(); j++){     //actual ROI setting
        float *x = &rawData->points[j].x, *y = &rawData->points[j].y, *z = &rawData->points[j].z;
        if(*x > ROI_xMin && *x < ROI_xMax && *y > ROI_yMin && *y < ROI_yMax && *z > ROI_zMin && *z < ROI_zMax){
            outputData->push_back(rawData->points[j]);
        }        
    }
}

// void UpSampling(PCXYZI::Ptr TotalCloud, PCXYZI::Ptr upsampledCloud){
//     PCXYZI Data_for_voxel;
//     pcl::MovingLeastSquares<PXYZI, PXYZI> filter;
//     pcl::search::KdTree<PXYZI>::Ptr kdtree;
    
//     cout << "PointCloud BEFORE US: \"" << TotalCloud->points.size ()  << "\" , \t" ; 

//     Data_for_voxel = *TotalCloud;
//     //copyPointCloud(TotalCloud, Data_for_voxel);
//     filter.setInputCloud(Data_for_voxel.makeShared());
//     filter.setSearchMethod(kdtree);
//     filter.setComputeNormals (true);
//     filter.setSearchRadius(US_searchRadius);       // Use all neighbors in a radius of 3cm.
//     filter.setUpsamplingMethod(pcl::MovingLeastSquares<PXYZI, PXYZI>::SAMPLE_LOCAL_PLANE);
//     filter.setUpsamplingRadius(US_upsamplingRadius);   // Radius around each point, where the local plane will be sampled.
//     filter.setUpsamplingStepSize(US_upsamplingStepSize); // Sampling step size. Bigger values will yield less (if any) new points.
//     filter.process(*upsampledCloud);
//     cout << "AFTER US: " << upsampledCloud->points.size ()  << " data points." << endl; 

//     sensor_msgs::PointCloud2 output; 
//     pcl::PCLPointCloud2 tmp_PCL;                               //declare PCL_PC2
//     pcl::toPCLPointCloud2(*upsampledCloud, tmp_PCL);           //PC -> PCL_PC2
//     pcl_conversions::fromPCL(tmp_PCL, output);                 //PCL_PC2 -> sensor_msg_PC2
//     output.header.frame_id = "velodyne";
//     pub_us.publish(output);

//     // // estimate normals
//     // pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>);

//     // pcl::IntegralImageNormalEstimation<PXYZI, pcl::Normal> ne;
//     // ne.setNormalEstimationMethod (ne.AVERAGE_3D_GRADIENT);
//     // ne.setMaxDepthChangeFactor(0.02f);
//     // ne.setNormalSmoothingSize(10.0f);
//     // ne.setInputCloud(upsampledCloud);
//     // ne.compute(*normals);

//     // // visualize normals
//     // pcl::visualization::PCLVisualizer viewer("PCL Viewer");
//     // viewer.setBackgroundColor (0.0, 0.0, 0.5);
//     // viewer.addPointCloudNormals<pcl::PointXYZI,pcl::Normal>(upsampledCloud, normals);
    

//     // while(!viewer.wasStopped()) { viewer.spinOnce (100); 
//     // boost::this_thread::sleep(boost::posix_time::microseconds (100000)); }
    
// }

void UPSMP(PCXYZI::Ptr input_cloud, PCXYZI::Ptr upsampledCloud){
    pcl::search::KdTree<PXYZI>::Ptr tree(new pcl::search::KdTree<PXYZI>);
    tree->setInputCloud(input_cloud);

    PCXYZI::Ptr new_point(new PCXYZI);
    SMPG<PXYZI> smp;
    smp.setCorePointMinPts(3);
    smp.setClusterTolerance(0.05);
    smp.setMinClusterSize(5);
    smp.setMaxClusterSize(25000);
    smp.setSearchMethod(tree);
    smp.setInputCloud(input_cloud);
    smp.upsample(new_point);
    *upsampledCloud = *new_point + *input_cloud;
    cout << "Input Cloud : " << input_cloud->points.size();
    cout << "\t New Points : " << new_point->points.size() << endl;
    cout << "Total Points : " << upsampledCloud->points.size() << endl;



    sensor_msgs::PointCloud2 output; 
    pcl::PCLPointCloud2 tmp_PCL;                               //declare PCL_PC2
    pcl::toPCLPointCloud2(*upsampledCloud, tmp_PCL);                //PC -> PCL_PC2
    pcl_conversions::fromPCL(tmp_PCL, output);                 //PCL_PC2 -> sensor_msg_PC2
    output.header.frame_id = "velodyne";
    pub_us.publish(output);
}

void smp(const sensor_msgs::PointCloud2ConstPtr& rawdata){

    PCXYZI::Ptr tmp(new PCXYZI);
    PCXYZI::Ptr input_cloud(new PCXYZI);
    PCXYZI::Ptr ROI_cloud(new PCXYZI);

    pcl::fromROSMsg(*rawdata,*tmp);
    ROI(tmp,ROI_cloud);

    UPSMP(ROI_cloud, input_cloud);
    //input_cloud = ROI_cloud;

    pcl::search::KdTree<PXYZI>::Ptr tree(new pcl::search::KdTree<PXYZI>);
    tree->setInputCloud(input_cloud);
    
    std::vector<pcl::PointIndices> cluster_indices;
    DBSCANSMP<PXYZI> DB;
    DB.setCorePointMinPts(3);
    DB.setClusterTolerance(0.05);
    DB.setMinClusterSize(5);
    DB.setMaxClusterSize(25000);
    DB.setSearchMethod(tree);
    DB.setInputCloud(input_cloud);
    DB.extract(cluster_indices);
    
    PCXYZI::Ptr cloud_clustered(new PCXYZI);
    vector<PXYZI> sorted_OBJ; 
    int j = 0;
    // visualization, use indensity to show different color for each cluster.
    for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin(); it != cluster_indices.end(); it++, j++) {
        pair<float,float> x(9999,-9999); //first = min, second = max
        pair<float,float> y(9999,-9999);
        pair<float,float> z(9999,-9999); 
        for(std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); ++pit) {
            PXYZI pt = input_cloud->points[*pit];
            pt.intensity = j%8;
            cloud_clustered->push_back(pt);
            if(pt.x < x.first)      x.first = pt.x;
            if(pt.x > x.second)     x.second = pt.x;
            if(pt.y < y.first)      y.first = pt.y ;
            if(pt.y > y.second)     y.second = pt.y; 
            if(pt.z < z.first)      z.first = pt.z;
            if(pt.z > z.second)     z.second = pt.z;  
        }
        PXYZI* tmp = new PXYZI(); //i dont know intensity initial format
        tmp->x = MidPt(x.first,x.second); tmp->y = MidPt(y.first,y.second); tmp->z = MidPt(z.first,z.second);
        sorted_OBJ.push_back(*tmp);
    }
    print_OBJ(sorted_OBJ);

    sensor_msgs::PointCloud2 output; 
    pcl::PCLPointCloud2 tmp_PCL;                               //declare PCL_PC2
    pcl::toPCLPointCloud2(*cloud_clustered, tmp_PCL);           //PC -> PCL_PC2
    pcl_conversions::fromPCL(tmp_PCL, output);                 //PCL_PC2 -> sensor_msg_PC2
    output.header.frame_id = "velodyne";
    pub.publish(output);
}


int main(int argc, char** argv){
	ros::init(argc, argv, "test_sampling"); //node name 
	ros::NodeHandle nh;         //nodehandle

    nh.getParam("/smp_tst/ROI_xMin", ROI_xMin);
    nh.getParam("/smp_tst/ROI_xMax", ROI_xMax);
    nh.getParam("/smp_tst/ROI_yMin", ROI_yMin);
    nh.getParam("/smp_tst/ROI_yMax", ROI_yMax);
    nh.getParam("/smp_tst/ROI_zMin", ROI_zMin);
    nh.getParam("/smp_tst/ROI_zMax", ROI_zMax);
    nh.getParam("/smp_tst/US_searchRadius", US_searchRadius);
    nh.getParam("/smp_tst/US_upsamplingRadius", US_upsamplingRadius);
    nh.getParam("/smp_tst/US_upsamplingStepSize", US_upsamplingStepSize);

	ros::Subscriber sub = nh.subscribe<sensor_msgs::PointCloud2> ("/velodyne_points", 100, smp);
    pub = nh.advertise<sensor_msgs::PointCloud2> ("/1_velodyne_points_smp_algorithm", 100);
    pub_us = nh.advertise<sensor_msgs::PointCloud2> ("/2_velodyne_points_smp_algorithm_us", 100);
    
	ros::spin();
}
