#ifndef SMPG_H
#define SMPG_H

#include <iostream>
#include <cmath>
#include <vector>
#include <set>
#include <utility>
#include <algorithm>
#include <boost/format.hpp>
#include <std_msgs/String.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/kdtree/kdtree_flann.h>

using namespace std;

class PointInfo_SAM{ 
    template<typename PointT> friend class SMPG;
public:
    PointInfo_SAM();
    PointInfo_SAM(int self_idx, double eps, pcl::PointCloud<pcl::PointXYZI>::Ptr input_cloud, pcl::search::KdTree<pcl::PointXYZI>::Ptr tree);
    void setSearchMethod(pcl::search::KdTree<pcl::PointXYZI>::Ptr tree) {search_method = tree;}  
    void calc_eps();
    void get_nearPoint_info();

    void PointAngle(){
       double z = input_cloud->points[self_idx].z;
       double xy_dist = sqrt(input_cloud->points[self_idx].x * input_cloud->points[self_idx].x + input_cloud->points[self_idx].y * input_cloud->points[self_idx].y);
       this -> laserId = round(atan2(z, xy_dist) * 180/M_PI);
    }

protected:
    int self_idx;
    double eps;
    int laserId;

    pcl::search::KdTree<pcl::PointXYZI>::Ptr search_method;
    pcl::PointCloud<pcl::PointXYZI>::Ptr input_cloud;
    vector<int> near_point; //indices로 near point 인덱스들 넣기로 수정
    vector<float> sqr_dist; //near point 거리
};

template <typename PointT>
class SMPG {
public:
    typedef typename pcl::PointCloud<PointT>::Ptr PointCloudPtr;
    typedef typename pcl::search::KdTree<PointT>::Ptr KdTreePtr;

    KdTreePtr search_method;

    SMPG();
    void setInputCloud(PointCloudPtr cloud) {input_cloud = cloud;}
    void setSearchMethod(KdTreePtr tree) {search_method = tree;}    
    void setClusterTolerance(double tolerance) {eps = tolerance;}
    void setUpsampleStepSize(double stepSize) {upsampleStepSize = stepSize;}
    void initializer_pointinfo();
    void upsample(PointCloudPtr cloud);

private:
    PointCloudPtr input_cloud;
    PointCloudPtr channelCloudPtr[16];
    KdTreePtr grid[16];

    double eps = 0;
    double upsampleStepSize = 0.1;
    vector<PointInfo_SAM> PCinfo;
};

#endif //smpg


PointInfo_SAM::PointInfo_SAM(){};

PointInfo_SAM::PointInfo_SAM(int self_idx_, double eps_, pcl::PointCloud<pcl::PointXYZI>::Ptr input_cloud_, pcl::search::KdTree<pcl::PointXYZI>::Ptr tree)
    :self_idx(self_idx_), eps(eps_), input_cloud(input_cloud_), search_method(tree)
{
    near_point.push_back(self_idx);
    calc_eps();
    PointAngle();
}

void PointInfo_SAM::calc_eps(){
    double safe_coeff = 0.05; //오차를 고려한 안전계수
    double cur_dist = sqrt(input_cloud->points[self_idx].x * input_cloud->points[self_idx].x 
                         + input_cloud->points[self_idx].y * input_cloud->points[self_idx].y 
                         + input_cloud->points[self_idx].z * input_cloud->points[self_idx].z);
    this -> eps = 0.037 * cur_dist; //실험적으로 구한 dist에 따른 eps함수
    if(cur_dist > 2){ 
        this -> eps += safe_coeff; //가까이에선 거의 오차가 없음. 오히려 기존 eps보다 안전계수가 커져버려서 과부하 발생
    }
    this -> eps = 0.05 * cur_dist;
}

template <typename PointT>
SMPG<PointT>::SMPG(){
        for(int i=0;i<16;i++){
            channelCloudPtr[i].reset(new pcl::PointCloud<pcl::PointXYZI>());
            grid[i].reset(new pcl::search::KdTree<pcl::PointXYZI>());
        }
}

template <typename PointT>
void SMPG<PointT>::initializer_pointinfo(){
    for(int i = 0; i < input_cloud -> points.size(); i++){
        PointInfo_SAM tmp(i, eps, input_cloud, search_method);
        PCinfo.push_back(tmp);
        //채널로 분류된 포인트를 배열에 넣는다.
        channelCloudPtr[(tmp.laserId + 15) / 2]->push_back(input_cloud->points[tmp.self_idx]);        
    }

    //channelCloud 내부 포인트를 grid 트리 내부에 넣는다.
    for (int i = 0; i < 16; i++){
        grid[i]->setInputCloud(channelCloudPtr[i]);
    }
}

template <typename PointT>
void SMPG<PointT>::upsample(PointCloudPtr cloud){
    initializer_pointinfo();

    //for (int i = 0; i < 16; i++)
        //cout << "channel " << i << " : " << channelCloudPtr[i]->points.size() << endl;

    // intensity로 채널 부여
    for(int i = 0; i < input_cloud -> points.size(); i++){
        this->input_cloud->points[i].intensity = PCinfo[i].laserId;
    }

    for(int i = 0; i < input_cloud -> points.size(); i++){//모든 포인트에 대해서 연산
        //다른 채널인 것 중 가장 가까운 포인트 찾기
        vector<int> idx;
        vector<float> sqr_dist; //near point "제곱!!!"거리
        if((PCinfo[i].laserId + 15) / 2 + 1 == 16) continue;
        if(channelCloudPtr[(PCinfo[i].laserId + 15) / 2 + 1]->points.size() == 0) continue;
        grid[(PCinfo[i].laserId + 15) / 2 + 1]->nearestKSearch(input_cloud->points[i], 1, idx, sqr_dist);
        if(idx.size()==0) continue;
        if(sqrt(sqr_dist[0]) > PCinfo[i].eps) continue;

        double height = upsampleStepSize; //0.05m
        int line_num = sqrt(sqr_dist[0]) / height;

        //스템 사이즈 맞춰 포인트 추가        
        double line_dist_x, line_dist_y, line_dist_z;
        if (line_num != 0) {
            line_dist_x = abs(input_cloud->points[i].x - channelCloudPtr[(PCinfo[i].laserId + 15) / 2 + 1]->points[idx[0]].x) / (line_num + 1);
            line_dist_y = abs(input_cloud->points[i].y - channelCloudPtr[(PCinfo[i].laserId + 15) / 2 + 1]->points[idx[0]].y) / (line_num + 1);
            line_dist_z = abs(input_cloud->points[i].z - channelCloudPtr[(PCinfo[i].laserId + 15) / 2 + 1]->points[idx[0]].z) / (line_num + 1);
        }
        for(int j = 0; j < line_num; j++){
            double jmp_dist_x = (j+1) * line_dist_x, jmp_dist_y = (j+1) * line_dist_y, jmp_dist_z = (j+1) * line_dist_z;
            pcl::PointXYZI tmp;
            tmp.x = input_cloud->points[i].x + jmp_dist_x;
            tmp.y = input_cloud->points[i].y + jmp_dist_y;
            tmp.z = input_cloud->points[i].z + jmp_dist_z;
            tmp.intensity = input_cloud->points[i].intensity;
            cloud->push_back(tmp);
        }
    }
}

/*
//radius search 기반 upsample 알고리즘
template <typename PointT>
void SMPG<PointT>::upsample(PointCloudPtr cloud){
    initializer_pointinfo();

    for (int i = 0; i < 16; i++)
        cout << "channel " << i << " : " << channelCloud[i].points.size() << endl;

    // 채널 부여
    // for(int i = 0; i < input_cloud -> points.size(); i++){
    //     this->input_cloud->points[i].intensity = PCinfo[i].laserId;
    // }

    for(int i = 0; i < input_cloud -> points.size(); i++){//모든 포인트에 대해서 연산


        //다른 채널인 것 중 가장 가까운 포인트 찾기
        int nnPoint_index_IN_NEARPOINT = 0;
        int mini = 99999;
        int line_num = 0;
        double height = 0.05; //0.05m
        //cout<<"i =  "<< i << " near point size = " << PCinfo[i].near_point.size()<<endl;

        for(int j = 1; j < PCinfo[i].near_point.size(); j++){
            if((   PCinfo[i].laserId    < PCinfo[   PCinfo[i].near_point[j]   ].laserId) ){
                double ucl_dist = sqrt(pow(input_cloud->points[i].x - input_cloud->points[PCinfo[i].near_point[j]].x,2) + pow(input_cloud->points[i].y - input_cloud->points[PCinfo[i].near_point[j]].y,2)+ pow(input_cloud->points[i].z - input_cloud->points[PCinfo[i].near_point[j]].z,2));
                if(ucl_dist > mini) continue;
                //double diff_dist = PCinfo[i].sqr_dist[j];
                //cout << "j = "<<j <<"     "<<PCinfo[i].laserId <<"      "<<PCinfo[PCinfo[i].near_point[j]].laserId<<endl;
                line_num = ucl_dist / height;
                nnPoint_index_IN_NEARPOINT = j;
                mini = ucl_dist;
                //cout << "ucl_dist : " << ucl_dist << " line_num: " << line_num << endl;
                //cout<<"real_dist : " <<ucl_dist<<endl;

            }
        }

        //스템 사이즈 맞춰 포인트 추가
        
        int nnPoint_index_REAL = PCinfo[i].near_point[nnPoint_index_IN_NEARPOINT];
        double line_dist_x, line_dist_y, line_dist_z;
        if (line_num != 0) {
            line_dist_x = abs(input_cloud->points[i].x - input_cloud->points[nnPoint_index_REAL].x) / (line_num + 1);
            line_dist_y = abs(input_cloud->points[i].y - input_cloud->points[nnPoint_index_REAL].y) / (line_num + 1);
            line_dist_z = abs(input_cloud->points[i].z - input_cloud->points[nnPoint_index_REAL].z) / (line_num + 1);
        }
        for(int j = 0; j < line_num; j++){
            double jmp_dist_x = (j+1) * line_dist_x, jmp_dist_y = (j+1) * line_dist_y, jmp_dist_z = (j+1) * line_dist_z;
            pcl::PointXYZI tmp;
            tmp.x = input_cloud->points[i].x + jmp_dist_x;
            tmp.y = input_cloud->points[i].y + jmp_dist_y;
            tmp.z = input_cloud->points[i].z + jmp_dist_z;
            tmp.intensity = input_cloud->points[i].intensity;
            cloud->push_back(tmp);
        }
        

    }
    //*cloud = *input_cloud;
}
*/
