#ifndef REGISTRATION_GICP_HPP
#define REGISTRATION_GICP_HPP
#include <iostream>
#include <Eigen/Dense>
#include <Eigen/Core>
#include <ros/ros.h>
#include "std_msgs/String.h"
#include <pcl/point_cloud.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/registration/icp.h> 
#include <pcl/io/pcd_io.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/registration/ndt.h>
#include <cmath>
#include <vector>
#include <pcl/registration/gicp.h>
#include <pcl/filters/passthrough.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/point_types.h>
#include <eigen3/Eigen/Core>
#include <fast_gicp/gicp/fast_gicp.hpp>


class Regiestration_Gicp
{
public:
    Regiestration_Gicp()
    {
        Teach_pcd_path=ros::param::param<std::string>("Teach_pcd_path","/home/firefly/Documents/catkin_test/src/livox_mapping/data");
        Trans_path=ros::param::param<std::string>("Trans_path","/media/liwen/KK/Wen_Li/Key_frame/teach_keyframe/keyframe_trans.txt");
        repeat_ds_size=ros::param::param<double>("repeat_ds_size",0.25);
        teach_ds_size=ros::param::param<double>("teach_ds_size",0.25);
        gicp_threads=ros::param::param<int>("gicp_threads",4);

        last_frameNum=-1;
        cloud_teach.reset(new pcl::PointCloud<pcl::PointXYZ>());
        cloud_repeat_downsampled.reset(new pcl::PointCloud<pcl::PointXYZ>());
        cloud_teach_downsampled.reset(new pcl::PointCloud<pcl::PointXYZ>());
        cloud_aligned.reset(new pcl::PointCloud<pcl::PointXYZ>());
        Pre_trans=Eigen::Matrix4f::Identity();

        gicp.setNumThreads(gicp_threads);
        end_frame=false;

        Eigen::Matrix4d pose_Ti=Eigen::Matrix4d::Identity();
        ifstream inFile;
        std::string s;
        inFile.open(Trans_path);
        if (!inFile) 
        {
            std::cout << "Unable to open file";
            exit(1); 
        }
        int num=0;
        while(getline(inFile,s))
        {
            num++;
            string s_sigle;  
            istringstream isstring(s); 
            for(int p=0;p<3;p++)
            {
                for(int q=0;q<4;q++)
                {
                    isstring>>s_sigle;
                    pose_Ti(p,q)=stod(s_sigle);
                }
            }
            Eigen::Isometry3d pose_i(pose_Ti);
            Trans_teach.push_back(pose_i);   
        }
        inFile.close();
    }

    double compute_error_now() 
    {
        Eigen::Isometry3d T1=Eigen::Isometry3f(icp_matrix4f).cast<double>();
        return sqrt(gicp.compute_error(T1))/cloud_repeat_downsampled->size();
    }

    bool get_transToteach(int now_frameNum,pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_repeat)
    {
        if (now_frameNum==0)
        {   
            if (now_frameNum==last_frameNum)
            {
                voxelSampler.setInputCloud(cloud_repeat);
                voxelSampler.setLeafSize((float)repeat_ds_size,(float)repeat_ds_size, (float)repeat_ds_size);
                voxelSampler.filter(*cloud_repeat_downsampled);
                gicp.clearSource();
                gicp.setInputSource(cloud_repeat_downsampled);
            }
            else
            {
                std::string keyframe_pointcloud_filename(Teach_pcd_path+"/pointcloud"  + std::to_string(now_frameNum)+".pcd");
                if (pcl::io::loadPCDFile<pcl::PointXYZ>(keyframe_pointcloud_filename,*cloud_teach) == -1)
                {
                    cout<<"could not find pcd"<<endl;
                    // ros::shutdown();
                    return false;
                }
                voxelSampler.setInputCloud(cloud_repeat);
                voxelSampler.setLeafSize((float)repeat_ds_size,(float)repeat_ds_size, (float)repeat_ds_size);
                voxelSampler.filter(*cloud_repeat_downsampled);
                voxelSampler.setInputCloud(cloud_teach);
                voxelSampler.setLeafSize((float)teach_ds_size, (float)teach_ds_size, (float)teach_ds_size);
                voxelSampler.filter(*cloud_teach_downsampled);
                // statFilter.setInputCloud(cloud_repeat_downsampled);
                // statFilter.setMeanK(10);
                // statFilter.setStddevMulThresh(0.2);
                // statFilter.filter(*cloud_repeat_downsampled);
                gicp.clearSource();
                gicp.clearTarget();
                gicp.setInputSource(cloud_repeat_downsampled);
                gicp.setInputTarget(cloud_teach_downsampled);
                // int size;
                // if(cloud_repeat_downsampled->size()>cloud_teach_downsampled->size()){
                // 	size=cloud_teach_downsampled->size();
                // }
                // else{
                // 	size=cloud_repeat_downsampled->size();
                // }
                // if(size<1500)
                // {
                // 	size=1500;
                // }
                // float dis=size*0.00025;
                // gicp.setMaxCorrespondenceDistance(dis);
                gicp.setMaxCorrespondenceDistance(1.0);
            }
		    gicp.align(*cloud_aligned,Pre_trans);
            last_frameNum=now_frameNum;
        }
        else
        {
            if (now_frameNum==last_frameNum)
            {
                Pre_trans=icp_matrix4f;
                voxelSampler.setInputCloud(cloud_repeat);
                voxelSampler.setLeafSize((float)repeat_ds_size,(float)repeat_ds_size, (float)repeat_ds_size);
                voxelSampler.filter(*cloud_repeat_downsampled);
                gicp.clearSource();
                gicp.setInputSource(cloud_repeat_downsampled);
            }
            else 
            {
                pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_teach2(new pcl::PointCloud<pcl::PointXYZ>());
                std::string keyframe_pointcloud_filename2(Teach_pcd_path+"/pointcloud"  + std::to_string(now_frameNum+1)+".pcd");
                if (pcl::io::loadPCDFile<pcl::PointXYZ>(keyframe_pointcloud_filename2,*cloud_teach2) == -1)
                {
                    cout<<"The Last Frame!!!!!!!!!!!!!!!!!"<<endl;
                    end_frame=true;
                    // ros::shutdown();
                    // return false;
                }

                Pre_trans=Trans_teach[now_frameNum-1].matrix().inverse().cast<float>()*icp_matrix4f;
                std::string keyframe_pointcloud_filename(Teach_pcd_path+"/pointcloud"  + std::to_string(now_frameNum)+".pcd");
                if (pcl::io::loadPCDFile<pcl::PointXYZ>(keyframe_pointcloud_filename,*cloud_teach) == -1)
                {
                    cout<<"could not find pcd"<<endl;
                    // ros::shutdown();
                    return false;
                }
                voxelSampler.setInputCloud(cloud_repeat);
                voxelSampler.setLeafSize((float)repeat_ds_size,(float)repeat_ds_size, (float)repeat_ds_size);
                voxelSampler.filter(*cloud_repeat_downsampled);
                voxelSampler.setInputCloud(cloud_teach);
                voxelSampler.setLeafSize((float)teach_ds_size, (float)teach_ds_size, (float)teach_ds_size);
                voxelSampler.filter(*cloud_teach_downsampled);
                // statFilter.setInputCloud(cloud_repeat_downsampled);
                // statFilter.setMeanK(10);
                // statFilter.setStddevMulThresh(0.2);
                // statFilter.filter(*cloud_repeat_downsampled);
                gicp.clearSource();
                gicp.clearTarget();
                gicp.setInputSource(cloud_repeat_downsampled);
                gicp.setInputTarget(cloud_teach_downsampled);
                // int size;
                // if(cloud_repeat_downsampled->size()>cloud_teach_downsampled->size()){
                // 	size=cloud_teach_downsampled->size();
                // }
                // else{
                // 	size=cloud_repeat_downsampled->size();
                // }
                // if(size<1500)
                // {
                // 	size=1500;
                // }
                // float dis=size*0.00025;
                // gicp.setMaxCorrespondenceDistance(dis);
                gicp.setMaxCorrespondenceDistance(1.0);
            }
            // std::cout<<"Trans_teach:"<<Trans_teach[now_frameNum-1].matrix().cast<float>()<<std::endl;
            // std::cout<<"icp_matrix4f before:"<<icp_matrix4f<<std::endl;
            std::cout<<"Pre_tcrans:"<<Pre_trans<<std::endl;
            gicp.align(*cloud_aligned,Pre_trans);
            last_frameNum=now_frameNum;
        }
        icp_matrix4f=gicp.getFinalTransformation();
        std::cout<<"icp_matrix4f after:"<<icp_matrix4f<<std::endl;
        return true;
    }

protected:
    std::string Teach_pcd_path,Trans_path;
    int last_frameNum,gicp_threads;
    double repeat_ds_size,teach_ds_size;

    pcl::VoxelGrid<pcl::PointXYZ> voxelSampler;
    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> statFilter;
    fast_gicp::FastGICP<pcl::PointXYZ, pcl::PointXYZ> gicp;

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_teach;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_repeat_downsampled;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_teach_downsampled;
    pcl::PointCloud<pcl::PointXYZ>::Ptr  cloud_aligned; 

    Matrix4f Pre_trans;
    std::vector<Eigen::Isometry3d, Eigen::aligned_allocator<Eigen::Isometry3d>> Trans_teach;

public:
    
    bool end_frame;
    Matrix4f icp_matrix4f;

};

#endif