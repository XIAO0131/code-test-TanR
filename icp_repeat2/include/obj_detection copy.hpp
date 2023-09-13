#ifndef OBJ_DETECTION_HPP
#define OBJ_DETECTION_HPP
#include<iostream>
#include <ros/ros.h>
#include <pcl/point_cloud.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/registration/ndt.h>
#include <cmath>
#include <vector>
#include <pcl/registration/gicp.h>
#include <pcl/filters/passthrough.h>
#include <pcl/sample_consensus/sac_model_perpendicular_plane.h>
#include <pcl/common/angles.h>
#include <pcl/segmentation/extract_clusters.h>

#include <unordered_map>

#include "nav_msgs/OccupancyGrid.h"

#include "AStar.hpp"

#define HASH_P 116101
#define MAX_N 10000000000
class VOXEL_LOC {
public:
  int64_t x, y, z;

  VOXEL_LOC(int64_t vx = 0, int64_t vy = 0, int64_t vz = 0)
      : x(vx), y(vy), z(vz) {}

  bool operator==(const VOXEL_LOC &other) const {
    return (x == other.x && y == other.y && z == other.z);
  }
};

// Hash value
namespace std {
template <> struct hash<VOXEL_LOC> {
  int64_t operator()(const VOXEL_LOC &s) const {
    using std::hash;
    using std::size_t;
    // return ((((s.z) * HASH_P) % MAX_N + (s.y)) * HASH_P) % MAX_N + (s.x);

	// std::cout<<s.x<<","<<s.y<<","<<s.z<<std::endl;
	// std::cout<<(((s.z) * HASH_P) % MAX_N ) + (s.x)<<std::endl;
	return ((((s.z) * HASH_P) % MAX_N ) + (s.x));
	

  }
};
} // namespace std

class OctoTree 
{
public:
  pcl::PointCloud<pcl::PointXYZ> contain_points;
  pcl::PointCloud<pcl::PointXYZ> nonground_points;
  pcl::PointCloud<pcl::PointXYZ> ground_points;
  pcl::PointXYZ lowest_point;
  int point_num;

  int is_ground;

  Eigen::Vector2d last_vec;

  OctoTree()
  {
	contain_points.clear();
	nonground_points.clear();
	ground_points.clear();
	lowest_point.x=10;
	lowest_point.y=10;
	lowest_point.z=10;
	point_num=0;
	is_ground=0;
	last_vec=Eigen::Vector2d(1,0);
  }
};

class segmatation_sel
{
public:
	segmatation_sel()
	{
		//接收pcl_output节点的数据，在cloudCB回调函数进行处理，以pcl_filtered节点发出去
		std::cout<<"segmatation初始化"<<std::endl;
		pcl_sub = nh.subscribe("/sunny_topic/tof_frame/pointcloud",10,&segmatation_sel::cloudCB, this);//定义接收者
		pcl_pub = nh.advertise<sensor_msgs::PointCloud2>("pcl_segmented",1);//定义发布者
		pass_pub = nh.advertise<sensor_msgs::PointCloud2>("pass_pointcloud",1);//定义发布者
		obj_cloud_pub = nh.advertise<sensor_msgs::PointCloud2>("obj_cloud",1);//定义发布者
		obj_cluster_cloud_pub = nh.advertise<sensor_msgs::PointCloud2>("obj_cluster_cloud",1);//定义发布者

		voxel_size=ros::param::param<float>("voxel_size",0.05f);

		obj_dis_near=ros::param::param<double>("obj_dis_near",0.5);
		obj_dis_far=ros::param::param<double>("obj_dis_far",2);

		tof_height= ros::param::param<double>("tof_height",-0.47);
		point_to_groundsym= ros::param::param<double>("point_to_groundsym",0.1);
		angle_the= ros::param::param<double>("angle_the",25);
		angle_the=cos(angle_the*3.14159/180);

		range_x=ros::param::param<double>("range_x",10);
		range_y=ros::param::param<double>("range_y",1.5);
		range_z=ros::param::param<double>("range_z",4);
		detec_x=ros::param::param<double>("detec_x",1);
		nan_ground_num=ros::param::param<int>("detnan_ground_numec_x",3);

		obj_nearst_kdtree.reset(new pcl::KdTreeFLANN<pcl::PointXYZ>());
		tof_position.x=0;
		tof_position.y=0;
		tof_position.z=0;

		obj_count1=0;
		obj_count2=0;
		objnan_count=0;

		obj_symble=0;
		obj_symble_temp=0;
		swiach_sym=false;
	}
	void para_reset()
	{
		voxel_map.clear();
		cloud_ground_sample.clear();
		cloud_ground_final.clear();
		cloud_nanground_final.clear();
		cloud_obj_detection.clear();
		cloud_obj_cloud.clear();
		x_max=0;
		x_min=0;
		y_max=0;
		y_min=0;
		z_max=0;
		z_min=0;
		Num_x=0;
		Num_y=0;
		Num_z=0;
	}

	void ground_remove(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_input)
	{
		//find max and min
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in(new pcl::PointCloud<pcl::PointXYZ>());
		for(int i = 0; i < cloud_input->points.size(); i++)
		{
			if(std::isnan(cloud_input->points[i].x) || std::isnan(cloud_input->points[i].y) ||std::isnan(cloud_input->points[i].z))
			{
				continue;
			}
			if ((abs(cloud_input->points[i].x)<(range_x/2))&&((cloud_input->points[i].y>(tof_height-0.5))&&(cloud_input->points[i].y<(tof_height+range_y)))&&(cloud_input->points[i].z>0.01&&cloud_input->points[i].z<range_z))
			{
				cloud_in->points.push_back(cloud_input->points[i]);
			}
		}
		for(int i = 0; i < cloud_in->points.size(); i++)
		{
			double x = cloud_in->points[i].x;
			double y = cloud_in->points[i].y;
			double z = cloud_in->points[i].z;
			if(x < x_min) x_min = x;
			if(x > x_max) x_max = x;
			if(y < y_min) y_min = y;
			if(y > y_max) y_max = y;
			if(z < z_min) z_min = z;
			if(z > z_max) z_max = z;
		}
		Num_x=(x_max-x_min)/voxel_size;
		Num_y=(y_max-y_min)/voxel_size;
		Num_z=(z_max-z_min)/voxel_size;
		//ground
		for (size_t i = 0; i < cloud_in->size(); i++)
		{
			pcl::PointXYZ p_v = cloud_in->points[i];
			int64_t position=(int64_t)((p_v.z-z_min)/voxel_size)*Num_x+(int64_t)((p_v.x-x_min)/voxel_size);
			auto iter = voxel_map.find(position);
			if (iter != voxel_map.end()) 
			{
				voxel_map[position]->contain_points.push_back(p_v);
				voxel_map[position]->point_num++;
				
				if(p_v.y<(voxel_map[position]->lowest_point.y))
				{
					voxel_map[position]->lowest_point.x=p_v.x;
					voxel_map[position]->lowest_point.y=p_v.y;
					voxel_map[position]->lowest_point.z=p_v.z;
				}
			} 
			else 
			{
				OctoTree *octo_tree =new OctoTree();
				voxel_map[position] = octo_tree;
				voxel_map[position]->contain_points.push_back(p_v);
				voxel_map[position]->point_num++;
				if(p_v.y<(voxel_map[position]->lowest_point.y))
				{
					voxel_map[position]->lowest_point.x=p_v.x;
					voxel_map[position]->lowest_point.y=p_v.y;
					voxel_map[position]->lowest_point.z=p_v.z;
				}
			}
		}
		std::vector<bool> first_show(Num_x);
		std::fill(first_show.begin(), first_show.end(),true);
		pcl::PointXYZ p_low;
		for (size_t i = 0; i < Num_z; i++)
		{
			for (size_t j = 0; j < Num_x; j++)
			{
				int64_t position_i=i*Num_x+j;
				auto iter = voxel_map.find(position_i);
				if (iter != voxel_map.end()) 
				{
					p_low = iter->second->lowest_point;
					
					if(first_show[j]==true)
					{
						// 没有限制
						Eigen::Vector3d nowpoint=Eigen::Vector3d(p_low.x,p_low.y,p_low.z);
						if (nowpoint.norm()<0.2)
						{
							cloud_nanground_final+=iter->second->contain_points;
					 		iter->second->nonground_points=iter->second->contain_points;
						}
						else
						{
							first_show[j]=false;
							cloud_ground_sample.push_back(p_low);
							voxel_map[position_i]->is_ground=1;
							for (size_t l = 0; l < iter->second->contain_points.size(); l++)
							{
								if (iter->second->contain_points[l].y<(p_low.y+point_to_groundsym))
								{
									cloud_ground_final.push_back(iter->second->contain_points[l]);
									iter->second->ground_points.push_back(iter->second->contain_points[l]);
								}
								else
								{
									cloud_nanground_final.push_back(iter->second->contain_points[l]);
									iter->second->nonground_points.push_back(iter->second->contain_points[l]);
								}				
							}
						}
						
						// //高度限制
						// if (abs(p_low.y-tof_height)<(0.2+0.05*p_low.z/4))
						// {
						// 	first_show[j]=false;
						// 	cloud_ground_sample.push_back(p_low);
						// 	voxel_map[position_i]->is_ground=1;
						// 	for (size_t l = 0; l < iter->second->contain_points.size(); l++)
						// 	{
						// 		if (iter->second->contain_points[l].y<(p_low.y+point_to_groundsym))
						// 		{
						// 			cloud_ground_final.push_back(iter->second->contain_points[l]);
						// 			iter->second->ground_points.push_back(iter->second->contain_points[l]);
						// 		}
						// 		else
						// 		{
						// 			cloud_nanground_final.push_back(iter->second->contain_points[l]);
						// 			iter->second->nonground_points.push_back(iter->second->contain_points[l]);
						// 		}				
						// 	}
						// }
						// else
						// {
						// 	// voxel_map[position_i]->is_ground=2;
						// 	cloud_nanground_final+=iter->second->contain_points;
					 	// 	iter->second->nonground_points=iter->second->contain_points;
						// }

						//角度限制
					}
					else
					{
						cloud_nanground_temp.clear();
						int nanground_count=0;
						for (size_t k = 1; k <=i; k++)
						{
							int64_t position_last=(i-k)*Num_x+j;
							auto iter_last = voxel_map.find(position_last);
							if (iter_last != voxel_map.end())
							{
								if(iter_last->second->is_ground==2)
								{
									nanground_count++;
									cloud_nanground_temp.push_back(position_last);
								}
								else if(iter_last->second->is_ground==1)
								{
									Eigen::Vector2d last_point=Eigen::Vector2d(iter_last->second->lowest_point.z,iter_last->second->lowest_point.y);
									Eigen::Vector2d cur_point=Eigen::Vector2d(iter->second->lowest_point.z,iter->second->lowest_point.y);
									Eigen::Vector2d ground_vec;
									Eigen::Vector2d cur_vec=cur_point-last_point;
									if (cur_vec[1]*iter_last->second->last_vec[1]<0)
									{
										ground_vec=Eigen::Vector2d(1,0);
									}
									else
									{
										ground_vec=iter_last->second->last_vec;
									}
									if ((abs(cur_vec.dot(ground_vec)/(cur_vec.norm()*ground_vec.norm()))>angle_the&& cur_vec[1]<0.1)||cur_vec[1]<0)
									{
										voxel_map[position_i]->is_ground=1;
										voxel_map[position_i]->last_vec=cur_vec;
										cloud_ground_sample.push_back(p_low);
										for (size_t l = 0; l < iter->second->contain_points.size(); l++)
										{
											if (iter->second->contain_points[l].y<(p_low.y+point_to_groundsym))
											{
												cloud_ground_final.push_back(iter->second->contain_points[l]);
												iter->second->ground_points.push_back(iter->second->contain_points[l]);
											}
											else
											{
												cloud_nanground_final.push_back(iter->second->contain_points[l]);
												iter->second->nonground_points.push_back(iter->second->contain_points[l]);
											}				
										}
									}
									else if(cur_vec[1]<0.1)
									{
										voxel_map[position_i]->is_ground=2;
									}
									else
									{
										cloud_nanground_final+=iter->second->contain_points;
										iter->second->nonground_points=iter->second->contain_points;
									}
									if (voxel_map[position_i]->is_ground==1)
									{
										if (nanground_count<=nan_ground_num)
										{
											for (size_t m = 0; m< cloud_nanground_temp.size(); m++)
											{
												cloud_ground_sample.push_back(voxel_map[cloud_nanground_temp[m]]->lowest_point);
												for (size_t l = 0; l < voxel_map[cloud_nanground_temp[m]]->contain_points.size(); l++)
												{
													if (voxel_map[cloud_nanground_temp[m]]->contain_points[l].y<(voxel_map[cloud_nanground_temp[m]]->lowest_point.y+point_to_groundsym))
													{
														cloud_ground_final.push_back(voxel_map[cloud_nanground_temp[m]]->contain_points[l]);
														voxel_map[cloud_nanground_temp[m]]->ground_points.push_back(voxel_map[cloud_nanground_temp[m]]->contain_points[l]);
													}
													else
													{
														cloud_nanground_final.push_back(voxel_map[cloud_nanground_temp[m]]->contain_points[l]);
														voxel_map[cloud_nanground_temp[m]]->nonground_points.push_back(voxel_map[cloud_nanground_temp[m]]->contain_points[l]);
													}				
												}
											}
										}
										else
										{
											for (size_t m = 0; m< cloud_nanground_temp.size(); m++)
											{
												cloud_nanground_final+=voxel_map[cloud_nanground_temp[m]]->contain_points;
												voxel_map[cloud_nanground_temp[m]]->nonground_points=voxel_map[cloud_nanground_temp[m]]->contain_points;
												// cloud_ground_sample.push_back(voxel_map[cloud_nanground_temp[m]]->lowest_point);
												// for (size_t l = 0; l < voxel_map[cloud_nanground_temp[m]]->contain_points.size(); l++)
												// {
												// 	cloud_nanground_final+=voxel_map[cloud_nanground_temp[m]]->contain_points;
												// 	voxel_map[cloud_nanground_temp[m]]->nonground_points=voxel_map[cloud_nanground_temp[m]]->contain_points;				
												// }
											}
										}
									}
									break;
								}																
							}
						}
					}
				}
			}
		}
		// std::cout<<"cloud_ground_sample.size:"<<cloud_ground_sample.size()<<std::endl;
	}

	void obj_detection()
	{
		int64_t voxel_num_x=(int64_t)((detec_x/2)/voxel_size)+1;
		int64_t zero_x=(int64_t)((0-x_min)/voxel_size);
		int64_t dt_num_z=(int64_t)(range_z/voxel_size);
		for (size_t i = zero_x-voxel_num_x; i < zero_x+voxel_num_x; i++)
		{
			for (size_t j = 0; j < dt_num_z; j++)
			{
				int64_t position_obj=j*Num_x+i;
				auto iter_obj = voxel_map.find(position_obj);
				if (iter_obj != voxel_map.end())
				{
					cloud_obj_detection+=voxel_map[position_obj]->contain_points;
					if (voxel_map[position_obj]->nonground_points.size()>=3)
					{
						cloud_obj_cloud+=voxel_map[position_obj]->nonground_points;
					}
				}
			}
		}
		obj_distance=10;
		if (cloud_obj_cloud.size()>=3)
		{
			KDtree_idx.clear();
			KDtree_dis.clear();
			obj_nearst_kdtree->setInputCloud(cloud_obj_cloud.makeShared());
			obj_nearst_kdtree->nearestKSearch(tof_position, 3, KDtree_idx, KDtree_dis);
			obj_distance=(sqrt(KDtree_dis[0])+sqrt(KDtree_dis[0])+sqrt(KDtree_dis[0]))/3;
		}
		if(obj_distance>obj_dis_far)
		{
			obj_symble_temp=0;
		}
		else if(obj_distance<obj_dis_near)
		{
			obj_symble_temp=2;
		}
		else
		{
			obj_symble_temp=1;
		}
		std::cout<<"obj_distance:"<<obj_distance<<std::endl;
		// std::cout<<"cloud_obj_detection.size:"<<cloud_obj_detection.size()<<std::endl;
	}


	void cloudCB(const sensor_msgs::PointCloud2& input)
	{
		clock_t start =clock();
		pcl::PointCloud<pcl::PointXYZ> cloud;
		pcl::PointCloud<pcl::PointXYZ> cloud_segmented;
		pcl::PointCloud<pcl::PointXYZ> obj_detected_cloud;
		pcl::fromROSMsg(input,cloud);
		pcl::ModelCoefficients coefficients;
		pcl::PointIndices::Ptr inliers(new pcl::PointIndices());

		Eigen::VectorXf coefficients_v;
		pcl::PointIndices::Ptr inliers_v(new pcl::PointIndices());

		// //降采样
		// pcl::VoxelGrid<pcl::PointXYZ> sor;
		// sor.setInputCloud(cloud.makeShared());
		// sor.setLeafSize(down_sampling_size, down_sampling_size, down_sampling_size);//设置叶大小为1cm
		// sor.filter(cloud);//执行滤波，将体素滤波(下采样)后的点云放置到cloud_filtered_blob
		// int all_point=cloud.size();
		// std::cout<<"point size is ::"<<all_point<<std::endl;

		// // 滤波
        // pcl::StatisticalOutlierRemoval<pcl::PointXYZ> statFilter;
		// statFilter.setMeanK(5);
		// statFilter.setStddevMulThresh(0.05);
        // statFilter.setInputCloud(cloud.makeShared());
		// statFilter.filter(cloud);

		// pcl::RadiusOutlierRemoval<pcl::PointXYZ> ror;
		// ror.setInputCloud(cloud.makeShared());
		// ror.setRadiusSearch(0.1f);
		// ror.setMinNeighborsInRadius(10);
		// ror.filter(cloud);  //保存滤波结果到cloud_filtered

		ground_remove(cloud.makeShared());
		obj_detection();
		
		if (obj_symble_temp==1)
		{
			obj_count1++;
		}
		else
		{
			obj_count1=0;
		}

		if (obj_symble_temp==2)
		{
			obj_count2++;
		}
		else
		{
			obj_count2=0;
		}

		if ((obj_count2>=2)||(obj_count1>=2))
		{
			obj_symble=obj_symble_temp;
		}
		else if((obj_count2>=1)||(obj_count1>=1))
		{
			if (obj_symble==0)
			{
				obj_symble=0;
			}
			else
			{
				obj_symble=obj_symble_temp;
			}
		}
		else
		{
			obj_symble=0;
		}

		//pub cloud
		sensor_msgs::PointCloud2 output;
		pcl::toROSMsg(cloud_ground_final,output);
		output.header.frame_id = "camera_point_cloud_frame";
		pcl_pub.publish(output);
		pcl::toROSMsg(cloud_nanground_final,output);
		output.header.frame_id = "camera_point_cloud_frame";
		obj_cloud_pub.publish(output);
		// pcl::toROSMsg(cloud_ground_sample,output);
		pcl::toROSMsg(cloud_obj_detection,output);
		output.header.frame_id = "camera_point_cloud_frame";
		pass_pub.publish(output);
		pcl::toROSMsg(cloud_obj_cloud,output);
		output.header.frame_id = "camera_point_cloud_frame";
		obj_cluster_cloud_pub.publish(output);
		
		clock_t end =clock();
		std::cout<<"obj_symble1:"<<obj_symble<<std::endl;
		// std::cout<<"分割time为:"<<((double)(end-start)/CLOCKS_PER_SEC)<<std::endl;
		std::cout<<"分割频率为:"<<1/((double)(end-start)/CLOCKS_PER_SEC)<<std::endl;
		// std::cout<<"obj_detected_cloud size is ::"<<obj_detected_cloud.size()<<std::endl;
		para_reset();
	}

protected:
	//创建节点 接受者 发布者
	ros::NodeHandle nh;
	ros::Subscriber pcl_sub;
	ros::Publisher pcl_pub,pass_pub,obj_cloud_pub,obj_cluster_cloud_pub;

	pcl::KdTreeFLANN<pcl::PointXYZ>::Ptr obj_nearst_kdtree;
    std::vector<int>  KDtree_idx; 
    std::vector<float> KDtree_dis; 

	double x_max,x_min,y_max,y_min,z_max,z_min;
	int Num_x,Num_y,Num_z;
	float voxel_size;
	
	double obj_dis_near;
	double obj_dis_far;
	double tof_height;
	double point_to_groundsym;
	double range_x,range_y,range_z,detec_x;
	int nan_ground_num;
	double angle_the;


	pcl::PointXYZ tof_position;
    Eigen::Vector3f normal_vector;

	std::unordered_map<VOXEL_LOC, OctoTree *> voxel_map;
	pcl::PointCloud<pcl::PointXYZ> cloud_ground_sample;
	pcl::PointCloud<pcl::PointXYZ> cloud_ground_final;
	pcl::PointCloud<pcl::PointXYZ> cloud_nanground_final;
	pcl::PointCloud<pcl::PointXYZ> cloud_obj_cloud;
	pcl::PointCloud<pcl::PointXYZ> cloud_obj_detection;

	std::vector<int64_t> cloud_nanground_temp;

	int obj_count1,obj_count2,objnan_count;
	bool swiach_sym;

	int obj_symble_temp;

public:
	int obj_symble;
	float obj_distance;
};

#endif;