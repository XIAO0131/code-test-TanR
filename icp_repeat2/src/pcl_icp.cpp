#include <iostream>
using namespace std;
// 稠密矩阵的代数运算（逆，特征值等）
#include <Eigen/Dense>
#include <Eigen/Core>
using namespace Eigen;
//迭代最近点ICP算法
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

#include<geometry_msgs/Twist.h>
#include <eigen3/Eigen/Core>
#include "pclomp/ndt_omp.h"
#include "pclomp/gicp_omp.h"
#include <fast_gicp/gicp/fast_gicp.hpp>
#include <fast_gicp/gicp/fast_vgicp.hpp>
#include <obj_detection.hpp>
#include <registration_gicp.hpp>
#include <planning.hpp>

class cloudHandler
{
public:
	cloudHandler()
	{
		ros::param::get("~Kp1",Kp1);
		ros::param::get("~Kp2",Kp2);
		ros::param::get("~Kd1",Kd1);
		ros::param::get("~Kd2",Kd2);
		ros::param::get("~Kp3",Kp3);
		ros::param::get("~Endframe",Endframe);
		ros::param::get("~Startframe",Startframe);
		ros::param::get("~Angular_limit",Angular_limit);
		ros::param::get("~Linear_limit",Linear_limit);
		ros::param::get("~Circle_Angular_Low",Circle_Angular_Low);
		ros::param::get("~Circle_Angular_High",Circle_Angular_High);
		ros::param::get("~Circle_Linear_limit",Circle_Linear_limit);
		icp_sub=nh.subscribe("/livox/lidar",1,&cloudHandler::cloudCB,this);
		vel_pub=nh.advertise<geometry_msgs::Twist>("/cmd_vel",1);

		distance_far=ros::param::param<double>("obj_dis_far",3);
		distance_near=ros::param::param<double>("obj_dis_near",1);

		enable_planning=ros::param::param<int>("enable_planning",0);
	}
	
	void cloudCB(const sensor_msgs::PointCloud2 &input)
	{
		double start_now=omp_get_wtime();
		pcl::fromROSMsg(input,cloud_repeat);
		if(!reg_gicp.get_transToteach(frameNum,cloud_repeat.makeShared()))
		{
			frameNum=0;
			vel_msg.linear.x = 0;
			vel_msg.angular.z = 0;
			vel_pub.publish(vel_msg);
			ros::shutdown();
            return;
		}
		icp_matrix4f=reg_gicp.icp_matrix4f;
		Ztheta=atan2(icp_matrix4f(1,0),icp_matrix4f(0.0)) ;
		x=icp_matrix4f(0,3);
		if (abs(x)<0.01){
			x=0;
		}
		y=icp_matrix4f(1,3);
		if(abs(y)<0.01){
			y=0;
		}
		cout<<"x方向位移为:\n"<<icp_matrix4f(0,3)<<endl;
		cout<<"y方向位移为:\n"<<icp_matrix4f(1,3)<<endl;
		cout<<"z轴旋转角度为:\n"<<atan2(icp_matrix4f(1,0),icp_matrix4f(0.0)) * 180 / pi<<endl;
		if (reg_gicp.compute_error_now()>error)
		{
			vel_msg.linear.x = 0;
			vel_msg.angular.z=0;
			vel_pub.publish(vel_msg);
			counticp++;
			std::cout <<"Wrong icp, score too much!!!!!!!!!!!!!!!!!!!!!!!!"<<endl;
			if(reg_gicp.compute_error_now()>0.02)
			{
				frameNum=frameNum2;
			}
			return;
		}
		double gicp_now=omp_get_wtime();
		std::cout<<"use_planning:"<<obj_detection.use_planning<<endl;
		if(!obj_detection.use_planning)
		{
			PID_controller(x,y,Ztheta);
			double pid_now =omp_get_wtime();
			std::cout<<"gicp耗时:"<<(gicp_now-start_now)<<endl;
			std::cout<<"pid耗时:"<<(pid_now-gicp_now)<<endl;
			std::cout<<"sigle频率:"<<1/(pid_now-start_now)<<endl;
			// std::cout<<"周期耗时:"<<(pid_now-last_time)<<endl;
			std::cout<<"总频率:"<<1/(pid_now-last_time)<<endl;
			last_time=pid_now;
		}
		else
		{
			std::cout<<"tof_obj_cloud.size()before:"<<obj_detection.tof_obj_cloud.size()<<endl;
			std::cout<<"livox_obj_cloud.size()before:"<<obj_detection.livox_obj_cloud.size()<<endl;
			bool arrive_target=planing_deal.get_path(obj_detection.do_A_star,frameNum,icp_matrix4f,obj_detection.tof_obj_cloud.makeShared(),obj_detection.livox_obj_cloud.makeShared(),vel_msg.linear.x,vel_msg.angular.z);
			std::cout<<"tof_obj_cloud.size()after:"<<obj_detection.tof_obj_cloud.size()<<endl;
			std::cout<<"livox_obj_cloud.size()after:"<<obj_detection.livox_obj_cloud.size()<<endl;
			std::cout<<"obj_detection.do_A_star:"<<obj_detection.do_A_star<<endl;
			// obj_detection.do_A_star=false;
			if(arrive_target)
			{
				obj_detection.use_planning=false;
				frameNum++;
				if(vel_msg.linear.x <0)
				{
					vel_msg.linear.x =0;
				}
				else if(vel_msg.linear.x >Linear_limit)
				{
					vel_msg.linear.x =Linear_limit;
				}
				if(vel_msg.angular.z>Angular_limit)
				{
					vel_msg.angular.z=Angular_limit;
				}
				else if(vel_msg.angular.z<-Angular_limit)
				{
					vel_msg.angular.z=-Angular_limit;
				}

				//障碍检测
				if(obj_detection.obj_symble==2)
				{
					vel_msg.linear.x = 0;
					// vel_msg.angular.z = 0;
					// double base_z=0.1;
					// if(vel_msg.angular.z<=base_z)
					// {
					// 	vel_msg.angular.z=0.5*vel_msg.angular.z;
					// }
					// if(vel_msg.angular.z>=base_z)
					// {
					// 	vel_msg.angular.z=base_z+(vel_msg.angular.z-base_z)*(obj_detection.obj_distance-distance_near)/(distance_far-distance_near);
					// }
				}
				else if(obj_detection.obj_symble==1)
				{
					// // double base_v=0.2;
					// if(vel_msg.linear.x<=base_v)
					// {
					// 	vel_msg.linear.x=0.8*vel_msg.linear.x;
					// }
					// if(vel_msg.linear.x>=base_v)
					// {
					// 	vel_msg.linear.x=base_v+(vel_msg.linear.x-base_v)*(obj_detection.obj_distance-distance_near)/(distance_far-distance_near);
					// }
								double base_v=0.1;
					if(vel_msg.linear.x<=base_v)
					{
						vel_msg.linear.x=0.5*vel_msg.linear.x;
					}
					if(vel_msg.linear.x>=base_v)
					{
						vel_msg.linear.x=base_v+(vel_msg.linear.x-base_v)*(obj_detection.obj_distance-distance_near)/(distance_far-distance_near);
					}
				}
				vel_pub.publish(vel_msg);
			}
			else
			{
				if(vel_msg.linear.x <0)
				{
					vel_msg.linear.x =0;
				}
				else if(vel_msg.linear.x >Linear_limit)
				{
					vel_msg.linear.x =Linear_limit;
				}
				if(vel_msg.angular.z>Angular_limit)
				{
					vel_msg.angular.z=Angular_limit;
				}
				else if(vel_msg.angular.z<-Angular_limit)
				{
					vel_msg.angular.z=-Angular_limit;
				}

				//障碍检测
				if(obj_detection.obj_symble==2)
				{
					vel_msg.linear.x = 0;
					vel_msg.angular.z = 0;
					// double base_z=0.1;
					// if(vel_msg.angular.z<=base_z)
					// {
					// 	vel_msg.angular.z=0.5*vel_msg.angular.z;
					// }
					// if(vel_msg.angular.z>=base_z)
					// {
					// 	vel_msg.angular.z=base_z+(vel_msg.angular.z-base_z)*(obj_detection.obj_distance-distance_near)/(distance_far-distance_near);
					// }
				}
				else if(obj_detection.obj_symble==1)
				{
					// // double base_v=0.2;
					// if(vel_msg.linear.x<=base_v)
					// {
					// 	vel_msg.linear.x=0.8*vel_msg.linear.x;
					// }
					// if(vel_msg.linear.x>=base_v)
					// {
					// 	vel_msg.linear.x=base_v+(vel_msg.linear.x-base_v)*(obj_detection.obj_distance-distance_near)/(distance_far-distance_near);
					// }
					double base_v=0.1;
					if(vel_msg.linear.x<=base_v)
					{
						vel_msg.linear.x=0.5*vel_msg.linear.x;
					}
					if(vel_msg.linear.x>=base_v)
					{
						vel_msg.linear.x=base_v+(vel_msg.linear.x-base_v)*(obj_detection.obj_distance-distance_near)/(distance_far-distance_near);
					}
				}

				vel_pub.publish(vel_msg);
			}
		}
	}


	void PID_controller(float x, float y, double Ztheta)
	{
		ecurrentX=x;
		if(abs(ecurrentX)<0.2)
		{
			UX=Kp1*ecurrentX;
		}
		else
		{
			UX=Linear_limit;
		}
		if(ecurrentX> 0 && abs(Ztheta*180/pi)<5)
		{
			frameNum++;
			count=0;
			vel_msg.linear.x = 0;
			vel_msg.angular.z = 0;
			vel_pub.publish(vel_msg);
			cout<<"最终控制指令角速度"<<vel_msg.angular.z<<endl;
			std::cout<<"Linear velocity is:"<<vel_msg.linear.x<<endl;
			std::cout<<"stop11111, x over frame!!!!!!!!!!!!!!!!"<<endl;
			return;
		}
		if(ecurrentX> 0 && abs(Ztheta*180/pi)>5)
		{
			count=0;
			ecurrentX=0;
		}
		else if(frameNum>0 && ecurrentX<-10)
		{
			frameNum=frameNum2;
			count=0;
			vel_msg.linear.x = 0;
			vel_msg.angular.z = 0;
			vel_pub.publish(vel_msg);
			cout<<"最终控制指令角速度"<<vel_msg.angular.z<<endl;
			std::cout<<"Linear velocity is:"<<vel_msg.linear.x<<endl;
			std::cout<<"stop222222, x too far!!!!!!!!!!!!!!!!"<<endl;
			
			if(enable_planning)
			{
				obj_detection.use_planning=true;
				obj_detection.do_A_star=true;
			}
			return;
		}
		else 
		{
			if (UX>Linear_limit)
			{
				UX=Linear_limit;
				std::cout<<"Linear velocity limitation is:"<<UX<<endl;
			}
			else if (UX<-1*Linear_limit)
			{
				UX=0;
				std::cout<<"Linear velocity limitation is:"<<UX<<endl;
			}
			vel_msg.linear.x = UX;
		}
		ecurrentTheta=Ztheta;
		// int symbol_beta=x*y/abs(x*y);
		// float beta=-pi/2-atan2(x,y);
		// beta=symbol_beta*abs(beta);
		// std::cout<<"Beta is:"<<beta* 180 / pi <<endl;
		// std::cout<<"theta is:"<<Ztheta* 180 / pi <<endl;
		// UTheta=Kp2*(-beta+ecurrentTheta);

		if(abs(Ztheta*180/pi)<10)
		{
			UTheta=Kp2*ecurrentTheta+Kd2*y;
			
		}
		else{
			UTheta=Kp2*ecurrentTheta;
			// UTheta=Kp2*(-beta+ecurrentTheta);
		}


		// UTheta=Kp2*ecurrentTheta+Kd2*y;
		// if(beta* 180 / pi <-80)
		// {
		// 	UTheta=Kp2*ecurrentTheta;
		// }
		// else{
		// 	UTheta=Kp2*ecurrentTheta-Kd2*beta;
		// }
		
		if (UTheta>Angular_limit )
		{
			UTheta=Angular_limit;
			vel_msg.angular.z = UTheta;
			// std::cout<<"Angular velocity limitation is:"<<UTheta<<endl;
		}
		else if(UTheta<-1*Angular_limit )
		{
			UTheta=-1*Angular_limit;
			vel_msg.angular.z = UTheta;
			// std::cout<<"Angular velocity limitation is:"<<UTheta<<endl;
		}
		else
		{
			vel_msg.angular.z = UTheta;
			// std::cout<<"Angular velocity is:"<<UTheta<<endl;
		}

		


		if((Ztheta*y<0)&&(!reg_gicp.end_frame))
		{
			float r=(sqrt(x*x+y*y)/2)/sin(abs(Ztheta)/2);			
			cout<<"ddddddddddddddddd预计圆弧r为:"<<r<<endl;
			if(abs(vel_msg.linear.x)>0.015 && abs(vel_msg.angular.z)>0.015)
			{
				cout<<"ddddddddddddddd正在利用圆弧构建比值移动"<<endl;
				// float diretion=vel_msg.angular.z/abs(vel_msg.angular.z);
				// cout<<"diretion:"<<diretion<<endl;
				// vel_msg.angular.z=diretion*0.25;
				// vel_msg.linear.x=abs(vel_msg.angular.z*r);			
				// if(vel_msg.linear.x>Linear_limit){
				// 	vel_msg.linear.x=Linear_limit;
				// 	vel_msg.angular.z=diretion*vel_msg.linear.x/r;
				// }
				float diretion=vel_msg.angular.z/abs(vel_msg.angular.z);
				cout<<"diretion_org:"<<vel_msg.angular.z<<endl;
				cout<<"diretion:"<<diretion<<endl;

				//完全依靠PID初值算法结果
				if(abs(vel_msg.angular.z)<Circle_Angular_Low)//转弯的速度期望为0.25以上
				{
					vel_msg.angular.z=diretion*Circle_Angular_Low;
				}
				double temp_line_v=abs(vel_msg.angular.z*r);	
				if(temp_line_v>Circle_Linear_limit)
				{
					temp_line_v=Circle_Linear_limit;
				}	

				if(temp_line_v>vel_msg.linear.x)
				{
					vel_msg.linear.x=temp_line_v;
				}
				else
				{
					vel_msg.angular.z=diretion*vel_msg.linear.x/r;
					if(abs(vel_msg.angular.z)>Circle_Angular_High)
					{
						vel_msg.angular.z=diretion*Circle_Angular_High;
						vel_msg.linear.x=abs(vel_msg.angular.z*r);
					}
				}

				// vel_msg.angular.z=diretion*0.25;//转弯的速度期望为0.25
				// // vel_msg.linear.x=abs(vel_msg.angular.z*r);	
				// vel_msg.linear.x=abs(vel_msg.angular.z*r);			
				// if(vel_msg.linear.x>Linear_limit){
				// 	vel_msg.linear.x=Linear_limit;
				// 	vel_msg.angular.z=diretion*vel_msg.linear.x/r;
				// }


			}

			if(sqrt(x*x+y*y)<0.15 && abs(vel_msg.linear.x)>0.01)
			{
				frameNum++;
				vel_pub.publish(vel_msg);
				cout<<"obj_detection:"<<obj_detection.obj_symble<<endl;
				cout<<"最终控制指令角速度"<<vel_msg.angular.z<<endl;
				std::cout<<"Linear velocity is:"<<vel_msg.linear.x<<endl;
				std::cout<<"Circle too small!!!!!!!!!!!"<<endl;
				return;
				
			}
		}

		if(abs(Ztheta*180/pi)>30)
		{
			vel_msg.linear.x = 0;
		}
		//障碍检测
		if(obj_detection.obj_symble==2)
		{
			vel_msg.linear.x = 0;
			vel_msg.angular.z = 0;
		}
		else if(obj_detection.obj_symble==1)
		{
			double base_v=0.1;
			if(vel_msg.linear.x<=base_v)
			{
				vel_msg.linear.x=0.5*vel_msg.linear.x;
			}
			if(vel_msg.linear.x>=base_v)
			{
				vel_msg.linear.x=base_v+(vel_msg.linear.x-base_v)*(obj_detection.obj_distance-distance_near)/(distance_far-distance_near);
			}

		}
        vel_pub.publish(vel_msg);
		cout<<"obj_detection:"<<obj_detection.obj_symble<<endl;
		cout<<"最终控制指令角速度"<<vel_msg.angular.z<<endl;
		std::cout<<"Linear velocity is:"<<vel_msg.linear.x<<endl;
		icp_matrix4f(0,3)=icp_matrix4f(0,3)+0.1*vel_msg.linear.x;
		icp_matrix4f(1,3)=icp_matrix4f(1,3)+vel_msg.linear.x*0.1*tan(vel_msg.angular.z*0.2);
		count++;
		if(vel_msg.angular.z<0.1)
		{
			x_limitation=0.5;
			z_limitation=15;
		}
		else
		{
			x_limitation=0.15;
			z_limitation=15;
		}

		if(reg_gicp.end_frame)
		{
			x_limitation=0.05;
			z_limitation=5;
		}
		if(abs(x)<x_limitation && abs(Ztheta)* 180 / pi <z_limitation)
		{
			frameNum++;
			count=0;
			cout<<"Change to next scan!!!!!!"<<endl;
			frameNum2=frameNum;//保留上一次成功的帧序号
			cout<<"Next scan is scan"<<frameNum<<endl;
		}
		// else if (frameNum >0 && count==20)
		// {
		// 	frameNum++;
		// 	count=0;
		// 	cout<<"Scan fail!!!!!!Failed scan is:"<<frameNum-1<<endl;
		// 	cout<<"Next scan is scan"<<frameNum<<endl;
		// }
		cout<<"Tracking scan"<<frameNum<<""<<endl;

		// if(frameNum==472)
		// {
		// 	frameNum=0;
		// }
		return;
	};

protected:
	ros::NodeHandle nh;
	ros::Subscriber icp_sub;
	ros::Publisher icp_pub;
	ros::Publisher pcd_pub;
	ros::Publisher vel_pub;
	double last_time;
	// ros::Time last_time=pid_now;uiuu
	float elast=0;
	float x_limitation;
	float z_limitation;
	int size;
	float dis;
	const double pi=3.141592653589793238462643383279502884197;
	float error=0.031;
	// ofstream outfile("/home/xrx/mid360_ws/src/icp_repeat/icp_trans_matrix4f.txt", ios::trunc);
	Matrix4f trans_matrixs;
	Matrix4f icp_matrix4f=Eigen::Matrix4f::Identity(4,4);
	pcl::PointCloud<pcl::PointXYZ> cloud_teach;
	sensor_msgs::PointCloud2 output;
	pcl::PointCloud<pcl::PointXYZ> cloud_repeat; //要转换的点云
	pcl::PointCloud<pcl::PointXYZ> cloud_ndt; 
	pcl::PointCloud<pcl::PointXYZ> cloud_aligned; //最终点云
	pcl::PointCloud<pcl::PointXYZ> cloud_teach_filtered;
 	pcl::PointCloud<pcl::PointXYZ> cloud_teach_downsampled;
	pcl::PointCloud<pcl::PointXYZ> cloud_repeat_filtered;
 	pcl::PointCloud<pcl::PointXYZ> cloud_repeat_downsampled;
	pcl::PointCloud<pcl::PointXYZ>  cloud_repeat_detect;

	geometry_msgs::Twist vel_msg;
	float ecurrentX,ecurrentTheta,UX=0,UTheta=0,ThetaY=0,ecurrent1Y=0,Lastlinear;
	int frameNum=0;
	int frameNum2=0;
	double Kp1,Kd1,Kp2,Kd2,Angular_limit,Linear_limit,Circle_Angular_Low,Circle_Angular_High,Circle_Linear_limit,Kp3;
	double distance_far,distance_near;
	float x,y;
	double Ztheta;
	int count=0,counticp=0;
	int Endframe,Startframe;

	int enable_planning;
	

public:
	segmatation_sel obj_detection;
	Regiestration_Gicp reg_gicp;
	Planning planing_deal;
};
 
 
 
main (int argc, char **argv)
{
	ros::init(argc,argv,"icp_matching");

	cloudHandler handler;
	
	ros::spin();
	return 0;
}
