
#ifndef PLANNING_HPP
#define PLANNING_HPP
#include <Eigen/Dense>
#include <Eigen/Core>
#include <eigen3/Eigen/Core>
#include <pcl/common/transforms.h>
#include <pcl/filters/approximate_voxel_grid.h>

#include <obj_detection.hpp>
#include "AStar.hpp"

#include "matplotlibcpp.h"

class Planning
{
public:
    Planning()
    {
        map_grid_size=ros::param::param<double>("map_grid_size",0.1);
        car_length=ros::param::param<double>("car_length",0.1);
        plan_safe_dis=ros::param::param<double>("plan_safe_dis",0.1);
        last_frame=-1;
        local_map.clear();
        finish_planning=false;
        path_id_num=0;
        last_path_id_num=0;
        theta_move=0;
        x_move=0;
        expend_num=1+(car_length/2+plan_safe_dis)/map_grid_size;
        expend_num_x=1+(car_length+plan_safe_dis)/map_grid_size;
        target_useless=false;
        nan_mun=0;
        min_point_num=5;
        last_path_astar.clear();

    }
    bool get_path(bool & A_star,int & frameNum,Eigen::Matrix4f T2target, pcl::PointCloud<pcl::PointXYZ>::Ptr tof_cloud,pcl::PointCloud<pcl::PointXYZ>::Ptr livox_cloud,double & msg_x,double & msg_z)
    {
        std::cout<<"get_path"<<std::endl;
        std::cout<<"A_star"<<A_star<<std::endl;
        if(A_star)
        {
            A_star=false;
            local_map.clear();
            Planning_path.clear();
            path_id_num=0;
            pcl::PointCloud<pcl::PointXYZ> trans_temp_cloud,trans_temp_cloud_livox;
            pcl::transformPointCloud(*tof_cloud,trans_temp_cloud,T2target);
            local_map+=trans_temp_cloud;

            // pcl::io::savePCDFileBinary("/media/liwen/KK/Wen_Li/Key_frame/teach_keyframe/cloud_tof.pcd", trans_temp_cloud);

            trans_temp_cloud.clear();

            for(int i = 0; i < livox_cloud->size(); i++)
            {
                if((livox_cloud->points[i].x==0)&&(livox_cloud->points[i].y==0)&&(livox_cloud->points[i].z==0))
                {
                    continue;
                }
                if((livox_cloud->points[i].z>0.2))
                continue;
                trans_temp_cloud_livox.push_back(livox_cloud->points[i]);
            }
            pcl::transformPointCloud(trans_temp_cloud_livox,trans_temp_cloud,T2target);
            
            local_map+=trans_temp_cloud;  
            std::cout<<"tof_cloud:"<<tof_cloud->size()<<::endl;
            std::cout<<"livox_cloud:"<<livox_cloud->size()<<::endl;
            std::cout<<"local_map:"<<local_map.size()<<::endl;

            // tof_cloud->clear();
            // livox_cloud->clear();
            
            // pcl::io::savePCDFileBinary("/media/liwen/KK/Wen_Li/Key_frame/teach_keyframe/cloud_livox.pcd", trans_temp_cloud);
            // pcl::io::savePCDFileBinary("/media/liwen/KK/Wen_Li/Key_frame/teach_keyframe/cloud_all.pcd", local_map);


            // for(int i = 0; i < local_map.points.size(); i++)
            // {
            //     double x = local_map.points[i].x;
            //     double y = local_map.points[i].y;
            //     double z = local_map.points[i].z;
            //     if(x < x_min) x_min = x;
            //     if(x > x_max) x_max = x;
            //     if(y < y_min) y_min = y;
            //     if(y > y_max) y_max = y;
            // }
            // Num_x=(x_max-x_min)/map_grid_size;
            // Num_y=(y_max-y_min)/map_grid_size;
            x_min=-5;
            y_min=-5;
            Num_x=100;
            Num_y=100;
            path_generator.setWorldSize({Num_x, Num_y});
            path_generator.setHeuristic(AStar::Heuristic::diagonal);
            path_generator.setHeuristicWeight(1.5);
            path_generator.setDiagonalMovement(true);
            path_generator.clearCollisions();

            std::vector<std::vector<int> > wall_temp(Num_x, std::vector<int>(Num_y, 0));
            std::cout<<"expend_num:"<<expend_num<<std::endl;
            std::cout<<"expend_num_x:"<<expend_num_x<<std::endl;
            for(int i = 0; i < local_map.points.size(); i++)
            {
                int xtemp=(local_map.points[i].x-x_min)/map_grid_size;
                int ytemp=(local_map.points[i].y-y_min)/map_grid_size;
                // std::cout<<"xtemp:"<<xtemp<<","<<xtemp<<std::endl;
                // if (xtemp<0||xtemp>=Num_x||ytemp<0||ytemp>=Num_y)
                if (xtemp<0||xtemp>=Num_x||)
                {
                    continue;
                }
                for (size_t j = xtemp-expend_num; j <= xtemp+expend_num_x; j++)
                {
                    for (size_t k =ytemp-expend_num; k <=ytemp+expend_num; k++)
                    {
                        if (j<0||j>=Num_x||k<0||k>=Num_y)
                        {
                            continue;
                        }
                        wall_temp[j][k]++;
                    }
                }
            }

            for (size_t j = 0; j <Num_x; j++)
            {
                for (size_t k =0; k <Num_y; k++)
                {
                    if (wall_temp[j][k]>min_point_num)
                    {
                        AStar::Vec2i cod_xy;
                        cod_xy.x=j;
                        cod_xy.y=k;
                        path_generator.addCollision(cod_xy);
                    }
                } 
            }
            AStar::Vec2i cod_target;
            cod_target.x=(0-x_min)/map_grid_size;
            cod_target.y=(0-y_min)/map_grid_size;

            AStar::Vec2i cod_target_old;
            cod_target_old=cod_target;
            
            // if(std::find(walls.begin(), walls.end(), coordinates_) != walls.end())
            if(wall_temp[cod_target.x][cod_target.y]>min_point_num)
            {
                target_useless=true;
                pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_nanwall(new pcl::PointCloud<pcl::PointXYZ>());
                pcl::PointXYZ point_i;
                for (size_t i =cod_target.x; i < Num_x; i++)
                {
                    for (size_t j = -Num_y; j < Num_y; j++)
                    {
                        if (i<0||i>=Num_x||j<-Num_y||j>=Num_y)
                        {
                            continue;
                        }
                        if (!(wall_temp[i][j]>min_point_num))
                        {
                            point_i.x=i;
                            point_i.y=j;
                            point_i.z=0;
                            cloud_nanwall->push_back(point_i);
                        } 
                    } 
                }
                point_i.x=cod_target.x;
                point_i.y=cod_target.y;
                std::vector<int>  KDtree_idx; 
                std::vector<float> KDtree_dis; 
                pcl::KdTreeFLANN<pcl::PointXYZ>::Ptr nanwall_kdtree(new pcl::KdTreeFLANN<pcl::PointXYZ>());
                nanwall_kdtree->setInputCloud(cloud_nanwall);
                nanwall_kdtree->nearestKSearch(point_i,1, KDtree_idx, KDtree_dis);
                cod_target.x=cloud_nanwall->points[KDtree_idx[0]].x;
                cod_target.y=cloud_nanwall->points[KDtree_idx[0]].y;
            }
            else
            {
                target_useless=false;
            }
            AStar::Vec2i cod_now;
            cod_now.x=(T2target(0,3)-x_min)/map_grid_size;
            cod_now.y=(T2target(1,3)-y_min)/map_grid_size;
            

            std::cout<<"wall_temp[cod_now.x][cod_now.y]:"<<wall_temp[cod_now.x][cod_now.y]<<std::endl;
            std::cout<<"wall_temp[cod_target.x][cod_target.y]:"<<wall_temp[cod_target.x][cod_target.y]<<std::endl;
            std::cout<<"cod_target.xy:"<<cod_target.x<<","<<cod_target.y<<std::endl;
            std::cout<<"cod_now.xy"<<cod_now.x<<","<<cod_now.y<<std::endl;

            path_generator.setHeuristicWeight(1);
            auto local_path=path_generator.findPath(cod_now,cod_target);

            std::cout<<"local_path.size()"<<local_path.size()<<std::endl;

            auto local_path_temp=local_path;//保证夹角小于38.6
            local_path.clear();
            for (size_t i = 0; i < local_path_temp.size(); i++)
            {
                if (i==0)
                {
                    local_path.push_back(local_path_temp[i]);
                    continue;
                }
                if(i==local_path_temp.size()-1)
                {
                    local_path.push_back(local_path_temp[i]);
                    continue;
                }
                Eigen::Vector2d next_vec(local_path_temp[i+1].x,local_path_temp[i+1].y);
                Eigen::Vector2d now_vec(local_path_temp[i].x,local_path_temp[i].y);
                Eigen::Vector2d last_vec(local_path_temp[i-1].x,local_path_temp[i-1].y);
                if(std::abs((((next_vec-now_vec).dot(now_vec-last_vec))/((next_vec-now_vec).norm()*(now_vec-last_vec).norm())))<=0.79)
                {
                    local_path.push_back(local_path_temp[i]);
                } 
            }


            auto local_path_temp2=local_path;
            local_path.clear();
            while (local_path_temp2.size())
            {
                if (local_path.size()==0)
                {
                    local_path.push_back(local_path_temp2[local_path_temp2.size()-1]);
                    local_path_temp2.pop_back();
                }
                else if (local_path_temp2.size()>1)
                {
                    Eigen::Vector2d dis_vec1(local_path_temp2[local_path_temp2.size()-1].x,local_path_temp2[local_path_temp2.size()-1].y);
                    Eigen::Vector2d dis_vec2(local_path.back().x,local_path.back().y);
                    if ((dis_vec1-dis_vec2).norm()>=3)
                    {
                        local_path.push_back(local_path_temp2[local_path_temp2.size()-1]);
                    }
                    local_path_temp2.pop_back();
                }
                else
                {
                    local_path.push_back(local_path_temp2[local_path_temp2.size()-1]);
                    local_path_temp2.pop_back();
                }
            }

            auto local_path_temp3=local_path;
            local_path.clear();
            while (local_path_temp3.size())
            {
                local_path.push_back(local_path_temp3.back());
                local_path_temp3.pop_back();
            }

            if (last_path_id_num)
            {
                bool continue_id=true;
                int last_temp_id=last_path_astar.size()-1;
                int now_temp_id=local_path.size()-1;
                for (size_t i = 0; i <=last_path_id_num; i++)
                {
                   if (!((local_path[now_temp_id-i].x==last_path_astar[last_temp_id-i].x)&&(local_path[now_temp_id-i].y==last_path_astar[last_temp_id-i].y)))
                   {
                        continue_id=false;
                   }
                }
                if (continue_id)
                {
                    path_id_num=last_path_id_num;
                }
            }
            last_path_astar=local_path;

            for(auto& coordinate : local_path) 
            {
                std::cout << coordinate.x << " " << coordinate.y << "\n";
            }

            std::cout<<"local_path.size()2:"<<local_path.size()<<std::endl;
            cout<<"findPath2!"<<endl;

            // if (local_path.size()>0)
            // {
            //     cout<<"trans_temp_cloud:"<<trans_temp_cloud.size()<<endl;
            //     cout<<"trans_temp_cloud:"<<livox_cloud->size()<<endl;
            //     pcl::io::savePCDFileBinary("/media/liwen/KK/Wen_Li/Key_frame/teach_keyframe/cloud_livox.pcd", trans_temp_cloud);
            //     // pcl::io::savePCDFileBinary("/media/liwen/KK/Wen_Li/Key_frame/teach_keyframe/livox_cloud.pcd", *livox_cloud);
            //     // pcl::io::savePCDFileBinary("/media/liwen/KK/Wen_Li/Key_frame/teach_keyframe/trans_temp_cloud_livox.pcd", trans_temp_cloud_livox);
            //     pcl::io::savePCDFileBinary("/media/liwen/KK/Wen_Li/Key_frame/teach_keyframe/cloud_all.pcd", local_map);
            // }
            tof_cloud->clear();
            livox_cloud->clear();


            Eigen::Vector3d direction_x_tar(1, 0, 0);
            // Eigen::Vector3d direction_x_now=T2target.block<3,3>(0,0)*direction_x_tar;
            // // direction_x_now[2]=0;
            // Eigen::Vector3d lastvec=direction_x_now;
            
            if ((local_path.size()>1)&&(local_path.front().x==cod_target.x)&&(local_path.front().y==cod_target.y))
            {
                //each pose
                while (local_path.size())
                {
                    if (local_path.size()>1)
                    {
                        AStar::Vec2i this_pos=local_path[local_path.size()-1];
                        AStar::Vec2i next_pos=local_path[local_path.size()-2];
                        Eigen::Vector3d posi_this((this_pos.x-cod_target_old.x)*map_grid_size,(this_pos.y-cod_target_old.y)*map_grid_size,0);
                        Eigen::Vector3d posi_next((next_pos.x-cod_target_old.x)*map_grid_size,(next_pos.y-cod_target_old.y)*map_grid_size,0);
                        Eigen::Matrix3d rotMatrix_now=Eigen::Quaterniond::FromTwoVectors(direction_x_tar, posi_next-posi_this).toRotationMatrix();
                        Eigen::Matrix4f T2target_nowpath=Eigen::Matrix4f::Identity();
                        T2target_nowpath.block<3,3>(0,0)=rotMatrix_now.cast<float>();
                        T2target_nowpath.block<3,1>(0,3)=posi_this.cast<float>();
                        Eigen::Isometry3f pose_this(T2target_nowpath);
                        Planning_path.push_back(pose_this);
                        local_path.pop_back();
                        // std::cout<<"pose"<<pose_this.matrix()<<std::endl;
                    }
                    else
                    {
                        Eigen::Matrix4f T2target_nowpath=Eigen::Matrix4f::Identity();
                        Eigen::Isometry3f pose_this(T2target_nowpath);
                        Planning_path.push_back(pose_this);
                        local_path.pop_back();
                        // std::cout<<"pose"<<pose_this.matrix()<<std::endl;
                    }
                    // float x, y, z, roll, pitch, yaw;
                    // pcl::getTranslationAndEulerAngles(Planning_path.back(), x, y, z, roll, pitch, yaw);
                    // std::cout<<"xyz_yaw:"<<x<<","<<y<<","<<z<<","<<roll<<","<<pitch<<","<<yaw<<std::endl;

                }
                finish_planning=true;
            }
            else
            {
                // std::cout<<"Can't find useful path:"<<nan_mun<<std::endl;
                // std::string save_nan="/media/liwen/KK/Wen_Li/Key_frame/teach_keyframe/cloud_all_"+std::to_string(nan_mun)+".pcd";
                // pcl::io::savePCDFileBinary(save_nan, local_map);
                nan_mun++;
                msg_x = 0;
		        msg_z = 0;
                A_star=true;
                finish_planning=false;
                return false;
            }

            if(finish_planning)
            {
                std::vector<Eigen::Isometry3f, Eigen::aligned_allocator<Eigen::Isometry3f>> Planning_path_temp;
                Planning_path_temp=Planning_path;
                Planning_path.clear();
                for (size_t i = 0; i < Planning_path_temp.size(); i++)
                {
                    if (i==0)
                    {
                        Planning_path.push_back(Planning_path_temp[i]);
                        // std::cout<<"Planning_path_temp:"<<Planning_path.back().matrix()<<std::endl;
                    }
                    else
                    {
                        Eigen::Matrix4f trans_bt_temp=Planning_path_temp[i].matrix();
                        trans_bt_temp.block<3,3>(0,0)=Planning_path_temp[i-1].matrix().block<3,3>(0,0);
                        Eigen::Isometry3f pose_this_temp(trans_bt_temp);
                        Planning_path.push_back(pose_this_temp);
                        // std::cout<<"Planning_path_temp:"<<Planning_path.back().matrix()<<std::endl;
                    }
                }
                Planning_path.push_back(Planning_path_temp.back());
                // std::cout<<"Planning_path_temp:"<<Planning_path.back().matrix()<<std::endl;
            }
            
        }
        if(finish_planning)
        { 
            std::cout<<"path_id_num:"<<path_id_num<<std::endl;  

            float x, y, z, roll, pitch, yaw;
            Eigen::Isometry3f pose_this_temp2(T2target);
            pcl::getTranslationAndEulerAngles(pose_this_temp2, x, y, z, roll, pitch, yaw);

            // if ((Planning_path[path_id_num](0,3)>0)&& (!target_useless))
            if ((x>-0.2)&&(abs(yaw)<0.3)&& (!target_useless))
            {
                frameNum++;
                finish_planning=false;
                last_path_id_num=0;
                msg_x = 0;
		        msg_z = 0;
                std::cout<<"arrive_true"<<std::endl;
                return true;
            }
            if ((Planning_path[path_id_num](0,3)>0)&& (target_useless))
            {
                frameNum++;
                A_star=true;
                finish_planning=false;
                last_path_id_num=0;
                msg_x = 0;
		        msg_z = 0;
                std::cout<<"arrive_false"<<std::endl;
                return false;
            }
            

            Eigen::Matrix4f T2next=T2target.inverse()*Planning_path[path_id_num].matrix();
            // float theta_move0=atan2(Planning_path[path_id_num].matrix()(1,0),Planning_path[path_id_num].matrix()(0.0)) ;
            // float x_move0=Planning_path[path_id_num].matrix()(0,3);
            
            // float x, y, z, roll, pitch, yaw;
            pcl::getTranslationAndEulerAngles(Planning_path[path_id_num], x, y, z, roll, pitch, yaw);
            std::cout<<"xyz_yaw:"<<x<<","<<y<<","<<z<<","<<roll<<","<<pitch<<","<<yaw<<std::endl;

            theta_move=atan2(T2next(1,0),T2next(0.0)) ;
		    x_move=T2next(0,3);
            std::cout<<"frameNum:"<<frameNum<<std::endl;
            std::cout<<"x:"<<x_move<<","<<"theta:"<<theta_move<<std::endl;
            if(x_move<0.2&&abs(theta_move*180/pi)<15)
            {
                path_id_num++;
                last_path_id_num=path_id_num;
                // msg_x = 0;
		        // msg_z = 0;
            }
            else
            {
                msg_x =x_move;
		        msg_z =theta_move;
                if ((theta_move* 180 / pi)>30)
                {
                    msg_x =0;
                }

                if (msg_x>0.5)
                {
                    msg_x=0.5;
                }
                if (msg_x<0)
                {
                    msg_x=0;
                }

                if (msg_z>0.5)
                {
                    msg_z=0.5;
                }
                if (msg_z<-0.5)
                {
                    msg_z=-0.5;
                }
            }
        }
        else
        {
            msg_x = 0;
		    msg_z = 0;
        }
        std::cout<<"msg_x:"<<msg_x<<","<<"msg_z:"<<msg_z<<std::endl;
        return false;
    }
private:
    ros::Publisher vel_pub_plan;
    const double pi=3.141592653589793238462643383279502884197;

    AStar::Generator path_generator;
    double x_max,x_min,y_max,y_min,plan_safe_dis;
    int Num_x,Num_y,expend_num,expend_num_x;

    int path_id_num,last_path_id_num;
    AStar::CoordinateList last_path_astar;

    double map_grid_size,car_length;
    int last_frame;
    pcl::PointCloud<pcl::PointXYZ> local_map;
    std::vector<Eigen::Isometry3f, Eigen::aligned_allocator<Eigen::Isometry3f>> Planning_path;
    std::vector<Eigen::Isometry3f, Eigen::aligned_allocator<Eigen::Isometry3f>> Planning_final_path;
    double theta_move;
    float x_move;
    bool finish_planning,target_useless;
    int nan_mun;
    int min_point_num;
};
#endif