/*
 * <one line to give the program's name and a brief idea of what it does.>
 * Copyright (C) 2016  <copyright holder> <email>
 * 
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 * 
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 * 
 */

#include "pointcloudmapping.h"
#include <KeyFrame.h>
#include <opencv2/highgui/highgui.hpp>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include "Converter.h"
#include "System.h"

PointCloudMapping::PointCloudMapping(double resolution_,double meank_,double thresh_)
{
    this->resolution = resolution_;
    this->meank = thresh_;
    this->thresh = thresh_;
    statistical_filter.setMeanK(meank);
    statistical_filter.setStddevMulThresh(thresh);
    voxel.setLeafSize( resolution, resolution, resolution);//体素的体积
    globalMap = boost::make_shared< PointCloud >( ); 
    globalTree = make_shared<octomap::ColorOcTree>(0.05);  
    viewerThread = make_shared<thread>( bind(&PointCloudMapping::viewer, this ) );
}
 
void PointCloudMapping::shutdown()
{
    {
        unique_lock<mutex> lck(shutDownMutex);
        shutDownFlag = true;
        keyFrameUpdated.notify_one();
    }
    viewerThread->join();
}
 
 
void PointCloudMapping::insertKeyFrame2(KeyFrame* kf, cv::Mat& color, cv::Mat& depth,int idk)
{
    cout<<"receive a keyframe, id = "<<idk<<" 第"<<kf->mnId<<"个"<<endl;
    PointCloude pointcloude;
    pointcloude.pcID = idk;
    pointcloude.T = ORB_SLAM2::Converter::toSE3Quat( kf->GetPose() );
    pointcloude.pcE = generatePointCloud(kf,color,depth);
    PointCloud::Ptr p (new PointCloud);
    pcl::transformPointCloud( *(pointcloude.pcE), *p, pointcloude.T.inverse().matrix());

    //离群点过滤
    PointCloud::Ptr tmp1 ( new PointCloud );
    statistical_filter.setInputCloud(p);
    statistical_filter.filter( *tmp1 );

    //过滤后生成八叉树地图
    octomap::Pointcloud cloud_octo;
    for (auto p1:tmp1->points)//把pcl点云转化为octomap的点云
        cloud_octo.push_back( p1.x, p1.y, p1.z );

    Eigen::Isometry3d pose = pointcloude.T.inverse();
    globalTree->insertPointCloud(cloud_octo,//将点云插入八叉树
        octomap::point3d(pose(0,3), pose(1,3), pose(2,3)));

    for(auto p_octo:tmp1->points)//添加色彩信息
        globalTree->integrateNodeColor(p_octo.x,p_octo.y,p_octo.z,p_octo.r,p_octo.g,p_octo.b);
    
    globalTree->updateInnerOccupancy();//八叉树更新


    *globalMap += *tmp1;//点云拼接
    
    //体素降采样
    PointCloud::Ptr tmp(new PointCloud());
    voxel.setInputCloud( globalMap);
    voxel.filter( *tmp );
    tmp->swap( *globalMap );
    cout<<"当前关键帧点云数量为："<<pointcloude.pcE->points.size()<<endl;
    cout<<"融合后关键帧点云数量为："<<globalMap->points.size()<<endl;

    if((kf->mnId)%20==10)
    {
      globalMap->clear();
      cout<<"****************清空global点云数据******************"<<endl;
    }
    if((kf->mnId)%20==5)
    {
      cout<<"*************唤醒view线程更新点云模型****************"<<endl;
      keyFrameUpdated.notify_one();
    }
}


pcl::PointCloud< PointCloudMapping::PointT >::Ptr PointCloudMapping::generatePointCloud(KeyFrame* kf, cv::Mat& color, cv::Mat& depth)//,Eigen::Isometry3d T
{
    PointCloud::Ptr tmp( new PointCloud() );
    // point cloud is null ptr
    for ( int m=0; m<depth.rows; m+=3 )
    {
        for ( int n=0; n<depth.cols; n+=3 )
        {
            float d = depth.ptr<float>(m)[n];
            if (d < 0.01 || d>5)
                continue;
            PointT p;
            p.z = d;
            p.x = ( n - kf->cx) * p.z / kf->fx;
            p.y = ( m - kf->cy) * p.z / kf->fy;
            
            p.b = color.ptr<uchar>(m)[n*3];
            p.g = color.ptr<uchar>(m)[n*3+1];
            p.r = color.ptr<uchar>(m)[n*3+2];
                
            tmp->points.push_back(p);
        }
    }
    
    return tmp;
}

void PointCloudMapping::viewer()
{
    pcl::visualization::CloudViewer viewer("viewer");
    while(1)
    {
        
        {
	    
            unique_lock<mutex> lck_shutdown( shutDownMutex );
            if (shutDownFlag)
            {
                break;
            }
        }
        {
	    cout<<"******************view线程进入睡眠*********************"<<endl;
            unique_lock<mutex> lck_keyframeUpdated( keyFrameUpdateMutex );
            keyFrameUpdated.wait( lck_keyframeUpdated );
        }
        
        // keyframe is updated 
        cout<<"***************在view线程中显示点云模型******************"<<endl;
        viewer.showCloud( globalMap );
    }
}

void PointCloudMapping::save()
{
	pcl::io::savePCDFile( "result0.01.pcd", *globalMap );
	cout<<"globalMap save finished"<<endl;

    cout<<"saving octomap ... "<<endl;
    globalTree->write( "resul_mask.ot" ); 
}
