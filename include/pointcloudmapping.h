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

#ifndef POINTCLOUDMAPPING_H
#define POINTCLOUDMAPPING_H

#include <pcl/common/transforms.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>//for pcl

#include <opencv2/core/core.hpp>//for opencv

#include <mutex>
#include <condition_variable>

#include </usr/local/octomap/include/octomap/octomap.h>    // for octomap 
#include </usr/local/octomap/include/octomap/ColorOcTree.h>

class ColorOcTree;
#include "System.h"

using namespace ORB_SLAM2;

class PointCloudMapping
{
public:

    typedef pcl::PointXYZRGBA PointT;
    typedef pcl::PointCloud<PointT> PointCloud;

    
    struct PointCloude
    {
        PointCloud::Ptr pcE;
        Eigen::Isometry3d T; 
        int pcID;
    };
    


    PointCloudMapping( double resolution_,double meank_,double thresh_ );
    void save();

    // 插入一个keyframe，会更新一次地图
    void insertKeyFrame2( KeyFrame* kf, cv::Mat& color, cv::Mat& depth,int idk);
    void shutdown();
    void viewer();
 
protected:
    PointCloud::Ptr generatePointCloud(KeyFrame* kf, cv::Mat& color, cv::Mat& depth);
    
    PointCloud::Ptr globalMap;
    shared_ptr<thread>  viewerThread;   
    
    bool    shutDownFlag    =false;
    mutex   shutDownMutex;  
    
    condition_variable  keyFrameUpdated;
    mutex               keyFrameUpdateMutex;
 
    mutex                   keyframeMutex;
    
    double resolution;
    double meank;
    double thresh;
    pcl::VoxelGrid<PointT>  voxel;//体素滤波器
    pcl::StatisticalOutlierRemoval<PointT> statistical_filter;//离群点滤波
    shared_ptr<octomap::ColorOcTree> globalTree;
    //octomap
    //octomap::ColorOcTree globalTree();
};
#endif // POINTCLOUDMAPPING_H
