#pragma once

#include <networktables/NetworkTable.h>
#include "networktables/NetworkTableEntry.h"
#include "networktables/NetworkTableInstance.h"

#include "ros/ros.h"

#include <memory>
#include <string>

#include "sensor_msgs/PointCloud2.h"

#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/conversions.h>
#include <pcl/common/common.h>
#include <pcl/common/centroid.h>
#include <pcl/point_types.h>
#include <pcl/PointIndices.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>

namespace plane_detection_node{
class plane_detection
{
    public:
    pcl::ModelCoefficients coefficients;
    pcl::PointIndices::Ptr inliers {new pcl::PointIndices ()};
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    pcl::ExtractIndices<pcl::PointXYZ> extract;
    pcl::PassThrough<pcl::PointXYZ> pass;

    nt::NetworkTableInstance inst;
    std::shared_ptr<NetworkTable> Table;

    nt::NetworkTableEntry xEntry;
    nt::NetworkTableEntry yEntry;
    nt::NetworkTableEntry zEntry;
    nt::NetworkTableEntry tEntry;
    
    plane_detection();
    ~plane_detection();

    void callback(const sensor_msgs::PointCloud2& cloud){
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered( new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_initial( new pcl::PointCloud<pcl::PointXYZ>);

        //convert pcl2 --> pcl<XYZ>
        pcl::PCLPointCloud2 pcl_pc2;
        pcl_conversions::toPCL(cloud, pcl_pc2);
        
        pcl::fromPCLPointCloud2 (pcl_pc2, *cloud_initial);
        //filter out bottom 1/2 add paramter in launch to set true or not
        if(true){
            pass.setInputCloud(cloud_initial);
            pass.setFilterFieldName("z");
            pass.setFilterLimits(0.0, 3.0);
            pass.filter(*cloud_filtered);
            seg.setInputCloud(cloud_filtered);   
        }else 
        {
            seg.setInputCloud(*cloud_initial);
        }

        //Create Segmentation object and segment
        
        seg.segment(*inliers, coefficients);

        if (*inliers->indices.size() == 0)
        {
            ROS_WARN("Could not estimate a planar model for the given dataset.");

        }
    
        //Calculate centroid for average distance of center 4 readings if in plane
        Eigen::Vector4f centroid;
        pcl::compute3DCentroid(cloud_filtered, *inliers, centroid);
        //calculate angle from centroid point and origin- check part of centroid
    
        //write to network tables
        xEntry.SetDouble(centroid(0));
        yEntry.SetDouble(centroid(1));
        zEntry.SetDouble(centroid(2));
        tEntry.SetDouble(centroid(3));
    }

    void init()
    {
        inst = nt::NetworkTableInstance::GetDefault();
        Table = inst.GetTable("Evo_64px_1");

        xEntry = Table->GetEntry("x");
        yEntry = Table->GetEntry("y");
        zEntry = Table->GetEntry("z");
        tEntry = Table->GetEntry("t");
        inst.StartClientTeam(7054);

        //optional
        seg.setOptimizeCoefficients(true);
        //Mandatory
        seg.setModelType(pcl::SACMODEL_PLANE);
        seg.setMethodType(pcl::SAC_RANSAC);
        seg.setDistanceThreshold(.01);
    }
    
};
}