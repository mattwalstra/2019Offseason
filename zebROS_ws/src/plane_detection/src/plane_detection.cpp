
#include <networktables/NetworkTable.h>
#include <networktables/NetworkTableEntry.h>
#include <networktables/NetworkTableInstance.h>

#include "ros/ros.h"

#include <memory>
#include <string>

#include "sensor_msgs/PointCloud2.h"
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/conversions.h>
#include <pcl/common/common.h>

//#include <Eigen/Core>

//create segmentation object and data
pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
pcl::PointIndicies::Ptr inliers (new pcl::PointIndicies);
pcl::SACSegmentation<pcl::PointXYZ> seg;
pcl::ExtractIndices<pcl::PointXYZ> extract;
pcl::PassThrough<pcl::PointXYZ> pass;

NetworkTable Table;
NetworkTableInstance inst;
NetworkTableEntry xEntry;
NetworkTableEntry yEntry;
NetworkTableEntry zEntry;
NetworkTableEntry tEntry;


void callback(const sensor_msgs::PointCloud2ConstPtr& cloud){
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered( new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_initial( new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_final( new pcl::PointCloud<pcl::PointXYZ>);
    //convert pcl2 --> pcl<XYZ>
    pcl::fromPCLPointCloud2 (*cloud, *cloud_initial);
    //filter out bottom 1/2 add paramter in launch to set true or not
    if(true){
        pass.setInputCloud(cloud_initial);
        pass.setFilterFieldName("z");
        pass.setFilterLimits(0.0, 3.0);
        pass.filter(*cloud_filtered)   
    }

    //Create Segmentation object and segment
    seg.setInputCloud(cloud_filtered);
    seg.segment(*inliers, *coefficients);

    if (inliers->indicies.size() == 0)
    {
        ROS_WARN("Could not estimate a planar model for the given dataset.");

    }
    
    //Calculate centroid for average distance of center 4 readings if in plane
    Eigen::Vector4f centroid;
    pcl::compute3DCentroid(cloud_filtered, inliers, centroid);
    //calculate angle from centroid point and origin- check part of centroid
    
    //write to network tables
    xEntry.SetDouble(centroid(0));
    yEntry.SetDouble(centroid(1));
    zEntry.SetDouble(centroid(2));
    tEntry.SetDouble(centroid(3));
}

main(int argc, char** argv){
    ros::init(arc, argv, "plane_detection");

    ros::NodeHandle n;
    ros::Subscriber sub = n.subscribe("evo_64px_1/point_cloud", 1000, callback);
    
    inst = NetworkTableInstance.GetDefault();
    Table = inst.GetTable("Evo_64px_1");

    xEntry = table.GetEntry("x");
    yEntry = table.GetEntry("y");
    zEntry = table.GetEntry("z");
    tEntry = table.GetEntry("t");
    inst.StartClientTeam(7054);

    //optional
    seg.setOptimizeCoefficients(true);
    //Mandatory
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC)
    seg.setDistanceThreshold(.01);

    ros::spin();
}

