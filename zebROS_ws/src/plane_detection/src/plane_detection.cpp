#include <networktables/NetworkTable.h>
#include "networktables/NetworkTableEntry.h"
#include "networktables/NetworkTableInstance.h"

#include "ros/ros.h"

#include <memory>
#include <string>
#include <cmath>

#include "sensor_msgs/PointCloud2.h"

#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl_conversions/pcl_conversions.h>
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
    pcl::ModelCoefficients::Ptr coefficients{ new pcl::ModelCoefficients()};
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
    
    void callback(const sensor_msgs::PointCloud2& cloud)
    {
    pcl::PointCloud<pcl::PointXYZ> cloud_filtered;
    pcl::PointCloud<pcl::PointXYZ> cloud_initial;

        //pcl::PointCloud<pcl::PointXYZ> cloud_filtered{new pcl::PointCloud<pcl::PointXYZ>};
        //pcl::PointCloud<pcl::PointXYZ> cloud_initial{new pcl::PointCloud<pcl::PointXYZ>};

        //convert pcl2 --> pcl<XYZ>
    pcl::fromROSMsg(cloud, cloud_initial);
    const pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud_initial_const {new pcl::PointCloud<pcl::PointXYZ>(cloud_initial)};
    //if (false)
    //{    
        //filter out bottom 1/2 add paramter in launch to set true or not
        //if(true){
    //pass.setInputCloud(cloud_initial_const);
    //pass.setFilterFieldName("z");
    //pass.setFilterLimits(0.0, 3.0);
    //pass.filter(cloud_filtered);

    //const pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud_filtered_const { new pcl::PointCloud<pcl::PointXYZ>(cloud_filtered)};
    //seg.setInputCloud(cloud_filtered_const);   
    //} else
    //{
        seg.setInputCloud(cloud_initial_const);
    //}
    
        //}else 
        //{
        //    seg.setInputCloud(cloud_initial);
        //}

    //Create Segmentation object and segment
        
    seg.segment(*inliers, *coefficients);

    if (inliers->indices.size() == 0)
    {
        ROS_WARN("Could not estimate a planar model for the given dataset.");

    }

    //Calculate centroid for average distance of plane
    Eigen::Vector4f centroid;
    //pcl::compute3DCentroid(cloud_filtered, *inliers, centroid);
    pcl::compute3DCentroid(cloud_inital, *inliers, centroid);
    //calculate angle from equation coefficients t=asin(A/B); Ax+By+Cz+D=0;
    double t = std::asin(coefficients->values[1]/coefficients->values[0]);
    
    ROS_INFO("X: %f, Y: %f, Z: %f, T: %f", centroid(0), centroid(1), centroid(2), t);
    ROS_INFO("A: %f, B: %f, C: %f, D: %f", coefficients->values[0],coefficients->values[1],coefficients->values[2],coefficients->values[3]);
    ROS_INFO("Size: %i", inliers->indices.size());
     
    //write to network tables
    xEntry.SetDouble(centroid(0));
    yEntry.SetDouble(centroid(1));
    zEntry.SetDouble(centroid(2));
    tEntry.SetDouble(t);
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

    xEntry.SetDouble(-1.0);
    yEntry.SetDouble(-1.0);
    zEntry.SetDouble(-1.0);
    tEntry.SetDouble(-1.0);

    //optional
    //seg.setOptimizeCoefficients(true);
    //Mandatory
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setDistanceThreshold(.01);
    }

    plane_detection()
    {
        init();
    }
    ~plane_detection(){}

    
    
    
};
}

using namespace plane_detection_node;






int main(int argc, char** argv){
    ros::init(argc, argv, "plane_detection");

    ros::NodeHandle n;

    plane_detection *plane_example = new plane_detection();
    ros::Subscriber sub = n.subscribe("/evo_64px_1/teraranger_evo_64px/point_cloud", 1000, &plane_detection::callback, plane_example);

    ros::spin();
}

