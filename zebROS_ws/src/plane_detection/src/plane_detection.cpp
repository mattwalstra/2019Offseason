#include "plane_detection/plane_detection.h"

using namespace plane_detection_node;

plane_detection::plane_detection()
{
    init();
}

plane_detection::~plane_detection(){}

int main(int argc, char** argv){
    ros::init(argc, argv, "plane_detection");

    ros::NodeHandle n;

    plane_detection *plane_example = new plane_detection();
    ros::Subscriber sub = n.subscribe("evo_64px_1/point_cloud", 1000, &plane_detection::callback, plane_example);

    ros::spin();
}

