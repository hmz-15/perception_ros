#include <iostream>
#include <ros/ros.h>
#include <ros/package.h>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/photo/photo.hpp>


int main(int argc, char **argv)
{
    ros::init(argc, argv,"image_test");
    ros::start();

    std::string path = ros::package::getPath("perception_ros");
    cv::Mat map = cv::imread(path + "/image/depth/377.png", CV_LOAD_IMAGE_ANYCOLOR | CV_LOAD_IMAGE_ANYDEPTH);

    double min;
    double max;
    cv::minMaxIdx(map, &min, &max);
    cv::Mat adjMap;
    cv::convertScaleAbs(map, adjMap, 255 / max);
    ROS_INFO("Adjust depth for visualization!");

    cv::imwrite(path + "/image/377.png",adjMap);


    ros::spin();

    

    ros::shutdown();
    return 0;
}