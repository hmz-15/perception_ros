#include <iostream>
#include <thread>

#include "ApproxMVBB/ComputeApproxMVBB.hpp"

#include <ros/ros.h>
#include <ros/package.h>
#include <pcl/io/ply_io.h>
// #include <pcl/io/pcd_io.h>

#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>

#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>

#include <pcl/features/moment_of_inertia_estimation.h>
#include <pcl/visualization/cloud_viewer.h>

#include <pcl/common/common.h>

#include <Eigen/Core>
// #include "ApproxMVBB/MinAreaRectangle.hpp"




using PointT = pcl::PointXYZ;

struct OBBox
{
    Eigen::Vector3f pos;
    Eigen::Vector3f scale; // x,y,z in world frame
    Eigen::Quaternionf quat;
    
};

void DetectPlane(const pcl::PointCloud<PointT>::Ptr& src_cloud, std::vector<pcl::PointCloud<PointT>::Ptr>& clouds)
{
    ros::Time start = ros::Time::now();

    // Eigen::Vector3f axis = Eigen::Vector3f(0.0,1.0,0.0); //y axis for sceneNN
    // seg.setAxis(axis);
    // seg.setEpsAngle( 30.0f * (3.14159265359/180.0f) ); // plane can be within 30 degrees of X-Z plane
    int i = 1;
    clouds.clear();
    pcl::PointCloud<PointT>::Ptr input_cloud (new pcl::PointCloud<PointT>);
    pcl::copyPointCloud(*src_cloud, *input_cloud);

    while (1)
    {
        pcl::PointCloud<PointT>::Ptr result_cloud (new pcl::PointCloud<PointT>);

        pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
        pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
        // Create the segmentation object
        pcl::SACSegmentation<PointT> seg;
        // Optional
        seg.setOptimizeCoefficients (true);
        // Mandatory
        // seg.setModelType (pcl::SACMODEL_PERPENDICULAR_PLANE);
        seg.setModelType (pcl::SACMODEL_PLANE);
        seg.setMethodType (pcl::SAC_RANSAC);
        seg.setDistanceThreshold (0.02);
        seg.setInputCloud (input_cloud);
        seg.segment (*inliers, *coefficients);
        if (inliers->indices.size () == 0)
        {
            PCL_ERROR ("Could not estimate a planar model for the given dataset.");
            return;
        }
        std::cout << "Iteration" + std::to_string(i) <<std::endl;
        std::cout << "Model coefficients: " << coefficients->values[0] << " " 
                                            << coefficients->values[1] << " "
                                            << coefficients->values[2] << " " 
                                            << coefficients->values[3] << std::endl;

        std::cout << "Model inliers: " << inliers->indices.size () << std::endl;

        pcl::ExtractIndices<PointT> extract;
        extract.setInputCloud (input_cloud);
        extract.setIndices (inliers);
        extract.setNegative (false);
        extract.filter (*result_cloud);

        extract.setIndices (inliers);
        extract.setNegative (true);
        extract.setKeepOrganized (true);
        extract.filter (*input_cloud);

        clouds.push_back(result_cloud);

        if ((inliers->indices.size () < 0.1 * src_cloud->points.size()) || (input_cloud->points.size() < 0.1 * src_cloud->points.size()))
        {
            std::cout<<"Stop estimation"<<std::endl;
            break;
        }
        i++;
    }
}


void DetectBBox(const pcl::PointCloud<PointT>::Ptr& src_cloud, OBBox& bbox)
{
    pcl::PointCloud<PointT>::Ptr cloud (new pcl::PointCloud<PointT>);
    pcl::copyPointCloud(*src_cloud, *cloud);

    for (int i = 0; i < cloud->points.size(); i++)
        cloud->points[i].y = 0.0;

    pcl::MomentOfInertiaEstimation <PointT> feature_extractor;
    feature_extractor.setInputCloud (cloud);
    feature_extractor.compute ();

    std::vector <float> moment_of_inertia;
    std::vector <float> eccentricity;
    pcl::PointXYZ min_point_AABB;
    pcl::PointXYZ max_point_AABB;
    pcl::PointXYZ min_point_OBB;
    pcl::PointXYZ max_point_OBB;
    pcl::PointXYZ position_OBB;
    Eigen::Matrix3f rotational_matrix_OBB;
    float major_value, middle_value, minor_value;
    Eigen::Vector3f major_vector, middle_vector, minor_vector;
    Eigen::Vector3f mass_center;

    // feature_extractor.getMomentOfInertia (moment_of_inertia);
    // feature_extractor.getEccentricity (eccentricity);
    // feature_extractor.getAABB (min_point_AABB, max_point_AABB);
    feature_extractor.getOBB (min_point_OBB, max_point_OBB, position_OBB, rotational_matrix_OBB);
    // feature_extractor.getEigenValues (major_value, middle_value, minor_value);
    // feature_extractor.getEigenVectors (major_vector, middle_vector, minor_vector);
    // feature_extractor.getMassCenter (mass_center);

    Eigen::Vector4f min;
    Eigen::Vector4f max;
    pcl::getMinMax3D(*src_cloud, min, max);

    bbox.pos << position_OBB.x, (min(1) + max(1))/2, position_OBB.z ;
    bbox.scale << (max_point_OBB.x - min_point_OBB.x), (max_point_OBB.y - min_point_OBB.y), (max(1) - min(1));
    // bbox.scale << (max_point_OBB.x - min_point_OBB.x), (max_point_OBB.y - min_point_OBB.y), (max_point_OBB.z - min_point_OBB.z) ;
    Eigen::Quaternionf quat (rotational_matrix_OBB);
    bbox.quat = quat;

    // pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    // viewer->setBackgroundColor (0, 0, 0);
    // viewer->addCoordinateSystem (1.0);
    // viewer->initCameraParameters ();
    // // viewer->addPointCloud<pcl::PointXYZ> (cloud, "sample cloud");
    // // viewer->addCube (min_point_AABB.x, max_point_AABB.x, min_point_AABB.y, max_point_AABB.y, min_point_AABB.z, max_point_AABB.z, 1.0, 1.0, 0.0, "AABB");
    // // viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION, pcl::visualization::PCL_VISUALIZER_REPRESENTATION_WIREFRAME, "AABB");

    // // Eigen::Vector3f position (position_OBB.x, position_OBB.y, position_OBB.z);
    // // Eigen::Quaternionf quat (rotational_matrix_OBB);
    // viewer->addCube (bbox.pos, bbox.quat, bbox.scale(0), bbox.scale(1), bbox.scale(2), "OBB");
    // viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION, pcl::visualization::PCL_VISUALIZER_REPRESENTATION_WIREFRAME, "OBB");

    // pcl::PointXYZ center (mass_center (0), mass_center (1), mass_center (2));
    // pcl::PointXYZ x_axis (major_vector (0) + mass_center (0), major_vector (1) + mass_center (1), major_vector (2) + mass_center (2));
    // pcl::PointXYZ y_axis (middle_vector (0) + mass_center (0), middle_vector (1) + mass_center (1), middle_vector (2) + mass_center (2));
    // pcl::PointXYZ z_axis (minor_vector (0) + mass_center (0), minor_vector (1) + mass_center (1), minor_vector (2) + mass_center (2));
    // viewer->addLine (center, x_axis, 1.0f, 0.0f, 0.0f, "major eigen vector");
    // viewer->addLine (center, y_axis, 0.0f, 1.0f, 0.0f, "middle eigen vector");
    // viewer->addLine (center, z_axis, 0.0f, 0.0f, 1.0f, "minor eigen vector");

    // while(!viewer->wasStopped())
    // {
    //     viewer->spinOnce (100);
    //     std::this_thread::sleep_for(std::chrono::milliseconds(100));
    // }
}


void DetectMinimalBBox(const pcl::PointCloud<PointT>::Ptr& src_cloud, OBBox& bbox)
{
    pcl::PointCloud<PointT>::Ptr cloud (new pcl::PointCloud<PointT>);
    pcl::copyPointCloud(*src_cloud, *cloud);

    int ground_axis = 1; // y axis

    ApproxMVBB::Matrix2Dyn points(2, cloud->points.size());  // x, y in approxMVBB
    
    for (size_t i = 0u; i < cloud->points.size(); ++i) 
    {
        if (ground_axis == 0)
        {
            points(0, i) = double(cloud->points[i].y);
            points(1, i) = double(cloud->points[i].z);
        }
        else if (ground_axis == 1)
        {
            points(0, i) = double(cloud->points[i].z);
            points(1, i) = double(cloud->points[i].x);
        }
        else if (ground_axis == 2)
        {
            points(0, i) = double(cloud->points[i].x);
            points(1, i) = double(cloud->points[i].y);
        }
    }

    ApproxMVBB::MinAreaRectangle min_rectangle(points);
    min_rectangle.compute();
    ApproxMVBB::MinAreaRectangle::Box2d box_2d = min_rectangle.getMinRectangle();

    Eigen::Vector2d pos_2d = box_2d.m_p + box_2d.m_v * box_2d.m_vL/2 + box_2d.m_u * box_2d.m_uL/2;
    Eigen::Vector4f min;
    Eigen::Vector4f max;
    pcl::getMinMax3D(*src_cloud, min, max);

    Eigen::Matrix3f rotation;
    if (ground_axis == 0)
        rotation << 1, 0,             0,
                    0, box_2d.m_u(0), box_2d.m_v(0),
                    0, box_2d.m_u(1), box_2d.m_v(1);
    else if (ground_axis == 1)
        rotation << box_2d.m_v(1), 0, box_2d.m_u(1),
                    0,             1, 0,
                    box_2d.m_v(0), 0, box_2d.m_u(0);
    else if (ground_axis == 2)
        rotation << box_2d.m_u(0), box_2d.m_v(0), 0,
                    box_2d.m_u(1), box_2d.m_v(1), 0,
                    0,             0,             1;


    bbox.pos(ground_axis) = (min(ground_axis) + max(ground_axis))/2;
    bbox.pos((ground_axis+1)%3) = pos_2d(0);
    bbox.pos((ground_axis+2)%3) = pos_2d(1);

    bbox.scale(ground_axis) = max(ground_axis) - min(ground_axis);
    bbox.scale((ground_axis+1)%3) = box_2d.m_uL;
    bbox.scale((ground_axis+2)%3) = box_2d.m_vL;

    bbox.quat = Eigen::Quaternionf(rotation);

    pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    viewer->setBackgroundColor (0, 0, 0);
    viewer->addCoordinateSystem (1.0);
    viewer->initCameraParameters ();
    viewer->addPointCloud<pcl::PointXYZ> (cloud, "sample cloud");

    viewer->addCube (bbox.pos, bbox.quat, bbox.scale(0), bbox.scale(1), bbox.scale(2), "OBB");
    viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION, pcl::visualization::PCL_VISUALIZER_REPRESENTATION_WIREFRAME, "OBB");

    while(!viewer->wasStopped())
    {
        viewer->spinOnce (100);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "alignment_test");
    ros::start();

    std::string path = ros::package::getPath("perception_ros");

    pcl::PLYReader reader;
    pcl::PLYWriter writer;
    pcl::PointCloud<PointT>::Ptr src_cloud (new pcl::PointCloud<PointT>);
    pcl::PointCloud<PointT>::Ptr tgt_cloud (new pcl::PointCloud<PointT>);

    

    reader.read (path + "/data/chair_cad_sample1.ply", *tgt_cloud);

    ROS_INFO("read chair_cad");
    reader.read (path + "/data/53.ply", *src_cloud);
    
    ROS_INFO("read chair_seg");

    // std::vector<pcl::PointCloud<PointT>::Ptr> clouds = {};
    // DetectPlane(src_cloud, clouds);
    
    // for (int j = 0; j < clouds.size(); j++)
    //     writer.write(path + "/data/result" + std::to_string(j) + ".ply", *(clouds[j]));
    OBBox bbox;
    DetectMinimalBBox(src_cloud, bbox);

    pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    viewer->setBackgroundColor (0, 0, 0);
    viewer->addCoordinateSystem (1.0);
    viewer->initCameraParameters ();
    viewer->addPointCloud<pcl::PointXYZ> (src_cloud, "sample cloud");
    // viewer->addCube (min_point_AABB.x, max_point_AABB.x, min_point_AABB.y, max_point_AABB.y, min_point_AABB.z, max_point_AABB.z, 1.0, 1.0, 0.0, "AABB");
    // viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION, pcl::visualization::PCL_VISUALIZER_REPRESENTATION_WIREFRAME, "AABB");

    // Eigen::Vector3f position (position_OBB.x, position_OBB.y, position_OBB.z);
    // Eigen::Quaternionf quat (rotational_matrix_OBB);
    std::cout<<bbox.scale<<std::endl;
    viewer->addCube (bbox.pos, bbox.quat, bbox.scale(0), bbox.scale(1), bbox.scale(2), "OBB");
    viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION, pcl::visualization::PCL_VISUALIZER_REPRESENTATION_WIREFRAME, "OBB");

    while(!viewer->wasStopped())
    {
        viewer->spinOnce (100);
        ros::spinOnce();
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    ros::shutdown();

    return 0;
}
