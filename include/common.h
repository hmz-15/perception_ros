#ifndef COMMON_H_
#define COMMON_H_

#include <iostream>
#include <vector>

#include <ros/ros.h>
#include <ros/console.h>
#include <ros/package.h>

// include pcl header first to avoid building error
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl_ros/point_cloud.h>

#include <opencv2/core/core.hpp>
#include <opencv2/core/eigen.hpp>
#include <opencv2/core/utility.hpp>

#include <Eigen/Core>

struct PointSurfelLabel {
  PCL_ADD_POINT4D;
  PCL_ADD_NORMAL4D;
  PCL_ADD_RGB;
  uint8_t instance_label;
  uint8_t semantic_label;

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;

POINT_CLOUD_REGISTER_POINT_STRUCT(
    PointSurfelLabel,
    (float, x, x)(float, y, y)(float, z, z)(float, normal_x, normal_x)(
        float, normal_y, normal_y)(float, normal_z, normal_z)(float, rgb, rgb)(
        uint8_t, instance_label, instance_label)(uint8_t, semantic_label,
                                                 semantic_label))


namespace PerceptionROS
{

struct Obj2D{
    int id;
    int category;
    float score;
    cv::Rect box; //top left x, y, width, height
    int area;
    bool is_dynamic;
    cv::Vec3b color;
    std::string cate_name;
};

struct Sem2D{
    int id;
    int category;
    int area;
    cv::Vec3b color;
    std::string cate_name;
};

struct ObjClass{
    cv::Vec3b color;
    int category_id;
    std::string name;
};

struct SemClass{
    cv::Vec3b color;
    std::vector<int> category_id;
    std::string name;
};

struct Obj3D{
    int id;
    // int category;
    std::string category_name;
    Eigen::Vector3f pos;
    Eigen::Vector3f aligned_dim; // x,y,z in world frame
    Eigen::Quaternionf quat; // x,y,z,w
    std::vector<Eigen::Vector4f> planes; // a,b,c,d
    pcl::PolygonMesh::Ptr mesh;
};

struct ObjCAD{
    std::string cad_id;
    std::string category_name;
    std::vector<Eigen::Vector4f> planes;
    Eigen::Vector3f aligned_dim;
};


enum GeometricSegMode{
    kDepth = 0,
    kPointCloud = 1
};


}
#endif