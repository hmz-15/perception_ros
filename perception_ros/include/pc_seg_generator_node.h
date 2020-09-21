#ifndef PC_SEG_GENERATOR_NODE_H_
#define PC_SEG_GENERATOR_NODE_H_

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_cloud.h>
// #include <tf/transform_broadcaster.h>
// #include <tf/transform_listener.h>

#include "common.h"
#include "pc_processing.h"
#include "depth_segmentation/depth_segmentation.h"
#include <seg_msgs/Seg.h>

namespace PerceptionROS
{
    
class PCSegGeneratorNode
{

public:
    PCSegGeneratorNode(ros::NodeHandle &node_handle);
    ~PCSegGeneratorNode();

    // image callback to generate point cloud segments
    void ImageCallback (const sensor_msgs::ImageConstPtr& msgRGB,const sensor_msgs::ImageConstPtr& msgD);
    void ImageSegCallback (const sensor_msgs::ImageConstPtr& msgRGB,const sensor_msgs::ImageConstPtr& msgD,const seg_msgs::SegConstPtr& msgSeg);

    // camera info callback to initialize camera intrinsics
    void CamInfoCallback (const sensor_msgs::CameraInfoConstPtr& msgInfo);

    // void SegCallback (const seg_msgs::SegConstPtr& msgSeg);

    void SetCamInfo();

    // process messages
    void ProcessImageMsg (const sensor_msgs::ImageConstPtr& msgRGB, const sensor_msgs::ImageConstPtr& msgD);
    void ProcessSegMsg (const seg_msgs::SegConstPtr& msgSeg);
    
    void Update();

    void LabelPC();

    cv::Mat DrawInstSeg();


private:
    // image subscibers
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image, seg_msgs::Seg> seg_sync_pol;
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> sync_pol;
    message_filters::Subscriber<sensor_msgs::Image> *rgb_subscriber_;
    message_filters::Subscriber<sensor_msgs::Image> *depth_subscriber_;
    message_filters::Subscriber<seg_msgs::Seg> *seg_subscriber_;
    message_filters::Synchronizer<sync_pol> *sync_;
    message_filters::Synchronizer<seg_sync_pol> *seg_sync_;     

    // camera info subscriber
    ros::Subscriber info_sub_;

    // operation modes
    bool use_semantic_segmentation = false;
    bool use_geometric_segmentation = false;
    bool use_distance_check = false;
    bool use_GT_camera_frame = true;
    std::string camera_frame;
    int geo_seg_mode = GeometricSegMode::kPointCloud;
    bool visualize_geo_seg = false;
    bool pub_seg_img = false;
    bool save_img = false;

    // Camera intrinsics
    cv::Mat K = cv::Mat::eye(3, 3, CV_32FC1);
    int width;
    int height;
    bool camera_info_ready = false;

    // Frame time and id
    std::string camera_frame_id_;
    ros::Time current_frame_time_ = ros::Time::now(); ;

    // Images to process
    cv::Mat imRGB;
    cv::Mat imDepth;
    cv::Mat imSeg;

    // Detection results
    std::vector<Obj2D> objects = {};
    std::vector<Sem2D> semantics = {}; 
    std::vector<int> mov_list = {};
    std::map<int, int> instance_category_pairs = {};
    std::map<int, int> instance_area_pairs = {};
    std::map<int, int> semantics_category_pairs = {};

    // Point cloud segments
    std::vector<pcl::PointCloud<PointSurfelLabel>::Ptr> clouds = {};
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud;

    // tf listener
    // tf::TransformListener tf_listener_;

    // Publishers
    ros::Publisher point_cloud_segment_publisher_;
    ros::Publisher inst_seg_image_publisher_;

    // Node handle
    ros::NodeHandle node_handle_;

    // segmenters
    PCProcessor* mpc_processor;
    depth_segmentation::DepthSegmenter* mdepth_segmenter;

    bool verbose;


};


}


#endif