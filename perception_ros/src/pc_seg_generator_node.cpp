#include "common.h"
#include "pc_seg_generator_node.h"
#include "depth_segmentation/depth_segmentation.h"


namespace PerceptionROS
{

const std::vector<ObjClass> obj_lib = {
    // rgb color, class id, class name
    {{220, 20, 60}, 0, "Person"}, 
    // {{119, 11, 32}, 1, "bicycle"}, 
    {{250, 0, 30}, 13, "Bench"}, 
    {{255, 179, 240}, 24, "Backpack"},
    // {{209, 0, 151}, 26, "handbag"}, 
    // {{0, 220, 176}, 28, "suitcase"}, 
    {{197, 226, 255}, 39, "Bottle"}, 
    // {{171, 134, 1}, 40, "wine glass"}, 
    {{109, 63, 54}, 41, "Cup"}, 
    // {{84, 105, 51}, 45, "bowl"}, 
    {{153, 69, 1}, 56, "Chair"}, 
    {{3, 95, 161}, 57, "Couch"}, 
    // {{163, 255, 0}, 58, "PottedPlant"}, 
    {{119, 0, 170}, 59, "Bed"}, 
    {{0, 182, 199}, 60, "Table"}, 
    {{0, 165, 120}, 61, "Toilet"}, 
    {{183, 130, 88}, 62, "TV"}, 
    {{95, 32, 0}, 63, "Laptop"}, 
    {{130, 114, 135}, 64, "Mouse"}, 
    // {{110, 129, 133}, 65, "Remote"}, 
    {{166, 74, 118}, 66, "Keyboard"}, 
    // {{219, 142, 185}, 67, "cell phone"}, 
    {{79, 210, 114}, 68, "Microwave"}, 
    {{178, 90, 62}, 69, "Oven"}, 
    // {{65, 70, 15}, 70, "toaster"}, 
    // {{127, 167, 115}, 71, "Sink"}, 
    {{59, 105, 106}, 72, "Refrigerator"}, 
    {{142, 108, 45}, 73, "Book"}
    // {{196, 172, 0}, 74, "Clock"}, 
    // {{95, 54, 80}, 75, "vase"}, 
    // {{128, 76, 255}, 76, "scissors"}, 
    // {{201, 57, 1}, 77, "teddy bear"}
    // {{246, 0, 122}, 78, "hair drier"}, 
    // {{191, 162, 208}, 79, "toothbrush"}
};

const std::vector<SemClass> sem_lib = {
    // {{92, 136, 89}, {87}, "Door"}, 
    {{209, 226, 140}, {122}, "Table"}, 
    // {{255, 160, 98}, {125}, "shelf"},
    {{134, 199, 156}, {121}, "Cabinet"},
    // {{218, 88, 184}, {123}, "floor"}, 
    {{96, 36, 108}, {88, 97, 98, 101, 124, 123}, "Floor"}, 
    // {{137, 54, 74}, {132}, "wall"}, 
    {{102, 102, 156}, {110, 111, 112, 113, 132}, "Wall"},
    // {{183, 121, 142}, {115, 116}, "window"},
    {{146, 139, 141}, {119}, "Ceiling"}
    // {{146, 139, 0}, {119}, "ceiling"}
};

const std::vector<int> pre_dynamic_obj = {0}; //Person category ID

int NFrame = 0;


PCSegGeneratorNode::PCSegGeneratorNode(ros::NodeHandle& node_handle): node_handle_(node_handle)
{

    cloud.reset(new pcl::PointCloud<pcl::PointXYZRGB>);

    Update();

    // load rosparam
    node_handle_.param<bool>("use_semantic_segmentation", use_semantic_segmentation, false);
    node_handle_.param<bool>("use_geometric_segmentation", use_geometric_segmentation, false);
    node_handle_.param<bool>("use_distance_check", use_distance_check, false);
    node_handle_.param<bool>("use_direct_fusion", use_direct_fusion, false);
    node_handle_.param<bool>("use_GT_camera_frame", use_GT_camera_frame, true);
    node_handle_.param<std::string>("camera_frame", camera_frame, "camera_link");
    node_handle_.param<int>("geo_seg_mode", geo_seg_mode, 1);
    node_handle_.param<bool>("visualize_geo_seg", visualize_geo_seg, false);
    node_handle_.param<bool>("visualize_pano_seg", visualize_pano_seg, false);
    node_handle_.param<bool>("visualize_fusion_seg", visualize_fusion_seg, false);
    node_handle_.param<bool>("pub_seg_img", pub_seg_img, false);
    node_handle_.param<bool>("save_img", save_img, false);

    node_handle_.param<bool>("verbose", verbose, true);

    // image subscriber
    rgb_subscriber_ = new message_filters::Subscriber<sensor_msgs::Image> (node_handle_, "/perception/rgb_image", 100);
    depth_subscriber_ = new message_filters::Subscriber<sensor_msgs::Image> (node_handle_, "/perception/depth_image", 100);

    if (!use_semantic_segmentation)
    {
        ROS_INFO("Disable semantic-instance segmentation!");
        sync_ = new message_filters::Synchronizer<sync_pol> (sync_pol(100), *rgb_subscriber_, *depth_subscriber_);
        sync_->registerCallback(boost::bind(&PCSegGeneratorNode::ImageCallback, this, _1, _2));
    }
    else
    {
        ROS_INFO("Enable semantic-instance segmentation!");
        seg_subscriber_ = new message_filters::Subscriber<seg_msgs::Seg> (node_handle_, "/perception/seg", 100);
        seg_sync_ = new message_filters::Synchronizer<seg_sync_pol> (seg_sync_pol(100), *rgb_subscriber_, *depth_subscriber_, *seg_subscriber_);
        seg_sync_->registerCallback(boost::bind(&PCSegGeneratorNode::ImageSegCallback, this, _1, _2, _3));
    }

    // camera info subscriber
    info_sub_ = node_handle_.subscribe("/perception/rgb_info", 5, &PCSegGeneratorNode::CamInfoCallback, this);
    // (TODO) remap the camerainfo topic in the launch file

    point_cloud_segment_publisher_ = node_handle_.advertise<sensor_msgs::PointCloud2>("/perception/seg_point_cloud", 100);
    inst_seg_image_publisher_ = node_handle_.advertise<sensor_msgs::Image>("/perception/inst_seg", 100);

    // initialize segmentor
    if (use_geometric_segmentation)
    {
        ROS_INFO("Enable geometric segmentation!");
        if (geo_seg_mode == GeometricSegMode::kPointCloud)
        {
            ROS_INFO("Use Point Cloud segmentation!");
            mpc_processor = new PCProcessor(node_handle_, visualize_geo_seg, save_img);
        }
        else if (geo_seg_mode == GeometricSegMode::kDepth)
        {
            ROS_INFO("Use depth segmentation!");
            mdepth_segmenter = new depth_segmentation::DepthSegmenter(node_handle_, visualize_geo_seg, save_img);
        }
        else
        {
            ROS_INFO("Use depth segmentation!");
            mdepth_segmenter = new depth_segmentation::DepthSegmenter(node_handle_, visualize_geo_seg, save_img);
            geo_seg_mode = GeometricSegMode::kDepth;
        }
    }
    else
    {
        ROS_INFO("Disable geometric segmentation!");
        mpc_processor = new PCProcessor(node_handle_, visualize_geo_seg, save_img);
    }

    ROS_INFO("Waiting for camera info...");
    while (1)
    {
        if (camera_info_ready)
            break;

        ros::spinOnce();    
    }
    ROS_INFO("Start point cloud segment generator node!");
}


PCSegGeneratorNode::~PCSegGeneratorNode()
{
    delete rgb_subscriber_;
    delete depth_subscriber_;

    if (use_semantic_segmentation)
        delete sync_;
    else
    {
        delete seg_subscriber_;
        delete seg_sync_;
    }

    if (use_geometric_segmentation)
    {
        if (geo_seg_mode == GeometricSegMode::kPointCloud)
            delete mpc_processor;
        else
            delete mdepth_segmenter;
    }
    else
        delete mpc_processor;
}


void PCSegGeneratorNode::ImageSegCallback (const sensor_msgs::ImageConstPtr& msgRGB,const sensor_msgs::ImageConstPtr& msgD,const seg_msgs::SegConstPtr& msgSeg)
{ 
    if (!camera_info_ready)
        return;

    ProcessImageMsg(msgRGB, msgD);

    // Process detection result
    ProcessSegMsg(msgSeg);

    // Generate point cloud segments  
    ros::Time begin = ros::Time::now(); 
    ros::Time end;
    
    if (verbose)
        ROS_INFO("Segment depth!");

    if (use_geometric_segmentation)
    {
        if (geo_seg_mode == GeometricSegMode::kPointCloud)
            mpc_processor->SegmentSingleFrame (imRGB, imDepth, clouds);
        else
            mdepth_segmenter->segmentSingleFrame (imRGB, imDepth, clouds);

        end = ros::Time::now();
        std::cout<<"segment  "<<(end-begin).toSec()<<std::endl;
        begin = end;
        LabelPC();
        end = ros::Time::now();
        std::cout<<"label  "<<(end-begin).toSec()<<std::endl;
        begin = end;
    }
    else
        mpc_processor->GeneratePCSem (imRGB, imDepth, imSeg, objects, semantics, mov_list, clouds);
    
    
    // Publish point cloud segments and clear variables
    Update();
    end = ros::Time::now();
    std::cout<<"publish  "<<(end-begin).toSec()<<std::endl;

    

}


void PCSegGeneratorNode::ImageCallback (const sensor_msgs::ImageConstPtr& msgRGB,const sensor_msgs::ImageConstPtr& msgD)
{
    if (!camera_info_ready)
        return;

    if (verbose)
        ROS_INFO("Get image!");

    ProcessImageMsg(msgRGB, msgD);

    // Generate point cloud segments  
    ros::Time begin = ros::Time::now(); 
    
    if (use_geometric_segmentation)
    {
        if (geo_seg_mode == GeometricSegMode::kPointCloud)
            mpc_processor->SegmentSingleFrame (imRGB, imDepth, clouds);
        else
            mdepth_segmenter->segmentSingleFrame (imRGB, imDepth, clouds);
    }
    else
        mpc_processor->GeneratePC (imRGB, imDepth, cloud);
    
        
    // Publish point cloud segments and clear variables
    Update();
    ros::Time end = ros::Time::now();
    std::cout<<(end-begin).toSec()<<std::endl;

}


void PCSegGeneratorNode::CamInfoCallback (const sensor_msgs::CameraInfoConstPtr& msgInfo)
{
    if (!camera_info_ready)
    {
        K.at<float>(0,0) = msgInfo->K[0];
        K.at<float>(0,2) = msgInfo->K[2];
        K.at<float>(1,1) = msgInfo->K[4];
        K.at<float>(1,2) = msgInfo->K[5];
        K.at<float>(2,2) = msgInfo->K[8];
        width = msgInfo->width;
        height = msgInfo->height;
        SetCamInfo();
        camera_info_ready = true;
    }
}


void PCSegGeneratorNode::SetCamInfo()
{
    if (use_geometric_segmentation)
    {
        if (geo_seg_mode == GeometricSegMode::kPointCloud)
            mpc_processor->SetCamIntrinsics(K);
        else
            mdepth_segmenter->setCamIntrinsics(K, width, height);
    }
    else
        mpc_processor->SetCamIntrinsics(K);
}
    


void PCSegGeneratorNode::Update()
{
    // Publish point cloud segments
    // std::cout<<"cloud size " <<clouds.size()<<std::endl;
    // if (clouds.size() == 1)
    //     std::cout<<"hamsterhamster!" <<std::endl;
    for (int i = 0; i < clouds.size(); i++)
    {
        // if (i == 0)
        // {
        // if (clouds[i]->points.size() < 1000)
        //     continue;
        // std::cout << "clouds " << (int)clouds[i]->points[0].semantic_label << " " << (int)clouds[i]->points[0].instance_label <<std::endl;
        sensor_msgs::PointCloud2 pcl2_msg;
        pcl::toROSMsg(*(clouds[i]), pcl2_msg);
        pcl2_msg.header.stamp = current_frame_time_;
        pcl2_msg.header.frame_id = camera_frame_id_;
        point_cloud_segment_publisher_.publish(pcl2_msg);
        // }
    }

    if (!(use_semantic_segmentation || use_geometric_segmentation))
    {
        sensor_msgs::PointCloud2 pcl2_msg;
        pcl::toROSMsg(*cloud, pcl2_msg);
        pcl2_msg.header.stamp = current_frame_time_;
        pcl2_msg.header.frame_id = camera_frame_id_;
        point_cloud_segment_publisher_.publish(pcl2_msg);
    }

    if (pub_seg_img)
    {
        cv::Mat seg_img = DrawInstSeg();
        std_msgs::Header header;
        header.stamp = current_frame_time_;
        header.frame_id = camera_frame_id_;
        const sensor_msgs::ImagePtr inst_seg_image_msg = cv_bridge::CvImage(header, "rgb8", seg_img).toImageMsg();  // rviz uses rgb as default
        inst_seg_image_publisher_.publish(inst_seg_image_msg);
    }

    // Clear variables
    objects.clear();
    semantics.clear();
    mov_list.clear();
    clouds.clear();
    cloud.reset(new pcl::PointCloud<pcl::PointXYZRGB>);
    instance_category_area_map.clear();
    semantics_category_area_map.clear();

    NFrame ++;
    std::cout << "No. frame: " << NFrame <<std::endl;
}


void PCSegGeneratorNode::ProcessImageMsg (const sensor_msgs::ImageConstPtr& msgRGB,const sensor_msgs::ImageConstPtr& msgD)
{

    current_frame_time_ = msgRGB->header.stamp;
    if (use_GT_camera_frame)
        camera_frame_id_ = msgRGB->header.frame_id;
    else
        camera_frame_id_ = camera_frame;

    // Copy the ros image message to cv::Mat.
    cv_bridge::CvImageConstPtr cv_ptrRGB;
    try {
        cv_ptrRGB = cv_bridge::toCvShare(msgRGB, sensor_msgs::image_encodings::RGB8);  //use rgb for all computations
    } catch (cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    cv_bridge::CvImagePtr cv_ptrD;
    try {
        cv_ptrD = cv_bridge::toCvCopy(msgD);  
    } catch (cv_bridge::Exception& e){
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
    
    // Get the RGB and depth image
    imRGB = cv_ptrRGB->image;
    if (cv_ptrD->encoding == sensor_msgs::image_encodings::TYPE_32FC1)
        imDepth = cv_ptrD->image;
    else if (cv_ptrD->encoding == sensor_msgs::image_encodings::TYPE_16UC1)
        cv::rgbd::rescaleDepth(cv_ptrD->image, CV_32FC1, imDepth);
    else
        imDepth = cv_ptrD->image;

    if (save_img)
    {
        if (verbose)
            ROS_INFO("Save image!");
        std::string path = ros::package::getPath("perception_ros");
        
        cv::Mat depth_save = (cv_ptrD->image).clone();
        if (cv_ptrD->encoding != sensor_msgs::image_encodings::TYPE_16UC1)
        {
            depth_save = 1000 * depth_save;
            depth_save.convertTo(depth_save, CV_16UC1);

        }
        cv::Mat rgb_save = imRGB.clone();
        cv::cvtColor(imRGB, rgb_save, CV_RGB2BGR);  // opencv uses bgr as default

        cv::imwrite(path + "/image/rgb/" + std::to_string(NFrame)+".jpg",rgb_save);
        cv::imwrite(path + "/image/depth/" + std::to_string(NFrame)+".png",depth_save);
    }

}


void PCSegGeneratorNode::ProcessSegMsg (const seg_msgs::SegConstPtr& msgSeg)
{
    seg_msgs::Seg seg_result;
    seg_result = *msgSeg;

    // Create seg image in cv::Mat
    imSeg.create(seg_result.height, seg_result.width, CV_16U);
    for (int i = 0; i < seg_result.height; i++)    
        for (int j = 0; j < seg_result.width; j++)
            imSeg.at<uchar>(i, j) = seg_result.seg_map[i*seg_result.width+j];

    // Process object detection result
    int obj_num = seg_result.obj_id.size();
    // auto pred = [id](const ObjClass & obj) {return obj.category_id == id;};
    for (int i = 0; i < obj_num; i++)
    {
        int cate_id = seg_result.obj_category[i];
        auto it = std::find_if(obj_lib.begin(), obj_lib.end(),
            [=] (const ObjClass& f) { return (f.category_id == cate_id); });
        if (it == obj_lib.end())
            continue;

        int x1 = seg_result.obj_boxes[4*i];
        int y1 = seg_result.obj_boxes[4*i+1];
        int x2 = seg_result.obj_boxes[4*i+2];
        int y2 = seg_result.obj_boxes[4*i+3];

        Obj2D obj = Obj2D();
        obj.id = seg_result.obj_id[i];
        obj.category = seg_result.obj_category[i];
        obj.score = seg_result.obj_scores[i];
        obj.is_dynamic = false;

        // Random color for instance
        // obj.color = (*it).color;
        cv::Vec3b color(rand() % 255,rand() % 255,rand() % 255);
        obj.color = color;

        obj.box = cv::Rect(x1,y1,x2-x1,y2-y1);
        obj.area = (x2-x1)*(y2-y1);
        obj.cate_name = (*it).name;

        if (find(pre_dynamic_obj.begin(), pre_dynamic_obj.end(), obj.category) != pre_dynamic_obj.end())
        {
            obj.is_dynamic = true;  
            mov_list.push_back(obj.id);
        }
        else
        {
            int area_accurate = 0;
            for (int j = x1; j < x2; j++)
                for (int k = y1; k < y2; k++)
                    area_accurate += ((int)imSeg.at<uchar>(k,j)==obj.id);
            obj.area = area_accurate;
        }
        instance_category_area_map.insert(std::make_pair(obj.id, std::make_pair(obj.category, obj.area)));
        // instance_area_pairs.insert(std::pair<int,int>(obj.id, obj.area));
                          
        objects.push_back(obj);
    }
    // Process semantic segmentation result
    int sem_num = seg_result.sem_id.size();
    for (int i = 0; i < sem_num; i++)
    {
        // if (seg_result.sem_area[i] < 5000)
        //     continue;
        for (int j = 0; j < sem_lib.size(); j++)
        {
            if (find(sem_lib[j].category_id.begin(), sem_lib[j].category_id.end(), seg_result.sem_category[i])!=sem_lib[j].category_id.end())
            {
                Sem2D sem = Sem2D();
                sem.id = seg_result.sem_id[i];
                // sem.category = seg_result.sem_category[i];
                sem.category = sem_lib[j].category_id.back();  // Assign all related semantics the same label
                sem.area = seg_result.sem_area[i];
                sem.color = sem_lib[j].color;
                sem.cate_name = sem_lib[j].name;
                semantics.push_back(sem);
                semantics_category_area_map.insert(std::make_pair(sem.id, std::make_pair(sem.category, sem.area)));
                // semantics_category_pairs.insert(std::pair<int,int>(sem.id, sem.category));
                break;
            }
        }     
    }
}


void PCSegGeneratorNode::LabelPC ()
{
    if (verbose)
        ROS_INFO("Label point cloud!");

    register float fx = K.at<float>(0,0);
    register float fy = K.at<float>(1,1);
    register float cx = K.at<float>(0,2);
    register float cy = K.at<float>(1,2);

    register int width = imRGB.cols;
    register int height = imRGB.rows;

    std::vector<pcl::PointCloud<PointSurfelLabel>::Ptr> extra_clouds = {};
    std::unordered_map<int, std::vector<pcl::PointCloud<PointSurfelLabel>::Ptr>> id_pc_pairs = {};

    for (auto cloud_it = clouds.begin(); cloud_it != clouds.end(); )
    {
        std::unordered_map<int, int> candidate_pairs;
        int sum_count = 0;
        if ((*cloud_it)->points.size() < 800)
        {
            clouds.erase(cloud_it);
            continue;
        }
        
        for (int j = 0; j < (*cloud_it)->points.size(); j++)
        {
            int x = (int) ((*cloud_it)->points[j].x * fx / (*cloud_it)->points[j].z + cx);
            int y = (int) ((*cloud_it)->points[j].y * fy / (*cloud_it)->points[j].z + cy);

            if (x < 0)
                x = 0;
            if (x > width - 1)
                x = width - 1;
            if (y < 0)
                y = 0;
            if (y > height - 1)
                y = height - 1;

            int label = (int)imSeg.at<uchar>(y, x);
            // Store temporally
            (*cloud_it)->points[j].instance_label = (uint8_t)label;

            auto it = candidate_pairs.find(label);
            if (it == candidate_pairs.end())
                candidate_pairs.insert({label, 1});
            else
                it->second++;

            sum_count ++;
        }
        
        // Extract instances that are not segmented geometrically
        std::pair<int, int> max_candidate = std::make_pair(0,80);
        int max_count = 0;
        std::unordered_map<int, std::pair<int, pcl::PointCloud<PointSurfelLabel>::Ptr>> extracted_instances;
        // std::vector<std::pair<int, int>> extracted_instances = {};
        // std::vector<pcl::PointCloud<PointSurfelLabel>::Ptr> extract_clouds = {};
        for (auto map_it = candidate_pairs.begin(); map_it != candidate_pairs.end(); map_it++)
        {
            auto object_it = instance_category_area_map.find(map_it->first);
            if (object_it != instance_category_area_map.end())
            {
                if (use_direct_fusion && (map_it->second > 0.95 * object_it->second.second) && (map_it->second < 0.5 * sum_count)) 
                // && (map_it->second < 0.5 * sum_count))
                {
                    pcl::PointCloud<PointSurfelLabel>::Ptr extract_cloud (new pcl::PointCloud<PointSurfelLabel>);                    
                    extract_cloud->is_dense = false;
                    extract_cloud->sensor_origin_.setZero ();
                    extract_cloud->sensor_orientation_.w () = 0.0f;
                    extract_cloud->sensor_orientation_.x () = 1.0f;
                    extract_cloud->sensor_orientation_.y () = 0.0f;
                    extract_cloud->sensor_orientation_.z () = 0.0f; 
                    // extract_clouds.push_back(extract_cloud); 
                    extracted_instances.insert(std::make_pair(object_it->first, std::make_pair(object_it->second.first, extract_cloud)));

                    sum_count -= map_it->second;
                }
                else
                {
                    if (map_it->second > max_count)
                    {
                        max_candidate = std::make_pair(object_it->first, object_it->second.first);
                        max_count = map_it->second;
                    }   
                }
                continue;
            }

            auto semantics_it = semantics_category_area_map.find(map_it->first);
            if (semantics_it != semantics_category_area_map.end())
            {
                // if (use_direct_fusion && (map_it->second > 0.6 * semantics_it->second.second))
                // // && (map_it->second < 0.5 * sum_count))
                // {
                //     pcl::PointCloud<PointSurfelLabel>::Ptr extract_cloud (new pcl::PointCloud<PointSurfelLabel>);                    
                //     extract_cloud->is_dense = false;
                //     extract_cloud->sensor_origin_.setZero ();
                //     extract_cloud->sensor_orientation_.w () = 0.0f;
                //     extract_cloud->sensor_orientation_.x () = 1.0f;
                //     extract_cloud->sensor_orientation_.y () = 0.0f;
                //     extract_cloud->sensor_orientation_.z () = 0.0f; 
                //     // extract_clouds.push_back(extract_cloud); 
                //     extracted_instances.insert(std::make_pair(semantics_it->first, std::make_pair(semantics_it->second.first, extract_cloud)));

                //     sum_count -= map_it->second;
                // }
                // else
                // {
                    if (map_it->second > max_count)
                    {
                        max_candidate = std::make_pair(semantics_it->first, semantics_it->second.first);
                        max_count = map_it->second;
                    }     
                    
                // }                
            }
        }

        // If the number of label with maximum counts still too small, assign to background
        if (max_count < 0.2 * sum_count)
            max_candidate = std::make_pair(0,80);

        // Remove clouds for moving objects
        if (std::find(mov_list.begin(),mov_list.end(),max_candidate.first) != mov_list.end())
        {
            clouds.erase(cloud_it);
            continue;
        }

        // Separate extracted instances and assign labels to the original segment and others
        float bad_point = std::numeric_limits<float>::quiet_NaN ();
        if (extracted_instances.size() > 0)
        {
            if (verbose)
                ROS_INFO("Start extracting instances!");
            for (int j = 0; j < (*cloud_it)->points.size(); j++)
            {
                int label = (int)(*cloud_it)->points[j].instance_label;
                auto it = extracted_instances.find(label);
                if (it != extracted_instances.end())
                {
                    PointSurfelLabel pt;
                    pcl::copyPoint((*cloud_it)->points[j], pt);
                    pt.instance_label = (uint8_t)(it->first);
                    pt.semantic_label = (uint8_t)(it->second.first);
                    it->second.second->push_back(pt);
                    (*cloud_it)->points[j].x = (*cloud_it)->points[j].y = (*cloud_it)->points[j].z = bad_point;
                }
                else
                {
                    (*cloud_it)->points[j].instance_label = (uint8_t)(max_candidate.first);
                    (*cloud_it)->points[j].semantic_label = (uint8_t)(max_candidate.second);
                }     
            }
            std::vector<int> map;
            pcl::removeNaNFromPointCloud(*(*cloud_it),*(*cloud_it),map);

            // Remove clouds for extra extracted instances if they are moving
            for (int k = 0; k < mov_list.size(); k++)
            {
                int mov_id = mov_list[k];
                auto inst_it = extracted_instances.find(mov_id);
                if (inst_it != extracted_instances.end())
                    extracted_instances.erase(inst_it);
            }
            // Append clouds of extra extracted instance to the list
            for (auto instance: extracted_instances)
                extra_clouds.push_back(instance.second.second);
        }
        else
        {
            for (int j = 0; j < (*cloud_it)->points.size(); j++)
            {
                (*cloud_it)->points[j].instance_label = (uint8_t)(max_candidate.first);
                (*cloud_it)->points[j].semantic_label = (uint8_t)(max_candidate.second);
            }
        }

        // Organize pairs of instance id and point cloud index (only consider object instances)
        if ((instance_category_area_map.find(max_candidate.first) != instance_category_area_map.end()) || (max_candidate.second == 122) || (max_candidate.second == 121))
        // if (instance_category_area_map.find(max_candidate.first) != instance_category_area_map.end())
        {
            auto id_it = id_pc_pairs.find(max_candidate.first);
            if (id_it != id_pc_pairs.end())
                id_it->second.push_back(*cloud_it);
            else
            {
                std::vector<pcl::PointCloud<PointSurfelLabel>::Ptr> pc_index = {*cloud_it};
                id_pc_pairs.insert(std::make_pair(max_candidate.first,pc_index));
            }
        }

        cloud_it ++;
    }

    // Distance check for segments with the same instance id
    if (use_distance_check)
    {
        float radius = 0.05;
        for (auto id_pc_pair: id_pc_pairs)
        {
            int size = id_pc_pair.second.size();
            if (size > 1)
            {
                if (verbose)
                    ROS_INFO("Distance check!");
                
                std::vector<int> neighbors(size, 0); // Check minimal distance between every two clouds, if smaller than radius, then set as neighbor
                std::map<int, std::set<int>> neighbor_pairs;
                std::set<std::set<int>> clusters; // Each cluster includes all pc that are neighbors
                std::set<int> outlier;
                std::set<int> inlier; // We assume the only cluster with maximun points is inlier
                int max_cluster_count = 0;

                // Get all neigbors
                for (int i = 0; i < size - 1; i++)
                {
                    pcl::KdTreeFLANN<PointSurfelLabel> kdtree;
                    kdtree.setInputCloud (id_pc_pair.second[i]);
                    for (int j = i + 1; j < size; j++)
                    {
                        std::vector<int> pointIdxRadiusSearch;
                        std::vector<float> pointRadiusSquaredDistance;
                        for (int k = 0; k < id_pc_pair.second[j]->points.size(); k += 5) // sample points
                        {
                            if (kdtree.radiusSearch (id_pc_pair.second[j]->points[k], radius, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0)
                            {
                                neighbors[i] += 1;
                                neighbors[j] += 1;
                                
                                auto it1 = neighbor_pairs.find(i);
                                auto it2 = neighbor_pairs.find(j);
                                if (it1 != neighbor_pairs.end())
                                    it1->second.insert(j);
                                else
                                {
                                    std::set<int> neighbor_set = {j};
                                    neighbor_pairs.insert(std::make_pair(i,neighbor_set));
                                }
                                if (it2 != neighbor_pairs.end())
                                    it2->second.insert(i);
                                else
                                {
                                    std::set<int> neighbor_set = {i};
                                    neighbor_pairs.insert(std::make_pair(j,neighbor_set));
                                }
                                break;
                            }
                        }
                    }
                }

                // Cluster the point cloud segments
                std::set<int> unclustered_set;
                for (int ii = 0; ii < size; ii++)
                    unclustered_set.insert(ii);   // Fill with 0, 1, ..
                while (unclustered_set.size() > 0)
                {
                    int current_segment_index = *(unclustered_set.begin());
                    std::set<int> current_cluster = {current_segment_index};
                    int current_cluster_count = id_pc_pair.second[current_segment_index]->size();
                    if (neighbors[current_segment_index] > 0)
                    {   // width-first search
                        std::set<int> next_check = neighbor_pairs.find(current_segment_index)->second;
                        while (next_check.size() > 0)
                        {
                            int current_check_index = *(next_check.begin());
                            auto ret = current_cluster.emplace(current_check_index);
                            if (ret.second)
                                current_cluster_count += id_pc_pair.second[current_check_index]->size();

                            if (neighbors[current_check_index] > 1)
                            {
                                std::set<int> future_check = neighbor_pairs.find(current_check_index)->second;
                                for (auto check: future_check)
                                {
                                    if (current_cluster.find(check) == current_cluster.end())
                                        next_check.emplace(check);
                                }
                            }
                            next_check.erase(next_check.find(current_check_index));
                        }   
                    }
                    for (auto index: current_cluster)
                        unclustered_set.erase(unclustered_set.find(index));

                    clusters.insert(current_cluster);  
                    if (current_cluster_count > max_cluster_count)
                    {
                        max_cluster_count = current_cluster_count;
                        outlier.insert(inlier.begin(), inlier.end());
                        inlier = current_cluster;
                    }
                    else
                        outlier.insert(current_cluster.begin(), current_cluster.end());          
                }
                // std::cout << "inlier size++++++++++ " << inlier.size() <<std::endl;
                // std::cout << "inlier point size++++++++++ " << (inst_id_it->second[*(inlier.begin())]->points.size()) <<std::endl;      
                
                // For outlier, assign background label
                if (clusters.size() > 1)
                {
                    if (verbose)
                        ROS_INFO("Filter outliers!");
                    for (int out: outlier)
                    {
                        for (int p = 0; p < id_pc_pair.second[out]->points.size(); p++)
                        {
                            id_pc_pair.second[out]->points[p].instance_label = 0u;
                            id_pc_pair.second[out]->points[p].semantic_label = 80u;
                        }
                    }
                }  
            }
        }
    }
        
    // Append clouds of extra extracted instance to the final list
    for (int p = 0; p < extra_clouds.size(); p++)
        if ((extra_clouds[p])->points.size() > 400)
            clouds.push_back(extra_clouds[p]);

    cv::Mat seg_img(height, width, CV_8UC3, cv::Scalar(0, 0, 0));

    for (int i = 0; i < clouds.size(); i++)
    {
        int semantic_label = (int)clouds[i]->points[0].semantic_label;
        int instance_label = (int)clouds[i]->points[0].instance_label;
        for (int j = 0; j < clouds[i]->points.size(); j++)
        {
            int x = (int) (clouds[i]->points[j].x * fx / clouds[i]->points[j].z + cx);
            int y = (int) (clouds[i]->points[j].y * fy / clouds[i]->points[j].z + cy);

            if (x < 0)
                x = 0;
            if (x > width - 2)
                x = width - 2;
            if (y < 0)
                y = 0;
            if (y > height - 2)
                y = height - 2;

            auto sem_it = std::find_if(sem_lib.begin(), sem_lib.end(),
            [=] (const SemClass& f) { return (std::find(f.category_id.begin(), f.category_id.end(), semantic_label) != f.category_id.end()); });
            if (sem_it != sem_lib.end())
            {
                seg_img.at<cv::Vec3b>(y, x) = sem_it->color;
                seg_img.at<cv::Vec3b>(y+1, x) = sem_it->color;
                seg_img.at<cv::Vec3b>(y, x+1) = sem_it->color;
                seg_img.at<cv::Vec3b>(y+1, x+1) = sem_it->color;
                continue;
            }

            auto obj_it = std::find_if(objects.begin(), objects.end(),
            [=] (const Obj2D& f) { return (f.id == instance_label); });
            if (obj_it != objects.end())
            {
                seg_img.at<cv::Vec3b>(y, x) = obj_it->color;
                seg_img.at<cv::Vec3b>(y+1, x) = obj_it->color;
                seg_img.at<cv::Vec3b>(y, x+1) = obj_it->color;
                seg_img.at<cv::Vec3b>(y+1, x+1) = obj_it->color;
                continue;
            }

            seg_img.at<cv::Vec3b>(y, x) = {200, 200, 200};
            seg_img.at<cv::Vec3b>(y+1, x) = {200, 200, 200};
            seg_img.at<cv::Vec3b>(y, x+1) = {200, 200, 200};
            seg_img.at<cv::Vec3b>(y+1, x+1) = {200, 200, 200};

        }
    }
    if (visualize_fusion_seg) 
    {
        static const std::string kWindowName = "FusionSeg";
        cv::namedWindow(kWindowName, cv::WINDOW_NORMAL);
        cv::resizeWindow(kWindowName, 480, 360);
        cv::imshow(kWindowName, seg_img);
        cv::waitKey(1);
    }
    if (save_img)
    {
        std::string path = ros::package::getPath("perception_ros");
        cv::Mat seg_img_save = seg_img.clone();
        cv::cvtColor(seg_img_save, seg_img_save, CV_RGB2BGR);  // opencv uses bgr as default
        cv::imwrite(path + "/image/fusion/" + std::to_string(NFrame)+".jpg",seg_img_save);
    }


        
}


cv::Mat PCSegGeneratorNode::DrawInstSeg()
{
    // cv::Mat inst_seg_img = imRGB.clone();
    int height = imRGB.rows;
    int width = imRGB.cols;

    cv::Mat inst_seg_img(height, width, CV_8UC3, cv::Scalar(200, 200, 200));

    for (int m = 0; m < semantics.size(); m++)
    {
        cv::Vec3b color = semantics[m].color;
        for (int i = 0; i < width; i++)
        {
            for (int j = 0; j < height; j++)
            {
                if (imSeg.at<uchar>(j,i) == (uint8_t)semantics[m].id)
                    inst_seg_img.at<cv::Vec3b>(j,i) = color;
            }
        }
    }

    for (int k = 0; k < objects.size(); k++)
    {
        cv::Rect box = objects[k].box;
        cv::Vec3b color = objects[k].color;
        cv::rectangle(inst_seg_img, box, color,3);
    
        for (int i = box.x; i < box.x+box.width; i++)
        {
            for (int j = box.y; j < box.y+box.height; j++)
            {
                if (imSeg.at<uchar>(j,i) == (uint8_t)objects[k].id)
                {
                    inst_seg_img.at<cv::Vec3b>(j,i) = color;
                    if (objects[k].is_dynamic)
                        inst_seg_img.at<cv::Vec3b>(j,i) = {255,0,0};
                }
            }
        }  
    }

    for (int k = 0; k < objects.size(); k++)
    {
        cv::Rect box = objects[k].box;
        cv::Vec3b color = objects[k].color;
        std::stringstream ss;
        ss << std::fixed << std::setprecision(2) << objects[k].score;
        std::string score = ss.str();
        std::string text = objects[k].cate_name + " " + score;
        int baseline;
        cv::Size text_size = cv::getTextSize(text, cv::FONT_HERSHEY_COMPLEX_SMALL, 0.8, 1.8, &baseline);
        cv::Point anchor(box.x,box.y-10);
        if (anchor.x+text_size.width > width)
        anchor.x = width - text_size.width - 2;
        cv::putText(inst_seg_img, text, anchor, cv::FONT_HERSHEY_COMPLEX_SMALL, 0.8, cv::viz::Color::green(), 1.8, cv::LINE_AA);
    }

    if (visualize_pano_seg) 
    {
        static const std::string kWindowName = "PanoSeg";
        cv::namedWindow(kWindowName, cv::WINDOW_NORMAL);
        cv::resizeWindow(kWindowName, 480, 360);
        cv::imshow(kWindowName, inst_seg_img);
        cv::waitKey(1);
    }
    if (save_img)
    {
        std::string path = ros::package::getPath("perception_ros");
        cv::Mat inst_seg_img_save = inst_seg_img.clone();
        cv::cvtColor(inst_seg_img_save, inst_seg_img_save, CV_RGB2BGR);  // opencv uses bgr as default
        cv::imwrite(path + "/image/pano_seg/" + std::to_string(NFrame)+".jpg",inst_seg_img_save);
    }
    
    return inst_seg_img;
}

}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "PC_seg_generator_node");
    ros::start();

    if(argc > 1) {
        ROS_WARN ("Arguments supplied via command line are neglected.");
    }

    ros::NodeHandle node_handle("~"); // resolve namespace of node handle

    PerceptionROS::PCSegGeneratorNode pc_seg_generator_node(node_handle);

    ros::spin(); // spin() will not return until the node has been shutdown

    // ros::MultiThreadedSpinner spinner(4); // Use 4 threads
    // spinner.spin(); // spin() will not return until the node has been shutdown

    ros::shutdown();

    return 0;
}
