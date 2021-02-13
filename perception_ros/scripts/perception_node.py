#!/usr/bin/env python

import rospy
import tf
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
import cv2
from sensor_msgs.msg import Image, CameraInfo
from seg_msgs.msg import Seg

from tcp_client import TcpClient
from utils import pack_img, bin2openpose, bin2detectron

from message_filters import ApproximateTimeSynchronizer, Subscriber


class PerceptionNode(object):

    def __init__(
        self,
        use_vrep,
        # check_dynamic,
        enable_openpose,
        openpose_ip,
        openpose_port,
        enable_detectron,
        detectron_ip,
        detectron_port,
        detectron_model
    ):
        # rospy.Subscriber("/camera/rgb/image_raw", Image, self.perceive_)
        
        self.use_vrep_ = use_vrep
        # self.check_dynamic_ = check_dynamic
        self.enable_openpose_ = enable_openpose
        self.op_client_ = TcpClient(ip=openpose_ip, port=openpose_port)

        self.enable_detectron_ = enable_detectron
        self.dt_client_ = TcpClient(ip=detectron_ip, port=detectron_port)
        self.dt_model_ = detectron_model

        # if not check_dynamic:
        #     self.enable_openpose_ = False
        #     self.enable_detectron_ = False


        self.rgb_sub_ = Subscriber('/camera/rgb/image_raw', Image)
        self.depth_sub_ = Subscriber('/camera/depth/image_raw', Image)
        self.rgb_info_sub_ = Subscriber('/camera/rgb/camera_info', CameraInfo)
        self.depth_info_sub_ = Subscriber('/camera/depth/camera_info', CameraInfo)

        # self.syn_sub_ = ApproximateTimeSynchronizer([self.rgb_sub_, self.depth_sub_, self.rgb_info_sub_, self.depth_info_sub_], queue_size=1, slop=0.05)
        self.syn_sub_ = ApproximateTimeSynchronizer([self.rgb_sub_, self.depth_sub_, self.rgb_info_sub_], queue_size=2, slop=0.1)
        self.syn_sub_.registerCallback(self.perceive_)

        self.dt_pub = rospy.Publisher("/perception/seg", Seg, queue_size=1)
        self.rgb_pub = rospy.Publisher("/perception/rgb_image", Image, queue_size=1)
        self.depth_pub = rospy.Publisher("/perception/depth_image", Image, queue_size=1)
        self.rgb_info_pub = rospy.Publisher("/perception/rgb_info", CameraInfo, queue_size=1)
        self.depth_info_pub = rospy.Publisher("/perception/depth_info", CameraInfo, queue_size=1)

        self.tf_listener_ = tf.TransformListener()

        self.bridge_ = CvBridge()

        self.rgb_image_ = None
        self.depth_image_ = None
        self.rgb_info_ = None
        self.depth_info_ = None
        self.flag_ = False  # if new message comes



    def __def__(self):
        pass


    def launch(self):
        while not rospy.is_shutdown():
            
            if not self.rgb_image_ == None and self.flag_:
                start = rospy.get_time()

                img = self.bridge_.imgmsg_to_cv2(self.rgb_image_)
                height, width, channels = img.shape
                time = self.rgb_image_.header.stamp
                
                # Get detection result
                img_bin = pack_img(img)
                # print("input")

                if self.enable_openpose_:
                    self.pub_op_(img_bin, time)

                
                if self.enable_detectron_:
                    self.pub_dt_(img_bin, time)


                end = rospy.get_time()
                # print(end-start)

                self.flag_ = False

    
    # def perceive_(self, rgb_img, depth_img, rgb_info, depth_info):
    def perceive_(self, rgb_img, depth_img, rgb_info):
        # print("percieve")
            
        self.rgb_image_ = rgb_img
        self.depth_image_ = depth_img
        self.rgb_info_ = rgb_info
        # self.depth_info_ = depth_info 

        if self.use_vrep_:
            dep_img = self.bridge_.imgmsg_to_cv2(depth_img, desired_encoding="16UC1")
            dep_flip_img = cv2.flip(dep_img, 0)
            depth_flip_img_ = self.bridge_.cv2_to_imgmsg(dep_flip_img, encoding="16UC1")
            depth_flip_img_.header = depth_img.header

        # img = self.bridge_.imgmsg_to_cv2(rgb_img)
        # height, width, channels = img.shape
        # if (channels == 4):
        #     img = cv2.cvtColor(img, cv2.COLOR_BGRA2BGR)

        # may include some other processing

        # Publish frame
        self.rgb_pub.publish(rgb_img)
        self.rgb_info_pub.publish(rgb_info)
        # self.depth_info_pub.publish(depth_info_)

        if self.use_vrep_:
            self.depth_pub.publish(depth_flip_img_)
        else:
            self.depth_pub.publish(depth_img)

        self.flag_ = True




    def pub_dt_(self, img_bin, time_):
        start = rospy.get_time()
        resp_bin = self.dt_client_.send(img_bin)
        # print("output")
        # print(len(resp_bin))
        resp = bin2detectron(resp_bin, self.dt_model_)
        end = rospy.get_time()

        # print("Received Detectron Result")
        # print(end-start)
        # print(resp)

        seg_msg = Seg()
        seg_msg.header.stamp = time_

        if self.dt_model_ == "Inst_seg":
            seg_map = resp["masks"]
            boxes = resp["boxes"]
            num = len(boxes)
            # print(num)
            boxes = np.reshape(boxes, (num*4))
            id = [(i+1) for i in range(num)]
            scores = resp["scores"]
            category = resp["classes"]

            seg_msg.obj_id = id
            seg_msg.obj_scores = scores
            seg_msg.obj_category = category
        
        elif self.dt_model_ == "Pano_seg":
            seg_map = resp["seg_map"]
            info  = resp["info"]
            boxes = resp["boxes"]
            boxes = np.reshape(boxes, len(boxes)*4)
            obj_id = []
            sem_id = []
            scores = []
            area = []
            obj_category = []
            sem_category = []

            for i in range(len(info)):
                info_current = info[i]
                if info_current["isthing"]:
                    scores.append(info_current["score"])
                    obj_category.append(info_current["category_id"])
                    obj_id.append(info_current["id"])  
                else:
                    area.append(info_current["area"])
                    sem_category.append(info_current["category_id"]+80) # start from 81, no class 80
                    sem_id.append(info_current["id"])   
                                            
        seg_msg.height = len(seg_map)
        seg_msg.width = len(seg_map[0])
        seg_map = seg_map.reshape(seg_msg.height*seg_msg.width)
        seg_msg.seg_map = seg_map
        seg_msg.obj_id = obj_id
        seg_msg.obj_category = obj_category
        seg_msg.obj_scores = scores
        seg_msg.obj_boxes = boxes

        seg_msg.sem_id = sem_id
        seg_msg.sem_category = sem_category
        seg_msg.sem_area = area
                        
        self.dt_pub.publish(seg_msg)

    
    def pub_op_(self, img_bin, time_):
        resp_bin = self.op_client_.send(img_bin)
        resp = bin2openpose(resp_bin)
        print("Received Openpose Result")



if __name__ == "__main__":
    rospy.init_node("robot_perception_node")

    use_vrep = rospy.get_param("~use_vrep", False)
    # check_dynamic = rospy.get_param("~check_dynamic", False)
    enable_openpose = rospy.get_param("~enable_openpose", False)
    openpose_ip = rospy.get_param("~openpose_ip", "0.0.0.0")
    openpose_port = rospy.get_param("~openpose_port", 8800)

    enable_detectron = rospy.get_param("~enable_detectron", False)
    detectron_ip = rospy.get_param("~detectron_ip", "0.0.0.0")
    detectron_port = rospy.get_param("~detectron_port", 8801)
    detectron_model = rospy.get_param("~detectron_model", "Pano_seg")

    print(enable_detectron)
    print(detectron_model)

    node = PerceptionNode(
        use_vrep=use_vrep,
        # check_dynamic=check_dynamic,
        enable_openpose=enable_openpose,
        openpose_ip=openpose_ip,
        openpose_port=openpose_port,
        enable_detectron=enable_detectron,
        detectron_ip=detectron_ip,
        detectron_port=detectron_port,
        detectron_model=detectron_model
    )

    node.launch()
