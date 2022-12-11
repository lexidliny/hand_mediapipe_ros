#! /home/kortex/anaconda3/envs/mediapipe/bin/python
# import imp
import rospy
from geometry_msgs.msg import Point
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
from sensor_msgs.msg import Image
import pyrealsense2 as rs
import numpy as np
from cv_bridge import CvBridge, CvBridgeError

from hand_mediapipe.srv import LandmarkPoint, LandmarkPointResponse

import mediapipe as mp
mp_drawing = mp.solutions.drawing_utils
mp_drawing_styles = mp.solutions.drawing_styles
mp_hands = mp.solutions.hands


def get_3d_camera_coordinate(depth_pixel, aligned_depth_frame, depth_intrin):
    x = depth_pixel[0]
    y = depth_pixel[1]
    dis = aligned_depth_frame.get_distance(int(x), int(y))  # 获取该像素点对应的深度
    # print ('depth: ',dis)       # 深度单位是m
    camera_coordinate = rs.rs2_deproject_pixel_to_point(depth_intrin, depth_pixel, dis)
    # print ('camera_coordinate: ',camera_coordinate)
    return camera_coordinate


def getpose(self):
  pipeline = rs.pipeline()
  config = rs.config()
  config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
  config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
  pipeline.start(config)
  align_to_color=rs.align(rs.stream.color)
  flag = 1
  with mp_hands.Hands(
    model_complexity=0,
    min_detection_confidence=0.5,
    min_tracking_confidence=0.5) as hands:

    while True:
      frames = pipeline.wait_for_frames()
      frames = align_to_color.process(frames)
  
      depth_frame = frames.get_depth_frame()
      color_frame = frames.get_color_frame()
      if not depth_frame or not color_frame:
          continue
        
      # get frame parameters
      depth_intrin = depth_frame.profile.as_video_stream_profile().intrinsics  
      color_intrin = color_frame.profile.as_video_stream_profile().intrinsics

      # Convert images to numpy arrays
      depth_image = np.asanyarray(depth_frame.get_data())
      color_image = np.asanyarray(color_frame.get_data())

      annotated_image = color_image.copy()
    
      results = hands.process(cv2.cvtColor(color_image, cv2.COLOR_BGR2RGB))
      if not results.multi_hand_landmarks:
          continue
      image_height, image_width, _ = color_image.shape
      
      for hand_landmarks in results.multi_hand_landmarks:
          middle_tip_pix_x = hand_landmarks.landmark[mp_hands.HandLandmark.MIDDLE_FINGER_TIP].x *image_width
          middle_tip_pix_y = hand_landmarks.landmark[mp_hands.HandLandmark.MIDDLE_FINGER_TIP].y *image_height
          
          middle_pip_pix_x = hand_landmarks.landmark[mp_hands.HandLandmark.MIDDLE_FINGER_PIP].x *image_width
          middle_pip_pix_y = hand_landmarks.landmark[mp_hands.HandLandmark.MIDDLE_FINGER_PIP].y *image_height

          index_pix_x = hand_landmarks.landmark[mp_hands.HandLandmark.INDEX_FINGER_MCP].x * image_width
          index_pix_y = hand_landmarks.landmark[mp_hands.HandLandmark.INDEX_FINGER_MCP].y * image_height

          ring_pix_x = hand_landmarks.landmark[mp_hands.HandLandmark.RING_FINGER_MCP].x * image_width
          ring_pix_y = hand_landmarks.landmark[mp_hands.HandLandmark.RING_FINGER_MCP].y * image_height
          
          landmarks = [(middle_tip_pix_x, middle_tip_pix_y), 
                       (middle_pip_pix_x, middle_pip_pix_y),
                       (index_pix_x, index_pix_y),
                       (ring_pix_x, ring_pix_y)]
          mp_drawing.draw_landmarks(
              annotated_image,
              hand_landmarks,
              mp_hands.HAND_CONNECTIONS,
              mp_drawing_styles.get_default_hand_landmarks_style(),
              mp_drawing_styles.get_default_hand_connections_style())    

      # cv2.imshow("hand_landmarks", annotated_image)
      
      self.landmark_img_publisher.publish(self.bridge.cv2_to_imgmsg(annotated_image, "bgr8"))
      flag += 1
      res = []

      if(flag == 200):
        for i in range(4):
          pos = Point()
          print(i)
          pos.x = get_3d_camera_coordinate(landmarks[i], depth_frame, depth_intrin)[0]
          pos.y = get_3d_camera_coordinate(landmarks[i], depth_frame, depth_intrin)[1]
          pos.z = get_3d_camera_coordinate(landmarks[i], depth_frame, depth_intrin)[2]
          res.append(pos)
        break
  
  pipeline.stop()
  # config.stop()
  # res = []
  return LandmarkPointResponse(res)
      

class Landmark_server:

  def __init__(self):

    self.bridge = CvBridge()

    self.landmark_server = rospy.Service('landmark_server', LandmarkPoint, self.callback)
    self.landmark_img_publisher = rospy.Publisher('annotated_image', Image, queue_size=3)

  def callback(self,req):
    
    return getpose(self)

def getpalmpose(self):
  pipeline = rs.pipeline()
  config = rs.config()
  config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
  config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
  pipeline.start(config)
  align_to_color=rs.align(rs.stream.color)
  flag = 1
  with mp_hands.Hands(
    model_complexity=0,
    min_detection_confidence=0.5,
    min_tracking_confidence=0.5) as hands:

    while True:
      frames = pipeline.wait_for_frames()
      frames = align_to_color.process(frames)
  
      depth_frame = frames.get_depth_frame()
      color_frame = frames.get_color_frame()
      if not depth_frame or not color_frame:
          continue
        
      # get frame parameters
      depth_intrin = depth_frame.profile.as_video_stream_profile().intrinsics  
      color_intrin = color_frame.profile.as_video_stream_profile().intrinsics

      # Convert images to numpy arrays
      depth_image = np.asanyarray(depth_frame.get_data())
      color_image = np.asanyarray(color_frame.get_data())

      annotated_image = color_image.copy()
    
      results = hands.process(cv2.cvtColor(color_image, cv2.COLOR_BGR2RGB))
      if not results.multi_hand_landmarks:
          continue
      image_height, image_width, _ = color_image.shape
      
      for hand_landmarks in results.multi_hand_landmarks:
          middle_tip_pix_x = hand_landmarks.landmark[mp_hands.HandLandmark.MIDDLE_FINGER_TIP].x *image_width
          middle_tip_pix_y = hand_landmarks.landmark[mp_hands.HandLandmark.MIDDLE_FINGER_TIP].y *image_height
          
          middle_pip_pix_x = hand_landmarks.landmark[mp_hands.HandLandmark.MIDDLE_FINGER_PIP].x *image_width
          middle_pip_pix_y = hand_landmarks.landmark[mp_hands.HandLandmark.MIDDLE_FINGER_PIP].y *image_height

          mcp_pix_x = hand_landmarks.landmark[mp_hands.HandLandmark.MCP_FINGER_MCP].x * image_width
          mcp_pix_y = hand_landmarks.landmark[mp_hands.HandLandmark.MCP_FINGER_MCP].y * image_height
          
          landmarks = [(middle_tip_pix_x, middle_tip_pix_y), 
                       (middle_pip_pix_x, middle_pip_pix_y),
                       (mcp_pix_x, mcp_pix_y)]
          mp_drawing.draw_landmarks(
              annotated_image,
              hand_landmarks,
              mp_hands.HAND_CONNECTIONS,
              mp_drawing_styles.get_default_hand_landmarks_style(),
              mp_drawing_styles.get_default_hand_connections_style())    

      # cv2.imshow("hand_landmarks", annotated_image)
      
      self.landmark_img_publisher2.publish(self.bridge.cv2_to_imgmsg(annotated_image, "bgr8"))
      flag += 1
      res = []

      if(flag == 200):
        for i in range(3):
          pos = Point()
          print(i)
          pos.x = get_3d_camera_coordinate(landmarks[i], depth_frame, depth_intrin)[0]
          pos.y = get_3d_camera_coordinate(landmarks[i], depth_frame, depth_intrin)[1]
          pos.z = get_3d_camera_coordinate(landmarks[i], depth_frame, depth_intrin)[2]
          res.append(pos)
        break
  
  pipeline.stop()
  # config.stop()
  # res = []
  return LandmarkPointResponse(res)





class Landmark_palm_server:

  def __init__(self):

    self.bridge = CvBridge()

    self.Landmark_palm_server = rospy.Service('Landmark_palm_server', LandmarkPoint, self.callback)
    self.landmark_img_publisher2 = rospy.Publisher('annotated_image', Image, queue_size=3)

  def callback(self,req):
    
    return getpalmpose(self)
    

if __name__ == '__main__':

    # ic = image_converter()
    rospy.init_node('landmark_server_node')
    rospy.loginfo("Starting landmark_server node")
    
    Landmark_server()
    Landmark_palm_server()
    
    rospy.spin()