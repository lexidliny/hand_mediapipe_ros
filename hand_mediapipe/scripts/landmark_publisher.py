#! /home/yxw/anaconda3/envs/mediapipe/bin/python
# import imp
from pyparsing import any_open_tag
import rospy
from geometry_msgs.msg import Point
from sensor_msgs.msg import Image
from std_msgs.msg import Header
from cv_bridge import CvBridge, CvBridgeError
import cv2
import tf
import message_filters
import pyrealsense2

import mediapipe as mp
mp_drawing = mp.solutions.drawing_utils
mp_drawing_styles = mp.solutions.drawing_styles
mp_hands = mp.solutions.hands



class Landmark_pub:

  def __init__(self):
    self.image_pub = rospy.Publisher("anotated_hand",Image, queue_size=2)

    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber("/camera/color/image_raw",Image,queue_size=2)


    self.landmark_pub = rospy.Publisher('hand_landmark_point', Point, queue_size=2)

  def callback(self,data1,data2):
    try:
      cv_image = self.bridge.imgmsg_to_cv2(data1, "bgr8")
      cv_depth = self.bridge.imgmsg_to_cv2(data2, "16UC1")
    except CvBridgeError as e:
      print(e)

    with mp_hands.Hands(
    static_image_mode=True,
    max_num_hands=2,
    min_detection_confidence=0.5) as hands:
        results = hands.process(cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB))
        if not results.multi_hand_landmarks:
            return 
        image_height, image_width, _ = cv_image.shape
        annotated_image = cv_image.copy()
        for hand_landmarks in results.multi_hand_landmarks:
            # print('hand_landmarks:', hand_landmarks)
            print(
                f'Index finger tip coordinates: (',
                f'{hand_landmarks.landmark[mp_hands.HandLandmark.INDEX_FINGER_TIP].x * image_width}, '
                f'{hand_landmarks.landmark[mp_hands.HandLandmark.INDEX_FINGER_TIP].y * image_height})'
            )
            mp_drawing.draw_landmarks(
                annotated_image,
                hand_landmarks,
                mp_hands.HAND_CONNECTIONS,
                mp_drawing_styles.get_default_hand_landmarks_style(),
                mp_drawing_styles.get_default_hand_connections_style())
            pix_x = hand_landmarks.landmark[mp_hands.HandLandmark.INDEX_FINGER_TIP].x * image_width
    fingertip_depth = cv_depth()

    # cv2.imshow("hand_landmarks", annotated_image)
    # (rows,cols,channels) = cv_image.shape
    # if cols > 60 and rows > 60 :
    #   cv2.circle(cv_image, (50,50), 10, 255)

    # cv2.waitKey(3)

    try:
      # header = Header(stamp=rospy.Time.now())
      # header.frame_id = 'annotated_image'
      img_pub = self.bridge.cv2_to_imgmsg(annotated_image, "bgr8")
      # img_pub.header = header
      # img_pub.step = 
      self.image_pub.publish(img_pub)
      
    except CvBridgeError as e:
      print(e)


if __name__ == '__main__':

    # ic = image_converter()
    rospy.init_node('landmark_publisher')
    rospy.loginfo("Starting landmark_publisher node")


    Landmark_pub()
    # br = tf.TransformBroadcaster()
    # br.sendTransform((0,0,0), tf.transformations.quaternion_from_euler(0,0,0),rospy.Time.now(), "annotated_image","camera_color_frame")

    # rate = rospy.Rate(10) # 10hz
    rospy.spin()