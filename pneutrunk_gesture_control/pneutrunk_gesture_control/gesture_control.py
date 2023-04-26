import rclpy
from rclpy.node import Node

import cv2
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
import time
from std_msgs.msg import String
import numpy as np
import math

import mediapipe as mp
mp_drawing = mp.solutions.drawing_utils
mp_drawing_styles = mp.solutions.drawing_styles
mp_hands = mp.solutions.hands



def main(args=None):
    rclpy.init(args=args)
    node = Node('pneutrunk_gesture_control')
    publisher_camera = node.create_publisher(Image, '/pneutrunk/gesture/camera', 1)
    publisher_gesture_cmd = node.create_publisher(String, '/pneutrunk/gesture/cmd', 1)

    cap = cv2.VideoCapture(0)
    bridge = CvBridge()

    with mp_hands.Hands(
    max_num_hands = 1,
    model_complexity=0,
    min_detection_confidence=0.7,
    min_tracking_confidence=0.7) as hands:

        while rclpy.ok() and cap.isOpened():
            success, image = cap.read()
            if not success:
                print("Ignoring empty camera frame.")
                break
            image.flags.writeable = False
            image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
            results = hands.process(image)
        
            # Draw the hand annotations on the image.
            image.flags.writeable = True
            image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
            
            if results.multi_hand_landmarks:
                for hand_landmarks in results.multi_hand_landmarks:
                    # print('hand_landmarks:', hand_landmarks)
                    # print(
                    #     f'Index finger tip coordinates: (',
                    #     f'{hand_landmarks.landmark[0].x}, '
                    #     f'{hand_landmarks.landmark[mp_hands.HandLandmark.INDEX_FINGER_TIP].y})'
                    # )
                    mp_drawing.draw_landmarks(
                        image,
                        hand_landmarks,
                        mp_hands.HAND_CONNECTIONS,
                        mp_drawing_styles.get_default_hand_landmarks_style(),
                        mp_drawing_styles.get_default_hand_connections_style())
                    
                    msg_gesture_cmd = String()
                    l2_norm = math.sqrt((hand_landmarks.landmark[4].x-hand_landmarks.landmark[8].x)**2+(hand_landmarks.landmark[4].y-hand_landmarks.landmark[8].y)**2)
                    if l2_norm < 0.05:
                        msg_gesture_cmd.data = "OK"
                        publisher_gesture_cmd.publish(msg_gesture_cmd)

            image = cv2.flip(image, 1)

            msg = bridge.cv2_to_imgmsg(image, "bgr8")
            publisher_camera.publish(msg)

            

    cap.release()
    cv2.destroyAllWindows()  
    rclpy.shutdown()


if __name__ == '__main__':
    main()