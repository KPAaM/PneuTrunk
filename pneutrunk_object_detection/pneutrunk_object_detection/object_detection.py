import pyrealsense2 as rs
import numpy as np
import cv2
import argparse
from ultralytics import YOLO
import supervision as sv

import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image

class RealsenseCamera:
    def __init__(self):
        # Configure depth and color streams
        print("Loading Intel RealSense Camera")
        self.pipeline = rs.pipeline()

        config = rs.config()
        config.enable_stream(rs.stream.color, 1280, 720, rs.format.bgr8, 30)
        config.enable_stream(rs.stream.depth, 1280, 720, rs.format.z16, 30)

        # Start streaming
        self.pipeline.start(config)
        align_to = rs.stream.color
        self.align = rs.align(align_to)

    def get_frame_stream(self):
        # Wait for a coherent pair of frames: depth and color
        frames = self.pipeline.wait_for_frames()
        aligned_frames = self.align.process(frames)
        depth_frame = aligned_frames.get_depth_frame()
        color_frame = aligned_frames.get_color_frame()



        if not depth_frame or not color_frame:
            # If there is no frame, probably camera not connected, return False
            print("Error, impossible to get the frame, make sure that the Intel RealSense camera is correctly connected")
            return False, None, None, None, None


        depth_profile = self.pipeline.get_active_profile().get_stream(rs.stream.depth)
        depth_intrinsics = depth_profile.as_video_stream_profile().get_intrinsics()

        fx = depth_intrinsics.fx
        fy = depth_intrinsics.fy
        focal_length_px = (fx + fy) / 2
        ins_x = depth_intrinsics.ppx
        ins_y = depth_intrinsics.ppy

        # Apply filter to fill the holes in the depth image
        spatial = rs.spatial_filter()
        spatial.set_option(rs.option.holes_fill, 3)
        filtered_depth = spatial.process(depth_frame)

        hole_filling = rs.hole_filling_filter()
        filled_depth = hole_filling.process(filtered_depth)

        depth_image = np.asanyarray(filled_depth.get_data())
        color_image = np.asanyarray(color_frame.get_data())

        return True, color_image, depth_image, depth_frame, color_frame,ins_x,ins_y,fx,fy

    def release(self):
        self.pipeline.stop()


def parse_arguments() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="YOLOv8 live")
    parser.add_argument(
        "--webcam-resolution",
        default=[1280, 720],
        nargs=2,
        type=int
    )
    args = parser.parse_args()
    return args


def main():
    args = parse_arguments()

    rclpy.init()
    node = Node('pneutrunk_object_detection')
    publisher = node.create_publisher(Image, 'pneutrunk_object_detection/camera', 1)
    bridge = CvBridge()



    frame_width, frame_height = args.webcam_resolution

    model = YOLO("yolov8s.pt")

    box_annotator = sv.BoxAnnotator(
        thickness=0,
        text_thickness=1,
        text_scale=0.7
    )

    realsense_camera = RealsenseCamera()
    while rclpy.ok():
            # Get frames from Realsense Camera
            ret, color_image, depth_image, frame, depth_frame,ins_x,ins_y,fx,fy = realsense_camera.get_frame_stream()
            if not ret:
                print("Ignoring empty camera frame.")
                break

            # Resize color image to desired dimensions
            frame = cv2.resize(color_image, (frame_width, frame_height))
            depth_frame = cv2.resize(depth_image, (frame_width, frame_height))

            frame = np.array(frame)
            depth_frame = np.array(depth_frame)

            # Run object detection on color image using YOLOv8 model
            result = model(frame, agnostic_nms=True)[0]
            detections = sv.Detections.from_yolov8(result)

            # Print the position of the detected objects in
            for index, detection in enumerate(detections):
                object = str(detection)
                end = object[-9:-7]
                swapped = end + object[7:-7]
                crop_id = swapped[0:54]
                id_obj = crop_id[0:2]
                crop = crop_id[3:53]
                replace = crop.replace(" ", "")
                rep = replace.split(",")
                float_list = [float(x) for x in rep]
                x1, y1, x2, y2 = float_list
                width = x2-x1
                height = y2-y1
                center_x = x1+(width/2)
                center_y = y1+(height/2)
                # Define a margin to expand the bounding box for the ball
           

                # Calculate the average depth value within the area of the detected object
                distance = depth_frame[int(center_y), int(center_x)]
                # ball_distance = np.mean(distance)

                # center_x_mm = ((center_x - ins_x) * (ball_distance*0.66) / fx)
                # center_y_mm = ((center_y - ins_y) * (ball_distance*0.66) / fy)
                center_x_mm = (1460/1280)*center_x
                center_y_mm = (821/720)*center_y

                for detection in detections:
                    if id_obj == "32" or id_obj == "64":   
                        cv2.rectangle(frame, (int(x1), int(y1)), (int(x2), int(y2)), (10, 10, 10), 8)
                        cv2.rectangle(frame, (int(x1-4), int(y1)), (int(x1+130), int(y1-40)), (10, 10, 10), -1)
                        cv2.rectangle(frame, (int(x1-4), int(y2)), (int(x1+180), int(y2+100)), (10, 10, 10), -1)
                        cv2.putText(frame, f'Sports Ball', (int(x1), int(y1-10)), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 1)
                        cv2.putText(frame, f' x={center_x_mm:.1f}[mm]', (int(x1), int(y2) + 28), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 1)
                        cv2.putText(frame, f' y={center_y_mm:.1f}[mm]', (int(x1), int(y2) + 56), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 1)
                        cv2.putText(frame, f' z={distance:.1f}[mm]', (int(x1), int(y2) + 84), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 1)
                        
            msg = bridge.cv2_to_imgmsg(frame, "bgr8")
            publisher.publish(msg)
            
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()




#   0: person
#   1: bicycle
#   2: car
#   3: motorcycle
#   4: airplane
#   5: bus
#   6: train
#   7: truck
#   8: boat
#   9: traffic light
#   10: fire hydrant
#   11: stop sign
#   12: parking meter
#   13: bench
#   14: bird
#   15: cat
#   16: dog
#   17: horse
#   18: sheep
#   19: cow
#   20: elephant
#   21: bear
#   22: zebra
#   23: giraffe
#   24: backpack
#   25: umbrella
#   26: handbag
#   27: tie
#   28: suitcase
#   29: frisbee
#   30: skis
#   31: snowboard
#   32: sports ball
#   33: kite
#   34: baseball bat
#   35: baseball glove
#   36: skateboard
#   37: surfboard
#   38: tennis racket
#   39: bottle
#   40: wine glass
#   41: cup
#   42: fork
#   43: knife
#   44: spoon
#   45: bowlki
#   49: orange
#   50: broccoli
#   51: carrot
#   52: hot dog
#   53: pizza
#   54: donut
#   55: cake
#   56: chair
#   57: couch
#   58: potted plant
#   59: bed
#   60: dining table
#   61: toilet
#   62: tv
#   63: laptop
#   64: mouse
#   65: remote
#   66: keyboard
#   67: cell phone
#   68: microwave
#   69: oven
#   70: toaster
#   71: sink
#   72: refrigerator
#   73: book
#   74: clock
#   75: vase
#   76: scissors
#   77: teddy bear
#   78: hair drier
#   79: toothbrush