# pip install ros_numpy
# pip install ultralytics
import time

import ros_numpy
import rospy
from sensor_msgs.msg import Image

from ultralytics import YOLO


model = YOLO("yolov8m.pt")
rospy.init_node("ultralytics")
time.sleep(1)

det_image_pub = rospy.Publisher("yolo/detection/annotated_camera_stream", Image, queue_size=5)


def callback(data):
    """Callback function to process image and publish annotated images."""
    array = ros_numpy.numpify(data)
    if det_image_pub.get_num_connections():
        det_result = model(array)
        det_annotated = det_result[0].plot(show=False)
        det_image_pub.publish(ros_numpy.msgify(Image, det_annotated, encoding="rgb8"))


rospy.Subscriber('camera_stream', Image, callback)

while True:
    rospy.spin()
