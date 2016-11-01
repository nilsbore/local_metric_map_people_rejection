#!/usr/bin/python

import rospy
from sensor_msgs.msg import CameraInfo
from semantic_map.msg import RoomObservation
from upper_body_detector.msg import UpperBodyDetector
from os import path
import json

class RejectionServer(object):

    def observation_cb(self, msg):

        self.sweep_detections.append(latest_detections)

    def finished_cb(self, msg):

        sweep_dir = path.join(path.abspath(msg.xml_file_name), path.pardir)
        for ind, det in enumerate(self.sweep_detections):
            name = "intermediate_detection%4d.json" % ind
            filename = path.join(sweep_dir, name)
            vec = []
            for x, y, w, h in zip(det.pos_x, det.pos_y, det.width, det.height):
                vals = {'x': x, 'y': y, 'width': w, 'height': h}
                vec.append(vals)
            with open(filename, 'w') as outfile:
                json.dump(vec, outfile)
        self.sweep_detections = []

    def detection_cb(self, msg):

        self.latest_detections = msg

    def __init__(self):

        self.observation_sub = rospy.Subscriber('/local_metric_map/depth/camera_info', CameraInfo, callback=self.observation_cb)
        self.finished_sub = rospy.Subscriber('/local_metric_map/room_observations', RoomObservation, callback=self.finished_cb)
        self.detection_sub = rospy.Subscriber('/upper_body_detector/detections', UpperBodyDetector, callback=self.detection_cb)

        self.latest_detections = UpperBodyDetector()
        self.sweep_detections = []

if __name__ == '__main__':

    rospy.init_node('people_rejection_node')
    rs = RejectionServer()
    rospy.spin()
