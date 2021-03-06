#!/usr/bin/python

import rospy
from sensor_msgs.msg import CameraInfo, Image
from semantic_map.msg import RoomObservation
from upper_body_detector.msg import UpperBodyDetector
from os import path
import sys
import json
from deep_object_detection.msg import Object
from deep_object_detection.srv import DetectObjects, DetectObjectsRequest
from std_msgs.msg import String

class RejectionServer(object):

    def observation_cb(self, msg):

        self.sweep_detections.append(self.latest_detections)
        self.images.append(msg)

    def finished_cb(self, msg):

        sweep_dir = path.abspath(path.join(path.abspath(msg.data), path.pardir))

        req = DetectObjectsRequest()
        req.images = self.images
        req.observation_path = msg.data
        rospy.wait_for_service('/deep_object_detection/detect_objects')
        try:
            server = rospy.ServiceProxy('/deep_object_detection/detect_objects', DetectObjects)
            resp = server(req)
            detections = [[] for im in self.images]
            for obj in resp.objects:
                if obj.label != "person":
                    continue
                vals = {'x': obj.x, 'y': obj.y, 'width': obj.width, 'height': obj.height}
                detections[obj.imageID].append(vals)

            for ind, vec in enumerate(detections):
                name = "intermediate_deep_detection%04d.json" % ind
                filename = path.join(sweep_dir, name)
                with open(filename, 'w') as outfile:
                    json.dump(vec, outfile)
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e
            return

        for ind, det in enumerate(self.sweep_detections):
            name = "intermediate_detection%04d.json" % ind
            filename = path.join(sweep_dir, name)
            vec = []
            for x, y, w, h in zip(det.pos_x, det.pos_y, det.width, det.height):
                vals = {'x': x, 'y': y, 'width': w, 'height': h}
                vec.append(vals)
            with open(filename, 'w') as outfile:
                json.dump(vec, outfile)

        self.sweep_detections = []
        self.images = []

    def simulate_sweep(self, room_xml):
        msg = String()
        msg.data = room_xml
        for i in range(0, 17):
            det = UpperBodyDetector()
            det.pos_x = [1.]
            det.pos_y = [2.]
            det.width = [3.]
            det.height = [4.]
            self.sweep_detections.append(det)
        self.finished_cb(msg)

    def detection_cb(self, msg):

        self.latest_detections = msg

    def __init__(self):

        self.observation_sub = rospy.Subscriber('/local_metric_map/rgb/rgb_filtered', Image, callback=self.observation_cb)
        self.finished_sub = rospy.Subscriber('/segmentation_done', String, callback=self.finished_cb)
        self.detection_sub = rospy.Subscriber('/upper_body_detector/detections', UpperBodyDetector, callback=self.detection_cb)

        self.latest_detections = UpperBodyDetector()
        self.sweep_detections = []
        self.images = []

if __name__ == '__main__':

    rospy.init_node('people_rejection_node')
    rs = RejectionServer()

    if len(sys.argv) > 1:
        rs.simulate_sweep(sys.argv[1])

    rospy.spin()
