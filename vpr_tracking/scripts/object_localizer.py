#! /usr/bin/env python2
from __future__ import print_function
import sys
import threading

import numpy as np
import rospy
import tf
import tf_conversions
import tf2_ros
import tf2_geometry_msgs

import aruco_msgs.msg
import geometry_msgs.msg
import std_srvs.srv

PUBLISH_TRANSFORM_PERIOD = 0.1 # seconds
WAIT_SERVICE_PERIOD = 10.0 # seconds
LOCALIZATION_TIMEOUT = 5.0 # seconds
LOCALIZATION_INFO_PARAM = 'localization_info'
LOCALIZE_SERVICE = 'localize_object'
MARKER_TOPIC = 'markers'
CLOSE_TRANSLATION = 0.03 # meters
CLOSE_ROTATION = 5.0 * (3.141592/180.0)

def transform_from_pose_msg(pose):
    from tf.transformations import quaternion_matrix, translation_matrix
    q = [pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w]
    t = [pose.position.x, pose.position.y, pose.position.z]
    return np.matmul(translation_matrix(t), quaternion_matrix(q))

def transform_from_array(vals):
    from tf.transformations import translation_matrix, euler_matrix
    T = np.matmul(translation_matrix(vals[0:3]), euler_matrix(vals[3], vals[4], vals[5]))
    return T

class LocateableObject:
    def __init__(self, param):
        self.pose = transform_from_array(param['pose'])
        self.name = param['name']
        self.parent = param['parent']
        self.marker_refs = dict()
        for marker_info in param['markers']:
            self.marker_refs[marker_info['id']] = transform_from_array(marker_info['pose'])

    def marker_ids(self):
        return list(self.marker_refs.keys())

    def get_tf_transform(self):
        from tf.transformations import quaternion_from_matrix

        t = geometry_msgs.msg.TransformStamped()
        t.header.frame_id = self.parent
        t.child_frame_id = self.name

        transl = self.pose[0:3,3]
        q = quaternion_from_matrix(self.pose)
        t.transform.translation.x, t.transform.translation.y, t.transform.translation.z = transl
        t.transform.rotation.x, t.transform.rotation.y, t.transform.rotation.z, t.transform.rotation.w = q
        return t

    def localize(self, measured_data):
        """
        Compute a pose for this object given measured poses of a set of markers.

        Uses a least squares fit on the sum of the reference and measured poses to estimate the pose
        of the object. Unfortunately this currently doesn't work reliably for more than one marker.

        Args:
            measured_data: dict of IDs to PoseStamped messages
        :type measured_data: dict
        """
        Rs = np.zeros((4,4))
        Ms = np.zeros((4,4))
        for id, marker in measured_data.items():
            if marker.header.frame_id != self.parent:
                raise RuntimeError("Frame id mismatch, expected marker {} in frame '{}', got '{}'".format(
                    id, self.parent, marker.header.frame_id))
            Rs = Rs + self.marker_refs[id]
            Ms = Ms + transform_from_pose_msg(marker.pose)

        X,res,rank,s = np.linalg.lstsq(Rs, Ms)
        R = X[0:3,0:3]
        U,d,V = np.linalg.svd(R) # Renormalize to orthonormal matrix
        Rn = np.matmul(U,V)
        X[0:3,0:3] = Rn

        self.pose = X
        rospy.loginfo("Pose orientation estimation singular values: {}".format(str(d)))
        rospy.loginfo("Localization estimate:\n{}".format(str(self.pose)))

        # Now that we have an estimate, check if any marker estimates something notably different
        for id, marker in measured_data.items():
            E = np.matmul(np.linalg.inv(self.marker_refs[id]), transform_from_pose_msg(marker.pose))
            dt = np.linalg.norm(self.pose[0:3,3] - E[0:3,3])
            dR33 = np.matmul(np.linalg.inv(self.pose[0:3,0:3]), E[0:3,0:3])
            dR = np.zeros((4,4))
            dR[0:3,0:3] = dR33
            dR[3,3] = 1.0
            dang = abs(tf.transformations.rotation_from_matrix(dR)[0])
            if dt > CLOSE_TRANSLATION or dang > CLOSE_ROTATION:
                rospy.logwarn("Marker {} pose estimation of '{}' is far from combined estimation".format(
                    id, self.name))

class Localizer:
    def __init__(self):
        self.objects = list()

        self.localize_srv = rospy.Service(LOCALIZE_SERVICE, std_srvs.srv.Trigger, self.localize_srv_callback)
        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer)
        self._tf_broadcaster = tf2_ros.TransformBroadcaster()
        self._pub_tf_timer = rospy.Timer(rospy.Duration(PUBLISH_TRANSFORM_PERIOD),self._broadcast_tf_cb_)

    def add_object(self, obj):
        self.objects.append(obj)

    def localize_srv_callback(self, srv_req):
        """
        :type srv_req: std_srvs.srv.TriggerRequest
        """
        srv_resp = std_srvs.srv.TriggerResponse()
        for object in self.objects: # type: LocateableObject
            rospy.loginfo("Looking for '{}'...".format(object.name))
            try:
                marker_data = self.collect_marker_info(object.marker_ids(), LOCALIZATION_TIMEOUT)
                trans_data = {id: self._tf_buffer.transform(m, object.parent, timeout=rospy.Duration(0.25)) for
                             (id, m) in marker_data.items()}
                object.localize(trans_data)
                rospy.loginfo("Localization finished")
                srv_resp.success = True
            except Exception as e:
                rospy.logerr("Failed to localize object '{}': {}".format(object.name, str(e)))
                srv_resp.success = False
        return srv_resp

    def collect_marker_info(self, marker_ids, timeout):
        """
        Subscribe to the marker topic on-the-fly and wait until a marker message arrives with all
        the markers requested for localization.

        :type marker_ids: list
        :type timeout: float
        """
        marker_data = dict()
        sorted_ids = sorted(marker_ids)

        t0 = rospy.Time.now()
        done = False
        while not done:
            marker_data.clear()

            try:
                msg = rospy.wait_for_message(MARKER_TOPIC, aruco_msgs.msg.MarkerArray, timeout)
            except rospy.ROSException as e:
                raise RuntimeError("Time out waiting for required localization marker data")

            if rospy.Time.now() - t0 > rospy.Duration(timeout):
                raise RuntimeError("Time out waiting for required localization marker data")

            for marker in msg.markers:
                if marker.id in marker_ids:
                    pose_stamped = geometry_msgs.msg.PoseStamped()
                    pose_stamped.header = marker.header
                    pose_stamped.pose = marker.pose.pose
                    marker_data[marker.id] = pose_stamped
            # check if we recorded all required markers
            if sorted(marker_data.keys()) == sorted_ids:
                rospy.loginfo('Marker data acquired')
                done = True

        return marker_data

    def _broadcast_tf_cb_(self, evnt):
        for object in self.objects:
            tf = object.get_tf_transform()
            tf.header.stamp = evnt.current_real
            self._tf_broadcaster.sendTransform(tf)

if __name__ == '__main__':
    rospy.init_node('object_localizer')

    loc = Localizer()
    loc_param = rospy.get_param('~' + LOCALIZATION_INFO_PARAM)

    for obj_param in loc_param:
        obj = LocateableObject(obj_param)
        loc.add_object(obj)
        rospy.loginfo("Loaded object '{}' for localization".format(obj.name))

    rospy.spin()
