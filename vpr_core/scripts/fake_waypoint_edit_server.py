#! /usr/bin/env python

from __future__ import absolute_import, division, print_function

import rospy
import roslib
import math
from vpr_msgs.srv._EnterEditMode import *
from std_srvs.srv._SetBool import *
from tf.transformations import *
import numpy as np
from numpy import dtype

roslib.load_manifest('vpr_core')

ENTER_EDIT_MODE_SERVICE = 'enter_edit_mode'
CONFIRM_EDIT_SERVICE = 'confirm_edit'


def create2DBox(side_length, num_points):

    # create corners
    l = 0.5 * side_length
    start_angle = math.pi / 4.0
    step_angle = math.pi * 0.5
    num_corner_points = 4
    num_points_per_side = int(math.ceil(num_points / 4.0))
    corner_points = []

    points_buffer = np.array([], dtype=np.float).reshape(3, 0)
    for i in range(0, 4):
        x = math.cos(start_angle + i * step_angle)
        y = math.sin(start_angle + i * step_angle)
        corner_points.append([x, y, 0.0])

        x_prev = None
        y_prev = None
        if i == 0:
            continue

        elif i == num_corner_points - 1:
            x_prev = corner_points[0][0]
            y_prev = corner_points[0][1]

        else:
            x_prev = corner_points[i - 1][0]
            y_prev = corner_points[i - 1][0]

        # interpolating
        x_points = np.linspace(x, x_prev, num=num_points_per_side)
        y_points = np.linspace(y, y_prev, num=num_points_per_side)
        z_points = np.zeros((1, num_points_per_side), dtype=np.float)

        temp = np.vstack([x_points, y_points])
        temp = np.vstack([temp, z_points])
        points_buffer = np.hstack([points_buffer, temp])

    return points_buffer


def transformPoints(points, position, rotation):

    xaxis, yaxis, zaxis = (1, 0, 0), (0, 1, 0), (0, 0, 1)

    P = translation_matrix(tuple(position))
    Rx = rotation_matrix(rotation[0], xaxis)
    Ry = rotation_matrix(rotation[1], yaxis)
    Rz = rotation_matrix(rotation[2], zaxis)
    T = concatenate_matrices(P, Rx, Ry, Rz)

    print('Transformation Matrix')
    print(T)

    # padding points
    num_points = points.shape[1]
    padded_points = np.vstack(
        [points, np.ones((1, num_points), dtype=np.float)])
    transformed_points = np.dot(T, padded_points)

    return transformed_points[0:3, :]

class WaypointEditorServer:
    
    __metaclass__ = type
    
    def __init__(self):
        self.box_side_length_ = 0.5
        self.num_points_ = 20
        self.box_position_ = (0.2, 0.0, 0.5)  # meters
        self.box_rpy_ = [0.0, 0.0, 0.0]  # radians
        self.current_point_index_ = 0
        self.points_buffer_ = np.array([], dtype=np.float).reshape([3, 0])
        self.on_edit_mode_ = False

    def run(self):

        # read parameters
        self.box_side_length_ = rospy.get_param('~box_side_length',
                                                self.box_side_length_)
        self.num_points_ = rospy.get_param('~num_points', self.num_points_)
        self.box_position_ = rospy.get_param('~box_position',
                                             self.box_position_)
        self.box_rpy_ = rospy.get_param('~box_rpy', self.box_rpy_)

        # create box points
        self.box_points_ = create2DBox(self.box_side_length_, self.num_points_)
        rospy.loginfo('Box Points:')
        print(np.transpose(self.box_points_))

        # transforming points
        self.box_points_ = transformPoints(self.box_points_,
                                           self.box_position_, self.box_rpy_)
        rospy.loginfo('Transformed Box Points:')
        print(np.transpose(self.box_points_))

        self.enter_edit_mode_server_ = rospy.Service(
            ENTER_EDIT_MODE_SERVICE, EnterEditMode, self.enterEditModeCb)
        self.confirm_edit_server_ = rospy.Service(CONFIRM_EDIT_SERVICE,
                                                  SetBool, self.confirmEditCb)

        rospy.loginfo('Waypoint Editor Server Ready ...')
        rospy.spin()

    def enterEditModeCb(self, req):

        res = EnterEditModeResponse()

        if self.on_edit_mode_:
            rospy.logerr('Already on Edit Mode, must accept/cancel first')
            res.success = False
            return res

        rospy.loginfo("Entered Edit Mode")

        success = True
        if req.operation is EnterEditModeRequest.ADD:

            if self.current_point_index_ < self.box_points_.shape[1]:
                p = self.box_points_[:, self.current_point_index_]
                print('Adding point: %s' % (str(p)))
                self.points_buffer_ = np.hstack(
                    [self.points_buffer_,
                     p.reshape((3, 1))])
                self.current_point_index_ += 1
            else:
                rospy.logerr(
                    "ADD operation failed, All points have been added")
                success = False

        elif req.operation is EnterEditModeRequest.DELETE:

            if self.points_buffer_.shape[1] > 0:
                np.delete(self.points_buffer_, -1, axis=1)
                self.current_point_index_ -= 1
            else:
                rospy.logerr(
                    'DELETE operation failed, there are no points in the buffer to delete'
                )
                success = False

        elif req.operation is EnterEditModeRequest.CLEAR_ALL:
            self.current_point_index_ = 0
            self.points_buffer_ = np.array([], dtype=np.float).reshape([3, 0])

        elif req.operation is EnterEditModeRequest.SELECT or req.operation is EnterEditModeRequest.MODIFY:
            pass

        else:
            rospy.logerr('Invalid operation requested')
            success = False

        res.success = success
        if (res.success):
            self.on_edit_mode_ = True

        return res

    def confirmEditCb(self, req):

        res = SetBoolResponse()
        if not self.on_edit_mode_:
            rospy.log("Not on edit mode, must enter edit mode first")
            res.success = False
            return res

        rospy.loginfo("Edit Confirmed, exiting edit mode")
        self.on_edit_mode_ = False
        res.success = True
        return res


if __name__ == '__main__':

    rospy.init_node('fake_waypoint_edit_server')
    server = WaypointEditorServer()
    server.run()
