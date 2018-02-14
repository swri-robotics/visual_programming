#! /usr/bin/env python

import rospy
from std_msgs.msg import String
from vpr_msgs.srv import ProcessAction
from vpr_msgs.srv._ProcessAction import ProcessActionRequest
from vpr_msgs.msg import VprErrorCode
from rospy.exceptions import ROSInterruptException, ROSException
from geometry_msgs.msg import Pose, Point, Quaternion
from visualization_msgs.msg import Marker
from collections import namedtuple
from enum import Enum
import geometry_msgs

KEYBOARD_TOPIC = 'key_pressed'
PROCESS_ACTION_SERVICE = 'process_action'
INSTRUCTIONS_MARKER_TOPIC = 'instructions'
CURRENT_STATE_TOPIC = 'current_state'


MapEntry = namedtuple('MapEntry', ['label','action'])    
TRACKING_MODE_ACTIONS_MAP = {'a': MapEntry('ADD_WAYPOINT','ADD_WAYPOINT'),
                         's': MapEntry('ADD_SEGMENT','ADD_SEGMENT'),
                         'd': MapEntry('DELETE_WAYPOINT','DELETE_WAYPOINT'),
                         'p': MapEntry('PREVIEW PATH','SHOW_PREVIEW'),
                         'c': MapEntry('CLEAR_WAYPOINTS','CLEAR_WAYPOINTS'),
                         'h': MapEntry('MOVE_HOME','MOVE_HOME')}   

CREATE_SEGMENT_MODE_ACTIONS_MAP = {'s': MapEntry('SAVE_SEGMENT','SAVE_SEGMENT'),
                         'x': MapEntry('CANCEL_SEGMENT','CANCEL_SEGMENT')}   

PREVIEW_MODE_ACTIONS_MAP = {'p': MapEntry('PREVIEW PATH','SHOW_PREVIEW'),
                         'e': MapEntry('EXECUTE_PATH','EXECUTE'),
                         'x': MapEntry('RETURN','EXIT_PREVIEW')}  

EXECUTE_MODE_ACTIONS_MAP = {} 


class Mode(Enum):
    
    TRACKING = 1
    CREATE_SEGMENT = 2
    PREVIEW = 3
    EXECUTE = 4
    
    
STATES_MAPPINGS ={'Tracking':(Mode.TRACKING,TRACKING_MODE_ACTIONS_MAP), 
                  'ContinuousEdit':(Mode.CREATE_SEGMENT,CREATE_SEGMENT_MODE_ACTIONS_MAP),
                  'Preview': (Mode.PREVIEW,PREVIEW_MODE_ACTIONS_MAP),
                  'Execute': (Mode.EXECUTE,EXECUTE_MODE_ACTIONS_MAP)}


class KeyActionMapper:
        
    def __init__(self):
            
        self.key_actions_map_ = TRACKING_MODE_ACTIONS_MAP 
        self.current_mode_ = Mode.TRACKING
        self.text_markers_ = None
        self.current_state_listener_ = rospy.Subscriber(CURRENT_STATE_TOPIC,String,self.currentStateCallback)
        
    def currentStateCallback(self,msg):
        
        new_mode = None
        if msg.data in STATES_MAPPINGS:
            new_mode = STATES_MAPPINGS[msg.data][0]   
        else:
            return      
            
        if new_mode is not self.current_mode_:
            self.current_mode_ = new_mode      
            self.key_actions_map_ = STATES_MAPPINGS[msg.data][1]
            self.text_markers_ = self.createInstructionsMarker()   
        
        
    def run(self):
        
        try:
            rospy.wait_for_service(PROCESS_ACTION_SERVICE,5.0)
        except ROSException as e:
            rospy.logerr('Service %s was not found'%(PROCESS_ACTION_SERVICE))
            return
        except ROSInterruptException as e:
            rospy.logerr(str(e))
            return
        
        self.hz_ = rospy.get_param('~hz',10.0) 
        self.text_marker_size_ = rospy.get_param('~text_marker_size',0.04)
        self.text_frame_id_ = rospy.get_param('~frame_id','instructions_frame')           
        
        self.keyboard_subs = rospy.Subscriber(KEYBOARD_TOPIC,String,self.keyboardCallback)
        self.process_action_client_ = rospy.ServiceProxy(PROCESS_ACTION_SERVICE,ProcessAction)
        self.marker_pub_ = rospy.Publisher(INSTRUCTIONS_MARKER_TOPIC,Marker,queue_size=10)
        
        loop_rate = rospy.Rate(self.hz_)    
        self.text_markers_ = self.createInstructionsMarker()    
        while not rospy.is_shutdown():
            self.marker_pub_.publish(self.text_markers_)
            loop_rate.sleep()
            
    def createInstructionsMarker(self):
        
        marker = Marker()
        marker.header.frame_id = self.text_frame_id_
        marker.id = 1
        marker.ns = ''
        marker.action = marker.ADD
        marker.type = marker.TEXT_VIEW_FACING
        marker.lifetime = rospy.Duration(0.0)
        marker.scale.z = self.text_marker_size_
        marker.color.r = 0.8
        marker.color.g = 0.8
        marker.color.b = 1.0
        marker.color.a = 1.0
        
        if len(self.key_actions_map_) > 0:
            
            marker.text = ' KEYBOARD COMMANDS: \n'
            for k, v in self.key_actions_map_.iteritems():
                marker.text += '  - ' + k + ' : ' + v.label + '\n' 
        else:
            marker.text = ' NO COMMANDS AVAILABLE \n'
            
        marker.pose = Pose()
        marker.pose.position = Point(0.0,0.0,0.0)
        marker.pose.orientation = Quaternion(0.0, 0.0, 0.0, 1.0)
        
        return marker
        
    def keyboardCallback(self,msg):
        
        key_pressed = msg.data
        
        try:
            if key_pressed in self.key_actions_map_:
                
                action_id = self.key_actions_map_[key_pressed].action
                res = self.process_action_client_(action_id)
                    
                if res.error.val is not  VprErrorCode.SUCCESS:
                    rospy.logerr('Action failed to be processed')
                    return
                    
        except rospy.ServiceException as e:
            rospy.logerr('Service call %s failed'%(PROCESS_ACTION_SERVICE))
            
if __name__ == '__main__':
    
    rospy.init_node('key_action_mapper')
    app = KeyActionMapper()
    app.run()
            
        
        