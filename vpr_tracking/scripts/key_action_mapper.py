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
from jsk_rviz_plugins.msg import OverlayText
import geometry_msgs

STATES_PARAM = 'states'
KEYBOARD_TOPIC = 'key_pressed'
PROCESS_ACTION_SERVICE = 'process_action'
CURRENT_STATE_TOPIC = 'current_state'
COMMAND_OVERLAY_TXT_TOPIC = 'application_commands'
MARKER_STRING_LINE_WIDTH = 30

class KeyActions(Enum):    
    UP = 'q'
    DOWN = 'z'
    ACCEPT = 'a'

class Mode(Enum):
    
    TRACKING = 1
    CREATE_SEGMENT = 2
    PREVIEW = 3
    EXECUTE = 4

StateInfo = namedtuple('StateInfo', ['id','label','action_map'])
ActionInfo = namedtuple('ActionInfo', ['id', 'order','label'])

def parseStatesConfigParam(states_list_param):

    states_dict = {}
    for item in states_list_param:
        actions_dict = {}
        actions_list = item['actions']
        for sub_item in actions_list:
            action_info = ActionInfo(sub_item['id'],sub_item['order'],sub_item['label'])
            actions_dict[action_info.label] = action_info

        state_info = StateInfo(item['id'], item['label'], actions_dict)
        states_dict[state_info.id] = state_info

    return states_dict


class KeyActionMapper:
        
    def __init__(self):
        
        self.current_state_ = None
        self.current_action_list_ = None
        self.current_action_index_ = 0
        self.states_dict_ = None
                
    def currentStateCallback(self,msg):
            
        new_state = None
        if msg.data in self.states_dict_:
            new_state = msg.data
        else:
            return

        if new_state != self.current_state_:
            self.selectState(new_state)
            self.cmd_overlay_text_msg_ = self.createCmdOverlayText()    
            

    def selectState(self,state_id):
        self.current_state_ = state_id
        actions_dict = self.states_dict_[self.current_state_].action_map
        sorted_action_infos = sorted(actions_dict.items(), key=lambda v: v[1].order)
        self.current_action_list_ = [a[1] for a in sorted_action_infos]
        self.current_action_index_ = 0
        
        # print commands
        print('Commands List: %s' %(str(list(KeyActions))))
        
        rospy.loginfo('Selected state "%s"'%(state_id))                    
            
    def createCmdOverlayText(self):
        overlay_txt = OverlayText()
        overlay_txt.action = overlay_txt.ADD
        overlay_txt.text_size = 15
        overlay_txt.width = 300
        overlay_txt.height = 250
        overlay_txt.left = 750
        overlay_txt.top = 20
        overlay_txt.fg_color.r, overlay_txt.fg_color.g, overlay_txt.fg_color.b, overlay_txt.fg_color.a = (1.0,1.0,1.0,0.8)
        overlay_txt.bg_color.r, overlay_txt.bg_color.g, overlay_txt.bg_color.b, overlay_txt.bg_color.a = (135.0/255.0,206.0/255.0,1.0,0.2)
        
        current_st_info = self.states_dict_[self.current_state_]
        indentation_str = '  '
        overlay_txt.text = ('\n|%s State\n|%sCommands:'%(current_st_info.label,indentation_str)).ljust(MARKER_STRING_LINE_WIDTH) + '\n'
        if len(self.current_action_list_) > 0:


            for i, action_info in enumerate(self.current_action_list_):

                action_id = action_info.id
                action_label = action_info.label

                new_line_str = ''
                if i == self.current_action_index_:
                    new_line_str = '|%s=>> ['%(indentation_str*2) + action_label +  ']'
                else:
                    new_line_str = '|%s- '%(indentation_str*2) + action_label

                new_line_str = new_line_str.ljust(MARKER_STRING_LINE_WIDTH)
                overlay_txt.text += new_line_str + '\n'
        else:
            overlay_txt.text += '\tNO COMMANDS AVAILABLE \n'
            
        return overlay_txt        
        
    def run(self):
        
        try:
            rospy.wait_for_service(PROCESS_ACTION_SERVICE,5.0)
        except ROSException as e:
            rospy.logerr('Service %s was not found'%(PROCESS_ACTION_SERVICE))
            return
        except ROSInterruptException as e:
            rospy.logerr(str(e))
            return
        
        # loading required parameters
        param = rospy.get_param('~' + STATES_PARAM)
        self.states_dict_ = parseStatesConfigParam(param)
        
        # selecting state
        kv_list = list(self.states_dict_.items())
        state_id = kv_list[0][1].id
        self.selectState(state_id)

        # loading optional parameters
        self.hz_ = rospy.get_param('~hz',10.0) 
        self.text_marker_size_ = rospy.get_param('~text_marker_size',0.04)
        self.text_frame_id_ = rospy.get_param('~frame_id','instructions_frame')       
        self.cmd_overlay_text_msg_ = self.createCmdOverlayText()    
        
        # setting subscribers, publishes and services
        self.keyboard_subs = rospy.Subscriber(KEYBOARD_TOPIC,String,self.keyboardCallback)
        self.process_action_client_ = rospy.ServiceProxy(PROCESS_ACTION_SERVICE,ProcessAction)
        self.cmd_overlay_txt_pub_ = rospy.Publisher(COMMAND_OVERLAY_TXT_TOPIC,OverlayText,queue_size=10)        
        self.current_state_listener_ = rospy.Subscriber(CURRENT_STATE_TOPIC,String,self.currentStateCallback)
                
        loop_rate = rospy.Rate(self.hz_)    
        
        while not rospy.is_shutdown():
            self.cmd_overlay_txt_pub_.publish(self.cmd_overlay_text_msg_)
            loop_rate.sleep() 
    
    def callActionService(self):

        rospy.logdebug("calling action service")
        try:
            action_id = self.current_action_list_[self.current_action_index_].id
            res = self.process_action_client_(action_id)
            if res.error.val !=  VprErrorCode.SUCCESS:
                rospy.logerr('Action failed to be processed')
                return

        except rospy.ServiceException as e:
            rospy.logerr('Service call %s failed'%(PROCESS_ACTION_SERVICE))
        
    def keyboardCallback(self,msg):
        
        key_pressed = msg.data
        
        if not key_pressed:
            return 
        
        #rospy.loginfo('got key %s\r' %( key_pressed))     
            
        if KeyActions.UP.value == key_pressed:
            self.current_action_index_ -= 1
        elif KeyActions.DOWN.value == key_pressed:
            self.current_action_index_ += 1
        elif KeyActions.ACCEPT.value == key_pressed:
            self.callActionService()
            return
        
        # capping index
        max_index = len(self.current_action_list_) - 1
        min_index = 0
        self.current_action_index_ = max_index if self.current_action_index_ < 0 else (
            min_index if self.current_action_index_ > max_index else self.current_action_index_)
        
        # updating string markers
        self.cmd_overlay_text_msg_ = self.createCmdOverlayText()

            
if __name__ == '__main__':
    
    rospy.init_node('key_action_mapper')
    app = KeyActionMapper()
    app.run()
            
        
        