#! /usr/bin/env python

from __future__ import print_function
import rospy
from std_msgs.msg import String
from vpr_msgs.srv import ProcessAction
from vpr_msgs.srv._ProcessAction import ProcessActionRequest
from vpr_msgs.msg import VprErrorCode
from rospy.exceptions import ROSInterruptException, ROSException
from geometry_msgs.msg import Pose, Point, Quaternion
from visualization_msgs.msg import Marker
from collections import namedtuple
from bitarray import bitarray
from enum import Enum
from sensor_msgs.msg import Joy
from copy import copy
from jsk_rviz_plugins.msg import OverlayText
import geometry_msgs
import yaml
import rosparam
import std_msgs

STATES_PARAM = 'states'
BUTTON_MAPPINGS_PARAM = 'button_mappings'
PROCESS_ACTION_SERVICE = 'process_action'
CURRENT_STATE_TOPIC = 'current_state'
JOYSTICK_TOPIC = 'joy'
COMMAND_OVERLAY_TXT_TOPIC = 'application_commands'
MARKER_STRING_LINE_WIDTH = 30

ButtonInfo = namedtuple('ButtonInfo', ['name','bitmask'])
ButtonConfig = namedtuple('ButtonConfig', ['index','value','bitmask'])
StateInfo = namedtuple('StateInfo', ['id','label','action_map'])
ActionInfo = namedtuple('ActionInfo', ['id', 'order','label'])

class ButtonBitmasks(Enum):

    __BUTTON_MASK_LENGHT__ = 10

    NONE =          __BUTTON_MASK_LENGHT__ * bitarray('0')
    AXIS_DOWN =     bitarray('1000000000')
    AXIS_UP =       bitarray('0100000000')
    AXIS_LEFT =     bitarray('0010000000')
    AXIS_RIGHT =    bitarray('0001000000')
    TRIGGER =       bitarray('0000100000')



class JoystickAxisState:


    def __init__(self,axis_up_config, axis_down_config, axis_left_config, axis_right_config ):
        self.axis_up_config_ = axis_up_config
        self.axis_down_config_ = axis_down_config
        self.axis_left_config_ = axis_left_config
        self.axis_right_config_ = axis_right_config

        self.axis_configs_ = [self.axis_up_config_, self.axis_down_config_ ,
                              self.axis_left_config_, self.axis_right_config_]

    def captureState(self, msg):
        buttons_down = copy(ButtonBitmasks.NONE.value)

        for btn_cfg in self.axis_configs_:
            val = msg.axes[btn_cfg.index]
            buttons_down |= btn_cfg.bitmask if val == btn_cfg.value else ButtonBitmasks.NONE.value

        return buttons_down

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

def parseButtonMappingsParam(button_mappings_param):

    button_mappings_dict = {}
    for k,v in button_mappings_param.items():
        button_name = k.upper()
        if button_name not in ButtonBitmasks.__members__:
            rospy.logerr("The button '%s' is not supported"%button_name)
            exit(-1)
        button_config = ButtonConfig(v['index'],v['value'],ButtonBitmasks[button_name].value)
        button_mappings_dict[ButtonBitmasks[button_name]] = button_config

    return button_mappings_dict

class JoystickManager(object):

    def __init__(self):
        self.current_state_ = None
        self.current_action_list_ = None
        self.current_action_index_ = 0
        self.states_dict_ = None

        # button management
        self.button_mappings_ = None
        self.joystick_axes_state_ = None
        self.previous_buttons_down_ = copy(ButtonBitmasks.NONE.value)
        self.buttons_callbacks_dict_ ={ButtonBitmasks.TRIGGER: self.callActionService}

        # published data
        self.text_frame_id_ = ''
        self.cmd_overlay_text_msg_ = None


    def run(self):

        try:
            rospy.wait_for_service(PROCESS_ACTION_SERVICE,5.0)
        except ROSException as e:
            rospy.logerr('Service "%s" was not found'%(PROCESS_ACTION_SERVICE))
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

        param = rospy.get_param('~' + BUTTON_MAPPINGS_PARAM)
        self.button_mappings_ = parseButtonMappingsParam(param)

        axes_configs = [self.button_mappings_[ButtonBitmasks.AXIS_UP],
                        self.button_mappings_[ButtonBitmasks.AXIS_DOWN],
                        self.button_mappings_[ButtonBitmasks.AXIS_LEFT],
                        self.button_mappings_[ButtonBitmasks.AXIS_RIGHT]]

        self.joystick_axes_state_ = JoystickAxisState(*axes_configs)

        # loading optional parameters
        self.hz_ = rospy.get_param('~hz',10.0)
        self.text_marker_size_ = rospy.get_param('~text_marker_size',0.08)
        self.text_frame_id_ = rospy.get_param('~frame_id','instructions_frame')
        self.cmd_overlay_text_msg_ = self.createCmdOverlayText()

        # setting subscribers, publishes and services
        self.joystick_subs_ = rospy.Subscriber(JOYSTICK_TOPIC,Joy,self.processJoystickMsg)
        self.process_action_client_ = rospy.ServiceProxy(PROCESS_ACTION_SERVICE,ProcessAction)
        self.cmd_overlay_txt_pub_ = rospy.Publisher(COMMAND_OVERLAY_TXT_TOPIC,OverlayText,queue_size=10)
        self.current_state_listener_ = rospy.Subscriber(CURRENT_STATE_TOPIC,String,self.currentStateCallback)

        loop_rate = rospy.Rate(self.hz_)

        while not rospy.is_shutdown():
            self.cmd_overlay_txt_pub_.publish(self.cmd_overlay_text_msg_)
            loop_rate.sleep()


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

    def createCmdTextMarker(self):

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

        # creating text
        current_st_info = self.states_dict_[self.current_state_]
        marker.text = ('%s\n\tCommands:'%(current_st_info.label)).ljust(MARKER_STRING_LINE_WIDTH) + '\n'
        if len(self.current_action_list_) > 0:


            for i, action_info in enumerate(self.current_action_list_):

                action_id = action_info.id
                action_label = action_info.label

                new_line_str = ''
                if i == self.current_action_index_:
                    new_line_str = '\t=>> [' + action_label +  ']'
                else:
                    new_line_str = '\t- ' + action_label

                new_line_str = new_line_str.ljust(MARKER_STRING_LINE_WIDTH)
                marker.text += new_line_str + '\n'
        else:
            marker.text += '\tNO COMMANDS AVAILABLE \n'

        marker.pose = Pose()
        marker.pose.position = Point(0.0,0.0,0.0)
        marker.pose.orientation = Quaternion(0.0, 0.0, 0.0, 1.0)

        return marker

    def processAxesInputs(self,combined_buttons_bitmask):
        if combined_buttons_bitmask & ButtonBitmasks.AXIS_DOWN.value == ButtonBitmasks.AXIS_DOWN.value:
            self.current_action_index_ += 1
        elif combined_buttons_bitmask & ButtonBitmasks.AXIS_UP.value == ButtonBitmasks.AXIS_UP.value:
            self.current_action_index_ -= 1

        # wrap around
        max_index = len(self.current_action_list_) - 1
        min_index = 0
        self.current_action_index_ = max_index if self.current_action_index_ < 0 else (
            min_index if self.current_action_index_ > max_index else self.current_action_index_)

        # updating string markers
        self.cmd_overlay_text_msg_ = self.createCmdOverlayText()

    def processButtonsInputs(self, combined_buttons_bitmask):
        for b, cb in self.buttons_callbacks_dict_.items():
            button_bitmask = self.button_mappings_[b].bitmask
            if (button_bitmask & combined_buttons_bitmask) == button_bitmask:
                cb()

    def processJoystickMsg(self, msg):

        buttons_pressed = copy(ButtonBitmasks.NONE.value)
        buttons_down = copy(ButtonBitmasks.NONE.value)

        # decoding msg axis data
        directions_down = self.joystick_axes_state_.captureState(msg)
        buttons_down |= directions_down

        # decoding msg buttons data
        for b, cb in self.buttons_callbacks_dict_.items():
            button_ind = self.button_mappings_[b].index
            down_val = self.button_mappings_[b].value
            button_bitmask = self.button_mappings_[b].bitmask

            if msg.buttons[button_ind] == down_val:
                buttons_down |= button_bitmask

        buttons_pressed = buttons_down & ~self.previous_buttons_down_

        # dispatching axes actions
        self.processAxesInputs(buttons_pressed)

        # dispatching buttons actions
        self.processButtonsInputs(buttons_pressed)
        rospy.logdebug('buttons down bitmask: %s'%(str(buttons_down)))

        self.previous_buttons_down_ = buttons_down

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


if __name__ == '__main__':

    rospy.init_node('joystick_action_mapper')
    jm = JoystickManager()
    jm.run()

