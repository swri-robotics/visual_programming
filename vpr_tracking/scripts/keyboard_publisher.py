#! /usr/bin/env python
# -*- coding: utf-8 -*-
#
# Copyright (c) 2013 PAL Robotics SL.
# Released under the BSD License.
#
# Authors:
#   * Siegfried-A. Gevatter

import curses
import math
from std_msgs.msg import String
import rospy
import std_msgs
import sys

KEY_TOPIC = 'key_pressed'

class TextWindow():

    _screen = None
    _window = None
    _num_lines = None

    def __init__(self, stdscr, lines=10):
        self._screen = stdscr
        self._screen.nodelay(True)
        curses.curs_set(0)

        self._num_lines = lines

    def read_key(self):
        keycode = self._screen.getch()
        return keycode if keycode != -1 else None

    def clear(self):
        self._screen.clear()

    def write_line(self, lineno, message):
        if lineno < 0 or lineno >= self._num_lines:
            raise ValueError, 'lineno out of bounds'
        height, width = self._screen.getmaxyx()
        y = (height / self._num_lines) * lineno
        x = 10
        for text in message.split('\n'):
            text = text.ljust(width)
            self._screen.addstr(y, x, text)
            y += 1

    def refresh(self):
        self._screen.refresh()

    def beep(self):
        curses.flash()


class SimpleKeyboardPublisher():
    def __init__(self, interface):
        self._interface = interface
        self._hz = rospy.get_param('~hz', 10)
        self.key_pub_ = rospy.Publisher(KEY_TOPIC,String,queue_size=10)


    def run(self):
        
        rate = rospy.Rate(self._hz)                    
        while not rospy.is_shutdown():
            keycode = self._interface.read_key()  
            
            print('key pressed %s'%(keycode))          
            self._interface.clear()
            self._interface.refresh()
            
            msg = String()
            if keycode is not None:
                try:
                    msg.data = chr(keycode)
                except ValueError as e:
                    rospy.logwarn("Only characters or numbers are reported")
                    
            else:
                msg.data = ''
                
            self.key_pub_.publish(msg)
            rate.sleep()


def main(stdscr):
    rospy.init_node('keyboard_publisher')
    app = SimpleKeyboardPublisher(TextWindow(stdscr))
    app.run()

if __name__ == '__main__':
    try:
        curses.wrapper(main)
    except rospy.ROSInterruptException:
        pass
