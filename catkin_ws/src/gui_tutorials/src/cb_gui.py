#!/usr/bin/env python3.8

# # from kivy.core.window import Window
# # from kivy.uix.popup import Popup
# # from kivy.uix.floatlayout import FloatLayout
# # from kivy.core.window import Window
# # 

import rospy
from roscore import *
import rosgraph
import subprocess
import sys
import time
from threading import *
from dynamixel_sdk import *
from dynamixel_sdk_examples.srv import *
from dynamixel_sdk_examples.msg import *
from kivymd.app import MDApp
from kivy.app import App
from kivy.lang import Builder
import os
os.environ['KIVY_GL_BACKEND'] = 'sdl2'
from kivymd.uix.tab import MDTabsBase
from kivy.uix.dropdown import DropDown
from kivymd.uix.floatlayout import MDFloatLayout
from kivymd.icon_definitions import md_icons
from kivy.uix.screenmanager import ScreenManager, Screen

# IMU Imports
import roslaunch
from sensor import *
import re

from kivymd.uix.menu import MDDropdownMenu
from kivymd.uix.snackbar import Snackbar

from kivymd.uix.list import OneLineListItem
from kivy.properties import *
from kivy.network.urlrequest import UrlRequest
import urllib.request
from sensor_msgs.msg import JointState
from std_msgs.msg import Int64
import math
from robot_section import *
from pickletools import uint8
from std_msgs.msg import Bool

#roscore = Roscore()

# for purpose of dmeos
left_arm = [2,4,6,8,10,12,14]
right_arm = [1,3,5,7,9,11,13]

#Builder.load_string()
KV = """
#:import rgba kivy.utils.get_color_from_hex

<MenuScreen>:
    name: 'home'
    img: asyn_image
    FloatLayout:
        orientation: "vertical"

        MDTopAppBar:
            title: "[b]Humanoid Control Panel[/b]"
            anchor_title: "center"
            left_action_items: [["home-analytics", lambda x: app.set_screen('home')]]
            right_action_items: [["engine", lambda x: app.callback(x)], ["robot-confused-outline", lambda x: app.set_screen('id')], ["power", lambda x: app.callback_2()]]
            size_hint: 1, .1
            pos_hint: {"center_x": .5, "center_y": 0.95}

        AsyncImage:
            id: asyn_image
            source: root.image_path
            size_hint: None, None
            width: 400
            height: 400
            pos_hint: {'center_x':.25, 'center_y': .6}
            nocache: True
        
        Label:
            pos_hint: {"center_x": 0.25, "center_y": 0.325}
            bold: True
            font_size: '20sp'
            text: 'LIVE URDF MODEL'
            size_hint_x: None
            width: 1
            color: (0,0,0,1)

        Label:
            pos_hint: {"center_x": 0.25, "center_y": 0.3}
            bold: True
            font_size: '12sp'
            text: 'POSITION IN DEGREES | VELOCITY IN ROTATIONS/S'
            size_hint_x: None
            width: 1
            color: (0,0,0,1)

        Button:
            id: emergency
            bold: True
            font_size: '18sp'
            text: "EMERGENCY STOP"
            background_normal: ''
            background_color: rgba("#ff0a01")
            color: 1,1,1,1
            halign: "center"
            # background_color: get_color_from_hex('#FF0A01')
            # background_color: 0,1,0,1
            size_hint: 0.2, 0.15
            pos_hint: {"center_x": .13, "center_y": 0.18}
            on_press: app.emergency_stop()

        Button:
            id: run_btn
            bold: True
            font_size: '20sp'
            text: "REFRESH"
            background_normal: ''
            background_color: rgba("#00ab66")
            color:1,1,1,1
            halign: "center"
            size_hint: 0.2, 0.15
            pos_hint: {"center_x": .35, "center_y": 0.18}
            on_press: root.change_image("new_robotImage.png")
        
        Label:
            pos_hint: {"center_x": 0.75, "center_y": 0.85}
            bold: True
            font_size: '20sp'
            text: 'DEMOS'
            size_hint_x: None
            width: 1
            color: (0,0,0,1)
        
        Button: 
            id: wave_left_arm
            text: "Wave Left Arm"
            size_hint: 0.15, 0.1
            pos_hint: {"center_x": .65, "center_y": 0.775}
            on_press: app.wave_left_arm('wave_left_arm')

        Button: 
            id: wave_right_arm
            text: "Wave Right Arm"
            size_hint: 0.15, 0.1
            pos_hint: {"center_x": .8, "center_y": 0.775}
            on_press: app.wave_right_arm('wave_right_arm')
            
        MDTabs:
            size_hint: 1, 0.08
            tab_hint_x: True
            allow_stretch: True
            text_color_active: [1, 1, 1, 1]
            underline_color: [33/255, 150/255, 243/255, 1]
            on_tab_switch: app.on_tab_switch(*args)
            Tab:
                title: '[b]LiDAR[/b]'
                icon: 'motion-sensor'
            Tab:
                title: '[b]RealSense[/b]'
                icon: 'camera-outline'
            Tab:
                title: '[b]IMU[/b]'
                icon: 'chart-waterfall'
            Tab:
                title: '[b]Force Torque[/b]'
                icon: 'cog'

<MenuLeftArmScreen>:
    name: 'home_leftarm_control'
    img: la_asyn_image
    m1_pos: m1_textinput
    m2_pos: m2_textinput
    m3_pos: m3_textinput
    m4_pos: m4_textinput
    m5_pos: m5_textinput
    m6_pos: m6_textinput
    m7_pos: m7_textinput
    m8_pos: m8_textinput
    m9_pos: m9_textinput
    m10_pos: m10_textinput
    
    m1_vel: velo1_textinput
    m2_vel: velo2_textinput
    m3_vel: velo3_textinput
    m4_vel: velo4_textinput
    m5_vel: velo5_textinput
    m6_vel: velo6_textinput
    m7_vel: velo7_textinput
    m8_vel: velo8_textinput
    m9_vel: velo9_textinput
    m10_vel: velo10_textinput

    voltage: voltage_id
    current: current_id
    temperature: temperature_id

    check1: m1_checkbox
    check2: m2_checkbox
    check3: m3_checkbox
    check4: m4_checkbox
    check5: m5_checkbox
    check6: m6_checkbox
    check7: m7_checkbox
    check8: m8_checkbox
    check9: m9_checkbox
    check10: m10_checkbox

    FloatLayout:
        orientation: "vertical"

        MDTopAppBar:
            title: "[b]Humanoid Control Panel[/b]"
            anchor_title: "center"
            left_action_items: [["home-analytics", lambda x: app.set_screen('home')]]
            right_action_items: [["engine", lambda x: app.callback(x)], ["robot-confused-outline", lambda x: app.set_screen('id')], ["power", lambda x: app.callback_2()]]
            size_hint: 1, .1
            pos_hint: {"center_x": .5, "center_y": 0.95}
        
        AsyncImage:
            id: la_asyn_image
            source: root.image_path
            size_hint: None, None
            width: 400
            height: 400
            pos_hint: {'center_x':.25, 'center_y': .6}
            nocache: True

        Label:
            pos_hint: {"center_x": 0.25, "center_y": 0.325}
            bold: True
            font_size: '20sp'
            text: 'LIVE URDF MODEL'
            size_hint_x: None
            width: 1
            color: (0,0,0,1)

        Label:
            pos_hint: {"center_x": 0.25, "center_y": 0.3}
            bold: True
            font_size: '12sp'
            text: 'POSITION IN DEGREES | VELOCITY IN ROTATIONS/S'
            size_hint_x: None
            width: 1
            color: (0,0,0,1)

        Button:
            id: emergency
            bold: True
            font_size: '18sp'
            text: "EMERGENCY STOP"
            background_normal: ''
            background_color: rgba("#ff0a01")
            color: 1,1,1,1
            halign: "center"
            # background_color: get_color_from_hex('#FF0A01')
            # background_color: 0,1,0,1
            size_hint: 0.2, 0.15
            pos_hint: {"center_x": .13, "center_y": 0.18}
            on_press: app.emergency_stop()

        Button:
            id: run_btn
            bold: True
            font_size: '20sp'
            text: "REFRESH"
            background_normal: ''
            background_color: rgba("#00ab66")
            color:1,1,1,1
            halign: "center"
            size_hint: 0.2, 0.15
            pos_hint: {"center_x": .35, "center_y": 0.18}
            on_press: root.change_image("new_robotImage.png")

        Label:
            pos_hint: {"center_x": 0.75, "center_y": 0.84}
            bold: True
            font_size: '20sp'
            text: 'LEFT ARM MOTOR CONTROL PANEL'
            size_hint_x: None
            width: 1
            color: (0,0,0,1)

        MDCard:
            size_hint: .5, .45
            focus_behavior: True
            pos_hint: {"center_x": .74, "center_y": .575}
            md_bg_color: "lightgrey"
            unfocus_color: "lightgrey"
            focus_color: "grey"
            # elevation: 6

        Label:
            pos_hint: {"center_x": 0.75, "center_y": 0.3}
            bold: True
            font_size: '20sp'
            text: 'MOTOR SENSOR INFO'
            size_hint_x: None
            width: 1
            color: (0,0,0,1)

        FloatLayout:
            Label:
                pos_hint: {"center_x": 0.96, "center_y": 0.78}
                bold: True
                font_size: '10sp'
                text: 'SENSOR'
                width: 1
                color: (0,0,0,1)
            Label:
                pos_hint: {"center_x": 0.525, "center_y": 0.78}
                bold: True
                font_size: '10sp'
                text: 'ID'
                width: 0.5
                color: (0,0,0,1)
            Label:
                pos_hint: {"center_x": 0.575, "center_y": 0.78}
                bold: True
                font_size: '10sp'
                text: 'CUR POS' 
                size_hint: None, 1
                width: 10
                color: (0,0,0,1) 
            Label:
                pos_hint: {"center_x": 0.64, "center_y": 0.78}
                bold: True
                font_size: '10sp'
                text: 'POS CTRL' 
                width: 10
                color: (0,0,0,1) 
            Label:
                pos_hint: {"center_x": 0.675, "center_y": 0.78}
                bold: True
                font_size: '10sp'
                text: ' '  
                size_hint_x: None
                width: 1
                color: (0,0,0,1) 
            Label:
                pos_hint: {"center_x": 0.705, "center_y": 0.78}
                bold: True
                font_size: '10sp'
                text: ' '
                size_hint_x: None
                width: 1
                color: (0,0,0,1)
            Label:
                pos_hint: {"center_x": 0.75, "center_y": 0.78}
                bold: True
                font_size: '10sp'
                text: 'CUR VEL'
                size_hint_x: None
                width: 1
                color: (0,0,0,1) 
            Label:
                pos_hint: {"center_x": 0.815, "center_y": 0.78}
                bold: True
                font_size: '10sp'
                text: 'VEL CTRL'
                size_hint_x: None
                width: 1
                color: (0,0,0,1) 
            Label:
                pos_hint: {"center_x": 0.85, "center_y": 0.78}
                bold: True
                font_size: '10sp'
                text: ' ' 
                size_hint_x: None
                width: 1
                color: (0,0,0,1)
            Label:
                pos_hint: {"center_x": 0.88, "center_y": 0.78}
                bold: True
                font_size: '10sp'
                text: ' ' 
                size_hint_x: None
                width: 1
                color: (0,0,0,1)
            Label:
                pos_hint: {"center_x": 0.9125, "center_y": 0.78}
                bold: True
                font_size: '10sp'
                text: 'SEND'
                size_hint_x: None
                width: 1
                color: (0,0,0,1)

            MDCheckbox:
                id: m1_checkbox
                pos_hint: {"center_x": 0.96, "center_y": 0.74}
                size_hint: None, None
                size: "24dp", "24dp"
                on_active: root.show_sensor_info(app, "2", "1")

            Label:
                pos_hint: {"center_x": 0.525, "center_y": 0.74}
                font_size: '10sp'
                text: 'sh_p1'
                id: m1_id
                color: (0,0,1,1)
                # canvas.before:
                #     Color:
                #         rgba: 1, 1, 1, 1
                #     Rectangle:
                #         pos: self.pos
                #         size: self.size
                width: 0.6
            
            TextInput:
                pos_hint: {"center_x": 0.575, "center_y": 0.74}
                id: m1_reading
                size_hint: 0.04, 0.05
                width: 1
                input_filter: 'int'
                text: "0"
                multiline: False
                readonly: True
                
            TextInput:
                pos_hint: {"center_x": 0.635, "center_y": 0.74}
                id: m1_textinput
                size_hint: 0.04, 0.05
                width: .05
                input_filter: 'int'
                text: "0"
                multiline: False
                on_text_validate: root.m1_command_callback(0)

            MDFloatingActionButton:
                pos_hint: {"center_x": 0.675, "center_y": 0.74}
                id: minusbutton
                icon: "minus"
                size_hint: 0.025, 0.03
                width: 0.2 
                on_release: root.m1_command_callback(-5)

            MDFloatingActionButton:
                pos_hint: {"center_x": 0.705, "center_y": 0.74}
                id: plusbutton
                icon: "plus"
                size_hint: 0.025, 0.03
                width: 0.2
                on_release: root.m1_command_callback(5)

            TextInput:
                pos_hint: {"center_x": 0.75, "center_y": 0.74}
                id: vel1_reading
                size_hint: 0.04, 0.05
                width: 1
                input_filter: 'int'
                text: "0"
                multiline: False
                readonly: True
            
            TextInput:
                pos_hint: {"center_x": 0.81, "center_y": 0.74}
                id: velo1_textinput
                size_hint: 0.04, 0.05
                width: 0.3
                input_filter: 'int'
                multiline: False
                text: "0"

            MDFloatingActionButton:
                pos_hint: {"center_x": 0.85, "center_y": 0.74}
                id: minusbutton
                icon: "minus"
                size_hint: 0.025, 0.03
                width: 0.2
                on_release: root.vel1_command_callback(-10)

            MDFloatingActionButton:
                pos_hint: {"center_x": 0.88, "center_y": 0.74}
                id: plusbutton
                icon: "plus"
                size_hint: 0.025, 0.03
                width: 0.2
                on_release: root.vel1_command_callback(10)

            Button:
                id: pub1
                pos_hint: {"center_x": 0.915, "center_y": 0.74}
                size_hint: 0.04, 0.05
                text: 'Pub'
                width: 0.3
                on_press: app.send_position("2", root.m1_pos.text, root.m1_vel.text, "pub1")

            MDCheckbox:
                id: m2_checkbox
                pos_hint: {"center_x": 0.96, "center_y": 0.70}
                size_hint: None, None
                size: "24dp", "24dp"
                on_active: root.show_sensor_info(app, '4', "2")

            Label:
                pos_hint: {"center_x": 0.525, "center_y": 0.70}
                font_size: '10sp'
                text: 'sh_r'
                id: m2_id
                color: (0,0,1,1)
                size_hint_x: None
                width: 0.6
                # canvas.before:
                #     Color:
                #         rgba: 1, 1, 1, 1
                #     Rectangle:
                #         pos: self.pos
                #         size: self.size

            TextInput:
                pos_hint: {"center_x": 0.575, "center_y": 0.70}
                id: m2_reading
                size_hint: 0.04, 0.05
                width: 0.3
                input_filter: 'int'
                text: "0"
                multiline: False
                readonly: True

            TextInput:
                pos_hint: {"center_x": 0.635, "center_y": 0.70}
                id: m2_textinput
                size_hint: 0.04, 0.05
                width: 0.3
                input_filter: 'int'
                text: "0"
                multiline: False
                on_text_validate: root.m2_command_callback(0)

            MDFloatingActionButton:
                pos_hint: {"center_x": 0.675, "center_y": 0.70}
                id: minusbutton
                icon: "minus"
                size_hint: 0.025, 0.03
                width: 0.2
                on_release: root.m2_command_callback(-5)

            MDFloatingActionButton:
                pos_hint: {"center_x": 0.705, "center_y": 0.70}
                id: plusbutton
                icon: "plus"
                size_hint: 0.025, 0.03
                width: 0.2
                on_release: root.m2_command_callback(5)

            TextInput:
                pos_hint: {"center_x": 0.75, "center_y": 0.70}
                id: vel2_reading
                size_hint: 0.04, 0.05
                width: 0.3
                input_filter: 'int'
                text: "0"
                multiline: False
                readonly: True

            TextInput:
                pos_hint: {"center_x": 0.81, "center_y": 0.70}
                id: velo2_textinput
                size_hint: 0.04, 0.05
                width: 0.3
                input_filter: 'int'
                multiline: False
                text: "0"    

            MDFloatingActionButton:
                pos_hint: {"center_x": 0.85, "center_y": 0.70}
                id: minusbutton
                icon: "minus"
                size_hint: 0.025, 0.03
                width: 0.2
                on_release: root.vel2_command_callback(-10)

            MDFloatingActionButton:
                pos_hint: {"center_x": 0.88, "center_y": 0.70}
                id: plusbutton
                icon: "plus"
                size_hint: 0.025, 0.03
                width: 0.2
                on_release: root.vel2_command_callback(10)

            Button:
                pos_hint: {"center_x": 0.915, "center_y": 0.70}
                size_hint: 0.04, 0.05
                id: pub2
                text: 'Pub'
                width: 0.3
                on_press: app.send_position("4", root.m2_pos.text, root.m2_vel.text, "pub2")

            #MOTOR 3
            MDCheckbox:
                id: m3_checkbox
                pos_hint: {"center_x": 0.96, "center_y": 0.66}
                size_hint: None, None
                size: "24dp", "24dp"
                on_active: root.show_sensor_info(app, '6', "3")

            Label:
                pos_hint: {"center_x": 0.525, "center_y": 0.66}
                font_size: '10sp'
                text: 'sh_p2'
                id: m3_id
                color: (0,0,1,1)
                size_hint_x: None
                width: 0.6
                # canvas.before:
                #     Color:
                #         rgba: 1, 1, 1, 1
                #     Rectangle:
                #         pos: self.pos
                #         size: self.size

            TextInput:
                pos_hint: {"center_x": 0.575, "center_y": 0.66}
                id: m3_reading
                size_hint: 0.04, 0.05
                width: 0.3
                input_filter: 'int'
                text: "0"
                multiline: False
                readonly: True

            TextInput:
                pos_hint: {"center_x": 0.635, "center_y": 0.66}
                id: m3_textinput
                size_hint: 0.04, 0.05
                width: 0.3
                input_filter: 'int'
                text: "0"
                multiline: False
                on_text_validate: root.m3_command_callback(0)

            MDFloatingActionButton:
                pos_hint: {"center_x": 0.675, "center_y": 0.66}
                id: minusbutton
                icon: "minus"
                size_hint: 0.025, 0.03
                width: 0.2
                on_release: root.m3_command_callback(-5)

            MDFloatingActionButton:
                pos_hint: {"center_x": 0.705, "center_y": 0.66}
                id: plusbutton
                icon: "plus"
                size_hint: 0.025, 0.03
                width: 0.2
                on_release: root.m3_command_callback(5)

            TextInput:
                pos_hint: {"center_x": 0.75, "center_y": 0.66}
                id: vel3_reading
                size_hint: 0.04, 0.05
                width: 0.3
                input_filter: 'int'
                text: "0"
                multiline: False
                readonly: True

            TextInput:
                pos_hint: {"center_x": 0.81, "center_y": 0.66}
                id: velo3_textinput
                size_hint: 0.04, 0.05
                width: 0.3
                input_filter: 'int'
                multiline: False
                text: "0"    

            MDFloatingActionButton:
                pos_hint: {"center_x": 0.85, "center_y": 0.66}
                id: minusbutton
                icon: "minus"
                size_hint: 0.025, 0.03
                width: 0.2
                on_release: root.vel3_command_callback(-10)

            MDFloatingActionButton:
                pos_hint: {"center_x": 0.88, "center_y": 0.66}
                id: plusbutton
                icon: "plus"
                size_hint: 0.025, 0.03
                width: 0.2
                on_release: root.vel3_command_callback(10)

            Button:
                pos_hint: {"center_x": 0.915, "center_y": 0.66}
                size_hint: 0.04, 0.05
                id: pub3
                text: 'Pub'
                width: 0.3
                on_press: app.send_position("6", root.m3_pos.text, root.m3_vel.text, "pub3")

            #MOTOR 4
            MDCheckbox:
                id: m4_checkbox
                pos_hint: {"center_x": 0.96, "center_y": 0.62}
                size_hint: None, None
                size: "24dp", "24dp"
                on_active: root.show_sensor_info(app, '8', "4")

            Label:
                pos_hint: {"center_x": 0.525, "center_y": 0.62}
                font_size: '10sp'
                text: 'el_y'
                id: m4_id
                color: (0,0,1,1)
                size_hint_x: None
                width: 0.6
                # canvas.before:
                #     Color:
                #         rgba: 1, 1, 1, 1
                #     Rectangle:
                #         pos: self.pos
                #         size: self.size

            TextInput:
                pos_hint: {"center_x": 0.575, "center_y": 0.62}
                id: m4_reading
                size_hint: 0.04, 0.05
                width: 0.3
                input_filter: 'int'
                text: "0"
                multiline: False
                readonly: True

            TextInput:
                pos_hint: {"center_x": 0.635, "center_y": 0.62}
                id: m4_textinput
                size_hint: 0.04, 0.05
                width: 0.3
                input_filter: 'int'
                text: "0"
                multiline: False
                on_text_validate: root.m4_command_callback(0)

            MDFloatingActionButton:
                pos_hint: {"center_x": 0.675, "center_y": 0.62}
                id: minusbutton
                icon: "minus"
                size_hint: 0.025, 0.03
                width: 0.2
                on_release: root.m4_command_callback(-5)

            MDFloatingActionButton:
                pos_hint: {"center_x": 0.705, "center_y": 0.62}
                id: plusbutton
                icon: "plus"
                size_hint: 0.025, 0.03
                width: 0.2
                on_release: root.m4_command_callback(5)

            TextInput:
                pos_hint: {"center_x": 0.75, "center_y": 0.62}
                id: vel4_reading
                size_hint: 0.04, 0.05
                width: 0.3
                input_filter: 'int'
                text: "0"
                multiline: False
                readonly: True

            TextInput:
                pos_hint: {"center_x": 0.81, "center_y": 0.62}
                id: velo4_textinput
                size_hint: 0.04, 0.05
                width: 0.3
                input_filter: 'int'
                multiline: False
                text: "0"    

            MDFloatingActionButton:
                pos_hint: {"center_x": 0.85, "center_y": 0.62}
                id: minusbutton
                icon: "minus"
                size_hint: 0.025, 0.03
                width: 0.2
                on_release: root.vel4_command_callback(-10)

            MDFloatingActionButton:
                pos_hint: {"center_x": 0.88, "center_y": 0.62}
                id: plusbutton
                icon: "plus"
                size_hint: 0.025, 0.03
                width: 0.2
                on_release: root.vel4_command_callback(10)

            Button:
                pos_hint: {"center_x": 0.915, "center_y": 0.62}
                size_hint: 0.04, 0.05
                id: pub4
                text: 'Pub'
                width: 0.3
                on_press: app.send_position("8", root.m4_pos.text, root.m4_vel.text, "pub4")

            #MOTOR 5
            MDCheckbox:
                id: m5_checkbox
                pos_hint: {"center_x": 0.96, "center_y": 0.58}
                size_hint: None, None
                size: "24dp", "24dp"
                on_active: root.show_sensor_info(app, '10', "5")

            Label:
                pos_hint: {"center_x": 0.525, "center_y": 0.58}
                font_size: '10sp'
                text: 'wr_r'
                id: m5_id
                color: (0,0,1,1)
                size_hint_x: None
                width: 0.6
                # canvas.before:
                #     Color:
                #         rgba: 1, 1, 1, 1
                #     Rectangle:
                #         pos: self.pos
                #         size: self.size

            TextInput:
                pos_hint: {"center_x": 0.575, "center_y": 0.58}
                id: m5_reading
                size_hint: 0.04, 0.05
                width: 0.3
                input_filter: 'int'
                text: "0"
                multiline: False
                readonly: True

            TextInput:
                pos_hint: {"center_x": 0.635, "center_y": 0.58}
                id: m5_textinput
                size_hint: 0.04, 0.05
                width: 0.3
                input_filter: 'int'
                text: "0"
                multiline: False
                on_text_validate: root.m5_command_callback(0)

            MDFloatingActionButton:
                pos_hint: {"center_x": 0.675, "center_y": 0.58}
                id: minusbutton
                icon: "minus"
                size_hint: 0.025, 0.03
                width: 0.2
                on_release: root.m5_command_callback(-5)

            MDFloatingActionButton:
                pos_hint: {"center_x": 0.705, "center_y": 0.58}
                id: plusbutton
                icon: "plus"
                size_hint: 0.025, 0.03
                width: 0.2
                on_release: root.m5_command_callback(5)

            TextInput:
                pos_hint: {"center_x": 0.75, "center_y": 0.58}
                id: vel5_reading
                size_hint: 0.04, 0.05
                width: 0.3
                input_filter: 'int'
                text: "0"
                multiline: False
                readonly: True

            TextInput:
                pos_hint: {"center_x": 0.81, "center_y": 0.58}
                id: velo5_textinput
                size_hint: 0.04, 0.05
                width: 0.3
                input_filter: 'int'
                multiline: False
                text: "0"    

            MDFloatingActionButton:
                pos_hint: {"center_x": 0.85, "center_y": 0.58}
                id: minusbutton
                icon: "minus"
                size_hint: 0.025, 0.03
                width: 0.2
                on_release: root.vel5_command_callback(-10)

            MDFloatingActionButton:
                pos_hint: {"center_x": 0.88, "center_y": 0.58}
                id: plusbutton
                icon: "plus"
                size_hint: 0.025, 0.03
                width: 0.2
                on_release: root.vel5_command_callback(10)

            Button:
                pos_hint: {"center_x": 0.915, "center_y": 0.58}
                size_hint: 0.04, 0.05
                id: pub5
                text: 'Pub'
                width: 0.3
                on_press: app.send_position("10", root.m5_pos.text, root.m5_vel.text, "pub5")

            #MOTOR 6
            MDCheckbox:
                id: m6_checkbox
                pos_hint: {"center_x": 0.96, "center_y": 0.54}
                size_hint: None, None
                size: "24dp", "24dp"
                on_active: root.show_sensor_info(app, '12', "6")

            Label:
                pos_hint: {"center_x": 0.525, "center_y": 0.54}
                font_size: '10sp'
                text: 'wr_y'
                id: m6_id
                color: (0,0,1,1)
                size_hint_x: None
                width: 0.6
                # canvas.before:
                #     Color:
                #         rgba: 1, 1, 1, 1
                #     Rectangle:
                #         pos: self.pos
                #         size: self.size

            TextInput:
                pos_hint: {"center_x": 0.575, "center_y": 0.54}
                id: m6_reading
                size_hint: 0.04, 0.05
                width: 0.3
                input_filter: 'int'
                text: "0"
                multiline: False
                readonly: True

            TextInput:
                pos_hint: {"center_x": 0.635, "center_y": 0.54}
                id: m6_textinput
                size_hint: 0.04, 0.05
                width: 0.3
                input_filter: 'int'
                text: "0"
                multiline: False
                on_text_validate: root.m6_command_callback(0)

            MDFloatingActionButton:
                pos_hint: {"center_x": 0.675, "center_y": 0.54}
                id: minusbutton
                icon: "minus"
                size_hint: 0.025, 0.03
                width: 0.2
                on_release: root.m6_command_callback(-5)

            MDFloatingActionButton:
                pos_hint: {"center_x": 0.705, "center_y": 0.54}
                id: plusbutton
                icon: "plus"
                size_hint: 0.025, 0.03
                width: 0.2
                on_release: root.m6_command_callback(5)

            TextInput:
                pos_hint: {"center_x": 0.75, "center_y": 0.54}
                id: vel6_reading
                size_hint: 0.04, 0.05
                width: 0.3
                input_filter: 'int'
                text: "0"
                multiline: False
                readonly: True

            TextInput:
                pos_hint: {"center_x": 0.81, "center_y": 0.54}
                id: velo6_textinput
                size_hint: 0.04, 0.05
                width: 0.3
                input_filter: 'int'
                multiline: False
                text: "0"    

            MDFloatingActionButton:
                pos_hint: {"center_x": 0.85, "center_y": 0.54}
                id: minusbutton
                icon: "minus"
                size_hint: 0.025, 0.03
                width: 0.2
                on_release: root.vel6_command_callback(-10)

            MDFloatingActionButton:
                pos_hint: {"center_x": 0.88, "center_y": 0.54}
                id: plusbutton
                icon: "plus"
                size_hint: 0.025, 0.03
                width: 0.2
                on_release: root.vel6_command_callback(10)

            Button:
                pos_hint: {"center_x": 0.915, "center_y": 0.54}
                size_hint: 0.04, 0.05
                id: pub6
                text: 'Pub'
                width: 0.3
                on_press: app.send_position("12", root.m6_pos.text, root.m6_vel.text, "pub6")

            #MOTOR 7
            MDCheckbox:
                id: m7_checkbox
                pos_hint: {"center_x": 0.96, "center_y": 0.5}
                size_hint: None, None
                size: "24dp", "24dp"
                on_active: root.show_sensor_info(app, '14', "7")

            Label:
                pos_hint: {"center_x": 0.525, "center_y": 0.5}
                font_size: '10sp'
                text: 'wr_p'
                id: m7_id
                color: (0,0,1,1)
                size_hint_x: None
                width: 0.6
                # canvas.before:
                #     Color:
                #         rgba: 1, 1, 1, 1
                #     Rectangle:
                #         pos: self.pos
                #         size: self.size

            TextInput:
                pos_hint: {"center_x": 0.575, "center_y": 0.5}
                id: m7_reading
                size_hint: 0.04, 0.05
                width: 0.3
                input_filter: 'int'
                text: "0"
                multiline: False
                readonly: True

            TextInput:
                pos_hint: {"center_x": 0.635, "center_y": 0.5}
                id: m7_textinput
                size_hint: 0.04, 0.05
                width: 0.3
                input_filter: 'int'
                text: "0"
                multiline: False
                on_text_validate: root.m7_command_callback(0)

            MDFloatingActionButton:
                pos_hint: {"center_x": 0.675, "center_y": 0.5}
                id: minusbutton
                icon: "minus"
                size_hint: 0.025, 0.03
                width: 0.2
                on_release: root.m7_command_callback(-5)

            MDFloatingActionButton:
                pos_hint: {"center_x": 0.705, "center_y": 0.5}
                id: plusbutton
                icon: "plus"
                size_hint: 0.025, 0.03
                width: 0.2
                on_release: root.m7_command_callback(5)

            TextInput:
                pos_hint: {"center_x": 0.75, "center_y": 0.5}
                id: vel7_reading
                size_hint: 0.04, 0.05
                width: 0.3
                input_filter: 'int'
                text: "0"
                multiline: False
                readonly: True

            TextInput:
                pos_hint: {"center_x": 0.81, "center_y": 0.5}
                id: velo7_textinput
                size_hint: 0.04, 0.05
                width: 0.3
                input_filter: 'int'
                multiline: False
                text: "0"    

            MDFloatingActionButton:
                pos_hint: {"center_x": 0.85, "center_y": 0.5}
                id: minusbutton
                icon: "minus"
                size_hint: 0.025, 0.03
                width: 0.2
                on_release: root.vel7_command_callback(-10)

            MDFloatingActionButton:
                pos_hint: {"center_x": 0.88, "center_y": 0.5}
                id: plusbutton
                icon: "plus"
                size_hint: 0.025, 0.03
                width: 0.2
                on_release: root.vel7_command_callback(10)

            Button:
                pos_hint: {"center_x": 0.915, "center_y": 0.5}
                size_hint: 0.04, 0.05
                id: pub7
                text: 'Pub'
                width: 0.3
                on_press: app.send_position("14", root.m7_pos.text, root.m7_vel.text, "pub7")
            
            #MOTOR 8 (hand motors)
            MDCheckbox:
                id: m8_checkbox
                pos_hint: {"center_x": 0.96, "center_y": 0.46}
                size_hint: None, None
                size: "24dp", "24dp"
                on_active: root.show_sensor_info(app, '32', "8")

            Label:
                pos_hint: {"center_x": 0.525, "center_y": 0.46}
                font_size: '10sp'
                text: 'grip_thumb'
                id: m8_id
                color: (0,0,1,1)
                size_hint_x: None
                width: 0.6
                # canvas.before:
                #     Color:
                #         rgba: 1, 1, 1, 1
                #     Rectangle:
                #         pos: self.pos
                #         size: self.size

            TextInput:
                pos_hint: {"center_x": 0.575, "center_y": 0.46}
                id: m8_reading
                size_hint: 0.04, 0.05
                width: 0.3
                input_filter: 'int'
                text: "0"
                multiline: False
                readonly: True

            TextInput:
                pos_hint: {"center_x": 0.635, "center_y": 0.46}
                id: m8_textinput
                size_hint: 0.04, 0.05
                width: 0.3
                input_filter: 'int'
                text: "0"
                multiline: False
                on_text_validate: root.m8_command_callback(0)

            MDFloatingActionButton:
                pos_hint: {"center_x": 0.675, "center_y": 0.46}
                id: minusbutton
                icon: "minus"
                size_hint: 0.025, 0.03
                width: 0.2
                on_release: root.m8_command_callback(-5)

            MDFloatingActionButton:
                pos_hint: {"center_x": 0.705, "center_y": 0.46}
                id: plusbutton
                icon: "plus"
                size_hint: 0.025, 0.03
                width: 0.2
                on_release: root.m8_command_callback(5)

            TextInput:
                pos_hint: {"center_x": 0.75, "center_y": 0.46}
                id: vel8_reading
                size_hint: 0.04, 0.05
                width: 0.3
                input_filter: 'int'
                text: "0"
                multiline: False
                readonly: True

            TextInput:
                pos_hint: {"center_x": 0.81, "center_y": 0.46}
                id: velo8_textinput
                size_hint: 0.04, 0.05
                width: 0.3
                input_filter: 'int'
                multiline: False
                text: "0"    

            MDFloatingActionButton:
                pos_hint: {"center_x": 0.85, "center_y": 0.46}
                id: minusbutton
                icon: "minus"
                size_hint: 0.025, 0.03
                width: 0.2
                on_release: root.vel8_command_callback(-10)

            MDFloatingActionButton:
                pos_hint: {"center_x": 0.88, "center_y": 0.46}
                id: plusbutton
                icon: "plus"
                size_hint: 0.025, 0.03
                width: 0.2
                on_release: root.vel8_command_callback(10)

            Button:
                pos_hint: {"center_x": 0.915, "center_y": 0.46}
                size_hint: 0.04, 0.05
                id: pub8
                text: 'Pub'
                width: 0.3
                on_press: app.send_position("32", root.m8_pos.text, root.m8_vel.text, "pub8")

            #MOTOR 9 (hand)
            MDCheckbox:
                id: m9_checkbox
                pos_hint: {"center_x": 0.96, "center_y": 0.42}
                size_hint: None, None
                size: "24dp", "24dp"
                on_active: root.show_sensor_info(app, '34', "9")

            Label:
                pos_hint: {"center_x": 0.525, "center_y": 0.42}
                font_size: '10sp'
                text: 'grip_index'
                id: m9_id
                color: (0,0,1,1)
                size_hint_x: None
                width: 0.6
                # canvas.before:
                #     Color:
                #         rgba: 1, 1, 1, 1
                #     Rectangle:
                #         pos: self.pos
                #         size: self.size

            TextInput:
                pos_hint: {"center_x": 0.575, "center_y": 0.42}
                id: m9_reading
                size_hint: 0.04, 0.05
                width: 0.3
                input_filter: 'int'
                text: "0"
                multiline: False
                readonly: True

            TextInput:
                pos_hint: {"center_x": 0.635, "center_y": 0.42}
                id: m9_textinput
                size_hint: 0.04, 0.05
                width: 0.3
                input_filter: 'int'
                text: "0"
                multiline: False
                on_text_validate: root.m9_command_callback(0)

            MDFloatingActionButton:
                pos_hint: {"center_x": 0.675, "center_y": 0.42}
                id: minusbutton
                icon: "minus"
                size_hint: 0.025, 0.03
                width: 0.2
                on_release: root.m9_command_callback(-5)

            MDFloatingActionButton:
                pos_hint: {"center_x": 0.705, "center_y": 0.42}
                id: plusbutton
                icon: "plus"
                size_hint: 0.025, 0.03
                width: 0.2
                on_release: root.m9_command_callback(5)

            TextInput:
                pos_hint: {"center_x": 0.75, "center_y": 0.42}
                id: vel9_reading
                size_hint: 0.04, 0.05
                width: 0.3
                input_filter: 'int'
                text: "0"
                multiline: False
                readonly: True

            TextInput:
                pos_hint: {"center_x": 0.81, "center_y": 0.42}
                id: velo9_textinput
                size_hint: 0.04, 0.05
                width: 0.3
                input_filter: 'int'
                multiline: False
                text: "0"    

            MDFloatingActionButton:
                pos_hint: {"center_x": 0.85, "center_y": 0.42}
                id: minusbutton
                icon: "minus"
                size_hint: 0.025, 0.03
                width: 0.2
                on_release: root.vel9_command_callback(-10)

            MDFloatingActionButton:
                pos_hint: {"center_x": 0.88, "center_y": 0.42}
                id: plusbutton
                icon: "plus"
                size_hint: 0.025, 0.03
                width: 0.2
                on_release: root.vel9_command_callback(10)

            Button:
                pos_hint: {"center_x": 0.915, "center_y": 0.42}
                size_hint: 0.04, 0.05
                id: pub9
                text: 'Pub'
                width: 0.3
                on_press: app.send_position("34", root.m9_pos.text, root.m9_vel.text, "pub9")

            #MOTOR 10 (hand)
            MDCheckbox:
                id: m10_checkbox
                pos_hint: {"center_x": 0.96, "center_y": 0.38}
                size_hint: None, None
                size: "24dp", "24dp"
                on_active: root.show_sensor_info(app, '36', "10")

            Label:
                pos_hint: {"center_x": 0.525, "center_y": 0.38}
                font_size: '10sp'
                text: 'grip_middle'
                id: m10_id
                color: (0,0,1,1)
                size_hint_x: None
                width: 0.6
                # canvas.before:
                #     Color:
                #         rgba: 1, 1, 1, 1
                #     Rectangle:
                #         pos: self.pos
                #         size: self.size

            TextInput:
                pos_hint: {"center_x": 0.575, "center_y": 0.38}
                id: m10_reading
                size_hint: 0.04, 0.05
                width: 0.3
                input_filter: 'int'
                text: "0"
                multiline: False
                readonly: True

            TextInput:
                pos_hint: {"center_x": 0.635, "center_y": 0.38}
                id: m10_textinput
                size_hint: 0.04, 0.05
                width: 0.3
                input_filter: 'int'
                text: "0"
                multiline: False
                on_text_validate: root.m10_command_callback(0)

            MDFloatingActionButton:
                pos_hint: {"center_x": 0.675, "center_y": 0.38}
                id: minusbutton
                icon: "minus"
                size_hint: 0.025, 0.03
                width: 0.2
                on_release: root.m10_command_callback(-5)

            MDFloatingActionButton:
                pos_hint: {"center_x": 0.705, "center_y": 0.38}
                id: plusbutton
                icon: "plus"
                size_hint: 0.025, 0.03
                width: 0.2
                on_release: root.m10_command_callback(5)

            TextInput:
                pos_hint: {"center_x": 0.75, "center_y": 0.38}
                id: vel10_reading
                size_hint: 0.04, 0.05
                width: 0.3
                input_filter: 'int'
                text: "0"
                multiline: False
                readonly: True

            TextInput:
                pos_hint: {"center_x": 0.81, "center_y": 0.38}
                id: velo10_textinput
                size_hint: 0.04, 0.05
                width: 0.3
                input_filter: 'int'
                multiline: False
                text: "0"    

            MDFloatingActionButton:
                pos_hint: {"center_x": 0.85, "center_y": 0.38}
                id: minusbutton
                icon: "minus"
                size_hint: 0.025, 0.03
                width: 0.2
                on_release: root.vel10_command_callback(-10)

            MDFloatingActionButton:
                pos_hint: {"center_x": 0.88, "center_y": 0.38}
                id: plusbutton
                icon: "plus"
                size_hint: 0.025, 0.03
                width: 0.2
                on_release: root.vel10_command_callback(10)

            Button:
                pos_hint: {"center_x": 0.915, "center_y": 0.38}
                size_hint: 0.04, 0.05
                id: pub10
                text: 'Pub'
                width: 0.3
                on_press:app.send_position("36", root.m10_pos.text, root.m10_vel.text, "pub10")
        
        # # # SENSORS
        GridLayout:
            pos_hint: {"center_x": 0.75, "center_y": 0.2}
            canvas.before:
                Color:
                    rgba: 0, 0, 0, 1
                Rectangle:
                    pos: self.pos
                    size: self.size
            cols: 3
            size_hint: None, None
            height: 100
            width: 275

            Label:
                text: "Voltage"
            Label:
                text: "0.0"
                id: voltage_id
                color: (0,0,1,1)
                canvas.before:
                    Color:
                        rgba: 1, 1, 1, 1
                    Rectangle:
                        pos: self.pos
                        size: self.size
            Label:
                text: "V"
            Label:
                text: "Current"
            Label:
                text: "0.0"
                id: current_id
                color: (0,0,1,1)
                canvas.before:
                    Color:
                        rgba: 1, 1, 1, 1
                    Rectangle:
                        pos: self.pos
                        size: self.size
            Label:
                text: "mA"
            Label:
                text: "Temperature"
            Label:
                text: "0.0"
                id: temperature_id
                color: (0,0,1,1)
                canvas.before:
                    Color:
                        rgba: 1, 1, 1, 1
                    Rectangle:
                        pos: self.pos
                        size: self.size
            Label:
                text: "C"
            
        MDTabs:
            text_color_active: [1, 1, 1, 1]
            size_hint: 1, 0.08
            tab_hint_x: True
            allow_stretch: True
            on_tab_switch: app.on_tab_switch(*args)
            Tab:
                title: '[b]LiDAR[/b]'
                icon: 'motion-sensor'
            Tab:
                title: '[b]RealSense[/b]'
                icon: 'camera-outline'
            Tab:
                title: '[b]IMU[/b]'
                icon: 'chart-waterfall'
            Tab:
                title: '[b]Force Torque[/b]'
                icon: 'cog'

<MenuRightArmScreen>:
    name: 'home_rightarm_control'
    img: ra_asyn_image
    m1_pos: m1_textinput
    m2_pos: m2_textinput
    m3_pos: m3_textinput
    m4_pos: m4_textinput
    m5_pos: m5_textinput
    m6_pos: m6_textinput
    m7_pos: m7_textinput
    m8_pos: m8_textinput
    m9_pos: m9_textinput
    m10_pos: m10_textinput
    
    m1_vel: velo1_textinput
    m2_vel: velo2_textinput
    m3_vel: velo3_textinput
    m4_vel: velo4_textinput
    m5_vel: velo5_textinput
    m6_vel: velo6_textinput
    m7_vel: velo7_textinput
    m8_vel: velo8_textinput
    m9_vel: velo9_textinput
    m10_vel: velo10_textinput

    FloatLayout:
        orientation: "vertical"

        MDTopAppBar:
            title: "[b]Humanoid Control Panel[/b]"
            anchor_title: "center"
            left_action_items: [["home-analytics", lambda x: app.set_screen('home')]]
            right_action_items: [["engine", lambda x: app.callback(x)], ["robot-confused-outline", lambda x: app.set_screen('id')], ["power", lambda x: app.callback_2()]]
            size_hint: 1, .1
            pos_hint: {"center_x": .5, "center_y": 0.95}
        
        AsyncImage:
            id: ra_asyn_image
            source: root.image_path
            size_hint: None, None
            width: 400
            height: 400
            pos_hint: {'center_x':.25, 'center_y': .6}
            nocache: True

        Label:
            pos_hint: {"center_x": 0.25, "center_y": 0.325}
            bold: True
            font_size: '20sp'
            text: 'LIVE URDF MODEL'
            size_hint_x: None
            width: 1
            color: (0,0,0,1)

        Label:
            pos_hint: {"center_x": 0.25, "center_y": 0.3}
            bold: True
            font_size: '12sp'
            text: 'POSITION IN DEGREES | VELOCITY IN ROTATIONS/S'
            size_hint_x: None
            width: 1
            color: (0,0,0,1)

        Button:
            id: emergency
            bold: True
            font_size: '18sp'
            text: "EMERGENCY STOP"
            background_normal: ''
            background_color: rgba("#ff0a01")
            color: 1,1,1,1
            halign: "center"
            # background_color: get_color_from_hex('#FF0A01')
            # background_color: 0,1,0,1
            size_hint: 0.2, 0.15
            pos_hint: {"center_x": .13, "center_y": 0.18}
            on_press: app.emergency_stop()

        Button:
            id: run_btn
            bold: True
            font_size: '20sp'
            text: "REFRESH"
            background_normal: ''
            background_color: rgba("#00ab66")
            color:1,1,1,1
            halign: "center"
            size_hint: 0.2, 0.15
            pos_hint: {"center_x": .35, "center_y": 0.18}
            on_press: root.change_image("new_robotImage.png")

        Label:
            pos_hint: {"center_x": 0.75, "center_y": 0.84}
            bold: True
            font_size: '20sp'
            text: 'RIGHT ARM MOTOR CONTROL PANEL'
            size_hint_x: None
            width: 1
            color: (0,0,0,1)

        MDCard:
            size_hint: .5, .45
            focus_behavior: True
            pos_hint: {"center_x": .74, "center_y": .575}
            md_bg_color: "lightgrey"
            unfocus_color: "lightgrey"
            focus_color: "grey"
            # elevation: 6

        Label:
            pos_hint: {"center_x": 0.75, "center_y": 0.3}
            bold: True
            font_size: '20sp'
            text: 'MOTOR SENSOR INFO'
            size_hint_x: None
            width: 1
            color: (0,0,0,1)

        FloatLayout:
            Label:
                pos_hint: {"center_x": 0.96, "center_y": 0.78}
                bold: True
                font_size: '10sp'
                text: 'SENSOR'
                width: 1
                color: (0,0,0,1)
            Label:
                pos_hint: {"center_x": 0.525, "center_y": 0.78}
                bold: True
                font_size: '10sp'
                text: 'ID'
                width: 0.5
                color: (0,0,0,1)
            Label:
                pos_hint: {"center_x": 0.575, "center_y": 0.78}
                bold: True
                font_size: '10sp'
                text: 'CUR POS' 
                size_hint: None, 1
                width: 10
                color: (0,0,0,1) 
            Label:
                pos_hint: {"center_x": 0.64, "center_y": 0.78}
                bold: True
                font_size: '10sp'
                text: 'POS CTRL' 
                width: 10
                color: (0,0,0,1) 
            Label:
                pos_hint: {"center_x": 0.675, "center_y": 0.78}
                bold: True
                font_size: '10sp'
                text: ' '  
                size_hint_x: None
                width: 1
                color: (0,0,0,1) 
            Label:
                pos_hint: {"center_x": 0.705, "center_y": 0.78}
                bold: True
                font_size: '10sp'
                text: ' '
                size_hint_x: None
                width: 1
                color: (0,0,0,1)
            Label:
                pos_hint: {"center_x": 0.75, "center_y": 0.78}
                bold: True
                font_size: '10sp'
                text: 'CUR VEL'
                size_hint_x: None
                width: 1
                color: (0,0,0,1) 
            Label:
                pos_hint: {"center_x": 0.815, "center_y": 0.78}
                bold: True
                font_size: '10sp'
                text: 'VEL CTRL'
                size_hint_x: None
                width: 1
                color: (0,0,0,1) 
            Label:
                pos_hint: {"center_x": 0.85, "center_y": 0.78}
                bold: True
                font_size: '10sp'
                text: ' ' 
                size_hint_x: None
                width: 1
                color: (0,0,0,1)
            Label:
                pos_hint: {"center_x": 0.88, "center_y": 0.78}
                bold: True
                font_size: '10sp'
                text: ' ' 
                size_hint_x: None
                width: 1
                color: (0,0,0,1)
            Label:
                pos_hint: {"center_x": 0.9125, "center_y": 0.78}
                bold: True
                font_size: '10sp'
                text: 'SEND'
                size_hint_x: None
                width: 1
                color: (0,0,0,1)

            MDCheckbox:
                id: m1_checkbox
                pos_hint: {"center_x": 0.96, "center_y": 0.74}
                size_hint: None, None
                size: "24dp", "24dp"
                on_active: app.show_sensor_info('1')

            Label:
                pos_hint: {"center_x": 0.525, "center_y": 0.74}
                font_size: '10sp'
                text: 'sh_p1'
                id: m1_id
                color: (0,0,1,1)
                # canvas.before:
                #     Color:
                #         rgba: 1, 1, 1, 1
                #     Rectangle:
                #         pos: self.pos
                #         size: self.size
                width: 0.6
            
            TextInput:
                pos_hint: {"center_x": 0.575, "center_y": 0.74}
                id: m1_reading
                size_hint: 0.04, 0.05
                width: 1
                input_filter: 'int'
                text: "0"
                multiline: False
                readonly: True
                
            TextInput:
                pos_hint: {"center_x": 0.635, "center_y": 0.74}
                id: m1_textinput
                size_hint: 0.04, 0.05
                width: .05
                input_filter: 'int'
                text: "0"
                multiline: False
                on_text_validate: root.m1_command_callback(0)

            MDFloatingActionButton:
                pos_hint: {"center_x": 0.675, "center_y": 0.74}
                id: minusbutton
                icon: "minus"
                size_hint: 0.025, 0.03
                width: 0.2 
                on_release: root.m1_command_callback(-5)

            MDFloatingActionButton:
                pos_hint: {"center_x": 0.705, "center_y": 0.74}
                id: plusbutton
                icon: "plus"
                size_hint: 0.025, 0.03
                width: 0.2
                on_release: root.m1_command_callback(5)

            TextInput:
                pos_hint: {"center_x": 0.75, "center_y": 0.74}
                id: vel1_reading
                size_hint: 0.04, 0.05
                width: 1
                input_filter: 'int'
                text: "0"
                multiline: False
                readonly: True
            
            TextInput:
                pos_hint: {"center_x": 0.81, "center_y": 0.74}
                id: velo1_textinput
                size_hint: 0.04, 0.05
                width: 0.3
                input_filter: 'int'
                multiline: False
                text: "0"

            MDFloatingActionButton:
                pos_hint: {"center_x": 0.85, "center_y": 0.74}
                id: minusbutton
                icon: "minus"
                size_hint: 0.025, 0.03
                width: 0.2
                on_release: root.vel1_command_callback(-10)

            MDFloatingActionButton:
                pos_hint: {"center_x": 0.88, "center_y": 0.74}
                id: plusbutton
                icon: "plus"
                size_hint: 0.025, 0.03
                width: 0.2
                on_release: root.vel1_command_callback(10)

            Button:
                pos_hint: {"center_x": 0.915, "center_y": 0.74}
                size_hint: 0.04, 0.05
                id: pub1
                text: 'Pub'
                width: 0.3
                on_press:app.send_position("1", root.m1_pos.text, root.m1_vel.text, "pub1")

            #MOTOR 2
            MDCheckbox:
                id: m2_checkbox
                pos_hint: {"center_x": 0.96, "center_y": 0.70}
                size_hint: None, None
                size: "24dp", "24dp"
                on_active: app.show_sensor_info('3')

            Label:
                pos_hint: {"center_x": 0.525, "center_y": 0.70}
                font_size: '10sp'
                text: 'sh_r'
                id: m2_id
                color: (0,0,1,1)
                size_hint_x: None
                width: 0.6
                # canvas.before:
                #     Color:
                #         rgba: 1, 1, 1, 1
                #     Rectangle:
                #         pos: self.pos
                #         size: self.size

            TextInput:
                pos_hint: {"center_x": 0.575, "center_y": 0.70}
                id: m2_reading
                size_hint: 0.04, 0.05
                width: 0.3
                input_filter: 'int'
                text: "0"
                multiline: False
                readonly: True

            TextInput:
                pos_hint: {"center_x": 0.635, "center_y": 0.70}
                id: m2_textinput
                size_hint: 0.04, 0.05
                width: 0.3
                input_filter: 'int'
                text: "0"
                multiline: False
                on_text_validate: root.m2_command_callback(0)

            MDFloatingActionButton:
                pos_hint: {"center_x": 0.675, "center_y": 0.70}
                id: minusbutton
                icon: "minus"
                size_hint: 0.025, 0.03
                width: 0.2
                on_release: root.m2_command_callback(-5)

            MDFloatingActionButton:
                pos_hint: {"center_x": 0.705, "center_y": 0.70}
                id: plusbutton
                icon: "plus"
                size_hint: 0.025, 0.03
                width: 0.2
                on_release: root.m2_command_callback(5)

            TextInput:
                pos_hint: {"center_x": 0.75, "center_y": 0.70}
                id: vel2_reading
                size_hint: 0.04, 0.05
                width: 0.3
                input_filter: 'int'
                text: "0"
                multiline: False
                readonly: True

            TextInput:
                pos_hint: {"center_x": 0.81, "center_y": 0.70}
                id: velo2_textinput
                size_hint: 0.04, 0.05
                width: 0.3
                input_filter: 'int'
                multiline: False
                text: "0"    

            MDFloatingActionButton:
                pos_hint: {"center_x": 0.85, "center_y": 0.70}
                id: minusbutton
                icon: "minus"
                size_hint: 0.025, 0.03
                width: 0.2
                on_release: root.vel2_command_callback(-10)

            MDFloatingActionButton:
                pos_hint: {"center_x": 0.88, "center_y": 0.70}
                id: plusbutton
                icon: "plus"
                size_hint: 0.025, 0.03
                width: 0.2
                on_release: root.vel2_command_callback(10)

            Button:
                pos_hint: {"center_x": 0.915, "center_y": 0.70}
                size_hint: 0.04, 0.05
                id: pub2
                text: 'Pub'
                width: 0.3
                on_press:app.send_position("3", root.m2_pos.text, root.m2_vel.text, "pub2")

            #MOTOR 3
            MDCheckbox:
                id: m3_checkbox
                pos_hint: {"center_x": 0.96, "center_y": 0.66}
                size_hint: None, None
                size: "24dp", "24dp"
                on_active: app.show_sensor_info('5')

            Label:
                pos_hint: {"center_x": 0.525, "center_y": 0.66}
                font_size: '10sp'
                text: 'sh_p2'
                id: m3_id
                color: (0,0,1,1)
                size_hint_x: None
                width: 0.6
                # canvas.before:
                #     Color:
                #         rgba: 1, 1, 1, 1
                #     Rectangle:
                #         pos: self.pos
                #         size: self.size

            TextInput:
                pos_hint: {"center_x": 0.575, "center_y": 0.66}
                id: m3_reading
                size_hint: 0.04, 0.05
                width: 0.3
                input_filter: 'int'
                text: "0"
                multiline: False
                readonly: True

            TextInput:
                pos_hint: {"center_x": 0.635, "center_y": 0.66}
                id: m3_textinput
                size_hint: 0.04, 0.05
                width: 0.3
                input_filter: 'int'
                text: "0"
                multiline: False
                on_text_validate: root.m3_command_callback(0)

            MDFloatingActionButton:
                pos_hint: {"center_x": 0.675, "center_y": 0.66}
                id: minusbutton
                icon: "minus"
                size_hint: 0.025, 0.03
                width: 0.2
                on_release: root.m3_command_callback(-5)

            MDFloatingActionButton:
                pos_hint: {"center_x": 0.705, "center_y": 0.66}
                id: plusbutton
                icon: "plus"
                size_hint: 0.025, 0.03
                width: 0.2
                on_release: root.m3_command_callback(5)

            TextInput:
                pos_hint: {"center_x": 0.75, "center_y": 0.66}
                id: vel3_reading
                size_hint: 0.04, 0.05
                width: 0.3
                input_filter: 'int'
                text: "0"
                multiline: False
                readonly: True

            TextInput:
                pos_hint: {"center_x": 0.81, "center_y": 0.66}
                id: velo3_textinput
                size_hint: 0.04, 0.05
                width: 0.3
                input_filter: 'int'
                multiline: False
                text: "0"    

            MDFloatingActionButton:
                pos_hint: {"center_x": 0.85, "center_y": 0.66}
                id: minusbutton
                icon: "minus"
                size_hint: 0.025, 0.03
                width: 0.2
                on_release: root.vel3_command_callback(-10)

            MDFloatingActionButton:
                pos_hint: {"center_x": 0.88, "center_y": 0.66}
                id: plusbutton
                icon: "plus"
                size_hint: 0.025, 0.03
                width: 0.2
                on_release: root.vel3_command_callback(10)

            Button:
                pos_hint: {"center_x": 0.915, "center_y": 0.66}
                size_hint: 0.04, 0.05
                id: pub3
                text: 'Pub'
                width: 0.3
                on_press:app.send_position("5", root.m3_pos.text, root.m3_vel.text, "pub3")

            #MOTOR 4
            MDCheckbox:
                id: m4_checkbox
                pos_hint: {"center_x": 0.96, "center_y": 0.62}
                size_hint: None, None
                size: "24dp", "24dp"
                on_active: app.show_sensor_info('7')

            Label:
                pos_hint: {"center_x": 0.525, "center_y": 0.62}
                font_size: '10sp'
                text: 'el_y'
                id: m4_id
                color: (0,0,1,1)
                size_hint_x: None
                width: 0.6
                # canvas.before:
                #     Color:
                #         rgba: 1, 1, 1, 1
                #     Rectangle:
                #         pos: self.pos
                #         size: self.size

            TextInput:
                pos_hint: {"center_x": 0.575, "center_y": 0.62}
                id: m4_reading
                size_hint: 0.04, 0.05
                width: 0.3
                input_filter: 'int'
                text: "0"
                multiline: False
                readonly: True

            TextInput:
                pos_hint: {"center_x": 0.635, "center_y": 0.62}
                id: m4_textinput
                size_hint: 0.04, 0.05
                width: 0.3
                input_filter: 'int'
                text: "0"
                multiline: False
                on_text_validate: root.m4_command_callback(0)

            MDFloatingActionButton:
                pos_hint: {"center_x": 0.675, "center_y": 0.62}
                id: minusbutton
                icon: "minus"
                size_hint: 0.025, 0.03
                width: 0.2
                on_release: root.m4_command_callback(-5)

            MDFloatingActionButton:
                pos_hint: {"center_x": 0.705, "center_y": 0.62}
                id: plusbutton
                icon: "plus"
                size_hint: 0.025, 0.03
                width: 0.2
                on_release: root.m4_command_callback(5)

            TextInput:
                pos_hint: {"center_x": 0.75, "center_y": 0.62}
                id: vel4_reading
                size_hint: 0.04, 0.05
                width: 0.3
                input_filter: 'int'
                text: "0"
                multiline: False
                readonly: True

            TextInput:
                pos_hint: {"center_x": 0.81, "center_y": 0.62}
                id: velo4_textinput
                size_hint: 0.04, 0.05
                width: 0.3
                input_filter: 'int'
                multiline: False
                text: "0"    

            MDFloatingActionButton:
                pos_hint: {"center_x": 0.85, "center_y": 0.62}
                id: minusbutton
                icon: "minus"
                size_hint: 0.025, 0.03
                width: 0.2
                on_release: root.vel4_command_callback(-10)

            MDFloatingActionButton:
                pos_hint: {"center_x": 0.88, "center_y": 0.62}
                id: plusbutton
                icon: "plus"
                size_hint: 0.025, 0.03
                width: 0.2
                on_release: root.vel4_command_callback(10)

            Button:
                pos_hint: {"center_x": 0.915, "center_y": 0.62}
                size_hint: 0.04, 0.05
                id: pub4
                text: 'Pub'
                width: 0.3
                on_press:app.send_position("7", root.m4_pos.text, root.m4_vel.text, "pub4")

            #MOTOR 5
            MDCheckbox:
                id: m5_checkbox
                pos_hint: {"center_x": 0.96, "center_y": 0.58}
                size_hint: None, None
                size: "24dp", "24dp"
                on_active: app.show_sensor_info('9')

            Label:
                pos_hint: {"center_x": 0.525, "center_y": 0.58}
                font_size: '10sp'
                text: 'wr_r'
                id: m5_id
                color: (0,0,1,1)
                size_hint_x: None
                width: 0.6
                # canvas.before:
                #     Color:
                #         rgba: 1, 1, 1, 1
                #     Rectangle:
                #         pos: self.pos
                #         size: self.size

            TextInput:
                pos_hint: {"center_x": 0.575, "center_y": 0.58}
                id: m5_reading
                size_hint: 0.04, 0.05
                width: 0.3
                input_filter: 'int'
                text: "0"
                multiline: False
                readonly: True

            TextInput:
                pos_hint: {"center_x": 0.635, "center_y": 0.58}
                id: m5_textinput
                size_hint: 0.04, 0.05
                width: 0.3
                input_filter: 'int'
                text: "0"
                multiline: False
                on_text_validate: root.m5_command_callback(0)

            MDFloatingActionButton:
                pos_hint: {"center_x": 0.675, "center_y": 0.58}
                id: minusbutton
                icon: "minus"
                size_hint: 0.025, 0.03
                width: 0.2
                on_release: root.m5_command_callback(-5)

            MDFloatingActionButton:
                pos_hint: {"center_x": 0.705, "center_y": 0.58}
                id: plusbutton
                icon: "plus"
                size_hint: 0.025, 0.03
                width: 0.2
                on_release: root.m5_command_callback(5)

            TextInput:
                pos_hint: {"center_x": 0.75, "center_y": 0.58}
                id: vel5_reading
                size_hint: 0.04, 0.05
                width: 0.3
                input_filter: 'int'
                text: "0"
                multiline: False
                readonly: True

            TextInput:
                pos_hint: {"center_x": 0.81, "center_y": 0.58}
                id: velo5_textinput
                size_hint: 0.04, 0.05
                width: 0.3
                input_filter: 'int'
                multiline: False
                text: "0"    

            MDFloatingActionButton:
                pos_hint: {"center_x": 0.85, "center_y": 0.58}
                id: minusbutton
                icon: "minus"
                size_hint: 0.025, 0.03
                width: 0.2
                on_release: root.vel5_command_callback(-10)

            MDFloatingActionButton:
                pos_hint: {"center_x": 0.88, "center_y": 0.58}
                id: plusbutton
                icon: "plus"
                size_hint: 0.025, 0.03
                width: 0.2
                on_release: root.vel5_command_callback(10)

            Button:
                pos_hint: {"center_x": 0.915, "center_y": 0.58}
                size_hint: 0.04, 0.05
                id: pub5
                text: 'Pub'
                width: 0.3
                on_press:app.send_position("9", root.m5_pos.text, root.m5_vel.text, "pub5")

            #MOTOR 6
            MDCheckbox:
                id: m6_checkbox
                pos_hint: {"center_x": 0.96, "center_y": 0.54}
                size_hint: None, None
                size: "24dp", "24dp"
                on_active: app.show_sensor_info('11')

            Label:
                pos_hint: {"center_x": 0.525, "center_y": 0.54}
                font_size: '10sp'
                text: 'wr_y'
                id: m6_id
                color: (0,0,1,1)
                size_hint_x: None
                width: 0.6
                # canvas.before:
                #     Color:
                #         rgba: 1, 1, 1, 1
                #     Rectangle:
                #         pos: self.pos
                #         size: self.size

            TextInput:
                pos_hint: {"center_x": 0.575, "center_y": 0.54}
                id: m6_reading
                size_hint: 0.04, 0.05
                width: 0.3
                input_filter: 'int'
                text: "0"
                multiline: False
                readonly: True

            TextInput:
                pos_hint: {"center_x": 0.635, "center_y": 0.54}
                id: m6_textinput
                size_hint: 0.04, 0.05
                width: 0.3
                input_filter: 'int'
                text: "0"
                multiline: False
                on_text_validate: root.m6_command_callback(0)

            MDFloatingActionButton:
                pos_hint: {"center_x": 0.675, "center_y": 0.54}
                id: minusbutton
                icon: "minus"
                size_hint: 0.025, 0.03
                width: 0.2
                on_release: root.m6_command_callback(-5)

            MDFloatingActionButton:
                pos_hint: {"center_x": 0.705, "center_y": 0.54}
                id: plusbutton
                icon: "plus"
                size_hint: 0.025, 0.03
                width: 0.2
                on_release: root.m6_command_callback(5)

            TextInput:
                pos_hint: {"center_x": 0.75, "center_y": 0.54}
                id: vel6_reading
                size_hint: 0.04, 0.05
                width: 0.3
                input_filter: 'int'
                text: "0"
                multiline: False
                readonly: True

            TextInput:
                pos_hint: {"center_x": 0.81, "center_y": 0.54}
                id: velo6_textinput
                size_hint: 0.04, 0.05
                width: 0.3
                input_filter: 'int'
                multiline: False
                text: "0"    

            MDFloatingActionButton:
                pos_hint: {"center_x": 0.85, "center_y": 0.54}
                id: minusbutton
                icon: "minus"
                size_hint: 0.025, 0.03
                width: 0.2
                on_release: root.vel6_command_callback(-10)

            MDFloatingActionButton:
                pos_hint: {"center_x": 0.88, "center_y": 0.54}
                id: plusbutton
                icon: "plus"
                size_hint: 0.025, 0.03
                width: 0.2
                on_release: root.vel6_command_callback(10)

            Button:
                pos_hint: {"center_x": 0.915, "center_y": 0.54}
                size_hint: 0.04, 0.05
                id: pub6
                text: 'Pub'
                width: 0.3
                on_press:app.send_position("11", root.m6_pos.text, root.m6_vel.text, "pub6")

            #MOTOR 7
            MDCheckbox:
                id: m7_checkbox
                pos_hint: {"center_x": 0.96, "center_y": 0.5}
                size_hint: None, None
                size: "24dp", "24dp"
                on_active: app.show_sensor_info('13')

            Label:
                pos_hint: {"center_x": 0.525, "center_y": 0.5}
                font_size: '10sp'
                text: 'wr_p'
                id: m7_id
                color: (0,0,1,1)
                size_hint_x: None
                width: 0.6
                # canvas.before:
                #     Color:
                #         rgba: 1, 1, 1, 1
                #     Rectangle:
                #         pos: self.pos
                #         size: self.size

            TextInput:
                pos_hint: {"center_x": 0.575, "center_y": 0.5}
                id: m7_reading
                size_hint: 0.04, 0.05
                width: 0.3
                input_filter: 'int'
                text: "0"
                multiline: False
                readonly: True

            TextInput:
                pos_hint: {"center_x": 0.635, "center_y": 0.5}
                id: m7_textinput
                size_hint: 0.04, 0.05
                width: 0.3
                input_filter: 'int'
                text: "0"
                multiline: False
                on_text_validate: root.m7_command_callback(0)

            MDFloatingActionButton:
                pos_hint: {"center_x": 0.675, "center_y": 0.5}
                id: minusbutton
                icon: "minus"
                size_hint: 0.025, 0.03
                width: 0.2
                on_release: root.m7_command_callback(-5)

            MDFloatingActionButton:
                pos_hint: {"center_x": 0.705, "center_y": 0.5}
                id: plusbutton
                icon: "plus"
                size_hint: 0.025, 0.03
                width: 0.2
                on_release: root.m7_command_callback(5)

            TextInput:
                pos_hint: {"center_x": 0.75, "center_y": 0.5}
                id: vel7_reading
                size_hint: 0.04, 0.05
                width: 0.3
                input_filter: 'int'
                text: "0"
                multiline: False
                readonly: True

            TextInput:
                pos_hint: {"center_x": 0.81, "center_y": 0.5}
                id: velo7_textinput
                size_hint: 0.04, 0.05
                width: 0.3
                input_filter: 'int'
                multiline: False
                text: "0"    

            MDFloatingActionButton:
                pos_hint: {"center_x": 0.85, "center_y": 0.5}
                id: minusbutton
                icon: "minus"
                size_hint: 0.025, 0.03
                width: 0.2
                on_release: root.vel7_command_callback(-10)

            MDFloatingActionButton:
                pos_hint: {"center_x": 0.88, "center_y": 0.5}
                id: plusbutton
                icon: "plus"
                size_hint: 0.025, 0.03
                width: 0.2
                on_release: root.vel7_command_callback(10)

            Button:
                pos_hint: {"center_x": 0.915, "center_y": 0.5}
                size_hint: 0.04, 0.05
                id: pub7
                text: 'Pub'
                width: 0.3
                on_press:app.send_position("13", root.m7_pos.text, root.m7_vel.text, "pub7")
            
            #MOTOR 8 (hand motors)
            MDCheckbox:
                id: m8_checkbox
                pos_hint: {"center_x": 0.96, "center_y": 0.46}
                size_hint: None, None
                size: "24dp", "24dp"
                on_active: app.show_sensor_info('31')

            Label:
                pos_hint: {"center_x": 0.525, "center_y": 0.46}
                font_size: '10sp'
                text: 'grip_thumb'
                id: m8_id
                color: (0,0,1,1)
                size_hint_x: None
                width: 0.6
                # canvas.before:
                #     Color:
                #         rgba: 1, 1, 1, 1
                #     Rectangle:
                #         pos: self.pos
                #         size: self.size

            TextInput:
                pos_hint: {"center_x": 0.575, "center_y": 0.46}
                id: m8_reading
                size_hint: 0.04, 0.05
                width: 0.3
                input_filter: 'int'
                text: "0"
                multiline: False
                readonly: True

            TextInput:
                pos_hint: {"center_x": 0.635, "center_y": 0.46}
                id: m8_textinput
                size_hint: 0.04, 0.05
                width: 0.3
                input_filter: 'int'
                text: "0"
                multiline: False
                on_text_validate: root.m8_command_callback(0)

            MDFloatingActionButton:
                pos_hint: {"center_x": 0.675, "center_y": 0.46}
                id: minusbutton
                icon: "minus"
                size_hint: 0.025, 0.03
                width: 0.2
                on_release: root.m8_command_callback(-5)

            MDFloatingActionButton:
                pos_hint: {"center_x": 0.705, "center_y": 0.46}
                id: plusbutton
                icon: "plus"
                size_hint: 0.025, 0.03
                width: 0.2
                on_release: root.m8_command_callback(5)

            TextInput:
                pos_hint: {"center_x": 0.75, "center_y": 0.46}
                id: vel8_reading
                size_hint: 0.04, 0.05
                width: 0.3
                input_filter: 'int'
                text: "0"
                multiline: False
                readonly: True

            TextInput:
                pos_hint: {"center_x": 0.81, "center_y": 0.46}
                id: velo8_textinput
                size_hint: 0.04, 0.05
                width: 0.3
                input_filter: 'int'
                multiline: False
                text: "0"    

            MDFloatingActionButton:
                pos_hint: {"center_x": 0.85, "center_y": 0.46}
                id: minusbutton
                icon: "minus"
                size_hint: 0.025, 0.03
                width: 0.2
                on_release: root.vel8_command_callback(-10)

            MDFloatingActionButton:
                pos_hint: {"center_x": 0.88, "center_y": 0.46}
                id: plusbutton
                icon: "plus"
                size_hint: 0.025, 0.03
                width: 0.2
                on_release: root.vel8_command_callback(10)

            Button:
                pos_hint: {"center_x": 0.915, "center_y": 0.46}
                size_hint: 0.04, 0.05
                id: pub8
                text: 'Pub'
                width: 0.3
                on_press:app.send_position("31", root.m8_pos.text, root.m8_vel.text, "pub8")

            #MOTOR 9 (hand)
            MDCheckbox:
                id: m9_checkbox
                pos_hint: {"center_x": 0.96, "center_y": 0.42}
                size_hint: None, None
                size: "24dp", "24dp"
                on_active: app.show_sensor_info('33')

            Label:
                pos_hint: {"center_x": 0.525, "center_y": 0.42}
                font_size: '10sp'
                text: 'grip_index'
                id: m9_id
                color: (0,0,1,1)
                size_hint_x: None
                width: 0.6
                # canvas.before:
                #     Color:
                #         rgba: 1, 1, 1, 1
                #     Rectangle:
                #         pos: self.pos
                #         size: self.size

            TextInput:
                pos_hint: {"center_x": 0.575, "center_y": 0.42}
                id: m9_reading
                size_hint: 0.04, 0.05
                width: 0.3
                input_filter: 'int'
                text: "0"
                multiline: False
                readonly: True

            TextInput:
                pos_hint: {"center_x": 0.635, "center_y": 0.42}
                id: m9_textinput
                size_hint: 0.04, 0.05
                width: 0.3
                input_filter: 'int'
                text: "0"
                multiline: False
                on_text_validate: root.m9_command_callback(0)

            MDFloatingActionButton:
                pos_hint: {"center_x": 0.675, "center_y": 0.42}
                id: minusbutton
                icon: "minus"
                size_hint: 0.025, 0.03
                width: 0.2
                on_release: root.m9_command_callback(-5)

            MDFloatingActionButton:
                pos_hint: {"center_x": 0.705, "center_y": 0.42}
                id: plusbutton
                icon: "plus"
                size_hint: 0.025, 0.03
                width: 0.2
                on_release: root.m9_command_callback(5)

            TextInput:
                pos_hint: {"center_x": 0.75, "center_y": 0.42}
                id: vel9_reading
                size_hint: 0.04, 0.05
                width: 0.3
                input_filter: 'int'
                text: "0"
                multiline: False
                readonly: True

            TextInput:
                pos_hint: {"center_x": 0.81, "center_y": 0.42}
                id: velo9_textinput
                size_hint: 0.04, 0.05
                width: 0.3
                input_filter: 'int'
                multiline: False
                text: "0"    

            MDFloatingActionButton:
                pos_hint: {"center_x": 0.85, "center_y": 0.42}
                id: minusbutton
                icon: "minus"
                size_hint: 0.025, 0.03
                width: 0.2
                on_release: root.vel9_command_callback(-10)

            MDFloatingActionButton:
                pos_hint: {"center_x": 0.88, "center_y": 0.42}
                id: plusbutton
                icon: "plus"
                size_hint: 0.025, 0.03
                width: 0.2
                on_release: root.vel9_command_callback(10)

            Button:
                pos_hint: {"center_x": 0.915, "center_y": 0.42}
                size_hint: 0.04, 0.05
                id: pub9
                text: 'Pub'
                width: 0.3
                on_press:app.send_position("33", root.m9_pos.text, root.m9_vel.text, "pub9")

            #MOTOR 10 (hand)
            MDCheckbox:
                id: m10_checkbox
                pos_hint: {"center_x": 0.96, "center_y": 0.38}
                size_hint: None, None
                size: "24dp", "24dp"
                on_active: app.show_sensor_info('35')

            Label:
                pos_hint: {"center_x": 0.525, "center_y": 0.38}
                font_size: '10sp'
                text: 'grip_middle'
                id: m10_id
                color: (0,0,1,1)
                size_hint_x: None
                width: 0.6
                # canvas.before:
                #     Color:
                #         rgba: 1, 1, 1, 1
                #     Rectangle:
                #         pos: self.pos
                #         size: self.size

            TextInput:
                pos_hint: {"center_x": 0.575, "center_y": 0.38}
                id: m10_reading
                size_hint: 0.04, 0.05
                width: 0.3
                input_filter: 'int'
                text: "0"
                multiline: False
                readonly: True

            TextInput:
                pos_hint: {"center_x": 0.635, "center_y": 0.38}
                id: m10_textinput
                size_hint: 0.04, 0.05
                width: 0.3
                input_filter: 'int'
                text: "0"
                multiline: False
                on_text_validate: root.m10_command_callback(0)

            MDFloatingActionButton:
                pos_hint: {"center_x": 0.675, "center_y": 0.38}
                id: minusbutton
                icon: "minus"
                size_hint: 0.025, 0.03
                width: 0.2
                on_release: root.m10_command_callback(-5)

            MDFloatingActionButton:
                pos_hint: {"center_x": 0.705, "center_y": 0.38}
                id: plusbutton
                icon: "plus"
                size_hint: 0.025, 0.03
                width: 0.2
                on_release: root.m10_command_callback(5)

            TextInput:
                pos_hint: {"center_x": 0.75, "center_y": 0.38}
                id: vel10_reading
                size_hint: 0.04, 0.05
                width: 0.3
                input_filter: 'int'
                text: "0"
                multiline: False
                readonly: True

            TextInput:
                pos_hint: {"center_x": 0.81, "center_y": 0.38}
                id: velo10_textinput
                size_hint: 0.04, 0.05
                width: 0.3
                input_filter: 'int'
                multiline: False
                text: "0"    

            MDFloatingActionButton:
                pos_hint: {"center_x": 0.85, "center_y": 0.38}
                id: minusbutton
                icon: "minus"
                size_hint: 0.025, 0.03
                width: 0.2
                on_release: root.vel10_command_callback(-10)

            MDFloatingActionButton:
                pos_hint: {"center_x": 0.88, "center_y": 0.38}
                id: plusbutton
                icon: "plus"
                size_hint: 0.025, 0.03
                width: 0.2
                on_release: root.vel10_command_callback(10)

            Button:
                pos_hint: {"center_x": 0.915, "center_y": 0.38}
                size_hint: 0.04, 0.05
                id: pub10
                text: 'Pub'
                width: 0.3
                on_press:app.send_position("35", root.m10_pos.text, root.m10_vel.text, "pub10")

        # # # SENSORS
        GridLayout:
            pos_hint: {"center_x": 0.75, "center_y": 0.2}
            canvas.before:
                Color:
                    rgba: 0, 0, 0, 1
                Rectangle:
                    pos: self.pos
                    size: self.size
            cols: 3
            size_hint: None, None
            height: 100
            width: 275

            Label:
                text: "Voltage"
            Label:
                text: "0.0"
                id: voltage_id
                color: (0,0,1,1)
                canvas.before:
                    Color:
                        rgba: 1, 1, 1, 1
                    Rectangle:
                        pos: self.pos
                        size: self.size
            Label:
                text: "V"
            Label:
                text: "Current"
            Label:
                text: "0.0"
                id: current_id
                color: (0,0,1,1)
                canvas.before:
                    Color:
                        rgba: 1, 1, 1, 1
                    Rectangle:
                        pos: self.pos
                        size: self.size
            Label:
                text: "mA"
            Label:
                text: "Temperature"
            Label:
                text: "0.0"
                id: temperature_id
                color: (0,0,1,1)
                canvas.before:
                    Color:
                        rgba: 1, 1, 1, 1
                    Rectangle:
                        pos: self.pos
                        size: self.size
            Label:
                text: "C"
            
        MDTabs:
            text_color_active: [1, 1, 1, 1]
            size_hint: 1, 0.08
            tab_hint_x: True
            allow_stretch: True
            on_tab_switch: app.on_tab_switch(*args)
            Tab:
                title: '[b]LiDAR[/b]'
                icon: 'motion-sensor'
            Tab:
                title: '[b]RealSense[/b]'
                icon: 'camera-outline'
            Tab:
                title: '[b]IMU[/b]'
                icon: 'chart-waterfall'
            Tab:
                title: '[b]Force Torque[/b]'
                icon: 'cog'

<MenuHeadScreen>:
    name: 'home_head_control'
    img: h_asyn_image
    m1_pos: m1_textinput
    m2_pos: m2_textinput
    m1_vel: velo1_textinput
    m2_vel: velo2_textinput

    FloatLayout:
        orientation: "vertical"

        MDTopAppBar:
            title: "[b]Humanoid Control Panel[/b]"
            anchor_title: "center"
            left_action_items: [["home-analytics", lambda x: app.set_screen('home')]]
            right_action_items: [["engine", lambda x: app.callback(x)], ["robot-confused-outline", lambda x: app.set_screen('id')], ["power", lambda x: app.callback_2()]]
            size_hint: 1, .1
            pos_hint: {"center_x": .5, "center_y": 0.95}
        
        AsyncImage:
            id: h_asyn_image
            source: root.image_path
            size_hint: None, None
            width: 400
            height: 400
            pos_hint: {'center_x':.25, 'center_y': .6}
            nocache: True

        Label:
            pos_hint: {"center_x": 0.25, "center_y": 0.325}
            bold: True
            font_size: '20sp'
            text: 'LIVE URDF MODEL'
            size_hint_x: None
            width: 1
            color: (0,0,0,1)

        Label:
            pos_hint: {"center_x": 0.25, "center_y": 0.3}
            bold: True
            font_size: '12sp'
            text: 'POSITION IN DEGREES | VELOCITY IN ROTATIONS/S'
            size_hint_x: None
            width: 1
            color: (0,0,0,1)

        Button:
            id: emergency
            bold: True
            font_size: '18sp'
            text: "EMERGENCY STOP"
            background_normal: ''
            background_color: rgba("#ff0a01")
            color: 1,1,1,1
            halign: "center"
            # background_color: get_color_from_hex('#FF0A01')
            # background_color: 0,1,0,1
            size_hint: 0.2, 0.15
            pos_hint: {"center_x": .13, "center_y": 0.18}
            on_press: app.emergency_stop()

        Button:
            id: run_btn
            bold: True
            font_size: '20sp'
            text: "REFRESH"
            background_normal: ''
            background_color: rgba("#00ab66")
            color:1,1,1,1
            halign: "center"
            size_hint: 0.2, 0.15
            pos_hint: {"center_x": .35, "center_y": 0.18}
            on_press: root.change_image("new_robotImage.png")

        Label:
            pos_hint: {"center_x": 0.75, "center_y": 0.84}
            bold: True
            font_size: '20sp'
            text: 'HEAD MOTOR CONTROL PANEL'
            size_hint_x: None
            width: 1
            color: (0,0,0,1)

        MDCard:
            size_hint: .5, .4
            focus_behavior: True
            pos_hint: {"center_x": .74, "center_y": .6}
            md_bg_color: "lightgrey"
            unfocus_color: "lightgrey"
            focus_color: "grey"
            # elevation: 6

        Label:
            pos_hint: {"center_x": 0.75, "center_y": 0.35}
            bold: True
            font_size: '20sp'
            text: 'MOTOR SENSOR INFO'
            size_hint_x: None
            width: 1
            color: (0,0,0,1)
        
        FloatLayout:
            Label:
                pos_hint: {"center_x": 0.96, "center_y": 0.78}
                bold: True
                font_size: '10sp'
                text: 'SENSOR'
                width: 1
                color: (0,0,0,1)
            Label:
                pos_hint: {"center_x": 0.525, "center_y": 0.78}
                bold: True
                font_size: '10sp'
                text: 'ID'
                width: 0.5
                color: (0,0,0,1)
            Label:
                pos_hint: {"center_x": 0.575, "center_y": 0.78}
                bold: True
                font_size: '10sp'
                text: 'CUR POS' 
                size_hint: None, 1
                width: 10
                color: (0,0,0,1) 
            Label:
                pos_hint: {"center_x": 0.64, "center_y": 0.78}
                bold: True
                font_size: '10sp'
                text: 'POS CTRL' 
                width: 10
                color: (0,0,0,1) 
            Label:
                pos_hint: {"center_x": 0.675, "center_y": 0.78}
                bold: True
                font_size: '10sp'
                text: ' '  
                size_hint_x: None
                width: 1
                color: (0,0,0,1) 
            Label:
                pos_hint: {"center_x": 0.705, "center_y": 0.78}
                bold: True
                font_size: '10sp'
                text: ' '
                size_hint_x: None
                width: 1
                color: (0,0,0,1)
            Label:
                pos_hint: {"center_x": 0.75, "center_y": 0.78}
                bold: True
                font_size: '10sp'
                text: 'CUR VEL'
                size_hint_x: None
                width: 1
                color: (0,0,0,1) 
            Label:
                pos_hint: {"center_x": 0.815, "center_y": 0.78}
                bold: True
                font_size: '10sp'
                text: 'VEL CTRL'
                size_hint_x: None
                width: 1
                color: (0,0,0,1) 
            Label:
                pos_hint: {"center_x": 0.85, "center_y": 0.78}
                bold: True
                font_size: '10sp'
                text: ' ' 
                size_hint_x: None
                width: 1
                color: (0,0,0,1)
            Label:
                pos_hint: {"center_x": 0.88, "center_y": 0.78}
                bold: True
                font_size: '10sp'
                text: ' ' 
                size_hint_x: None
                width: 1
                color: (0,0,0,1)
            Label:
                pos_hint: {"center_x": 0.9125, "center_y": 0.78}
                bold: True
                font_size: '10sp'
                text: 'SEND'
                size_hint_x: None
                width: 1
                color: (0,0,0,1)

            MDCheckbox:
                id: m1_checkbox
                pos_hint: {"center_x": 0.96, "center_y": 0.74}
                size_hint: None, None
                size: "24dp", "24dp"
                on_active: app.show_sensor_info('41')

            Label:
                pos_hint: {"center_x": 0.525, "center_y": 0.74}
                font_size: '10sp'
                text: 'r_antenna'
                id: m1_id
                color: (0,0,1,1)
                # canvas.before:
                #     Color:
                #         rgba: 1, 1, 1, 1
                #     Rectangle:
                #         pos: self.pos
                #         size: self.size
                width: 0.6
            
            TextInput:
                pos_hint: {"center_x": 0.575, "center_y": 0.74}
                id: m1_reading
                size_hint: 0.04, 0.05
                width: 1
                input_filter: 'int'
                text: "0"
                multiline: False
                readonly: True
                
            TextInput:
                pos_hint: {"center_x": 0.635, "center_y": 0.74}
                id: m1_textinput
                size_hint: 0.04, 0.05
                width: .05
                input_filter: 'int'
                text: "0"
                multiline: False
                on_text_validate: root.m1_command_callback(0)

            MDFloatingActionButton:
                pos_hint: {"center_x": 0.675, "center_y": 0.74}
                id: minusbutton
                icon: "minus"
                size_hint: 0.025, 0.03
                width: 0.2 
                on_release: root.m1_command_callback(-5)

            MDFloatingActionButton:
                pos_hint: {"center_x": 0.705, "center_y": 0.74}
                id: plusbutton
                icon: "plus"
                size_hint: 0.025, 0.03
                width: 0.2
                on_release: root.m1_command_callback(5)

            TextInput:
                pos_hint: {"center_x": 0.75, "center_y": 0.74}
                id: vel1_reading
                size_hint: 0.04, 0.05
                width: 1
                input_filter: 'int'
                text: "0"
                multiline: False
                readonly: True
            
            TextInput:
                pos_hint: {"center_x": 0.81, "center_y": 0.74}
                id: velo1_textinput
                size_hint: 0.04, 0.05
                width: 0.3
                input_filter: 'int'
                multiline: False
                text: "0"

            MDFloatingActionButton:
                pos_hint: {"center_x": 0.85, "center_y": 0.74}
                id: minusbutton
                icon: "minus"
                size_hint: 0.025, 0.03
                width: 0.2
                on_release: root.vel1_command_callback(-10)

            MDFloatingActionButton:
                pos_hint: {"center_x": 0.88, "center_y": 0.74}
                id: plusbutton
                icon: "plus"
                size_hint: 0.025, 0.03
                width: 0.2
                on_release: root.vel1_command_callback(10)

            Button:
                id: pub1
                pos_hint: {"center_x": 0.915, "center_y": 0.74}
                size_hint: 0.04, 0.05
                text: 'Pub'
                width: 0.3
                on_press: app.send_position("41", root.m1_pos.text, root.m1_vel.text, "pub1")

            MDCheckbox:
                id: m2_checkbox
                pos_hint: {"center_x": 0.96, "center_y": 0.7}
                size_hint: None, None
                size: "24dp", "24dp"
                on_active: app.show_sensor_info('42')

            Label:
                pos_hint: {"center_x": 0.525, "center_y": 0.7}
                font_size: '10sp'
                text: 'l_antenna'
                id: m2_id
                color: (0,0,1,1)
                size_hint_x: None
                width: 0.6
                # canvas.before:
                #     Color:
                #         rgba: 1, 1, 1, 1
                #     Rectangle:
                #         pos: self.pos
                #         size: self.size

            TextInput:
                pos_hint: {"center_x": 0.575, "center_y": 0.7}
                id: m2_reading
                size_hint: 0.04, 0.05
                width: 0.3
                input_filter: 'int'
                text: "0"
                multiline: False
                readonly: True

            TextInput:
                pos_hint: {"center_x": 0.635, "center_y": 0.7}
                id: m2_textinput
                size_hint: 0.04, 0.05
                width: 0.3
                input_filter: 'int'
                text: "0"
                multiline: False
                on_text_validate: root.m2_command_callback(0)

            MDFloatingActionButton:
                pos_hint: {"center_x": 0.675, "center_y": 0.7}
                id: minusbutton
                icon: "minus"
                size_hint: 0.025, 0.03
                width: 0.2
                on_release: root.m2_command_callback(-5)

            MDFloatingActionButton:
                pos_hint: {"center_x": 0.705, "center_y": 0.7}
                id: plusbutton
                icon: "plus"
                size_hint: 0.025, 0.03
                width: 0.2
                on_release: root.m2_command_callback(5)

            TextInput:
                pos_hint: {"center_x": 0.75, "center_y": 0.7}
                id: vel2_reading
                size_hint: 0.04, 0.05
                width: 0.3
                input_filter: 'int'
                text: "0"
                multiline: False
                readonly: True

            TextInput:
                pos_hint: {"center_x": 0.81, "center_y": 0.7}
                id: velo2_textinput
                size_hint: 0.04, 0.05
                width: 0.3
                input_filter: 'int'
                multiline: False
                text: "0"    

            MDFloatingActionButton:
                pos_hint: {"center_x": 0.85, "center_y": 0.7}
                id: minusbutton
                icon: "minus"
                size_hint: 0.025, 0.03
                width: 0.2
                on_release: root.vel2_command_callback(-10)

            MDFloatingActionButton:
                pos_hint: {"center_x": 0.88, "center_y": 0.7}
                id: plusbutton
                icon: "plus"
                size_hint: 0.025, 0.03
                width: 0.2
                on_release: root.vel2_command_callback(10)

            Button:
                id: pub2
                pos_hint: {"center_x": 0.915, "center_y": 0.7}
                size_hint: 0.04, 0.05
                text: 'Pub'
                width: 0.3
                #on_press:app.send_position("42", root.m2_pos.text, root.m2_vel.text, "pub2")
                on_press: root.buton()


        # # # SENSORS
        GridLayout:
            pos_hint: {"center_x": 0.75, "center_y": 0.2}
            canvas.before:
                Color:
                    rgba: 0, 0, 0, 1
                Rectangle:
                    pos: self.pos
                    size: self.size
            cols: 3
            size_hint: None, None
            height: 100
            width: 275

            Label:
                text: "Voltage"
            Label:
                text: "0.0"
                id: voltage_id
                color: (0,0,1,1)
                canvas.before:
                    Color:
                        rgba: 1, 1, 1, 1
                    Rectangle:
                        pos: self.pos
                        size: self.size
            Label:
                text: "V"
            Label:
                text: "Current"
            Label:
                text: "0.0"
                id: current_id
                color: (0,0,1,1)
                canvas.before:
                    Color:
                        rgba: 1, 1, 1, 1
                    Rectangle:
                        pos: self.pos
                        size: self.size
            Label:
                text: "mA"
            Label:
                text: "Temperature"
            Label:
                text: "0.0"
                id: temperature_id
                color: (0,0,1,1)
                canvas.before:
                    Color:
                        rgba: 1, 1, 1, 1
                    Rectangle:
                        pos: self.pos
                        size: self.size
            Label:
                text: "C"
            
        MDTabs:
            text_color_active: [1, 1, 1, 1]
            size_hint: 1, 0.08
            tab_hint_x: True
            allow_stretch: True
            on_tab_switch: app.on_tab_switch(*args)
            Tab:
                title: '[b]LiDAR[/b]'
                icon: 'motion-sensor'
            Tab:
                title: '[b]RealSense[/b]'
                icon: 'camera-outline'
            Tab:
                title: '[b]IMU[/b]'
                icon: 'chart-waterfall'
            Tab:
                title: '[b]Force Torque[/b]'
                icon: 'cog'

<MenuTorsoScreen>:
    name: 'home_torso_control'
    img: t_asyn_image
    m1_pos: m1_textinput
    m2_pos: m2_textinput
    m3_pos: m3_textinput
    m1_vel: velo1_textinput
    m2_vel: velo2_textinput
    m3_vel: velo3_textinput

    FloatLayout:
        orientation: "vertical"

        MDTopAppBar:
            title: "[b]Humanoid Control Panel[/b]"
            anchor_title: "center"
            left_action_items: [["home-analytics", lambda x: app.set_screen('home')]]
            right_action_items: [["engine", lambda x: app.callback(x)], ["robot-confused-outline", lambda x: app.set_screen('id')], ["power", lambda x: app.callback_2()]]
            size_hint: 1, .1
            pos_hint: {"center_x": .5, "center_y": 0.95}
        
        AsyncImage:
            id: t_asyn_image
            source: root.image_path
            size_hint: None, None
            width: 400
            height: 400
            pos_hint: {'center_x':.25, 'center_y': .6}
            nocache: True

        Label:
            pos_hint: {"center_x": 0.25, "center_y": 0.325}
            bold: True
            font_size: '20sp'
            text: 'LIVE URDF MODEL'
            size_hint_x: None
            width: 1
            color: (0,0,0,1)

        Label:
            pos_hint: {"center_x": 0.25, "center_y": 0.3}
            bold: True
            font_size: '12sp'
            text: 'POSITION IN DEGREES | VELOCITY IN ROTATIONS/S'
            size_hint_x: None
            width: 1
            color: (0,0,0,1)

        Button:
            id: emergency
            bold: True
            font_size: '18sp'
            text: "EMERGENCY STOP"
            background_normal: ''
            background_color: rgba("#ff0a01")
            color: 1,1,1,1
            halign: "center"
            # background_color: get_color_from_hex('#FF0A01')
            # background_color: 0,1,0,1
            size_hint: 0.2, 0.15
            pos_hint: {"center_x": .13, "center_y": 0.18}
            on_press: app.emergency_stop()

        Button:
            id: run_btn
            bold: True
            font_size: '20sp'
            text: "REFRESH"
            background_normal: ''
            background_color: rgba("#00ab66")
            color:1,1,1,1
            halign: "center"
            size_hint: 0.2, 0.15
            pos_hint: {"center_x": .35, "center_y": 0.18}
            on_press: root.change_image("new_robotImage.png")

        Label:
            pos_hint: {"center_x": 0.75, "center_y": 0.84}
            bold: True
            font_size: '20sp'
            text: 'TORSO MOTOR CONTROL PANEL'
            size_hint_x: None
            width: 1
            color: (0,0,0,1)

        MDCard:
            size_hint: .5, .4
            focus_behavior: True
            pos_hint: {"center_x": .74, "center_y": .6}
            md_bg_color: "lightgrey"
            unfocus_color: "lightgrey"
            focus_color: "grey"
            # elevation: 6

        Label:
            pos_hint: {"center_x": 0.75, "center_y": 0.35}
            bold: True
            font_size: '20sp'
            text: 'MOTOR SENSOR INFO'
            size_hint_x: None
            width: 1
            color: (0,0,0,1)

        FloatLayout:
            Label:
                pos_hint: {"center_x": 0.96, "center_y": 0.78}
                bold: True
                font_size: '10sp'
                text: 'SENSOR'
                width: 1
                color: (0,0,0,1)
            Label:
                pos_hint: {"center_x": 0.525, "center_y": 0.78}
                bold: True
                font_size: '10sp'
                text: 'ID'
                width: 0.5
                color: (0,0,0,1)
            Label:
                pos_hint: {"center_x": 0.575, "center_y": 0.78}
                bold: True
                font_size: '10sp'
                text: 'CUR POS' 
                size_hint: None, 1
                width: 10
                color: (0,0,0,1) 
            Label:
                pos_hint: {"center_x": 0.64, "center_y": 0.78}
                bold: True
                font_size: '10sp'
                text: 'POS CTRL' 
                width: 10
                color: (0,0,0,1) 
            Label:
                pos_hint: {"center_x": 0.675, "center_y": 0.78}
                bold: True
                font_size: '10sp'
                text: ' '  
                size_hint_x: None
                width: 1
                color: (0,0,0,1) 
            Label:
                pos_hint: {"center_x": 0.705, "center_y": 0.78}
                bold: True
                font_size: '10sp'
                text: ' '
                size_hint_x: None
                width: 1
                color: (0,0,0,1)
            Label:
                pos_hint: {"center_x": 0.75, "center_y": 0.78}
                bold: True
                font_size: '10sp'
                text: 'CUR VEL'
                size_hint_x: None
                width: 1
                color: (0,0,0,1) 
            Label:
                pos_hint: {"center_x": 0.815, "center_y": 0.78}
                bold: True
                font_size: '10sp'
                text: 'VEL CTRL'
                size_hint_x: None
                width: 1
                color: (0,0,0,1) 
            Label:
                pos_hint: {"center_x": 0.85, "center_y": 0.78}
                bold: True
                font_size: '10sp'
                text: ' ' 
                size_hint_x: None
                width: 1
                color: (0,0,0,1)
            Label:
                pos_hint: {"center_x": 0.88, "center_y": 0.78}
                bold: True
                font_size: '10sp'
                text: ' ' 
                size_hint_x: None
                width: 1
                color: (0,0,0,1)
            Label:
                pos_hint: {"center_x": 0.9125, "center_y": 0.78}
                bold: True
                font_size: '10sp'
                text: 'SEND'
                size_hint_x: None
                width: 1
                color: (0,0,0,1)

            MDCheckbox:
                id: m1_checkbox
                pos_hint: {"center_x": 0.96, "center_y": 0.74}
                size_hint: None, None
                size: "24dp", "24dp"
                on_active: app.show_sensor_info('27')

            Label:
                pos_hint: {"center_x": 0.525, "center_y": 0.74}
                font_size: '10sp'
                text: 'torso_y'
                id: m1_id
                color: (0,0,1,1)
                # canvas.before:
                #     Color:
                #         rgba: 1, 1, 1, 1
                #     Rectangle:
                #         pos: self.pos
                #         size: self.size
                width: 0.6
            
            TextInput:
                pos_hint: {"center_x": 0.575, "center_y": 0.74}
                id: m1_reading
                size_hint: 0.04, 0.05
                width: 1
                input_filter: 'int'
                text: "0"
                multiline: False
                readonly: True
                
            TextInput:
                pos_hint: {"center_x": 0.635, "center_y": 0.74}
                id: m1_textinput
                size_hint: 0.04, 0.05
                width: .05
                input_filter: 'int'
                text: "0"
                multiline: False
                on_text_validate: root.m1_command_callback(0)

            MDFloatingActionButton:
                pos_hint: {"center_x": 0.675, "center_y": 0.74}
                id: minusbutton
                icon: "minus"
                size_hint: 0.025, 0.03
                width: 0.2 
                on_release: root.m1_command_callback(-5)

            MDFloatingActionButton:
                pos_hint: {"center_x": 0.705, "center_y": 0.74}
                id: plusbutton
                icon: "plus"
                size_hint: 0.025, 0.03
                width: 0.2
                on_release: root.m1_command_callback(5)

            TextInput:
                pos_hint: {"center_x": 0.75, "center_y": 0.74}
                id: vel1_reading
                size_hint: 0.04, 0.05
                width: 1
                input_filter: 'int'
                text: "0"
                multiline: False
                readonly: True
            
            TextInput:
                pos_hint: {"center_x": 0.81, "center_y": 0.74}
                id: velo1_textinput
                size_hint: 0.04, 0.05
                width: 0.3
                input_filter: 'int'
                multiline: False
                text: "0"

            MDFloatingActionButton:
                pos_hint: {"center_x": 0.85, "center_y": 0.74}
                id: minusbutton
                icon: "minus"
                size_hint: 0.025, 0.03
                width: 0.2
                on_release: root.vel1_command_callback(-10)

            MDFloatingActionButton:
                pos_hint: {"center_x": 0.88, "center_y": 0.74}
                id: plusbutton
                icon: "plus"
                size_hint: 0.025, 0.03
                width: 0.2
                on_release: root.vel1_command_callback(10)

            Button:
                pos_hint: {"center_x": 0.915, "center_y": 0.74}
                size_hint: 0.04, 0.05
                id: pub1
                text: 'Pub'
                width: 0.3
                on_press:app.send_position("27", root.m1_pos.text, root.m1_vel.text, "pub1")

            #MOTOR 2
            MDCheckbox:
                id: m2_checkbox
                pos_hint: {"center_x": 0.96, "center_y": 0.70}
                size_hint: None, None
                size: "24dp", "24dp"
                on_active: app.show_sensor_info('28')

            Label:
                pos_hint: {"center_x": 0.525, "center_y": 0.70}
                font_size: '10sp'
                text: 'head_y'
                id: m2_id
                color: (0,0,1,1)
                size_hint_x: None
                width: 0.6
                # canvas.before:
                #     Color:
                #         rgba: 1, 1, 1, 1
                #     Rectangle:
                #         pos: self.pos
                #         size: self.size

            TextInput:
                pos_hint: {"center_x": 0.575, "center_y": 0.70}
                id: m2_reading
                size_hint: 0.04, 0.05
                width: 0.3
                input_filter: 'int'
                text: "0"
                multiline: False
                readonly: True

            TextInput:
                pos_hint: {"center_x": 0.635, "center_y": 0.70}
                id: m2_textinput
                size_hint: 0.04, 0.05
                width: 0.3
                input_filter: 'int'
                text: "0"
                multiline: False
                on_text_validate: root.m2_command_callback(0)

            MDFloatingActionButton:
                pos_hint: {"center_x": 0.675, "center_y": 0.70}
                id: minusbutton
                icon: "minus"
                size_hint: 0.025, 0.03
                width: 0.2
                on_release: root.m2_command_callback(-5)

            MDFloatingActionButton:
                pos_hint: {"center_x": 0.705, "center_y": 0.70}
                id: plusbutton
                icon: "plus"
                size_hint: 0.025, 0.03
                width: 0.2
                on_release: root.m2_command_callback(5)

            TextInput:
                pos_hint: {"center_x": 0.75, "center_y": 0.70}
                id: vel2_reading
                size_hint: 0.04, 0.05
                width: 0.3
                input_filter: 'int'
                text: "0"
                multiline: False
                readonly: True

            TextInput:
                pos_hint: {"center_x": 0.81, "center_y": 0.70}
                id: velo2_textinput
                size_hint: 0.04, 0.05
                width: 0.3
                input_filter: 'int'
                multiline: False
                text: "0"    

            MDFloatingActionButton:
                pos_hint: {"center_x": 0.85, "center_y": 0.70}
                id: minusbutton
                icon: "minus"
                size_hint: 0.025, 0.03
                width: 0.2
                on_release: root.vel2_command_callback(-10)

            MDFloatingActionButton:
                pos_hint: {"center_x": 0.88, "center_y": 0.70}
                id: plusbutton
                icon: "plus"
                size_hint: 0.025, 0.03
                width: 0.2
                on_release: root.vel2_command_callback(10)

            Button:
                pos_hint: {"center_x": 0.915, "center_y": 0.70}
                size_hint: 0.04, 0.05
                id: pub2
                text: 'Pub'
                width: 0.3
                on_press: app.send_position("28", root.m2_pos.text, root.m2_vel.text, "pub2")

            #MOTOR 3
            MDCheckbox:
                id: m3_checkbox
                pos_hint: {"center_x": 0.96, "center_y": 0.66}
                size_hint: None, None
                size: "24dp", "24dp"
                on_active: app.show_sensor_info('29')

            Label:
                pos_hint: {"center_x": 0.525, "center_y": 0.66}
                font_size: '10sp'
                text: 'head_p'
                id: m3_id
                color: (0,0,1,1)
                size_hint_x: None
                width: 0.6
                # canvas.before:
                #     Color:
                #         rgba: 1, 1, 1, 1
                #     Rectangle:
                #         pos: self.pos
                #         size: self.size

            TextInput:
                pos_hint: {"center_x": 0.575, "center_y": 0.66}
                id: m3_reading
                size_hint: 0.04, 0.05
                width: 0.3
                input_filter: 'int'
                text: "0"
                multiline: False
                readonly: True

            TextInput:
                pos_hint: {"center_x": 0.635, "center_y": 0.66}
                id: m3_textinput
                size_hint: 0.04, 0.05
                width: 0.3
                input_filter: 'int'
                text: "0"
                multiline: False
                on_text_validate: root.m3_command_callback(0)

            MDFloatingActionButton:
                pos_hint: {"center_x": 0.675, "center_y": 0.66}
                id: minusbutton
                icon: "minus"
                size_hint: 0.025, 0.03
                width: 0.2
                on_release: root.m3_command_callback(-5)

            MDFloatingActionButton:
                pos_hint: {"center_x": 0.705, "center_y": 0.66}
                id: plusbutton
                icon: "plus"
                size_hint: 0.025, 0.03
                width: 0.2
                on_release: root.m3_command_callback(5)

            TextInput:
                pos_hint: {"center_x": 0.75, "center_y": 0.66}
                id: vel3_reading
                size_hint: 0.04, 0.05
                width: 0.3
                input_filter: 'int'
                text: "0"
                multiline: False
                readonly: True

            TextInput:
                pos_hint: {"center_x": 0.81, "center_y": 0.66}
                id: velo3_textinput
                size_hint: 0.04, 0.05
                width: 0.3
                input_filter: 'int'
                multiline: False
                text: "0"    

            MDFloatingActionButton:
                pos_hint: {"center_x": 0.85, "center_y": 0.66}
                id: minusbutton
                icon: "minus"
                size_hint: 0.025, 0.03
                width: 0.2
                on_release: root.vel3_command_callback(-10)

            MDFloatingActionButton:
                pos_hint: {"center_x": 0.88, "center_y": 0.66}
                id: plusbutton
                icon: "plus"
                size_hint: 0.025, 0.03
                width: 0.2
                on_release: root.vel3_command_callback(10)

            Button:
                pos_hint: {"center_x": 0.915, "center_y": 0.66}
                size_hint: 0.04, 0.05
                id: pub3
                text: 'Pub'
                width: 0.3
                on_press:app.send_position("29", root.m3_pos.text, root.m3_vel.text, "pub3")
        
        # # # SENSORS
        GridLayout:
            pos_hint: {"center_x": 0.75, "center_y": 0.2}
            canvas.before:
                Color:
                    rgba: 0, 0, 0, 1
                Rectangle:
                    pos: self.pos
                    size: self.size
            cols: 3
            size_hint: None, None
            height: 100
            width: 275

            Label:
                text: "Voltage"
            Label:
                text: "0.0"
                id: voltage_id
                color: (0,0,1,1)
                canvas.before:
                    Color:
                        rgba: 1, 1, 1, 1
                    Rectangle:
                        pos: self.pos
                        size: self.size
            Label:
                text: "V"
            Label:
                text: "Current"
            Label:
                text: "0.0"
                id: current_id
                color: (0,0,1,1)
                canvas.before:
                    Color:
                        rgba: 1, 1, 1, 1
                    Rectangle:
                        pos: self.pos
                        size: self.size
            Label:
                text: "mA"
            Label:
                text: "Temperature"
            Label:
                text: "0.0"
                id: temperature_id
                color: (0,0,1,1)
                canvas.before:
                    Color:
                        rgba: 1, 1, 1, 1
                    Rectangle:
                        pos: self.pos
                        size: self.size
            Label:
                text: "C"

        MDTabs:
            text_color_active: [1, 1, 1, 1]
            size_hint: 1, 0.08
            tab_hint_x: True
            allow_stretch: True
            on_tab_switch: app.on_tab_switch(*args)
            Tab:
                title: '[b]LiDAR[/b]'
                icon: 'motion-sensor'
            Tab:
                title: '[b]RealSense[/b]'
                icon: 'camera-outline'
            Tab:
                title: '[b]IMU[/b]'
                icon: 'chart-waterfall'
            Tab:
                title: '[b]Force Torque[/b]'
                icon: 'cog'

<MenuLeftLegScreen>:
    name: 'home_leftleg_control'
    img: ll_asyn_image
    m1_pos: m1_textinput
    m2_pos: m2_textinput
    m3_pos: m3_textinput
    m4_pos: m4_textinput
    m5_pos: m5_textinput
    m6_pos: m6_textinput

    m1_vel: velo1_textinput
    m2_vel: velo2_textinput
    m3_vel: velo3_textinput
    m4_vel: velo4_textinput
    m5_vel: velo5_textinput
    m6_vel: velo6_textinput

    FloatLayout:
        orientation: "vertical"

        MDTopAppBar:
            title: "[b]Humanoid Control Panel[/b]"
            anchor_title: "center"
            left_action_items: [["home-analytics", lambda x: app.set_screen('home')]]
            right_action_items: [["engine", lambda x: app.callback(x)], ["robot-confused-outline", lambda x: app.set_screen('id')], ["power", lambda x: app.callback_2()]]
            size_hint: 1, .1
            pos_hint: {"center_x": .5, "center_y": 0.95}
        
        AsyncImage:
            id: ll_asyn_image
            source: root.image_path
            size_hint: None, None
            width: 400
            height: 400
            pos_hint: {'center_x':.25, 'center_y': .6}
            nocache: True

        Label:
            pos_hint: {"center_x": 0.25, "center_y": 0.325}
            bold: True
            font_size: '20sp'
            text: 'LIVE URDF MODEL'
            size_hint_x: None
            width: 1
            color: (0,0,0,1)

        Label:
            pos_hint: {"center_x": 0.25, "center_y": 0.3}
            bold: True
            font_size: '12sp'
            text: 'POSITION IN DEGREES | VELOCITY IN ROTATIONS/S'
            size_hint_x: None
            width: 1
            color: (0,0,0,1)

        Button:
            id: emergency
            bold: True
            font_size: '18sp'
            text: "EMERGENCY STOP"
            background_normal: ''
            background_color: rgba("#ff0a01")
            color: 1,1,1,1
            halign: "center"
            # background_color: get_color_from_hex('#FF0A01')
            # background_color: 0,1,0,1
            size_hint: 0.2, 0.15
            pos_hint: {"center_x": .13, "center_y": 0.18}
            on_press: app.emergency_stop()

        Button:
            id: run_btn
            bold: True
            font_size: '20sp'
            text: "REFRESH"
            background_normal: ''
            background_color: rgba("#00ab66")
            color:1,1,1,1
            halign: "center"
            size_hint: 0.2, 0.15
            pos_hint: {"center_x": .35, "center_y": 0.18}
            on_press: root.change_image("new_robotImage.png")

        Label:
            pos_hint: {"center_x": 0.75, "center_y": 0.84}
            bold: True
            font_size: '20sp'
            text: 'LEFT LEG MOTOR CONTROL PANEL'
            size_hint_x: None
            width: 1
            color: (0,0,0,1)

        MDCard:
            size_hint: .5, .4
            focus_behavior: True
            pos_hint: {"center_x": .74, "center_y": .6}
            md_bg_color: "lightgrey"
            unfocus_color: "lightgrey"
            focus_color: "grey"
            # elevation: 6

        Label:
            pos_hint: {"center_x": 0.75, "center_y": 0.35}
            bold: True
            font_size: '20sp'
            text: 'MOTOR SENSOR INFO'
            size_hint_x: None
            width: 1
            color: (0,0,0,1)

        FloatLayout:
            Label:
                pos_hint: {"center_x": 0.96, "center_y": 0.78}
                bold: True
                font_size: '10sp'
                text: 'SENSOR'
                width: 1
                color: (0,0,0,1)
            Label:
                pos_hint: {"center_x": 0.525, "center_y": 0.78}
                bold: True
                font_size: '10sp'
                text: 'ID'
                width: 0.5
                color: (0,0,0,1)
            Label:
                pos_hint: {"center_x": 0.575, "center_y": 0.78}
                bold: True
                font_size: '10sp'
                text: 'CUR POS' 
                size_hint: None, 1
                width: 10
                color: (0,0,0,1) 
            Label:
                pos_hint: {"center_x": 0.64, "center_y": 0.78}
                bold: True
                font_size: '10sp'
                text: 'POS CTRL' 
                width: 10
                color: (0,0,0,1) 
            Label:
                pos_hint: {"center_x": 0.675, "center_y": 0.78}
                bold: True
                font_size: '10sp'
                text: ' '  
                size_hint_x: None
                width: 1
                color: (0,0,0,1) 
            Label:
                pos_hint: {"center_x": 0.705, "center_y": 0.78}
                bold: True
                font_size: '10sp'
                text: ' '
                size_hint_x: None
                width: 1
                color: (0,0,0,1)
            Label:
                pos_hint: {"center_x": 0.75, "center_y": 0.78}
                bold: True
                font_size: '10sp'
                text: 'CUR VEL'
                size_hint_x: None
                width: 1
                color: (0,0,0,1) 
            Label:
                pos_hint: {"center_x": 0.815, "center_y": 0.78}
                bold: True
                font_size: '10sp'
                text: 'VEL CTRL'
                size_hint_x: None
                width: 1
                color: (0,0,0,1) 
            Label:
                pos_hint: {"center_x": 0.85, "center_y": 0.78}
                bold: True
                font_size: '10sp'
                text: ' ' 
                size_hint_x: None
                width: 1
                color: (0,0,0,1)
            Label:
                pos_hint: {"center_x": 0.88, "center_y": 0.78}
                bold: True
                font_size: '10sp'
                text: ' ' 
                size_hint_x: None
                width: 1
                color: (0,0,0,1)
            Label:
                pos_hint: {"center_x": 0.9125, "center_y": 0.78}
                bold: True
                font_size: '10sp'
                text: 'SEND'
                size_hint_x: None
                width: 1
                color: (0,0,0,1)

            MDCheckbox:
                id: m1_checkbox
                pos_hint: {"center_x": 0.96, "center_y": 0.74}
                size_hint: None, None
                size: "24dp", "24dp"
                on_active: app.show_sensor_info('16')

            Label:
                pos_hint: {"center_x": 0.525, "center_y": 0.74}
                font_size: '10sp'
                text: 'hip_y'
                id: m1_id
                color: (0,0,1,1)
                # canvas.before:
                #     Color:
                #         rgba: 1, 1, 1, 1
                #     Rectangle:
                #         pos: self.pos
                #         size: self.size
                width: 0.6
            
            TextInput:
                pos_hint: {"center_x": 0.575, "center_y": 0.74}
                id: m1_reading
                size_hint: 0.04, 0.05
                width: 1
                input_filter: 'int'
                text: "0"
                multiline: False
                readonly: True
                
            TextInput:
                pos_hint: {"center_x": 0.635, "center_y": 0.74}
                id: m1_textinput
                size_hint: 0.04, 0.05
                width: .05
                input_filter: 'int'
                text: "0"
                multiline: False
                on_text_validate: root.m1_command_callback(0)

            MDFloatingActionButton:
                pos_hint: {"center_x": 0.675, "center_y": 0.74}
                id: minusbutton
                icon: "minus"
                size_hint: 0.025, 0.03
                width: 0.2 
                on_release: root.m1_command_callback(-5)

            MDFloatingActionButton:
                pos_hint: {"center_x": 0.705, "center_y": 0.74}
                id: plusbutton
                icon: "plus"
                size_hint: 0.025, 0.03
                width: 0.2
                on_release: root.m1_command_callback(5)

            TextInput:
                pos_hint: {"center_x": 0.75, "center_y": 0.74}
                id: vel1_reading
                size_hint: 0.04, 0.05
                width: 1
                input_filter: 'int'
                text: "0"
                multiline: False
                readonly: True
            
            TextInput:
                pos_hint: {"center_x": 0.81, "center_y": 0.74}
                id: velo1_textinput
                size_hint: 0.04, 0.05
                width: 0.3
                input_filter: 'int'
                multiline: False
                text: "0"

            MDFloatingActionButton:
                pos_hint: {"center_x": 0.85, "center_y": 0.74}
                id: minusbutton
                icon: "minus"
                size_hint: 0.025, 0.03
                width: 0.2
                on_release: root.vel1_command_callback(-10)

            MDFloatingActionButton:
                pos_hint: {"center_x": 0.88, "center_y": 0.74}
                id: plusbutton
                icon: "plus"
                size_hint: 0.025, 0.03
                width: 0.2
                on_release: root.vel1_command_callback(10)

            Button:
                pos_hint: {"center_x": 0.915, "center_y": 0.74}
                size_hint: 0.04, 0.05
                id: pub1
                text: 'Pub'
                width: 0.3
                on_press:app.send_position("16", root.m1_pos.text, root.m1_vel.text, "pub1")

            #MOTOR 2
            MDCheckbox:
                id: m2_checkbox
                pos_hint: {"center_x": 0.96, "center_y": 0.70}
                size_hint: None, None
                size: "24dp", "24dp"
                on_active: app.show_sensor_info('18')

            Label:
                pos_hint: {"center_x": 0.525, "center_y": 0.70}
                font_size: '10sp'
                text: 'hip_r'
                id: m2_id
                color: (0,0,1,1)
                size_hint_x: None
                width: 0.6
                # canvas.before:
                #     Color:
                #         rgba: 1, 1, 1, 1
                #     Rectangle:
                #         pos: self.pos
                #         size: self.size

            TextInput:
                pos_hint: {"center_x": 0.575, "center_y": 0.70}
                id: m2_reading
                size_hint: 0.04, 0.05
                width: 0.3
                input_filter: 'int'
                text: "0"
                multiline: False
                readonly: True

            TextInput:
                pos_hint: {"center_x": 0.635, "center_y": 0.70}
                id: m2_textinput
                size_hint: 0.04, 0.05
                width: 0.3
                input_filter: 'int'
                text: "0"
                multiline: False
                on_text_validate: root.m2_command_callback(0)

            MDFloatingActionButton:
                pos_hint: {"center_x": 0.675, "center_y": 0.70}
                id: minusbutton
                icon: "minus"
                size_hint: 0.025, 0.03
                width: 0.2
                on_release: root.m2_command_callback(-5)

            MDFloatingActionButton:
                pos_hint: {"center_x": 0.705, "center_y": 0.70}
                id: plusbutton
                icon: "plus"
                size_hint: 0.025, 0.03
                width: 0.2
                on_release: root.m2_command_callback(5)

            TextInput:
                pos_hint: {"center_x": 0.75, "center_y": 0.70}
                id: vel2_reading
                size_hint: 0.04, 0.05
                width: 0.3
                input_filter: 'int'
                text: "0"
                multiline: False
                readonly: True

            TextInput:
                pos_hint: {"center_x": 0.81, "center_y": 0.70}
                id: velo2_textinput
                size_hint: 0.04, 0.05
                width: 0.3
                input_filter: 'int'
                multiline: False
                text: "0"    

            MDFloatingActionButton:
                pos_hint: {"center_x": 0.85, "center_y": 0.70}
                id: minusbutton
                icon: "minus"
                size_hint: 0.025, 0.03
                width: 0.2
                on_release: root.vel2_command_callback(-10)

            MDFloatingActionButton:
                pos_hint: {"center_x": 0.88, "center_y": 0.70}
                id: plusbutton
                icon: "plus"
                size_hint: 0.025, 0.03
                width: 0.2
                on_release: root.vel2_command_callback(10)

            Button:
                pos_hint: {"center_x": 0.915, "center_y": 0.70}
                size_hint: 0.04, 0.05
                id: pub2
                text: 'Pub'
                width: 0.3
                on_press: app.send_position("18", root.m2_pos.text, root.m2_vel.text, "pub2")

            #MOTOR 3
            MDCheckbox:
                id: m3_checkbox
                pos_hint: {"center_x": 0.96, "center_y": 0.66}
                size_hint: None, None
                size: "24dp", "24dp"
                on_active: app.show_sensor_info('20')

            Label:
                pos_hint: {"center_x": 0.525, "center_y": 0.66}
                font_size: '10sp'
                text: 'hip_p'
                id: m3_id
                color: (0,0,1,1)
                size_hint_x: None
                width: 0.6
                # canvas.before:
                #     Color:
                #         rgba: 1, 1, 1, 1
                #     Rectangle:
                #         pos: self.pos
                #         size: self.size

            TextInput:
                pos_hint: {"center_x": 0.575, "center_y": 0.66}
                id: m3_reading
                size_hint: 0.04, 0.05
                width: 0.3
                input_filter: 'int'
                text: "0"
                multiline: False
                readonly: True

            TextInput:
                pos_hint: {"center_x": 0.635, "center_y": 0.66}
                id: m3_textinput
                size_hint: 0.04, 0.05
                width: 0.3
                input_filter: 'int'
                text: "0"
                multiline: False
                on_text_validate: root.m3_command_callback(0)

            MDFloatingActionButton:
                pos_hint: {"center_x": 0.675, "center_y": 0.66}
                id: minusbutton
                icon: "minus"
                size_hint: 0.025, 0.03
                width: 0.2
                on_release: root.m3_command_callback(-5)

            MDFloatingActionButton:
                pos_hint: {"center_x": 0.705, "center_y": 0.66}
                id: plusbutton
                icon: "plus"
                size_hint: 0.025, 0.03
                width: 0.2
                on_release: root.m3_command_callback(5)

            TextInput:
                pos_hint: {"center_x": 0.75, "center_y": 0.66}
                id: vel3_reading
                size_hint: 0.04, 0.05
                width: 0.3
                input_filter: 'int'
                text: "0"
                multiline: False
                readonly: True

            TextInput:
                pos_hint: {"center_x": 0.81, "center_y": 0.66}
                id: velo3_textinput
                size_hint: 0.04, 0.05
                width: 0.3
                input_filter: 'int'
                multiline: False
                text: "0"    

            MDFloatingActionButton:
                pos_hint: {"center_x": 0.85, "center_y": 0.66}
                id: minusbutton
                icon: "minus"
                size_hint: 0.025, 0.03
                width: 0.2
                on_release: root.vel3_command_callback(-10)

            MDFloatingActionButton:
                pos_hint: {"center_x": 0.88, "center_y": 0.66}
                id: plusbutton
                icon: "plus"
                size_hint: 0.025, 0.03
                width: 0.2
                on_release: root.vel3_command_callback(10)

            Button:
                pos_hint: {"center_x": 0.915, "center_y": 0.66}
                size_hint: 0.04, 0.05
                id: pub3
                text: 'Pub'
                width: 0.3
                on_press:app.send_position("20", root.m3_pos.text, root.m3_vel.text, "pub3")

            #MOTOR 4
            MDCheckbox:
                id: m4_checkbox
                pos_hint: {"center_x": 0.96, "center_y": 0.62}
                size_hint: None, None
                size: "24dp", "24dp"
                on_active: app.show_sensor_info('22')

            Label:
                pos_hint: {"center_x": 0.525, "center_y": 0.62}
                font_size: '10sp'
                text: 'kn_p'
                id: m4_id
                color: (0,0,1,1)
                # canvas.before:
                #     Color:
                #         rgba: 1, 1, 1, 1
                #     Rectangle:
                #         pos: self.pos
                #         size: self.size
                width: 0.6
            
            TextInput:
                pos_hint: {"center_x": 0.575, "center_y": 0.62}
                id: m4_reading
                size_hint: 0.04, 0.05
                width: 1
                input_filter: 'int'
                text: "0"
                multiline: False
                readonly: True
                
            TextInput:
                pos_hint: {"center_x": 0.635, "center_y": 0.62}
                id: m4_textinput
                size_hint: 0.04, 0.05
                width: .05
                input_filter: 'int'
                text: "0"
                multiline: False
                on_text_validate: root.m4_command_callback(0)

            MDFloatingActionButton:
                pos_hint: {"center_x": 0.675, "center_y": 0.62}
                id: minusbutton
                icon: "minus"
                size_hint: 0.025, 0.03
                width: 0.2 
                on_release: root.m4_command_callback(-5)

            MDFloatingActionButton:
                pos_hint: {"center_x": 0.705, "center_y": 0.62}
                id: plusbutton
                icon: "plus"
                size_hint: 0.025, 0.03
                width: 0.2
                on_release: root.m4_command_callback(5)

            TextInput:
                pos_hint: {"center_x": 0.75, "center_y": 0.62}
                id: vel4_reading
                size_hint: 0.04, 0.05
                width: 1
                input_filter: 'int'
                text: "0"
                multiline: False
                readonly: True
            
            TextInput:
                pos_hint: {"center_x": 0.81, "center_y": 0.62}
                id: velo4_textinput
                size_hint: 0.04, 0.05
                width: 0.3
                input_filter: 'int'
                multiline: False
                text: "0"

            MDFloatingActionButton:
                pos_hint: {"center_x": 0.85, "center_y": 0.62}
                id: minusbutton
                icon: "minus"
                size_hint: 0.025, 0.03
                width: 0.2
                on_release: root.vel4_command_callback(-10)

            MDFloatingActionButton:
                pos_hint: {"center_x": 0.88, "center_y": 0.62}
                id: plusbutton
                icon: "plus"
                size_hint: 0.025, 0.03
                width: 0.2
                on_release: root.vel4_command_callback(10)

            Button:
                pos_hint: {"center_x": 0.915, "center_y": 0.62}
                size_hint: 0.04, 0.05
                id: pub4
                text: 'Pub'
                width: 0.3
                on_press:app.send_position("22", root.m4_pos.text, root.m4_vel.text, "pub4")

            #MOTOR 5
            MDCheckbox:
                id: m5_checkbox
                pos_hint: {"center_x": 0.96, "center_y": 0.58}
                size_hint: None, None
                size: "24dp", "24dp"
                on_active: app.show_sensor_info('24')

            Label:
                pos_hint: {"center_x": 0.525, "center_y": 0.58}
                font_size: '10sp'
                text: 'an_p'
                id: m5_id
                color: (0,0,1,1)
                # canvas.before:
                #     Color:
                #         rgba: 1, 1, 1, 1
                #     Rectangle:
                #         pos: self.pos
                #         size: self.size
                width: 0.6
            
            TextInput:
                pos_hint: {"center_x": 0.575, "center_y": 0.58}
                id: m5_reading
                size_hint: 0.04, 0.05
                width: 1
                input_filter: 'int'
                text: "0"
                multiline: False
                readonly: True
                
            TextInput:
                pos_hint: {"center_x": 0.635, "center_y": 0.58}
                id: m5_textinput
                size_hint: 0.04, 0.05
                width: .05
                input_filter: 'int'
                text: "0"
                multiline: False
                on_text_validate: root.m5_command_callback(0)

            MDFloatingActionButton:
                pos_hint: {"center_x": 0.675, "center_y": 0.58}
                id: minusbutton
                icon: "minus"
                size_hint: 0.025, 0.03
                width: 0.2 
                on_release: root.m5_command_callback(-5)

            MDFloatingActionButton:
                pos_hint: {"center_x": 0.705, "center_y": 0.58}
                id: plusbutton
                icon: "plus"
                size_hint: 0.025, 0.03
                width: 0.2
                on_release: root.m5_command_callback(5)

            TextInput:
                pos_hint: {"center_x": 0.75, "center_y": 0.58}
                id: vel5_reading
                size_hint: 0.04, 0.05
                width: 1
                input_filter: 'int'
                text: "0"
                multiline: False
                readonly: True
            
            TextInput:
                pos_hint: {"center_x": 0.81, "center_y": 0.58}
                id: velo5_textinput
                size_hint: 0.04, 0.05
                width: 0.3
                input_filter: 'int'
                multiline: False
                text: "0"

            MDFloatingActionButton:
                pos_hint: {"center_x": 0.85, "center_y": 0.58}
                id: minusbutton
                icon: "minus"
                size_hint: 0.025, 0.03
                width: 0.2
                on_release: root.vel5_command_callback(-10)

            MDFloatingActionButton:
                pos_hint: {"center_x": 0.88, "center_y": 0.58}
                id: plusbutton
                icon: "plus"
                size_hint: 0.025, 0.03
                width: 0.2
                on_release: root.vel5_command_callback(10)

            Button:
                pos_hint: {"center_x": 0.915, "center_y": 0.58}
                size_hint: 0.04, 0.05
                id: pub5
                text: 'Pub'
                width: 0.3
                on_press:app.send_position("24", root.m5_pos.text, root.m5_vel.text, "pub5")

            #MOTOR 6
            MDCheckbox:
                id: m6_checkbox
                pos_hint: {"center_x": 0.96, "center_y": 0.54}
                size_hint: None, None
                size: "24dp", "24dp"
                on_active: app.show_sensor_info('26')

            Label:
                pos_hint: {"center_x": 0.525, "center_y": 0.54}
                font_size: '10sp'
                text: 'an_r'
                id: m6_id
                color: (0,0,1,1)
                # canvas.before:
                #     Color:
                #         rgba: 1, 1, 1, 1
                #     Rectangle:
                #         pos: self.pos
                #         size: self.size
                width: 0.6
            
            TextInput:
                pos_hint: {"center_x": 0.575, "center_y": 0.54}
                id: m6_reading
                size_hint: 0.04, 0.05
                width: 1
                input_filter: 'int'
                text: "0"
                multiline: False
                readonly: True
                
            TextInput:
                pos_hint: {"center_x": 0.635, "center_y": 0.54}
                id: m6_textinput
                size_hint: 0.04, 0.05
                width: .05
                input_filter: 'int'
                text: "0"
                multiline: False
                on_text_validate: root.m6_command_callback(0)

            MDFloatingActionButton:
                pos_hint: {"center_x": 0.675, "center_y": 0.54}
                id: minusbutton
                icon: "minus"
                size_hint: 0.025, 0.03
                width: 0.2 
                on_release: root.m6_command_callback(-5)

            MDFloatingActionButton:
                pos_hint: {"center_x": 0.705, "center_y": 0.54}
                id: plusbutton
                icon: "plus"
                size_hint: 0.025, 0.03
                width: 0.2
                on_release: root.m6_command_callback(5)

            TextInput:
                pos_hint: {"center_x": 0.75, "center_y": 0.54}
                id: vel6_reading
                size_hint: 0.04, 0.05
                width: 1
                input_filter: 'int'
                text: "0"
                multiline: False
                readonly: True
            
            TextInput:
                pos_hint: {"center_x": 0.81, "center_y": 0.54}
                id: velo6_textinput
                size_hint: 0.04, 0.05
                width: 0.3
                input_filter: 'int'
                multiline: False
                text: "0"

            MDFloatingActionButton:
                pos_hint: {"center_x": 0.85, "center_y": 0.54}
                id: minusbutton
                icon: "minus"
                size_hint: 0.025, 0.03
                width: 0.2
                on_release: root.vel6_command_callback(-10)

            MDFloatingActionButton:
                pos_hint: {"center_x": 0.88, "center_y": 0.54}
                id: plusbutton
                icon: "plus"
                size_hint: 0.025, 0.03
                width: 0.2
                on_release: root.vel6_command_callback(10)

            Button:
                pos_hint: {"center_x": 0.915, "center_y": 0.54}
                size_hint: 0.04, 0.05
                id: pub6
                text: 'Pub'
                width: 0.3
                on_press:app.send_position("24", root.m6_pos.text, root.m6_vel.text, "pub6")

        # # # SENSORS
        GridLayout:
            pos_hint: {"center_x": 0.75, "center_y": 0.2}
            canvas.before:
                Color:
                    rgba: 0, 0, 0, 1
                Rectangle:
                    pos: self.pos
                    size: self.size
            cols: 3
            size_hint: None, None
            height: 100
            width: 275

            Label:
                text: "Voltage"
            Label:
                text: "0.0"
                id: voltage_id
                color: (0,0,1,1)
                canvas.before:
                    Color:
                        rgba: 1, 1, 1, 1
                    Rectangle:
                        pos: self.pos
                        size: self.size
            Label:
                text: "V"
            Label:
                text: "Current"
            Label:
                text: "0.0"
                id: current_id
                color: (0,0,1,1)
                canvas.before:
                    Color:
                        rgba: 1, 1, 1, 1
                    Rectangle:
                        pos: self.pos
                        size: self.size
            Label:
                text: "mA"
            Label:
                text: "Temperature"
            Label:
                text: "0.0"
                id: temperature_id
                color: (0,0,1,1)
                canvas.before:
                    Color:
                        rgba: 1, 1, 1, 1
                    Rectangle:
                        pos: self.pos
                        size: self.size
            Label:
                text: "C"
            
        MDTabs:
            text_color_active: [1, 1, 1, 1]
            size_hint: 1, 0.08
            tab_hint_x: True
            allow_stretch: True
            on_tab_switch: app.on_tab_switch(*args)
            Tab:
                title: '[b]LiDAR[/b]'
                icon: 'motion-sensor'
            Tab:
                title: '[b]RealSense[/b]'
                icon: 'camera-outline'
            Tab:
                title: '[b]IMU[/b]'
                icon: 'chart-waterfall'
            Tab:
                title: '[b]Force Torque[/b]'
                icon: 'cog'

<MenuRightLegScreen>:
    name: 'home_rightleg_control'
    img: rl_asyn_image
    m1_pos: m1_textinput
    m2_pos: m2_textinput
    m3_pos: m3_textinput
    m4_pos: m4_textinput
    m5_pos: m5_textinput
    m6_pos: m6_textinput

    m1_vel: velo1_textinput
    m2_vel: velo2_textinput
    m3_vel: velo3_textinput
    m4_vel: velo4_textinput
    m5_vel: velo5_textinput
    m6_vel: velo6_textinput

    FloatLayout:
        orientation: "vertical"

        MDTopAppBar:
            title: "[b]Humanoid Control Panel[/b]"
            anchor_title: "center"
            left_action_items: [["home-analytics", lambda x: app.set_screen('home')]]
            right_action_items: [["engine", lambda x: app.callback(x)], ["robot-confused-outline", lambda x: app.set_screen('id')], ["power", lambda x: app.callback_2()]]
            size_hint: 1, .1
            pos_hint: {"center_x": .5, "center_y": 0.95}
        
        AsyncImage:
            id: rl_asyn_image
            source: root.image_path
            size_hint: None, None
            width: 400
            height: 400
            pos_hint: {'center_x':.25, 'center_y': .6}
            nocache: True

        Label:
            pos_hint: {"center_x": 0.25, "center_y": 0.325}
            bold: True
            font_size: '20sp'
            text: 'LIVE URDF MODEL'
            size_hint_x: None
            width: 1
            color: (0,0,0,1)

        Label:
            pos_hint: {"center_x": 0.25, "center_y": 0.3}
            bold: True
            font_size: '12sp'
            text: 'POSITION IN DEGREES | VELOCITY IN ROTATIONS/S'
            size_hint_x: None
            width: 1
            color: (0,0,0,1)

        Button:
            id: emergency
            bold: True
            font_size: '18sp'
            text: "EMERGENCY STOP"
            background_normal: ''
            background_color: rgba("#ff0a01")
            color: 1,1,1,1
            halign: "center"
            # background_color: get_color_from_hex('#FF0A01')
            # background_color: 0,1,0,1
            size_hint: 0.2, 0.15
            pos_hint: {"center_x": .13, "center_y": 0.18}
            on_press: app.emergency_stop()

        Button:
            id: run_btn
            bold: True
            font_size: '20sp'
            text: "REFRESH"
            background_normal: ''
            background_color: rgba("#00ab66")
            color:1,1,1,1
            halign: "center"
            size_hint: 0.2, 0.15
            pos_hint: {"center_x": .35, "center_y": 0.18}
            on_press: root.change_image("new_robotImage.png")

        Label:
            pos_hint: {"center_x": 0.75, "center_y": 0.84}
            bold: True
            font_size: '20sp'
            text: 'RIGHT LEG MOTOR CONTROL PANEL'
            size_hint_x: None
            width: 1
            color: (0,0,0,1)

        MDCard:
            size_hint: .5, .4
            focus_behavior: True
            pos_hint: {"center_x": .74, "center_y": .6}
            md_bg_color: "lightgrey"
            unfocus_color: "lightgrey"
            focus_color: "grey"
            # elevation: 6

        Label:
            pos_hint: {"center_x": 0.75, "center_y": 0.35}
            bold: True
            font_size: '20sp'
            text: 'MOTOR SENSOR INFO'
            size_hint_x: None
            width: 1
            color: (0,0,0,1)

        FloatLayout:
            Label:
                pos_hint: {"center_x": 0.96, "center_y": 0.78}
                bold: True
                font_size: '10sp'
                text: 'SENSOR'
                width: 1
                color: (0,0,0,1)
            Label:
                pos_hint: {"center_x": 0.525, "center_y": 0.78}
                bold: True
                font_size: '10sp'
                text: 'ID'
                width: 0.5
                color: (0,0,0,1)
            Label:
                pos_hint: {"center_x": 0.575, "center_y": 0.78}
                bold: True
                font_size: '10sp'
                text: 'CUR POS' 
                size_hint: None, 1
                width: 10
                color: (0,0,0,1) 
            Label:
                pos_hint: {"center_x": 0.64, "center_y": 0.78}
                bold: True
                font_size: '10sp'
                text: 'POS CTRL' 
                width: 10
                color: (0,0,0,1) 
            Label:
                pos_hint: {"center_x": 0.675, "center_y": 0.78}
                bold: True
                font_size: '10sp'
                text: ' '  
                size_hint_x: None
                width: 1
                color: (0,0,0,1) 
            Label:
                pos_hint: {"center_x": 0.705, "center_y": 0.78}
                bold: True
                font_size: '10sp'
                text: ' '
                size_hint_x: None
                width: 1
                color: (0,0,0,1)
            Label:
                pos_hint: {"center_x": 0.75, "center_y": 0.78}
                bold: True
                font_size: '10sp'
                text: 'CUR VEL'
                size_hint_x: None
                width: 1
                color: (0,0,0,1) 
            Label:
                pos_hint: {"center_x": 0.815, "center_y": 0.78}
                bold: True
                font_size: '10sp'
                text: 'VEL CTRL'
                size_hint_x: None
                width: 1
                color: (0,0,0,1) 
            Label:
                pos_hint: {"center_x": 0.85, "center_y": 0.78}
                bold: True
                font_size: '10sp'
                text: ' ' 
                size_hint_x: None
                width: 1
                color: (0,0,0,1)
            Label:
                pos_hint: {"center_x": 0.88, "center_y": 0.78}
                bold: True
                font_size: '10sp'
                text: ' ' 
                size_hint_x: None
                width: 1
                color: (0,0,0,1)
            Label:
                pos_hint: {"center_x": 0.9125, "center_y": 0.78}
                bold: True
                font_size: '10sp'
                text: 'SEND'
                size_hint_x: None
                width: 1
                color: (0,0,0,1)

            MDCheckbox:
                id: m1_checkbox
                pos_hint: {"center_x": 0.96, "center_y": 0.74}
                size_hint: None, None
                size: "24dp", "24dp"
                on_active: app.show_sensor_info('15')

            Label:
                pos_hint: {"center_x": 0.525, "center_y": 0.74}
                font_size: '10sp'
                text: 'hip_y'
                id: m1_id
                color: (0,0,1,1)
                # canvas.before:
                #     Color:
                #         rgba: 1, 1, 1, 1
                #     Rectangle:
                #         pos: self.pos
                #         size: self.size
                width: 0.6
            
            TextInput:
                pos_hint: {"center_x": 0.575, "center_y": 0.74}
                id: m1_reading
                size_hint: 0.04, 0.05
                width: 1
                input_filter: 'int'
                text: "0"
                multiline: False
                readonly: True
                
            TextInput:
                pos_hint: {"center_x": 0.635, "center_y": 0.74}
                id: m1_textinput
                size_hint: 0.04, 0.05
                width: .05
                input_filter: 'int'
                text: "0"
                multiline: False
                on_text_validate: root.m1_command_callback(0)

            MDFloatingActionButton:
                pos_hint: {"center_x": 0.675, "center_y": 0.74}
                id: minusbutton
                icon: "minus"
                size_hint: 0.025, 0.03
                width: 0.2 
                on_release: root.m1_command_callback(-5)

            MDFloatingActionButton:
                pos_hint: {"center_x": 0.705, "center_y": 0.74}
                id: plusbutton
                icon: "plus"
                size_hint: 0.025, 0.03
                width: 0.2
                on_release: root.m1_command_callback(5)

            TextInput:
                pos_hint: {"center_x": 0.75, "center_y": 0.74}
                id: vel1_reading
                size_hint: 0.04, 0.05
                width: 1
                input_filter: 'int'
                text: "0"
                multiline: False
                readonly: True
            
            TextInput:
                pos_hint: {"center_x": 0.81, "center_y": 0.74}
                id: velo1_textinput
                size_hint: 0.04, 0.05
                width: 0.3
                input_filter: 'int'
                multiline: False
                text: "0"

            MDFloatingActionButton:
                pos_hint: {"center_x": 0.85, "center_y": 0.74}
                id: minusbutton
                icon: "minus"
                size_hint: 0.025, 0.03
                width: 0.2
                on_release: root.vel1_command_callback(-10)

            MDFloatingActionButton:
                pos_hint: {"center_x": 0.88, "center_y": 0.74}
                id: plusbutton
                icon: "plus"
                size_hint: 0.025, 0.03
                width: 0.2
                on_release: root.vel1_command_callback(10)

            Button:
                pos_hint: {"center_x": 0.915, "center_y": 0.74}
                size_hint: 0.04, 0.05
                id: pub1
                text: 'Pub'
                width: 0.3
                on_press:app.send_position("15", root.m1_pos.text, root.m1_vel.text, "pub1")

            #MOTOR 2
            MDCheckbox:
                id: m2_checkbox
                pos_hint: {"center_x": 0.96, "center_y": 0.70}
                size_hint: None, None
                size: "24dp", "24dp"
                on_active: app.show_sensor_info('17')

            Label:
                pos_hint: {"center_x": 0.525, "center_y": 0.70}
                font_size: '10sp'
                text: 'hip_r'
                id: m2_id
                color: (0,0,1,1)
                size_hint_x: None
                width: 0.6
                # canvas.before:
                #     Color:
                #         rgba: 1, 1, 1, 1
                #     Rectangle:
                #         pos: self.pos
                #         size: self.size

            TextInput:
                pos_hint: {"center_x": 0.575, "center_y": 0.70}
                id: m2_reading
                size_hint: 0.04, 0.05
                width: 0.3
                input_filter: 'int'
                text: "0"
                multiline: False
                readonly: True

            TextInput:
                pos_hint: {"center_x": 0.635, "center_y": 0.70}
                id: m2_textinput
                size_hint: 0.04, 0.05
                width: 0.3
                input_filter: 'int'
                text: "0"
                multiline: False
                on_text_validate: root.m2_command_callback(0)

            MDFloatingActionButton:
                pos_hint: {"center_x": 0.675, "center_y": 0.70}
                id: minusbutton
                icon: "minus"
                size_hint: 0.025, 0.03
                width: 0.2
                on_release: root.m2_command_callback(-5)

            MDFloatingActionButton:
                pos_hint: {"center_x": 0.705, "center_y": 0.70}
                id: plusbutton
                icon: "plus"
                size_hint: 0.025, 0.03
                width: 0.2
                on_release: root.m2_command_callback(5)

            TextInput:
                pos_hint: {"center_x": 0.75, "center_y": 0.70}
                id: vel2_reading
                size_hint: 0.04, 0.05
                width: 0.3
                input_filter: 'int'
                text: "0"
                multiline: False
                readonly: True

            TextInput:
                pos_hint: {"center_x": 0.81, "center_y": 0.70}
                id: velo2_textinput
                size_hint: 0.04, 0.05
                width: 0.3
                input_filter: 'int'
                multiline: False
                text: "0"    

            MDFloatingActionButton:
                pos_hint: {"center_x": 0.85, "center_y": 0.70}
                id: minusbutton
                icon: "minus"
                size_hint: 0.025, 0.03
                width: 0.2
                on_release: root.vel2_command_callback(-10)

            MDFloatingActionButton:
                pos_hint: {"center_x": 0.88, "center_y": 0.70}
                id: plusbutton
                icon: "plus"
                size_hint: 0.025, 0.03
                width: 0.2
                on_release: root.vel2_command_callback(10)

            Button:
                pos_hint: {"center_x": 0.915, "center_y": 0.70}
                size_hint: 0.04, 0.05
                id: pub2
                text: 'Pub'
                width: 0.3
                on_press: app.send_position("17", root.m2_pos.text, root.m2_vel.text, "pub2")

            #MOTOR 3
            MDCheckbox:
                id: m3_checkbox
                pos_hint: {"center_x": 0.96, "center_y": 0.66}
                size_hint: None, None
                size: "24dp", "24dp"
                on_active: app.show_sensor_info('19')

            Label:
                pos_hint: {"center_x": 0.525, "center_y": 0.66}
                font_size: '10sp'
                text: 'hip_p'
                id: m3_id
                color: (0,0,1,1)
                size_hint_x: None
                width: 0.6
                # canvas.before:
                #     Color:
                #         rgba: 1, 1, 1, 1
                #     Rectangle:
                #         pos: self.pos
                #         size: self.size

            TextInput:
                pos_hint: {"center_x": 0.575, "center_y": 0.66}
                id: m3_reading
                size_hint: 0.04, 0.05
                width: 0.3
                input_filter: 'int'
                text: "0"
                multiline: False
                readonly: True

            TextInput:
                pos_hint: {"center_x": 0.635, "center_y": 0.66}
                id: m3_textinput
                size_hint: 0.04, 0.05
                width: 0.3
                input_filter: 'int'
                text: "0"
                multiline: False
                on_text_validate: root.m3_command_callback(0)

            MDFloatingActionButton:
                pos_hint: {"center_x": 0.675, "center_y": 0.66}
                id: minusbutton
                icon: "minus"
                size_hint: 0.025, 0.03
                width: 0.2
                on_release: root.m3_command_callback(-5)

            MDFloatingActionButton:
                pos_hint: {"center_x": 0.705, "center_y": 0.66}
                id: plusbutton
                icon: "plus"
                size_hint: 0.025, 0.03
                width: 0.2
                on_release: root.m3_command_callback(5)

            TextInput:
                pos_hint: {"center_x": 0.75, "center_y": 0.66}
                id: vel3_reading
                size_hint: 0.04, 0.05
                width: 0.3
                input_filter: 'int'
                text: "0"
                multiline: False
                readonly: True

            TextInput:
                pos_hint: {"center_x": 0.81, "center_y": 0.66}
                id: velo3_textinput
                size_hint: 0.04, 0.05
                width: 0.3
                input_filter: 'int'
                multiline: False
                text: "0"    

            MDFloatingActionButton:
                pos_hint: {"center_x": 0.85, "center_y": 0.66}
                id: minusbutton
                icon: "minus"
                size_hint: 0.025, 0.03
                width: 0.2
                on_release: root.vel3_command_callback(-10)

            MDFloatingActionButton:
                pos_hint: {"center_x": 0.88, "center_y": 0.66}
                id: plusbutton
                icon: "plus"
                size_hint: 0.025, 0.03
                width: 0.2
                on_release: root.vel3_command_callback(10)

            Button:
                pos_hint: {"center_x": 0.915, "center_y": 0.66}
                size_hint: 0.04, 0.05
                id: pub3
                text: 'Pub'
                width: 0.3
                on_press:app.send_position("19", root.m3_pos.text, root.m3_vel.text, "pub3")

            #MOTOR 4
            MDCheckbox:
                id: m4_checkbox
                pos_hint: {"center_x": 0.96, "center_y": 0.62}
                size_hint: None, None
                size: "24dp", "24dp"
                on_active: app.show_sensor_info('21')

            Label:
                pos_hint: {"center_x": 0.525, "center_y": 0.62}
                font_size: '10sp'
                text: 'kn_p'
                id: m4_id
                color: (0,0,1,1)
                # canvas.before:
                #     Color:
                #         rgba: 1, 1, 1, 1
                #     Rectangle:
                #         pos: self.pos
                #         size: self.size
                width: 0.6
            
            TextInput:
                pos_hint: {"center_x": 0.575, "center_y": 0.62}
                id: m4_reading
                size_hint: 0.04, 0.05
                width: 1
                input_filter: 'int'
                text: "0"
                multiline: False
                readonly: True
                
            TextInput:
                pos_hint: {"center_x": 0.635, "center_y": 0.62}
                id: m4_textinput
                size_hint: 0.04, 0.05
                width: .05
                input_filter: 'int'
                text: "0"
                multiline: False
                on_text_validate: root.m4_command_callback(0)

            MDFloatingActionButton:
                pos_hint: {"center_x": 0.675, "center_y": 0.62}
                id: minusbutton
                icon: "minus"
                size_hint: 0.025, 0.03
                width: 0.2 
                on_release: root.m4_command_callback(-5)

            MDFloatingActionButton:
                pos_hint: {"center_x": 0.705, "center_y": 0.62}
                id: plusbutton
                icon: "plus"
                size_hint: 0.025, 0.03
                width: 0.2
                on_release: root.m4_command_callback(5)

            TextInput:
                pos_hint: {"center_x": 0.75, "center_y": 0.62}
                id: vel4_reading
                size_hint: 0.04, 0.05
                width: 1
                input_filter: 'int'
                text: "0"
                multiline: False
                readonly: True
            
            TextInput:
                pos_hint: {"center_x": 0.81, "center_y": 0.62}
                id: velo4_textinput
                size_hint: 0.04, 0.05
                width: 0.3
                input_filter: 'int'
                multiline: False
                text: "0"

            MDFloatingActionButton:
                pos_hint: {"center_x": 0.85, "center_y": 0.62}
                id: minusbutton
                icon: "minus"
                size_hint: 0.025, 0.03
                width: 0.2
                on_release: root.vel4_command_callback(-10)

            MDFloatingActionButton:
                pos_hint: {"center_x": 0.88, "center_y": 0.62}
                id: plusbutton
                icon: "plus"
                size_hint: 0.025, 0.03
                width: 0.2
                on_release: root.vel4_command_callback(10)

            Button:
                pos_hint: {"center_x": 0.915, "center_y": 0.62}
                size_hint: 0.04, 0.05
                id: pub4
                text: 'Pub'
                width: 0.3
                on_press:app.send_position("21", root.m4_pos.text, root.m4_vel.text, "pub4")

            #MOTOR 5
            MDCheckbox:
                id: m5_checkbox
                pos_hint: {"center_x": 0.96, "center_y": 0.58}
                size_hint: None, None
                size: "24dp", "24dp"
                on_active: app.show_sensor_info('23')

            Label:
                pos_hint: {"center_x": 0.525, "center_y": 0.58}
                font_size: '10sp'
                text: 'an_p'
                id: m5_id
                color: (0,0,1,1)
                # canvas.before:
                #     Color:
                #         rgba: 1, 1, 1, 1
                #     Rectangle:
                #         pos: self.pos
                #         size: self.size
                width: 0.6
            
            TextInput:
                pos_hint: {"center_x": 0.575, "center_y": 0.58}
                id: m5_reading
                size_hint: 0.04, 0.05
                width: 1
                input_filter: 'int'
                text: "0"
                multiline: False
                readonly: True
                
            TextInput:
                pos_hint: {"center_x": 0.635, "center_y": 0.58}
                id: m5_textinput
                size_hint: 0.04, 0.05
                width: .05
                input_filter: 'int'
                text: "0"
                multiline: False
                on_text_validate: root.m5_command_callback(0)

            MDFloatingActionButton:
                pos_hint: {"center_x": 0.675, "center_y": 0.58}
                id: minusbutton
                icon: "minus"
                size_hint: 0.025, 0.03
                width: 0.2 
                on_release: root.m5_command_callback(-5)

            MDFloatingActionButton:
                pos_hint: {"center_x": 0.705, "center_y": 0.58}
                id: plusbutton
                icon: "plus"
                size_hint: 0.025, 0.03
                width: 0.2
                on_release: root.m5_command_callback(5)

            TextInput:
                pos_hint: {"center_x": 0.75, "center_y": 0.58}
                id: vel5_reading
                size_hint: 0.04, 0.05
                width: 1
                input_filter: 'int'
                text: "0"
                multiline: False
                readonly: True
            
            TextInput:
                pos_hint: {"center_x": 0.81, "center_y": 0.58}
                id: velo5_textinput
                size_hint: 0.04, 0.05
                width: 0.3
                input_filter: 'int'
                multiline: False
                text: "0"

            MDFloatingActionButton:
                pos_hint: {"center_x": 0.85, "center_y": 0.58}
                id: minusbutton
                icon: "minus"
                size_hint: 0.025, 0.03
                width: 0.2
                on_release: root.vel5_command_callback(-10)

            MDFloatingActionButton:
                pos_hint: {"center_x": 0.88, "center_y": 0.58}
                id: plusbutton
                icon: "plus"
                size_hint: 0.025, 0.03
                width: 0.2
                on_release: root.vel5_command_callback(10)

            Button:
                pos_hint: {"center_x": 0.915, "center_y": 0.58}
                size_hint: 0.04, 0.05
                id: pub5
                text: 'Pub'
                width: 0.3
                on_press:app.send_position("23", root.m5_pos.text, root.m5_vel.text, "pub5")

            #MOTOR 6
            MDCheckbox:
                id: m6_checkbox
                pos_hint: {"center_x": 0.96, "center_y": 0.54}
                size_hint: None, None
                size: "24dp", "24dp"
                on_active: app.show_sensor_info('25')

            Label:
                pos_hint: {"center_x": 0.525, "center_y": 0.54}
                font_size: '10sp'
                text: 'an_r'
                id: m6_id
                color: (0,0,1,1)
                # canvas.before:
                #     Color:
                #         rgba: 1, 1, 1, 1
                #     Rectangle:
                #         pos: self.pos
                #         size: self.size
                width: 0.6
            
            TextInput:
                pos_hint: {"center_x": 0.575, "center_y": 0.54}
                id: m6_reading
                size_hint: 0.04, 0.05
                width: 1
                input_filter: 'int'
                text: "0"
                multiline: False
                readonly: True
                
            TextInput:
                pos_hint: {"center_x": 0.635, "center_y": 0.54}
                id: m6_textinput
                size_hint: 0.04, 0.05
                width: .05
                input_filter: 'int'
                text: "0"
                multiline: False
                on_text_validate: root.m6_command_callback(0)

            MDFloatingActionButton:
                pos_hint: {"center_x": 0.675, "center_y": 0.54}
                id: minusbutton
                icon: "minus"
                size_hint: 0.025, 0.03
                width: 0.2 
                on_release: root.m6_command_callback(-5)

            MDFloatingActionButton:
                pos_hint: {"center_x": 0.705, "center_y": 0.54}
                id: plusbutton
                icon: "plus"
                size_hint: 0.025, 0.03
                width: 0.2
                on_release: root.m6_command_callback(5)

            TextInput:
                pos_hint: {"center_x": 0.75, "center_y": 0.54}
                id: vel6_reading
                size_hint: 0.04, 0.05
                width: 1
                input_filter: 'int'
                text: "0"
                multiline: False
                readonly: True
            
            TextInput:
                pos_hint: {"center_x": 0.81, "center_y": 0.54}
                id: velo6_textinput
                size_hint: 0.04, 0.05
                width: 0.3
                input_filter: 'int'
                multiline: False
                text: "0"

            MDFloatingActionButton:
                pos_hint: {"center_x": 0.85, "center_y": 0.54}
                id: minusbutton
                icon: "minus"
                size_hint: 0.025, 0.03
                width: 0.2
                on_release: root.vel6_command_callback(-10)

            MDFloatingActionButton:
                pos_hint: {"center_x": 0.88, "center_y": 0.54}
                id: plusbutton
                icon: "plus"
                size_hint: 0.025, 0.03
                width: 0.2
                on_release: root.vel6_command_callback(10)

            Button:
                pos_hint: {"center_x": 0.915, "center_y": 0.54}
                size_hint: 0.04, 0.05
                id: pub6
                text: 'Pub'
                width: 0.3
                on_press:app.send_position("25", root.m6_pos.text, root.m6_vel.text, "pub6")

        # # # SENSORS
        GridLayout:
            pos_hint: {"center_x": 0.75, "center_y": 0.2}
            canvas.before:
                Color:
                    rgba: 0, 0, 0, 1
                Rectangle:
                    pos: self.pos
                    size: self.size
            cols: 3
            size_hint: None, None
            height: 100
            width: 275

            Label:
                text: "Voltage"
            Label:
                text: "0.0"
                id: voltage_id
                color: (0,0,1,1)
                canvas.before:
                    Color:
                        rgba: 1, 1, 1, 1
                    Rectangle:
                        pos: self.pos
                        size: self.size
            Label:
                text: "V"
            Label:
                text: "Current"
            Label:
                text: "0.0"
                id: current_id
                color: (0,0,1,1)
                canvas.before:
                    Color:
                        rgba: 1, 1, 1, 1
                    Rectangle:
                        pos: self.pos
                        size: self.size
            Label:
                text: "mA"
            Label:
                text: "Temperature"
            Label:
                text: "0.0"
                id: temperature_id
                color: (0,0,1,1)
                canvas.before:
                    Color:
                        rgba: 1, 1, 1, 1
                    Rectangle:
                        pos: self.pos
                        size: self.size
            Label:
                text: "C"

            
        MDTabs:
            text_color_active: [1, 1, 1, 1]
            size_hint: 1, 0.08
            tab_hint_x: True
            allow_stretch: True
            on_tab_switch: app.on_tab_switch(*args)
            Tab:
                title: '[b]LiDAR[/b]'
                icon: 'motion-sensor'
            Tab:
                title: '[b]RealSense[/b]'
                icon: 'camera-outline'
            Tab:
                title: '[b]IMU[/b]'
                icon: 'chart-waterfall'
            Tab:
                title: '[b]Force Torque[/b]'
                icon: 'cog'

<IDScreen>:
    name: 'id'
    FloatLayout:
        orientation: "vertical"

        MDTopAppBar:
            title: "Humanoid Control Panel"
            left_action_items: [["home-analytics", lambda x: app.set_screen('home')]]
            right_action_items: [["robot-confused-outline", lambda x: app.callback_1()], ["power", lambda x: app.callback_2()]]
            size_hint: 1, .1
            pos_hint: {"center_x": .5, "center_y": 0.95}

        # Image
        AsyncImage:
            source: 'robot_map.jpg'
            size_hint: 1, 1
            size_hint: None, None
            width: 625
            height: 625
            pos_hint: {'center_x':.50, 'center_y': .50}
        
        MDTabs:
            size_hint: 1, .1
            pos_hint: {"center_x": .5, "center_y": 0.01}
    
<LIDARScreen>:
    name: 'lidar'
    FloatLayout:
        orientation: "vertical"

        MDTopAppBar:
            title: "LIDAR Panel"
            left_action_items: [["home-analytics", lambda x: app.set_screen('home')]]
            right_action_items: [["engine", lambda x: app.callback(x)], ["robot-confused-outline", lambda x: app.set_screen('id')], ["power", lambda x: app.callback()]]
            size_hint: 1, .1
            pos_hint: {"center_x": .5, "center_y": 0.95}

        
        
        MDLabel:
            text: "LiDAR RVIZ"
            halign: "center"
        
        Button:
            id: emergency
            bold: True
            text: "EM-STOP"
            background_normal: ''
            background_color: rgba("#ff0a01")
            color: 1,1,1,1
            halign: "center"
            # background_color: get_color_from_hex('#FF0A01')
            # background_color: 0,1,0,1
            size_hint: 0.15, 0.1
            pos_hint: {"center_x": .2, "center_y": 0.2}
            on_press: app.emergency_stop()
  
        MDTabs:
            text_color_active: [1, 1, 1, 1]
            size_hint: 1, 0.08
            tab_hint_x: True
            allow_stretch: True
            on_tab_switch: app.on_tab_switch(*args)
            Tab:
                title: '[b]LiDAR[/b]'
                text: 'lidar'
                icon: 'motion-sensor'
            Tab:
                title: '[b]RealSense[/b]'
                text: 'realsense'
                icon: 'camera-outline'
            Tab:
                title: '[b]IMU[/b]'
                text: 'imu'
                icon: 'chart-waterfall'
            Tab:
                title: '[b]Force Torque[/b]'
                text: 'ft'
                icon: 'cog'

<RealsenseScreen>:
    name: 'realsense'
    FloatLayout:
        orientation: "vertical"

        MDTopAppBar:
            title: "Realsense Panel"
            left_action_items: [["home-analytics", lambda x: app.set_screen('home')]]
            right_action_items: [["engine", lambda x: app.callback(x)], ["robot-confused-outline", lambda x: app.set_screen('id')], ["power", lambda x: app.callback()]]
            size_hint: 1, .1
            pos_hint: {"center_x": .5, "center_y": 0.95}

        MDLabel:
            text: "Realsense RVIZ"
            halign: "center"

        Button:
            id: emergency
            bold: True
            text: "EM-STOP"
            background_normal: ''
            background_color: rgba("#ff0a01")
            color: 1,1,1,1
            halign: "center"
            # background_color: get_color_from_hex('#FF0A01')
            # background_color: 0,1,0,1
            size_hint: 0.15, 0.1
            pos_hint: {"center_x": .2, "center_y": 0.2}
            on_press: app.emergency_stop()
  
        MDTabs:
            text_color_active: [1, 1, 1, 1]
            size_hint: 1, 0.08
            tab_hint_x: True
            allow_stretch: True
            on_tab_switch: app.on_tab_switch(*args)
            Tab:
                title: '[b]LiDAR[/b]'
                text: 'lidar'
                icon: 'motion-sensor'
            Tab:
                title: '[b]RealSense[/b]'
                text: 'realsense'
                icon: 'camera-outline'
            Tab:
                title: '[b]IMU[/b]'
                text: 'imu'
                icon: 'chart-waterfall'
            Tab:
                title: '[b]Force Torque[/b]'
                text: 'ft'
                icon: 'cog'

<IMUScreen>:
    name: 'imu'
    FloatLayout:
        orientation: "vertical"

        MDTopAppBar:
            title: "IMU Panel"
            left_action_items: [["home-analytics", lambda x: app.set_screen('home')]]
            right_action_items: [["engine", lambda x: app.callback(x)], ["robot-confused-outline", lambda x: app.set_screen('id')], ["power", lambda x: app.callback_2()]]
            size_hint: 1, .1
            pos_hint: {"center_x": .5, "center_y": 0.95}
        
        MDLabel:
            text: "IMU Log/readings"
            halign: "center"

        GridLayout:
            canvas.before:
                Color:
                    rgba: 0, 0, 0, 1
                Rectangle:
                    pos: self.pos
                    size: self.size
            cols: 3
            size_hint: None, None
            height: 100
            width: 400
            pos_hint: {'center_x':0.50, 'center_y':0.50}

            Button:
                id: imu_linear_acc
                text: 'Linear Acc'
                on_press:app.linear_acc(imu_linear_acc.text)
            Button:
                id: imu_angular_vel
                text: 'Angular velocity'
                on_press:app.angular_vel(imu_angular_vel.text)
            Button:
                id: IMU_rviz
                text: 'IMU rviz'
                on_press:app.imu_rviz(IMU_rviz.text)

        Button:
            id: emergency
            bold: True
            text: "EM-STOP"
            background_normal: ''
            background_color: rgba("#ff0a01")
            color: 1,1,1,1
            halign: "center"
            # background_color: get_color_from_hex('#FF0A01')
            # background_color: 0,1,0,1
            size_hint: 0.15, 0.1
            pos_hint: {"center_x": .2, "center_y": 0.2}
            on_press: app.emergency_stop()
  
        MDTabs:
            text_color_active: [1, 1, 1, 1]
            size_hint: 1, 0.08
            tab_hint_x: True
            allow_stretch: True
            on_tab_switch: app.on_tab_switch(*args)
            Tab:
                title: '[b]LiDAR[/b]'
                text: 'lidar'
                icon: 'motion-sensor'
            Tab:
                title: '[b]RealSense[/b]'
                text: 'realsense'
                icon: 'camera-outline'
            Tab:
                title: '[b]IMU[/b]'
                text: 'imu'
                icon: 'chart-waterfall'
            Tab:
                title: '[b]Force Torque[/b]'
                text: 'ft'
                icon: 'cog'

<FTScreen>:
    name: 'ft'
    FloatLayout:
        orientation: "vertical"

        MDTopAppBar:
            title: "Force-Torque Sensor Panel"
            left_action_items: [["home-analytics", lambda x: app.set_screen('home')]]
            right_action_items: [["engine", lambda x: app.callback(x)], ["robot-confused-outline", lambda x: app.set_screen('id')], ["power", lambda x: app.callback_2()]]
            size_hint: 1, .1
            pos_hint: {"center_x": .5, "center_y": 0.95}
        
        MDLabel:
            text: "Force Torque Data Log/readings"
            halign: "center"

        Button:
            id: emergency
            bold: True
            text: "EM-STOP"
            background_normal: ''
            background_color: rgba("#ff0a01")
            color: 1,1,1,1
            halign: "center"
            # background_color: get_color_from_hex('#FF0A01')
            # background_color: 0,1,0,1
            size_hint: 0.15, 0.1
            pos_hint: {"center_x": .2, "center_y": 0.2}
            on_press: app.emergency_stop()
  
        MDTabs:
            text_color_active: [1, 1, 1, 1]
            size_hint: 1, 0.08
            tab_hint_x: True
            allow_stretch: True
            on_tab_switch: app.on_tab_switch(*args)
            Tab:
                title: '[b]LiDAR[/b]'
                text: 'lidar'
                icon: 'motion-sensor'
            Tab:
                title: '[b]RealSense[/b]'
                text: 'realsense'
                icon: 'camera-outline'
            Tab:
                title: '[b]IMU[/b]'
                text: 'imu'
                icon: 'chart-waterfall'
            Tab:
                title: '[b]Force Torque[/b]'
                text: 'ft'
                icon: 'cog'
"""

class RobotJointState():
    joint_state = JointState()

    def __init__(self, **kwargs):
        self.joint_state.name.append('r_arm_sh_p1')
        self.joint_state.name.append('r_arm_sh_r')
        self.joint_state.name.append('r_arm_sh_p2')
        self.joint_state.name.append('r_arm_el_y')
        self.joint_state.name.append('r_arm_wr_r')
        self.joint_state.name.append('r_arm_wr_y')
        self.joint_state.name.append('r_arm_wr_p')
        self.joint_state.name.append('r_arm_grip_thumb')
        self.joint_state.name.append('r_arm_grip_index')
        self.joint_state.name.append('r_arm_grip_middle')

        self.joint_state.name.append('l_arm_sh_p1')
        self.joint_state.name.append('l_arm_sh_r')
        self.joint_state.name.append('l_arm_sh_p2')
        self.joint_state.name.append('l_arm_el_y')
        self.joint_state.name.append('l_arm_wr_r')
        self.joint_state.name.append('l_arm_wr_y')
        self.joint_state.name.append('l_arm_wr_p')
        self.joint_state.name.append('l_arm_grip_thumb')
        self.joint_state.name.append('l_arm_grip_index')
        self.joint_state.name.append('l_arm_grip_middle')
        
        self.joint_state.name.append('head_y')
        self.joint_state.name.append('head_p')
        self.joint_state.name.append('r_antenna')
        self.joint_state.name.append('l_antenna')
        self.joint_state.name.append('torso_y')

        self.joint_state.name.append('r_leg_hip_r')
        self.joint_state.name.append('r_leg_hip_y')
        self.joint_state.name.append('r_leg_hip_p')
        self.joint_state.name.append('r_leg_kn_p')
        self.joint_state.name.append('r_leg_an_p')
        self.joint_state.name.append('r_leg_an_r')
        self.joint_state.name.append('l_leg_hip_r')
        self.joint_state.name.append('l_leg_hip_y')
        self.joint_state.name.append('l_leg_hip_p')
        self.joint_state.name.append('l_leg_kn_p')
        self.joint_state.name.append('l_leg_an_p')
        self.joint_state.name.append('l_leg_an_r')

        for x in range(37):
            self.joint_state.position.append(0.0)

        self.joint_state.velocity = []
        self.joint_state.effort = []

    def copy(self, msg):
        self.joint_state.name = msg.name
        for x in range(37):
            self.joint_state.position[x] = msg.position[x]
        self.joint_state.velocity = msg.velocity
        self.joint_state.effort = msg.effort
        self.joint_state.header = msg.header
    
    def set_pos(self, id, val):
        self.joint_state.position[id] = val


#pub for joint state publisher
joint_pub = rospy.Publisher("joint_states", JointState, queue_size=10)
                                            
#kill rviz joint state publisher node
os.system("rosnode kill /joint_state_publisher")

class Tab(MDFloatLayout, MDTabsBase):
    pass

class MenuScreen(Screen):
    url = 'http://localhost:8080/snapshot?topic=/rviz1/camera1/image&type=png'
    image_path = StringProperty("robotImage.png")
    urllib.request.urlretrieve(url, "robotImage.png")
    img = ObjectProperty(source = "robotImage.png")
    pass

    def __init__(self, **kwargs):
        super(MenuScreen, self).__init__(**kwargs)
        
    def change_image(self, path):
        urllib.request.urlretrieve(self.url, path)
        os.system("mv new_robotImage.png robotImage.png")
        self.img.reload()

class MenuHeadScreen(Screen):
    #image info
    url = 'http://localhost:8080/snapshot?topic=/rviz1/camera1/image&type=png'
    image_path = StringProperty("robotImage.png")
    urllib.request.urlretrieve(url, "robotImage.png")
    img = ObjectProperty(source = "robotImage.png")

    rjs = RobotJointState()

    #r_antenna properties
    m1_pos = ObjectProperty(None)
    m1_vel = ObjectProperty(None)

    #l_antenna properties
    m2_pos = ObjectProperty(None)
    m2_vel = ObjectProperty(None)
    pass

    def __init__(self, **kwargs):
        super(MenuHeadScreen, self).__init__(**kwargs)
        self.command_sub = rospy.Subscriber("joint_states", JointState,
                                            self.update_rjs, queue_size=10)
        
    def change_image(self, path):
        urllib.request.urlretrieve(self.url, path)
        os.system("mv new_robotImage.png robotImage.png")
        self.img.reload()

    def update_rjs(self, msg):
        self.rjs.copy(msg)
        
    def m1_command_callback(self, val):
        curr = self.m1_pos.text
        self.m1_pos.text = str(int(curr) + val)
        self.rjs.joint_state.position[22] = int(self.m1_pos.text) * math.pi/180
        self.rjs.joint_state.header.stamp = rospy.Time.now()
        self.rjs.joint_state.header.seq += 1
        joint_pub.publish(self.rjs.joint_state)
        self.change_image("new_robotImage.png")
    
    def m2_command_callback(self, val):
        curr = self.m2_pos.text
        self.m2_pos.text = str(int(curr) + val)
        self.rjs.joint_state.position[23] = int(self.m2_pos.text) * math.pi/180
        self.rjs.joint_state.header.stamp = rospy.Time.now()
        self.rjs.joint_state.header.seq += 1
        joint_pub.publish(self.rjs.joint_state)
        self.change_image("new_robotImage.png")

    def vel1_command_callback(self, val):
        curr = self.m1_vel.text 
        self.m1_vel.text = str(int(curr) + val)

    def vel2_command_callback(self, val):
        curr = self.m2_vel.text 
        self.m2_vel.text = str(int(curr) + val)
    
    def buton(self):
        print("-----------BUTTON PRESSED----------")

class MenuTorsoScreen(Screen):
    url = 'http://localhost:8080/snapshot?topic=/rviz1/camera1/image&type=png'
    image_path = StringProperty("robotImage.png")
    urllib.request.urlretrieve(url, "robotImage.png")
    img = ObjectProperty(source = "robotImage.png")
    
    rjs = RobotJointState()

    #torso_y properties
    m1_pos = ObjectProperty(None)

    #head_y properties
    m2_pos = ObjectProperty(None)

    #head_p properties
    m3_pos = ObjectProperty(None)

    m1_vel = ObjectProperty(None)
    m2_vel = ObjectProperty(None)
    m3_vel = ObjectProperty(None)

    pass

    def __init__(self, **kwargs):
        super(MenuTorsoScreen, self).__init__(**kwargs)
        self.command_sub = rospy.Subscriber("joint_states", JointState,
                                            self.update_rjs, queue_size=10)
        
    def change_image(self, path):
        urllib.request.urlretrieve(self.url, path)
        os.system("mv new_robotImage.png robotImage.png")
        self.img.reload()

    def update_rjs(self, msg):
        self.rjs.copy(msg)
        
    def m1_command_callback(self, val):
        curr = self.m1_pos.text
        self.m1_pos.text = str(int(curr) + val)
        self.rjs.joint_state.position[24] = int(self.m1_pos.text) * math.pi/180
        self.rjs.joint_state.header.stamp = rospy.Time.now()
        self.rjs.joint_state.header.seq += 1
        joint_pub.publish(self.rjs.joint_state)
        self.change_image("new_robotImage.png")
    
    def m2_command_callback(self, val):
        curr = self.m2_pos.text
        self.m2_pos.text = str(int(curr) + val)
        self.rjs.joint_state.position[20] = int(self.m2_pos.text) * math.pi/180
        self.rjs.joint_state.header.stamp = rospy.Time.now()
        self.rjs.joint_state.header.seq += 1
        joint_pub.publish(self.rjs.joint_state)
        self.change_image("new_robotImage.png")

    def m3_command_callback(self, val):
        curr = self.m3_pos.text
        self.m3_pos.text = str(int(curr) + val)
        self.rjs.joint_state.position[21] = int(self.m3_pos.text)* math.pi/180
        self.rjs.joint_state.header.stamp = rospy.Time.now()
        self.rjs.joint_state.header.seq += 1
        joint_pub.publish(self.rjs.joint_state)
        self.change_image("new_robotImage.png")
    
    def vel1_command_callback(self, val):
        curr = self.m1_vel.text 
        self.m1_vel.text = str(int(curr) + val)

    def vel2_command_callback(self, val):
        curr = self.m2_vel.text 
        self.m2_vel.text = str(int(curr) + val)

    def vel3_command_callback(self, val):
        curr = self.m3_vel.text 
        self.m3_vel.text = str(int(curr) + val)
    

class MenuLeftArmScreen(Screen):
    url = 'http://localhost:8080/snapshot?topic=/rviz1/camera1/image&type=png'
    image_path = StringProperty("robotImage.png")
    urllib.request.urlretrieve(url, "robotImage.png")
    img = ObjectProperty(source = "robotImage.png")
    
    rjs = RobotJointState()

    #l_arm_sh_p1 properties
    m1_pos = ObjectProperty(None)
    m1_vel = ObjectProperty(None)

    #l_arm_sh_r properties
    m2_pos = ObjectProperty(None)
    m2_vel = ObjectProperty(None)

    #l_arm_sh_p2 properties
    m3_pos = ObjectProperty(None)
    m3_vel = ObjectProperty(None)

    #l_arm_el_y properties
    m4_pos = ObjectProperty(None)
    m4_vel = ObjectProperty(None)

    #l_arm_wr_r properties
    m5_pos = ObjectProperty(None)
    m5_vel = ObjectProperty(None)

    #l_arm_wr_y properties
    m6_pos = ObjectProperty(None)
    m6_vel = ObjectProperty(None)

    #l_arm_wr_p properties
    m7_pos = ObjectProperty(None)
    m7_vel = ObjectProperty(None)

    #l_arm_grip_thumb properties
    m8_pos = ObjectProperty(None)
    m8_vel = ObjectProperty(None)

    #l_arm_grip_index properties
    m9_pos = ObjectProperty(None)
    m9_vel = ObjectProperty(None)

    #l_arm_grip_middle properties
    m10_pos = ObjectProperty(None)
    m10_vel = ObjectProperty(None)

    #sensor vales
    voltage = ObjectProperty(None)
    current = ObjectProperty(None)
    temperature = ObjectProperty(None)

    #check boxes
    check1 = ObjectProperty(None)
    check2 = ObjectProperty(None)
    check3 = ObjectProperty(None)
    check4 = ObjectProperty(None)
    check5 = ObjectProperty(None)
    check6 = ObjectProperty(None)
    check7 = ObjectProperty(None)
    check8 = ObjectProperty(None)
    check9 = ObjectProperty(None)
    check10 = ObjectProperty(None)
    pass

    def __init__(self, **kwargs):
        super(MenuLeftArmScreen, self).__init__(**kwargs)
        self.command_sub = rospy.Subscriber("joint_states", JointState,
                                            self.update_rjs, queue_size=10)
        
    def change_image(self, path):
        urllib.request.urlretrieve(self.url, path)
        os.system("mv new_robotImage.png robotImage.png")
        self.img.reload()

    def update_rjs(self, msg):
        self.rjs.copy(msg)
        
    def m1_command_callback(self, val):
        curr = self.m1_pos.text
        self.m1_pos.text = str(int(curr) + val)
        self.rjs.joint_state.position[10] = int(self.m1_pos.text) * math.pi/180
        self.rjs.joint_state.header.stamp = rospy.Time.now()
        self.rjs.joint_state.header.seq += 1
        joint_pub.publish(self.rjs.joint_state)
        self.change_image("new_robotImage.png")
    
    def m2_command_callback(self, val):
        curr = self.m2_pos.text
        self.m2_pos.text = str(int(curr) + val)
        self.rjs.joint_state.position[11] = int(self.m2_pos.text) * math.pi/180
        self.rjs.joint_state.header.stamp = rospy.Time.now()
        self.rjs.joint_state.header.seq += 1
        joint_pub.publish(self.rjs.joint_state)
        self.change_image("new_robotImage.png")

    def m3_command_callback(self, val):
        curr = self.m3_pos.text
        self.m3_pos.text = str(int(curr) + val)
        self.rjs.joint_state.position[12] = int(self.m3_pos.text) * math.pi/180
        self.rjs.joint_state.header.stamp = rospy.Time.now()
        self.rjs.joint_state.header.seq += 1
        joint_pub.publish(self.rjs.joint_state)
        self.change_image("new_robotImage.png")
    
    def m4_command_callback(self, val):
        curr = self.m4_pos.text
        self.m4_pos.text = str(int(curr) + val)
        self.rjs.joint_state.position[13] = int(self.m4_pos.text) * math.pi/180
        self.rjs.joint_state.header.stamp = rospy.Time.now()
        self.rjs.joint_state.header.seq += 1
        joint_pub.publish(self.rjs.joint_state)
        self.change_image("new_robotImage.png")

    def m5_command_callback(self, val):
        curr = self.m5_pos.text
        self.m5_pos.text = str(int(curr) + val)
        self.rjs.joint_state.position[14] = int(self.m5_pos.text) * math.pi/180
        self.rjs.joint_state.header.stamp = rospy.Time.now()
        self.rjs.joint_state.header.seq += 1
        joint_pub.publish(self.rjs.joint_state)
        self.change_image("new_robotImage.png")
    
    def m6_command_callback(self, val):
        curr = self.m6_pos.text
        self.m6_pos.text = str(int(curr) + val)
        self.rjs.joint_state.position[15] = int(self.m6_pos.text) * math.pi/180
        self.rjs.joint_state.header.stamp = rospy.Time.now()
        self.rjs.joint_state.header.seq += 1
        joint_pub.publish(self.rjs.joint_state)
        self.change_image("new_robotImage.png")

    def m7_command_callback(self, val):
        curr = self.m7_pos.text
        self.m7_pos.text = str(int(curr) + val)
        self.rjs.joint_state.position[16] = int(self.m7_pos.text) * math.pi/180
        self.rjs.joint_state.header.stamp = rospy.Time.now()
        self.rjs.joint_state.header.seq += 1
        joint_pub.publish(self.rjs.joint_state)
        self.change_image("new_robotImage.png")
    
    def m8_command_callback(self, val):
        curr = self.m8_pos.text
        self.m8_pos.text = str(int(curr) + val)
        self.rjs.joint_state.position[17] = int(self.m8_pos.text) * math.pi/180
        self.rjs.joint_state.header.stamp = rospy.Time.now()
        self.rjs.joint_state.header.seq += 1
        joint_pub.publish(self.rjs.joint_state)
        self.change_image("new_robotImage.png")

    def m9_command_callback(self, val):
        curr = self.m9_pos.text
        self.m9_pos.text = str(int(curr) + val)
        self.rjs.joint_state.position[18] = int(self.m9_pos.text) * math.pi/180
        self.rjs.joint_state.header.stamp = rospy.Time.now()
        self.rjs.joint_state.header.seq += 1
        joint_pub.publish(self.rjs.joint_state)
        self.change_image("new_robotImage.png")
    
    def m10_command_callback(self, val):
        curr = self.m10_pos.text
        self.m10_pos.text = str(int(curr) + val)
        self.rjs.joint_state.position[19] = int(self.m10_pos.text) * math.pi/180
        self.rjs.joint_state.header.stamp = rospy.Time.now()
        self.rjs.joint_state.header.seq += 1
        joint_pub.publish(self.rjs.joint_state)
        self.change_image("new_robotImage.png")

    def vel1_command_callback(self, val):
        curr = self.m1_vel.text 
        self.m1_vel.text = str(int(curr) + val)

    def vel2_command_callback(self, val):
        curr = self.m2_vel.text 
        self.m2_vel.text = str(int(curr) + val)

    def vel3_command_callback(self, val):
        curr = self.m3_vel.text 
        self.m3_vel.text = str(int(curr) + val)
    
    def vel4_command_callback(self, val):
        curr = self.m4_vel.text 
        self.m4_vel.text = str(int(curr) + val)

    def vel5_command_callback(self, val):
        curr = self.m5_vel.text 
        self.m5_vel.text = str(int(curr) + val)

    def vel6_command_callback(self, val):
        curr = self.m6_vel.text 
        self.m6_vel.text = str(int(curr) + val)
    
    def vel7_command_callback(self, val):
        curr = self.m7_vel.text 
        self.m7_vel.text = str(int(curr) + val)

    def vel8_command_callback(self, val):
        curr = self.m8_vel.text 
        self.m8_vel.text = str(int(curr) + val)

    def vel9_command_callback(self, val):
        curr = self.m9_vel.text 
        self.m9_vel.text = str(int(curr) + val)
    
    def vel10_command_callback(self, val):
        curr = self.m10_vel.text 
        self.m10_vel.text = str(int(curr) + val)

    def show_sensor_info(self, app, id, check):
        self.unselect(check)
        vals = app.show_sensor_info(id)
        self.voltage.text = str(vals[0])
        self.current.text = str(vals[1])
        self.temperature.text = str(vals[2])

    def unselect(self, id):
        if id != str(1):
            self.check1.disabled = True
        
        if id != str(2):
            self.check2.disabled = True
       
        if id != str(3):
            self.check3.disabled = True
        
        if id != str(4):
            self.check4.disabled = True
        
        if id != str(5):
            self.check5.disabled = True
        
        if id != str(6):
            self.check6.disabled = True
        
        if id != str(7):
            self.check7.disabled = True
        
        if id != str(8):
            self.check8.disabled = True
        
        if id != str(9):
            self.check9.disabled = True
        
        if id != str(10):
            self.check10.disabled = True

class MenuRightArmScreen(Screen):
    url = 'http://localhost:8080/snapshot?topic=/rviz1/camera1/image&type=png'
    image_path = StringProperty("robotImage.png")
    urllib.request.urlretrieve(url, "robotImage.png")
    img = ObjectProperty(source = "robotImage.png")
    
    rjs = RobotJointState()

    #r_arm_sh_p1 properties
    m1_pos = ObjectProperty(None)
    m1_vel = ObjectProperty(None)

    #r_arm_sh_r properties
    m2_pos = ObjectProperty(None)
    m2_vel = ObjectProperty(None)

    #r_arm_sh_p2 properties
    m3_pos = ObjectProperty(None)
    m3_vel = ObjectProperty(None)

    #r_arm_el_y properties
    m4_pos = ObjectProperty(None)
    m4_vel = ObjectProperty(None)

    #r_arm_wr_r properties
    m5_pos = ObjectProperty(None)
    m5_vel = ObjectProperty(None)

    #r_arm_wr_y properties
    m6_pos = ObjectProperty(None)
    m6_vel = ObjectProperty(None)

    #r_arm_wr_p properties
    m7_pos = ObjectProperty(None)
    m7_vel = ObjectProperty(None)

    #r_arm_grip_thumb properties
    m8_pos = ObjectProperty(None)
    m8_vel = ObjectProperty(None)

    #r_arm_grip_index properties
    m9_pos = ObjectProperty(None)
    m9_vel = ObjectProperty(None)

    #r_arm_grip_middle properties
    m10_pos = ObjectProperty(None)
    m10_vel = ObjectProperty(None)

    pass

    def __init__(self, **kwargs):
        super(MenuRightArmScreen, self).__init__(**kwargs)
        self.command_sub = rospy.Subscriber("joint_states", JointState,
                                            self.update_rjs, queue_size=10)
        
    def change_image(self, path):
        urllib.request.urlretrieve(self.url, path)
        os.system("mv new_robotImage.png robotImage.png")
        self.img.reload()
    
    def update_rjs(self, msg):
        self.rjs.copy(msg)
        
    def m1_command_callback(self, val):
        curr = self.m1_pos.text
        self.m1_pos.text = str(int(curr) + val)
        self.rjs.joint_state.position[10] = int(self.m1_pos.text) * math.pi/180
        self.rjs.joint_state.header.stamp = rospy.Time.now()
        self.rjs.joint_state.header.seq += 1
        joint_pub.publish(self.rjs.joint_state)
        self.change_image("new_robotImage.png")
    
    def m2_command_callback(self, val):
        curr = self.m2_pos.text
        self.m2_pos.text = str(int(curr) + val)
        self.rjs.joint_state.position[11] = int(self.m2_pos.text) * math.pi/180
        self.rjs.joint_state.header.stamp = rospy.Time.now()
        self.rjs.joint_state.header.seq += 1
        joint_pub.publish(self.rjs.joint_state)
        self.change_image("new_robotImage.png")

    def m3_command_callback(self, val):
        curr = self.m3_pos.text
        self.m3_pos.text = str(int(curr) + val)
        self.rjs.joint_state.position[12] = int(self.m3_pos.text) * math.pi/180
        self.rjs.joint_state.header.stamp = rospy.Time.now()
        self.rjs.joint_state.header.seq += 1
        joint_pub.publish(self.rjs.joint_state)
        self.change_image("new_robotImage.png")
    
    def m4_command_callback(self, val):
        curr = self.m4_pos.text
        self.m4_pos.text = str(int(curr) + val)
        self.rjs.joint_state.position[13] = int(self.m4_pos.text) * math.pi/180
        self.rjs.joint_state.header.stamp = rospy.Time.now()
        self.rjs.joint_state.header.seq += 1
        joint_pub.publish(self.rjs.joint_state)
        self.change_image("new_robotImage.png")

    def m5_command_callback(self, val):
        curr = self.m5_pos.text
        self.m5_pos.text = str(int(curr) + val)
        self.rjs.joint_state.position[14] = int(self.m5_pos.text) * math.pi/180
        self.rjs.joint_state.header.stamp = rospy.Time.now()
        self.rjs.joint_state.header.seq += 1
        joint_pub.publish(self.rjs.joint_state)
        self.change_image("new_robotImage.png")
    
    def m6_command_callback(self, val):
        curr = self.m6_pos.text
        self.m6_pos.text = str(int(curr) + val)
        self.rjs.joint_state.position[15] = int(self.m6_pos.text) * math.pi/180
        self.rjs.joint_state.header.stamp = rospy.Time.now()
        self.rjs.joint_state.header.seq += 1
        joint_pub.publish(self.rjs.joint_state)
        self.change_image("new_robotImage.png")

    def m7_command_callback(self, val):
        curr = self.m7_pos.text
        self.m7_pos.text = str(int(curr) + val)
        self.rjs.joint_state.position[16] = int(self.m7_pos.text) * math.pi/180
        self.rjs.joint_state.header.stamp = rospy.Time.now()
        self.rjs.joint_state.header.seq += 1
        joint_pub.publish(self.rjs.joint_state)
        self.change_image("new_robotImage.png")
    
    def m8_command_callback(self, val):
        curr = self.m8_pos.text
        self.m8_pos.text = str(int(curr) + val)
        self.rjs.joint_state.position[17] = int(self.m8_pos.text) * math.pi/180
        self.rjs.joint_state.header.stamp = rospy.Time.now()
        self.rjs.joint_state.header.seq += 1
        joint_pub.publish(self.rjs.joint_state)
        self.change_image("new_robotImage.png")

    def m9_command_callback(self, val):
        curr = self.m9_pos.text
        self.m9_pos.text = str(int(curr) + val)
        self.rjs.joint_state.position[18] = int(self.m9_pos.text) * math.pi/180
        self.rjs.joint_state.header.stamp = rospy.Time.now()
        self.rjs.joint_state.header.seq += 1
        joint_pub.publish(self.rjs.joint_state)
        self.change_image("new_robotImage.png")
    
    def m10_command_callback(self, val):
        curr = self.m10_pos.text
        self.m10_pos.text = str(int(curr) + val)
        self.rjs.joint_state.position[19] = int(self.m10_pos.text) * math.pi/180
        self.rjs.joint_state.header.stamp = rospy.Time.now()
        self.rjs.joint_state.header.seq += 1
        joint_pub.publish(self.rjs.joint_state)
        self.change_image("new_robotImage.png")

    def vel1_command_callback(self, val):
        curr = self.m1_vel.text 
        self.m1_vel.text = str(int(curr) + val)

    def vel2_command_callback(self, val):
        curr = self.m2_vel.text 
        self.m2_vel.text = str(int(curr) + val)

    def vel3_command_callback(self, val):
        curr = self.m3_vel.text 
        self.m3_vel.text = str(int(curr) + val)
    
    def vel4_command_callback(self, val):
        curr = self.m4_vel.text 
        self.m4_vel.text = str(int(curr) + val)

    def vel5_command_callback(self, val):
        curr = self.m5_vel.text 
        self.m5_vel.text = str(int(curr) + val)

    def vel6_command_callback(self, val):
        curr = self.m6_vel.text 
        self.m6_vel.text = str(int(curr) + val)
    
    def vel7_command_callback(self, val):
        curr = self.m7_vel.text 
        self.m7_vel.text = str(int(curr) + val)

    def vel8_command_callback(self, val):
        curr = self.m8_vel.text 
        self.m8_vel.text = str(int(curr) + val)

    def vel9_command_callback(self, val):
        curr = self.m9_vel.text 
        self.m9_vel.text = str(int(curr) + val)
    
    def vel10_command_callback(self, val):
        curr = self.m10_vel.text 
        self.m10_vel.text = str(int(curr) + val)

class MenuLeftLegScreen(Screen):
    url = 'http://localhost:8080/snapshot?topic=/rviz1/camera1/image&type=png'
    image_path = StringProperty("robotImage.png")
    urllib.request.urlretrieve(url, "robotImage.png")
    img = ObjectProperty(source = "robotImage.png")
    
    rjs = RobotJointState()

    m1_pos = ObjectProperty(None)
    m1_vel = ObjectProperty(None)

    m2_pos = ObjectProperty(None)
    m2_vel = ObjectProperty(None)

    m3_pos = ObjectProperty(None)
    m3_vel = ObjectProperty(None)

    m4_pos = ObjectProperty(None)
    m4_vel = ObjectProperty(None)

    m5_pos = ObjectProperty(None)
    m5_vel = ObjectProperty(None)

    m6_pos = ObjectProperty(None)
    m6_vel = ObjectProperty(None)

    pass

    def __init__(self, **kwargs):
        super(MenuLeftLegScreen, self).__init__(**kwargs)
        self.command_sub = rospy.Subscriber("joint_states", JointState,
                                            self.update_rjs, queue_size=10)
        
    def change_image(self, path):
        urllib.request.urlretrieve(self.url, path)
        os.system("mv new_robotImage.png robotImage.png")
        self.img.reload()

    def update_rjs(self, msg):
        self.rjs.copy(msg)
        
    def m1_command_callback(self, val):
        curr = self.m1_pos.text
        self.m1_pos.text = str(int(curr) + val)
        self.rjs.joint_state.position[10] = int(self.m1_pos.text) * math.pi/180
        self.rjs.joint_state.header.stamp = rospy.Time.now()
        self.rjs.joint_state.header.seq += 1
        joint_pub.publish(self.rjs.joint_state)
        self.change_image("new_robotImage.png")
    
    def m2_command_callback(self, val):
        curr = self.m2_pos.text
        self.m2_pos.text = str(int(curr) + val)
        self.rjs.joint_state.position[11] = int(self.m2_pos.text) * math.pi/180
        self.rjs.joint_state.header.stamp = rospy.Time.now()
        self.rjs.joint_state.header.seq += 1
        joint_pub.publish(self.rjs.joint_state)
        self.change_image("new_robotImage.png")

    def m3_command_callback(self, val):
        curr = self.m3_pos.text
        self.m3_pos.text = str(int(curr) + val)
        self.rjs.joint_state.position[12] = int(self.m3_pos.text) * math.pi/180
        self.rjs.joint_state.header.stamp = rospy.Time.now()
        self.rjs.joint_state.header.seq += 1
        joint_pub.publish(self.rjs.joint_state)
        self.change_image("new_robotImage.png")
    
    def m4_command_callback(self, val):
        curr = self.m4_pos.text
        self.m4_pos.text = str(int(curr) + val)
        self.rjs.joint_state.position[13] = int(self.m4_pos.text) * math.pi/180
        self.rjs.joint_state.header.stamp = rospy.Time.now()
        self.rjs.joint_state.header.seq += 1
        joint_pub.publish(self.rjs.joint_state)
        self.change_image("new_robotImage.png")

    def m5_command_callback(self, val):
        curr = self.m5_pos.text
        self.m5_pos.text = str(int(curr) + val)
        self.rjs.joint_state.position[14] = int(self.m5_pos.text) * math.pi/180
        self.rjs.joint_state.header.stamp = rospy.Time.now()
        self.rjs.joint_state.header.seq += 1
        joint_pub.publish(self.rjs.joint_state)
        self.change_image("new_robotImage.png")
    
    def m6_command_callback(self, val):
        curr = self.m6_pos.text
        self.m6_pos.text = str(int(curr) + val)
        self.rjs.joint_state.position[15] = int(self.m6_pos.text) * math.pi/180
        self.rjs.joint_state.header.stamp = rospy.Time.now()
        self.rjs.joint_state.header.seq += 1
        joint_pub.publish(self.rjs.joint_state)
        self.change_image("new_robotImage.png")

    def vel1_command_callback(self, val):
        curr = self.m1_vel.text 
        self.m1_vel.text = str(int(curr) + val)

    def vel2_command_callback(self, val):
        curr = self.m2_vel.text 
        self.m2_vel.text = str(int(curr) + val)

    def vel3_command_callback(self, val):
        curr = self.m3_vel.text 
        self.m3_vel.text = str(int(curr) + val)
    
    def vel4_command_callback(self, val):
        curr = self.m4_vel.text 
        self.m4_vel.text = str(int(curr) + val)

    def vel5_command_callback(self, val):
        curr = self.m5_vel.text 
        self.m5_vel.text = str(int(curr) + val)

    def vel6_command_callback(self, val):
        curr = self.m6_vel.text 
        self.m6_vel.text = str(int(curr) + val)

class MenuRightLegScreen(Screen):
    url = 'http://localhost:8080/snapshot?topic=/rviz1/camera1/image&type=png'
    image_path = StringProperty("robotImage.png")
    urllib.request.urlretrieve(url, "robotImage.png")
    img = ObjectProperty(source = "robotImage.png")
    
    rjs = RobotJointState()

    m1_pos = ObjectProperty(None)
    m1_vel = ObjectProperty(None)

    m2_pos = ObjectProperty(None)
    m2_vel = ObjectProperty(None)

    m3_pos = ObjectProperty(None)
    m3_vel = ObjectProperty(None)

    m4_pos = ObjectProperty(None)
    m4_vel = ObjectProperty(None)

    m5_pos = ObjectProperty(None)
    m5_vel = ObjectProperty(None)

    m6_pos = ObjectProperty(None)
    m6_vel = ObjectProperty(None)

    pass

    def __init__(self, **kwargs):
        super(MenuRightLegScreen, self).__init__(**kwargs)
        self.command_sub = rospy.Subscriber("joint_states", JointState,
                                            self.update_rjs, queue_size=10)
        
    def change_image(self, path):
        urllib.request.urlretrieve(self.url, path)
        os.system("mv new_robotImage.png robotImage.png")
        self.img.reload()

    def update_rjs(self, msg):
        self.rjs.copy(msg)
        
    def m1_command_callback(self, val):
        curr = self.m1_pos.text
        self.m1_pos.text = str(int(curr) + val)
        self.rjs.joint_state.position[10] = int(self.m1_pos.text) * math.pi/180
        self.rjs.joint_state.header.stamp = rospy.Time.now()
        self.rjs.joint_state.header.seq += 1
        joint_pub.publish(self.rjs.joint_state)
        self.change_image("new_robotImage.png")
    
    def m2_command_callback(self, val):
        curr = self.m2_pos.text
        self.m2_pos.text = str(int(curr) + val)
        self.rjs.joint_state.position[11] = int(self.m2_pos.text) * math.pi/180
        self.rjs.joint_state.header.stamp = rospy.Time.now()
        self.rjs.joint_state.header.seq += 1
        joint_pub.publish(self.rjs.joint_state)
        self.change_image("new_robotImage.png")

    def m3_command_callback(self, val):
        curr = self.m3_pos.text
        self.m3_pos.text = str(int(curr) + val)
        self.rjs.joint_state.position[12] = int(self.m3_pos.text) * math.pi/180
        self.rjs.joint_state.header.stamp = rospy.Time.now()
        self.rjs.joint_state.header.seq += 1
        joint_pub.publish(self.rjs.joint_state)
        self.change_image("new_robotImage.png")
    
    def m4_command_callback(self, val):
        curr = self.m4_pos.text
        self.m4_pos.text = str(int(curr) + val)
        self.rjs.joint_state.position[13] = int(self.m4_pos.text) * math.pi/180
        self.rjs.joint_state.header.stamp = rospy.Time.now()
        self.rjs.joint_state.header.seq += 1
        joint_pub.publish(self.rjs.joint_state)
        self.change_image("new_robotImage.png")

    def m5_command_callback(self, val):
        curr = self.m5_pos.text
        self.m5_pos.text = str(int(curr) + val)
        self.rjs.joint_state.position[14] = int(self.m5_pos.text) * math.pi/180
        self.rjs.joint_state.header.stamp = rospy.Time.now()
        self.rjs.joint_state.header.seq += 1
        joint_pub.publish(self.rjs.joint_state)
        self.change_image("new_robotImage.png")
    
    def m6_command_callback(self, val):
        curr = self.m6_pos.text
        self.m6_pos.text = str(int(curr) + val)
        self.rjs.joint_state.position[15] = int(self.m6_pos.text) * math.pi/180
        self.rjs.joint_state.header.stamp = rospy.Time.now()
        self.rjs.joint_state.header.seq += 1
        joint_pub.publish(self.rjs.joint_state)
        self.change_image("new_robotImage.png")

    def vel1_command_callback(self, val):
        curr = self.m1_vel.text 
        self.m1_vel.text = str(int(curr) + val)

    def vel2_command_callback(self, val):
        curr = self.m2_vel.text 
        self.m2_vel.text = str(int(curr) + val)

    def vel3_command_callback(self, val):
        curr = self.m3_vel.text 
        self.m3_vel.text = str(int(curr) + val)
    
    def vel4_command_callback(self, val):
        curr = self.m4_vel.text 
        self.m4_vel.text = str(int(curr) + val)

    def vel5_command_callback(self, val):
        curr = self.m5_vel.text 
        self.m5_vel.text = str(int(curr) + val)

    def vel6_command_callback(self, val):
        curr = self.m6_vel.text 
        self.m6_vel.text = str(int(curr) + val)
        
class IDScreen(Screen):
    pass

class LIDARScreen(Screen):
    pass

class RealsenseScreen(Screen):
    pass

class IMUScreen(Screen):
    pass

class FTScreen(Screen):
    pass

def LA_button():
    command1= "rqt_plot /linear_acceleration_X:linear_acceleration_Y:linear_acceleration_Z"
    p = subprocess.run(command1, shell=True) 

def AV_button():
    command1= "rqt_plot /angular_velocity_X:angular_velocity_Y:angular_velocity_Z"
    p = subprocess.run(command1, shell=True) 

def Rviz_button():
    command2= "rosrun rviz rviz -f vectornav"
    p = subprocess.run(command2, shell=True) 

#initializing the motors using a seperate thread
#subprocess open up the python file which starts the launch file
def init_motors():
    print("initing motors")
    Thread(target=lambda *largs: subprocess.run([sys.executable, os.getcwd() + "/src/gui_tutorials/src/read_write_node_launch.py"])).start()
    #Thread(target=lambda *largs: subprocess.run([sys.executable, "/home/glorycode/catkin_ws/src/gui_tutorials/src/read_write_node_launch.py"])).start()

#initializing motor sensors 
#subprocess open up the python file which starts the launch file
def init_motor_sensors():
    #Thread(target=lambda *largs: subprocess.run([sys.executable, "/home/glorycode/catkin_ws/src/gui_tutorials/src/sensors_launch.py"])).start()
    Thread(target=lambda *largs: subprocess.run([sys.executable, os.getcwd() + "/src/gui_tutorials/src/sensors_launch.py"])).start()

def init_urdf():
    command = "roslaunch " + os.getcwd() + "/src/urdf-rviz/launch/urdf-rviz.launch"
    p = subprocess.run(command, shell=True)

def init_force():
    command = "roslaunch rokubimini_serial rokubimini_serial.launch"
    p = subprocess.run(command, shell=True)

def init_velo():
    command = "roslaunch velodyne_pointcloud VLP16_points.launch"
    p = subprocess.run(command, shell=True)

#initializing the other sensors
#subprocess open up the python file which starts the launch file
def init_sensors():
    Thread(target=lambda *largs: subprocess.run([sys.executable, os.getcwd() +  "/src/gui_tutorials/src/all_sensors_launch.py"])).start()

#runs the command to open the velodyne visualization in rviz through a subprocess
def start_velo():
    print("collecting data")
    command = "rosrun rviz rviz -f velodyne"
    p = subprocess.run(command, shell=True)

#runs the command to open the camera visualization in rviz through a subprocess
def start_camera():
    print("collecting data")
    command = "roslaunch realsense2_camera demo_t265.launch"
    p = subprocess.run(command, shell=True)

#runs the command to record the force torque data to a rosbag file for later processing
def start_force():
    print("collecting data")
    command = "rosbag record -O forceTorque /bus0/ft_sensor0/ft_sensor_readings/wrench"
    p = subprocess.run(command, shell=True)

def stop_button():
    PROTOCOL_VERSION            = 2.0               # See which protocol version is used in the Dynamixel
    PROTOCOL_VERSION_HANDS      = 1.0
    DEVICENAME1  = '/dev/ttyUSB0'    # Check which port is being used on your controller using the command "ls -l /dev/ttyUSB*"
    DEVICENAME2  = '/dev/ttyUSB1'
    DEVICENAME3  = '/dev/ttyUSB2'
    DEVICENAME4  = '/dev/ttyUSB3'

    portHandler1 = PortHandler(DEVICENAME1)             # left arm, head and torso     
    portHandler2 = PortHandler(DEVICENAME2)          	# right arm
    portHandler3 = PortHandler(DEVICENAME3)             # left leg
    portHandler4 = PortHandler(DEVICENAME4)         	# right leg
    packetHandler = PacketHandler(PROTOCOL_VERSION)
    packetHandler2 = PacketHandler(PROTOCOL_VERSION_HANDS)

    try:
        portHandler1.openPort()
        portHandler2.openPort()
        portHandler3.openPort()
        portHandler4.openPort() #comment the porthandler corresponding to the port that is not being used
    except:
        print("failed to connect port in stop_button")
        getch()
        quit()

    try:
        portHandler1.setBaudRate(BAUDRATE)
        portHandler2.setBaudRate(BAUDRATE)
        portHandler3.setBaudRate(BAUDRATE)
        portHandler4.setBaudRate(BAUDRATE)  #comment the porthandler corresponding to the port that is not being used

    except:
        print("Failed to change the stop_button baudrate")
        getch()
        quit()

    #arrays which contain the motor ids for their corresponding sections
    head = [41,42]
    left_arm = [2,4,6,8,10,12,14]
    right_arm = [1,3,5,7,9,11,13]
    left_leg = [16,18,20,22,24,26]
    right_leg =[15,17,19,21,23,25]
    torso = [27,28,29]
    left_hand = [32,34,36]
    right_hand = [31,33,35]

    TORQUE_ENABLE = 0
    ADDR_TORQUE_ENABLE = 512
    ADDR_VEL_POSITION      = 560
    SPEED                  = 2
    ADDR_VEL_POSITION_HANDS      = 32
    SPEED_HANDS = 1
    ADDR_ACC               = 556
    ACC                    = 0
    
    for i in head:
        dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler1, i, ADDR_VEL_POSITION, SPEED)
        dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler1, i, ADDR_TORQUE_ENABLE, TORQUE_ENABLE)
        dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler1, i, ADDR_ACC, ACC)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
            print("WARNING MOTOR ", i, " NOT CONNECTED!!!")
            # getch()
        elif dxl_error != 0:
            print("%s" % packetHandler.getRxPacketError(dxl_error))
            print("WARNING MOTOR ", i, " NOT CONNECTED!!!")
            # getch()
        else:
            print("DYNAMIXEL has been successfully connected")
    
    for i in left_arm:
        print("left arm")
        dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler1, i, ADDR_VEL_POSITION, SPEED)
        print("Set Velocity of ID %s = %s" % (i, SPEED))
        dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler1, i, ADDR_TORQUE_ENABLE, TORQUE_ENABLE)
        dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler1, i, ADDR_ACC, ACC)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
            print("WARNING MOTOR ", i, " NOT CONNECTED!!!")
            # getch()
        elif dxl_error != 0:
            print("%s" % packetHandler.getRxPacketError(dxl_error))
            print("WARNING MOTOR ", i, " NOT CONNECTED!!!")
            # getch()
        else:
            print("DYNAMIXEL has been successfully connected")

    for i in right_arm:
        dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler2, i, ADDR_VEL_POSITION, SPEED)
        dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler2, i, ADDR_TORQUE_ENABLE, TORQUE_ENABLE)
        dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler1, i, ADDR_ACC, ACC)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
            print("WARNING MOTOR ", i, " NOT CONNECTED!!!")
            # getch()
        elif dxl_error != 0:
            print("%s" % packetHandler.getRxPacketError(dxl_error))
            print("WARNING MOTOR ", i, " NOT CONNECTED!!!")
            # getch()
        else:
            print("DYNAMIXEL has been successfully connected")

    for i in right_leg:
        dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler4, i, ADDR_GOAL_POSITION, SPEED)
        dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler4, i, ADDR_TORQUE_ENABLE, TORQUE_ENABLE)
        dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler1, i, ADDR_ACC, ACC)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
            print("WARNING MOTOR ", i, " NOT CONNECTED!!!")
            # getch()
        elif dxl_error != 0:
            print("%s" % packetHandler.getRxPacketError(dxl_error))
            print("WARNING MOTOR ", i, " NOT CONNECTED!!!")
            # getch()
        else:
            print("DYNAMIXEL has been successfully connected")

    for i in left_leg:
        dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler3, i, ADDR_VEL_POSITION, SPEED)
        dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler3, i, ADDR_TORQUE_ENABLE, TORQUE_ENABLE)
        dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler1, i, ADDR_ACC, ACC)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
            print("WARNING MOTOR ", i, " NOT CONNECTED!!!")
            # getch()
        elif dxl_error != 0:
            print("%s" % packetHandler.getRxPacketError(dxl_error))
            print("WARNING MOTOR ", i, " NOT CONNECTED!!!")
            # getch()
        else:
            print("DYNAMIXEL has been successfully connected")

    for i in torso:
        dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler1, i, ADDR_VEL_POSITION, SPEED)
        dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler1, i, ADDR_TORQUE_ENABLE, TORQUE_ENABLE)
        dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler1, i, ADDR_ACC, ACC)
        print("dxl_comm_result, dxl_error", dxl_comm_result, dxl_error, COMM_SUCCESS)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
            print("WARNING MOTOR ", i, " NOT CONNECTED!!!")
            getch()
        elif dxl_error != 0:
            print("%s" % packetHandler.getRxPacketError(dxl_error))
            print("WARNING MOTOR ", i, " NOT CONNECTED!!!")
            # getch()
        else:
            print("DYNAMIXEL has been successfully connected")
    
    # for i in left_hand:
    #     dxl_comm_result, dxl_error = packetHandler2.write2ByteTxRx(portHandler1, i, ADDR_VEL_POSITION_HANDS, SPEED_HANDS)
    #     dxl_comm_result, dxl_error = packetHandler2.write1ByteTxRx(portHandler1, i, ADDR_TORQUE_ENABLE_HANDS, TORQUE_ENABLE)
    #     dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler1, i, ADDR_ACC, ACC)
    #     if dxl_comm_result != COMM_SUCCESS:
    #         print("%s" % packetHandler2.getTxRxResult(dxl_comm_result))
    #         print("WARNING MOTOR ", i, " NOT CONNECTED!!!")
    #         # getch()
    #     elif dxl_error != 0:
    #         print("%s" % packetHandler2.getRxPacketError(dxl_error))
    #         print("WARNING MOTOR ", i, " NOT CONNECTED!!!")
    #         # getch()
    #     else:
    #         print("DYNAMIXEL has been successfully connected")
    
    # for i in right_hand:
    #     dxl_comm_result, dxl_error = packetHandler2.write2ByteTxRx(portHandler2, i, ADDR_VEL_POSITION_HANDS, SPEED_HANDS)
    #     dxl_comm_result, dxl_error = packetHandler2.write1ByteTxRx(portHandler2, i, ADDR_TORQUE_ENABLE_HANDS, TORQUE_ENABLE)
    #     dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler1, i, ADDR_ACC, ACC)
    #     if dxl_comm_result != COMM_SUCCESS:
    #         print("%s" % packetHandler2.getTxRxResult(dxl_comm_result))
    #         print("WARNING MOTOR ", i, " NOT CONNECTED!!!")
    #         # getch()
    #     elif dxl_error != 0:
    #         print("%s" % packetHandler2.getRxPacketError(dxl_error))
    #         print("WARNING MOTOR ", i, " NOT CONNECTED!!!")
    #         # getch()
    #     else:
    #         print("DYNAMIXEL has been successfully connected")
    
    App.get_running_app().stop
    Window.close()
    
    command = "rosnode kill --all" #once the gui has exited, then kill all rosnodes
    p = subprocess.run(command, shell=True) 
    roscore.terminate() #close roscore

class HumanoidApp(MDApp):
    # Redefine init template
    def __init__(self, **kwargs):
        super().__init__(**kwargs)
        
        #self.screen=Builder.load_file('ros_gui.kv') #load the .kv file for the main window
        self.screen=Builder.load_string(KV) 
        # TO-DO- USE getcwd()
        
    def build(self):
        sm = ScreenManager()
        sm.add_widget(MenuScreen(name='home'))
        sm.add_widget(MenuHeadScreen(name='home_head_control'))
        sm.add_widget(MenuTorsoScreen(name='home_torso_control'))
        sm.add_widget(MenuLeftArmScreen(name='home_leftarm_control'))
        sm.add_widget(MenuRightArmScreen(name='home_rightarm_control'))
        sm.add_widget(MenuLeftLegScreen(name='home_leftleg_control'))
        sm.add_widget(MenuRightLegScreen(name='home_rightleg_control'))
        sm.add_widget(IDScreen(name='id'))
        sm.add_widget(LIDARScreen(name='lidar'))
        sm.add_widget(RealsenseScreen(name='realsense'))
        sm.add_widget(IMUScreen(name='imu'))
        sm.add_widget(FTScreen(name='ft'))

        self.counter = 0

        # menu_items = [
        #     {
        #         "viewclass": "OneLineListItem",
        #         "text": f"Item {i}",
        #         "height": dp(56),
        #         "on_release": lambda x=f"Item {i}": self.menu_callback(x),
        #      } for i in range(5)
        # ]
        menu_items = [
            {
                "viewclass": "OneLineListItem",
                "text": "Head",
                "on_release": lambda *args: self.set_screen('home_head_control')
            },
            {
                "viewclass": "OneLineListItem",
                "text": "Torso",
                "on_release": lambda *args: self.set_screen('home_torso_control')
            },
            {
                "viewclass": "OneLineListItem",
                "text": "Left Arm",
                "on_release": lambda *args: self.set_screen('home_leftarm_control')
            },
            {
                "viewclass": "OneLineListItem",
                "text": "Right Arm",
                "on_release": lambda *args: self.set_screen('home_rightarm_control')
            },
            {
                "viewclass": "OneLineListItem",
                "text": "Left Leg",
                "on_release": lambda *args: self.set_screen('home_leftleg_control')
            },
            {
                "viewclass": "OneLineListItem",
                "text": "Right Leg",
                "on_release": lambda *args: self.set_screen('home_rightleg_control')
            }
        ]
        self.menu = MDDropdownMenu(
            items=menu_items,
            width_mult=4,
        )
        # screen = Builder.load_string(screen_helper)
        return sm
    
    def callback(self, button):
        self.menu.caller = button
        self.menu.open()

    # def menu_callback(self, text_item):
    #     self.menu.dismiss()
    #     Snackbar(text=text_item).open()
    
    def set_screen(self, screen_name):
        self.root.current = screen_name

    def get_counter(self, action):
        if action == 'increment':
            # print(type(int(self.root.id.test_textinput.text)))
            self.counter = self.counter + 1
        
        if action == 'decrement':
            # print(type(int(self.root.id.test_textinput.text)))
            self.counter = self.counter - 1
        
        print('Counter is:')
        print(self.counter)

    def get_counter_value(self):
        self.n = '19'
        return str(self.n)
        #return '12'
       # return self.root.counter
    
    # def on_start(self):
    #     # for i in range(3):
    #     #         self.root.ids.tabs.add_widget(Tab(title=f"LiDAR {i}"))
    #     for name_tab in list(md_icons.keys())[15:18]:
    #         self.root.ids.tabs.add_widget(Tab(icon=name_tab, title=name_tab))

    def on_tab_switch(
            self, instance_tabs, instance_tab, instance_tab_label, tab_text
    ):
        """
        Called when switching tabs
        :type instance_tabs: <kivymd.uix.tab.MDTabs object>;
        :param instance_tab: <__main__.Tab object>;
        :param instance_tab_label: <kivymd.uix.tab.MDTabsLabel object>;
        :param tab_text: text or name icon of tab;
        """
        #print(f"{instance_tabs} {instance_tab_label} {tab_text}")
        count_icon = instance_tab.icon

        if count_icon == 'motion-sensor':
            #self.root.screen_manager.current = 'screen1'
            self.root.current = 'lidar'
        elif count_icon == 'camera-outline':
            self.root.current = 'realsense'
        elif count_icon == 'chart-waterfall':
            self.root.current = 'imu'
        elif count_icon == 'cog':
            self.root.current = 'ft'

    #IMU buttons functionaities.
    def linear_acc(self, id):
        threadStart = Thread(target= LA_button, args=())
        threadStart.start()

    def angular_vel(self, id):
        threadStart = Thread(target= AV_button, args=())
        threadStart.start()
    
    def imu_rviz(self, id):
        threadStart = Thread(target= Rviz_button, args=())
        threadStart.start() 
    
    def emergency_stop(self):
        print("emergency_stop is pressed")
        threatStart = Thread(target = stop_button, args=())
        threatStart.start() 
    
    #in a seperate thread to prevent main window from freezing visaluze the velodyne data
    def velo_bag(self):
        threatStart = Thread(target = init_velo, args=())
        threatStart.start()
        thread2 = Thread(target=start_velo, args=()) 
        thread2.setDaemon(True)
        thread2.start()
    
    #in a seperate thread to prevent main window from freezing visualize the camera data
    def camera_bag(self):
        thread4 = Thread(target=start_camera, args=()) 
        thread4.setDaemon(True)
        thread4.start()

    #in a seperate thread to prevent main window from freezing bag the force torque data
    def force_bag(self):
        threadStart = Thread(target = init_force, args=())
        threadStart.start()
        thread5 = Thread(target=start_force, args=()) 
        thread5.setDaemon(True)
        thread5.start()
        
    #thread function which greys out the publish button for 3 seconds when it is pressed
    #prevents the rest of the gui from sleeping when time.sleep is called
    def send_position_callback(self, id, position, velocity, buttonid):
        pub.publish(int(id), int(position), int(velocity))
        self.screen.ids[buttonid].disabled = True #disables the buttons
        time.sleep(3) #disbaled for 3 seconds
        self.screen.ids[buttonid].disabled = False

    #function which converts degrees to the appropriate position values for the motors based on their types
    def send_position(self, id, position, velocity, buttonid):
        print("-----------BUTTON PRESSED----------")
        id_i = int(id) #casting the id to an int 
        if ((id_i > 0 and id_i < 9) or (id_i==15) or (id_i == 16) or (id_i == 27)): #converts degrees to appropriate position value
            position = ((int(position)*(303750 *2))/360) #initial position *2/360
            # velocity = 750
           # position = ((int(position)*(501433*2))/360) #initial position *2/360
        elif ((id_i > 8 and id_i < 15) or (id_i==28) or (id_i == 29)):
            position = ((int(position)*(303454*2))/360) #initial position *2/360
            # velocity = 750
        elif((id_i > 30 and id_i < 37)):    #ids for hands
            position = ((int(position)*(1023))/300)
            # velocity = 100
        print("position = ", position, "velocity = ", velocity)
        thread1 = Thread(target=self.send_position_callback, args=(id, position, velocity, buttonid)) #starts the thread which controls the button functionality
        thread1.setDaemon(True)
        thread1.start()
        print("started thread")

    def send_position_test(self, id, position, velocity, buttonid):
        print("-----------BUTTON PRESSED----------")
        print(str(id))
        print(str(position))
        print(str(velocity))
        print(str(buttonid))
    
    def buton(self):
        print("-----------BUTTON PRESSED----------")

    #thread function which contains the sequence to move the right arm in a wave
    def wave_right_arm_callback(self, buttonid):
        print("Hello User, Humanoid is waving right arm")
        pub.publish(int(right_arm[1]), int((20*(501433*2))/360),int(750)) #moving the motors to hardcoded value, converting the angle from sliders to a positional value for the motors
        rospy.sleep(1)
        pub.publish(int(right_arm[3]), int((-77*(501433*2))/360),int(750))
        rospy.sleep(1)
        pub.publish(int(right_arm[4]), int((-1*(303454*2))/360),int(750))
        rospy.sleep(1)
        pub.publish(int(right_arm[5]), int((-71*(303454*2))/360),int(750))
        rospy.sleep(2)
        pub.publish(int(right_arm[4]), int((29*(303454*2))/360),int(750))
        rospy.sleep(1)
        pub.publish(int(right_arm[4]), int((-29*(303454*2))/360),int(750))
        rospy.sleep(1)
        pub.publish(int(right_arm[4]), int((29*(303454*2))/360),int(750))
        rospy.sleep(1)
        pub.publish(int(right_arm[1]), int((20*(501433*2))/360),int(750))
        rospy.sleep(1)
        pub.publish(int(right_arm[3]), int((-16*(501433*2))/360),int(750))
        rospy.sleep(1)
        pub.publish(int(right_arm[4]), int((0*(303454*2))/360),int(750))
        rospy.sleep(1)
        pub.publish(int(right_arm[5]), int((0*(303454*2))/360),int(750))
        self.screen.ids[buttonid].disabled = True  #disables the button to keep users from repeatedly pushing while the movement sequence is taking place
        time.sleep(10) #keep disabled for 10 seconds
        self.screen.ids[buttonid].disabled = False
    
    #demo right arm wave through a thread to keep main window from freezing while demo takes place
    def wave_right_arm(self, buttonid):
        thread1 = Thread(target=self.wave_right_arm_callback, args=(buttonid,)) #starts the thread which controls the button functionality
        thread1.start()

    #thread function which contains the sequence to move the left arm in a wave
    def wave_left_arm_callback(self, buttonid):
        print("waving left arm")
        pub.publish(int(left_arm[1]), int((-90*(501433*2))/360),int(750))
        rospy.sleep(2)
        pub.publish(int(left_arm[2]), int((-90*(501433*2))/360),int(750))
        rospy.sleep(2)
        pub.publish(int(left_arm[3]), int((67*(501433*2))/360),int(750))
        rospy.sleep(2)
        pub.publish(int(left_arm[3]), int((51*(501433*2))/360),int(750))
        rospy.sleep(1)
        pub.publish(int(left_arm[3]), int((67*(501433*2))/360),int(750))
        rospy.sleep(1)
        pub.publish(int(left_arm[3]), int((51*(501433*2))/360),int(750))
        rospy.sleep(1)
        pub.publish(int(left_arm[3]), int((67*(501433*2))/360),int(750))
        rospy.sleep(1)
        pub.publish(int(left_arm[3]), int((51*(501433*2))/360),int(750))
        rospy.sleep(1)
        pub.publish(int(left_arm[1]), int((-20*(501433*2))/360),int(750))
        rospy.sleep(1)
        pub.publish(int(left_arm[2]), int((0*(501433*2))/360),int(750))
        rospy.sleep(1)
        pub.publish(int(left_arm[3]), int((0*(501433*2))/360),int(750))
        self.screen.ids[buttonid].disabled = True  #disables the button to keep users from repeatedly pushing while the movement sequence is taking place
        time.sleep(10) #keep disabled for 10 seconds
        self.screen.ids[buttonid].disabled = False

    # # demo left arm wave through a thread to keep main window from freezing while demo takes place
    def wave_left_arm(self, buttonid):
        thread1 = Thread(target=self.wave_left_arm_callback, args=(buttonid,)) #starts the thread which controls the button functionality
        thread1.start()

    #launches the seperate the popup windows for the motor sliders with different threads
    @staticmethod
    def motor_section_show(id):
        print("New window open")
        if str(id) == 'Head': #if the button pressed on main window is 'Head' pass robot_section.py 'Head' as argument
            Thread(target=lambda *largs: subprocess.run([sys.executable, os.getcwd() + "/src/gui_tutorials/src/robot_section.py"], input=b"Head")).start() #starting up the code for each different section in a seperate thread
        elif str(id) == 'Left Arm': #if the button pressed on main window is 'Left Arm' pass robot_section.py 'Left Arm' as argument
            Thread(target=lambda *largs: subprocess.run([sys.executable, os.getcwd() + "/src/gui_tutorials/src/robot_section.py"], input=b"Left Arm")).start()
        elif str(id) == 'Right Arm': #if button pressed on main window is 'Right Arm' pass robot_section.py 'Right Arm' as argumeny
            Thread(target=lambda *largs: subprocess.run([sys.executable, os.getcwd() + "/src/gui_tutorials/src/robot_section.py"], input=b"Right Arm")).start()
        elif str(id) == 'Torso': #if button pressed on main window is 'Torso' pass robot_section.py 'Torso' as argument
            Thread(target=lambda *largs: subprocess.run([sys.executable, os.getcwd() + "/src/gui_tutorials/src/robot_section.py"], input=b"Torso")).start()
        elif str(id) == 'Left Leg': #if button pressed on main window is 'Left Leg' launch the left leg popup pass robot_section.py 'Left Leg' as argument
            Thread(target=lambda *largs: subprocess.run([sys.executable, os.getcwd() + "/src/gui_tutorials/src/robot_section.py"], input=b"Left Leg")).start()
        elif str(id) == 'Right Leg': #if the button pressed on main window is 'Right Leg' pass robot_section.py 'Right Leg' as argument
            Thread(target=lambda *largs: subprocess.run([sys.executable, os.getcwd() + "/src/gui_tutorials/src/robot_section.py"], input=b"Right Leg")).start()
        elif str(id) == 'IMU': #if the button pressed on main window is 'Left Arm' pass robot_section.py 'Left Arm' as argument
            Thread(target=lambda *largs: subprocess.run([sys.executable, os.getcwd() + "/src/gui_tutorials/src/robot_section.py"], input=b"IMU")).start()
            threatStart = Thread(target = launch_button, args=())
            threatStart.start()
            Thread(target=lambda *largs: subprocess.run([sys.executable, os.getcwd() + "/src/gui_tutorials/src/imu_subscriber.py"])).start()

    def show_sensor_info(self, id):
        print(str(id)+" was checked")
        m = "id: " +str(id)
        thread1 = Thread(target=self.send_position_callback, args=(m,))
        thread1.start()

        # #sending the command line argument to start up the service through a subprocess
        process = subprocess.Popen(['rosservice', 'call', '/get_motor_sensors', id], stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        i = 0
        out = [0]*3
        for line in process.stdout:
            out[i] = (re.sub('[^0-9]','', line.decode("ascii"))) #gets rid of any extra characters
            i+=1
        # print(str(out[0]))
        # print(str(float(out[1])/10))
        # print(str(out[2]))
        return out
        # #displays the values of the current, voltage and temp to the gui screen
        #self.current_id.text = str(out[0])    
        # self.voltage_id.text = str(float(out[1])/10)
        # self.temperature_id.text = str(out[2])

if __name__ == '__main__':	

    # if rosgraph.is_master_online(): # Checks the master uri and results boolean (True or False)
    #     print('ROS MASTER is Online')
    #     roscore.terminate()
    # else:
    #     print('ROS MASTER is Offline') #if ros master is offline then start a new instance of roscore

    # roscore.run()
    # time.sleep(2)
    
    # rospy.init_node('simple_gui', anonymous=True)
    # pub = rospy.Publisher('set_position', SetPosition, queue_size=10)

    # init_motors() #initializing the motors
    # init_motor_sensors() #initializing the motor sensors
    # # init_sensors() #initializing the sensors
    
    ########################################
    #init_urdf()
    rospy.init_node('mc_gui', anonymous=True)
    
    init_motors() #initializing the motors
    init_motor_sensors() #initializing the motor sensors
    pub = rospy.Publisher('set_position', SetPosition, queue_size=10)
   

    
    # Runs KIVY HumanoidApp Class
    HumanoidApp().run() 
    command = "rosnode kill --all" #once the gui has exited, then kill all rosnodes
    p = subprocess.run(command, shell=True) 
    # roscore.terminate() #close roscore
    #############################

######################################################

# #arrays for the motors ids of arms for demo purposes
# left_arm = [2,4,6,8,10,12,14]
# right_arm = [1,3,5,7,9,11,13]

# def launch_button():
#     cmd1="roslaunch vectornav vectornav.launch"
#     p = subprocess.run(cmd1, shell=True)
    
# class P(FloatLayout):
#     pass

# #launches the popup window for the motor sliders
# def show_popup():
#     show = P()
#     time.sleep(3)
#     popupWindow.open()