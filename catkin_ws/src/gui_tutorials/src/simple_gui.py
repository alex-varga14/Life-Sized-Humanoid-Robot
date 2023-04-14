#!/usr/bin/env python3.8

# # from pickletools import uint8
# # from std_msgs.msg import Bool
# # from kivy.core.window import Window
# # from kivy.uix.popup import Popup
# # from kivy.uix.floatlayout import FloatLayout
# # from kivy.core.window import Window
# # from robot_section import *

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

from kivymd.uix.menu import MDDropdownMenu
from kivymd.uix.snackbar import Snackbar

from kivymd.uix.list import OneLineListItem
from kivy.properties import StringProperty
from kivy.network.urlrequest import UrlRequest
import urllib.request

# roscore = Roscore()

#Builder.load_string()
KV = """
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

        Button:
            id: refresh_btn
            text: "Refresh"
            halign: "center"
            size_hint: 0.15, 0.1
            pos_hint: {"center_x": .5, "center_y": 0.5}
            on_press: root.change_image("new_robotImage.png")

        Button:
            id: run_btn
            text: "Run"
            halign: "center"
            size_hint: 0.15, 0.1
            pos_hint: {"center_x": .1, "center_y": 0.2}
        
        Button:
            id: emergency
            text: "EM-STOP"
            halign: "center"
            background_color: get_color_from_hex('#FF0A01')
            #background_color: 0,1,0,1
            size_hint: 0.15, 0.1
            pos_hint: {"center_x": .35, "center_y": 0.2}
            on_press: app.emergency_stop()

        Button:
            id: btn 
            text: "Load Demo:"
            halign: "center"
            size_hint: 0.25, 0.1
            pos_hint: {"center_x": .65, "center_y": 0.8}
            on_parent: drop_content.dismiss()
            on_release: drop_content.open(self)

        DropDown:
            id: drop_content
            on_select: btn.text = '{}'.format(args[1])

            Button:
                id: btn1
                text: 'First Item'
                size_hint_y: None
                height: 35
                on_release: drop_content.select('First Item')
            
            Button:
                id: btn2
                text: 'Second Item'
                size_hint_y: None
                height: 35
                on_release: drop_content.select('Second Item')

        FloatLayout:
            MDRaisedButton:
                id: caller
                text: "Motor Selection"
                pos_hint: {"center_x": .75, "center_y": .5}
                on_release: app.menu.open()
            
        MDTabs:
            text_color_active: [0, 0, 0, 1]
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

<MenuLeftArmScreen>:
    name: 'home_leftarm_control'
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

        Button:
            id: refresh_btn
            text: "Refresh"
            halign: "center"
            size_hint: 0.15, 0.1
            pos_hint: {"center_x": .5, "center_y": 0.5}
            on_press: root.change_image("new_robotImage.png")

        Button:
            id: run_btn
            text: "Run"
            halign: "center"
            size_hint: 0.15, 0.1
            pos_hint: {"center_x": .1, "center_y": 0.2}
        
        Button:
            id: emergency
            text: "EM-STOP"
            halign: "center"
            background_color: 1,0,0,1
            size_hint: 0.15, 0.1
            pos_hint: {"center_x": .35, "center_y": 0.2}
            on_press: app.emergency_stop()

        Button:
            id: btn 
            text: "Load Demo:"
            halign: "center"
            size_hint: 0.25, 0.1
            pos_hint: {"center_x": .65, "center_y": 0.8}
            on_parent: drop_content.dismiss()
            on_release: drop_content.open(self)

        DropDown:
            id: drop_content
            on_select: btn.text = '{}'.format(args[1])

            Button:
                id: btn1
                text: 'First Item'
                size_hint_y: None
                height: 35
                on_release: drop_content.select('First Item')
            
            Button:
                id: btn2
                text: 'Second Item'
                size_hint_y: None
                height: 35
                on_release: drop_content.select('Second Item')

        FloatLayout:
            #pos_hint: {"right": 1}
            # size_hint: 0.5, 0.5
            GridLayout:
                pos_hint: {"center_x": 0.5, "center_y": 0.4}
                col_default_width: '56dp'
                size_hint_x: None
                cols: 8
                row_default_height: '32dp'
                size_hint_y: None
                height: self.minimum_height
                Label:
                    text: 'SENSOR'
                    color: (0,0,0,1)
                Label:
                    text: 'ID'
                    color: (0,0,0,1)
                Label:
                    text: 'POS CONTROL' 
                    color: (0,0,0,1)           
                Label:
                    text: 'GOAL POS'  
                    color: (0,0,0,1) 
                Label:
                    text: 'CUR POS'
                    color: (0,0,0,1)
                Label:
                    text: 'VEL CONTROL'
                    color: (0,0,0,1) 
                Label:
                    text: 'GOAL VEL' 
                    color: (0,0,0,1)
                Label:
                    text: 'SEND POS'
                    color: (0,0,0,1)
                # Motor 1
                CheckBox:
                    id: m1_checkbox
                    group: "checkboxes"
                    color: 1, 1, 1, 1
                    on_active: app.test(m1_id.text)
                    size_hint_x: .20
                    canvas.before:
                        Color:
                            rgb: 1,0,0
                        Ellipse:
                            pos:self.center_x-8, self.center_y-8
                            size:[16,16]
                        Color:
                            rgb: 0,0,0
                        Ellipse:
                            pos:self.center_x-7, self.center_y-7
                            size:[14,14]
                Label:
                    text: '2'
                    id: m1_id
                    color: (0,0,1,1)
                    canvas.before:
                        Color:
                            rgba: 1, 1, 1, 1
                        Rectangle:
                            pos: self.pos
                            size: self.size
                Slider:
                    id: m1_slider
                    value: 0
                    min: -180
                    max: 180
                    step: 1
                    #size_hint:(None, .5)
                    #width: 200

                TextInput:
                    id: m1_textinput
                    # size_hint_x: None
                    # width: 30
                    input_filter: 'int'
                    multiline: False
                    text: str(m1_slider.value)
                Label:
                    text: str(m1_slider.value)
                    color: (0,0,1,1)
                    canvas.before:
                        Color:
                            rgba: 1, 1, 1, 1
                        Rectangle:
                            pos: self.pos
                            size: self.size
                Slider:
                    id: velo1_slider
                    value: 750
                    min: 1
                    max: 1500
                    step: 1
                    #size_hint:(None, .5)
                    #width: 200    
                TextInput:
                    id: velo1_textinput
                    # size_hint_x: None
                    # width: 30
                    input_filter: 'int'
                    multiline: False
                    text: str(velo1_slider.value)
                # Label:
                #     text: str(velo_slider.value)
                #     color: (0,0,1,1)
                #     canvas.before:
                #         Color:
                #             rgba: 1, 1, 1, 1
                #         Rectangle:
                #             pos: self.pos
                #             size: self.size

                Button:
                    id: pub1
                    text: 'Pub'
                    on_press:app.send_position(m1_id.text, m1_slider.value, velo1_slider.value, "pub1") 
                # Motor 2
                CheckBox:
                    id: m2_checkbox
                    group: "checkboxes"
                    color: 1, 1, 1, 1
                    on_active: app.test(m2_id.text)
                    size_hint_x: .20
                    canvas.before:
                        Color:
                            rgb: 1,0,0
                        Ellipse:
                            pos:self.center_x-8, self.center_y-8
                            size:[16,16]
                        Color:
                            rgb: 0,0,0
                        Ellipse:
                            pos:self.center_x-7, self.center_y-7
                            size:[14,14]
                Label:
                    text: '4'
                    id: m2_id
                    color: (0,0,1,1)
                    canvas.before:
                        Color:
                            rgba: 1, 1, 1, 1
                        Rectangle:
                            pos: self.pos
                            size: self.size
                Slider:
                    id: m2_slider
                    value: 0
                    min: -180
                    max: 180
                    step: 1
                    #size_hint:(None, .5)sectiition(m2_id.text, m2_slider.value)
                TextInput:
                    id: m2_textinput
                    # size_hint_x: None
                    # width: 30
                    input_filter: 'int'
                    multiline: False
                    text: str(m2_slider.value)
                Label:
                    text: str(m2_slider.value)
                    color: (0,0,1,1)
                    canvas.before:
                        Color:
                            rgba: 1, 1, 1, 1
                        Rectangle:
                            pos: self.pos
                            size: self.size
                Slider:
                    id: velo2_slider
                    value: 750
                    min: 1
                    max: 1500
                    step: 1
                    #size_hint:(None, .5)
                    #width: 200    
                TextInput:
                    id: velo2_textinput
                    # size_hint_x: None
                    # width: 30
                    input_filter: 'int'
                    multiline: False
                    text: str(velo2_slider.value)                        
                Button:
                    id: pub2
                    text: 'Pub'
                    on_press:app.send_position(m2_id.text, m2_slider.value, velo2_slider.value, "pub2")

                # Motor 3
                CheckBox:
                    id: m3_checkbox
                    group: "checkboxes"
                    color: 1, 1, 1, 1
                    on_active: app.test(m3_id.text)
                    size_hint_x: .20
                    canvas.before:
                        Color:
                            rgb: 1,0,0
                        Ellipse:
                            pos:self.center_x-8, self.center_y-8
                            size:[16,16]
                        Color:
                            rgb: 0,0,0
                        Ellipse:
                            pos:self.center_x-7, self.center_y-7
                            size:[14,14]
                Label:
                    text: '6'
                    id: m3_id
                    color: (0,0,1,1)
                    canvas.before:
                        Color:
                            rgba: 1, 1, 1, 1
                        Rectangle:
                            pos: self.pos
                            size: self.size
                Slider:
                    id: m3_slider
                    value: 0
                    min: -180
                    max: 180
                    step: 1
                    #size_hint:(None, .5)sectiition(m2_id.text, m2_slider.value)
                TextInput:
                    id: m3_textinput
                    # size_hint_x: None
                    # width: 30
                    input_filter: 'int'
                    multiline: False
                    text: str(m3_slider.value)
                Label:
                    text: str(m3_slider.value)
                    color: (0,0,1,1)
                    canvas.before:
                        Color:
                            rgba: 1, 1, 1, 1
                        Rectangle:
                            pos: self.pos
                            size: self.size
                Slider:
                    id: velo3_slider
                    value: 750
                    min: 1
                    max: 1500
                    step: 1
                    #size_hint:(None, .5)
                    #width: 200    
                TextInput:
                    id: velo3_textinput
                    # size_hint_x: None
                    # width: 30
                    input_filter: 'int'
                    multiline: False
                    text: str(velo3_slider.value)                           
                Button:
                    id: pub3
                    text: 'Pub'
                    on_press:app.send_position(m3_id.text, m3_slider.value,velo3_slider.value, "pub3")

                # Motor 4
                CheckBox:
                    id: m4_checkbox
                    group: "checkboxes"
                    color: 1, 1, 1, 1
                    on_active: app.test(m4_id.text)
                    size_hint_x: .20
                    canvas.before:
                        Color:
                            rgb: 1,0,0
                        Ellipse:
                            pos:self.center_x-8, self.center_y-8
                            size:[16,16]
                        Color:
                            rgb: 0,0,0
                        Ellipse:
                            pos:self.center_x-7, self.center_y-7
                            size:[14,14]
                Label:
                    text: '8'
                    id: m4_id
                    color: (0,0,1,1)
                    canvas.before:
                        Color:
                            rgba: 1, 1, 1, 1
                        Rectangle:
                            pos: self.pos
                            size: self.size
                Slider:
                    id: m4_slider
                    value: 0
                    min: -180
                    max: 180
                    step: 1
                    #size_hint:(None, .5)sectiition(m2_id.text, m2_slider.value)
                TextInput:
                    id: m4_textinput
                    # size_hint_x: None
                    # width: 30
                    input_filter: 'int'
                    multiline: False
                    text: str(m4_slider.value)
                Label:
                    text: str(m4_slider.value)
                    color: (0,0,1,1)
                    canvas.before:
                        Color:
                            rgba: 1, 1, 1, 1
                        Rectangle:
                            pos: self.pos
                            size: self.size
                Slider:
                    id: velo4_slider
                    value: 750
                    min: 1
                    max: 1500
                    step: 1
                    #size_hint:(None, .5)
                    #width: 200    
                TextInput:
                    id: velo4_textinput
                    # size_hint_x: None
                    # width: 30
                    input_filter: 'int'
                    multiline: False
                    text: str(velo4_slider.value)                           
                Button:
                    id: pub4
                    text: 'Pub'
                    on_press:app.send_position(m4_id.text, m4_slider.value, velo4_slider.value, "pub4")

                # Motor 5
                CheckBox:
                    id: m5_checkbox
                    group: "checkboxes"
                    color: 1, 1, 1, 1
                    on_active: app.test(m5_id.text)
                    size_hint_x: .20
                    canvas.before:
                        Color:
                            rgb: 1,0,0
                        Ellipse:
                            pos:self.center_x-8, self.center_y-8
                            size:[16,16]
                        Color:
                            rgb: 0,0,0
                        Ellipse:
                            pos:self.center_x-7, self.center_y-7
                            size:[14,14]
                Label:
                    text: '10'
                    id: m5_id
                    color: (0,0,1,1)
                    canvas.before:
                        Color:
                            rgba: 1, 1, 1, 1
                        Rectangle:
                            pos: self.pos
                            size: self.size
                Slider:
                    id: m5_slider
                    value: 0
                    min: -180
                    max: 180
                    step: 1
                    #size_hint:(None, .5)sectiition(m2_id.text, m2_slider.value)
                TextInput:
                    id: m5_textinput
                    # size_hint_x: None
                    # width: 30
                    input_filter: 'int'
                    multiline: False
                    text: str(m5_slider.value)
                Label:
                    text: str(m5_slider.value)
                    color: (0,0,1,1)
                    canvas.before:
                        Color:
                            rgba: 1, 1, 1, 1
                        Rectangle:
                            pos: self.pos
                            size: self.size
                Slider:
                    id: velo5_slider
                    value: 750
                    min: 1
                    max: 1500
                    step: 1
                    #size_hint:(None, .5)
                    #width: 200    
                TextInput:
                    id: velo5_textinput
                    # size_hint_x: None
                    # width: 30
                    input_filter: 'int'
                    multiline: False
                    text: str(velo5_slider.value)                           
                Button:
                    id: pub5
                    text: 'Pub'
                    on_press:app.send_position(m5_id.text, m5_slider.value, velo5_slider.value, "pub5")

                # Motor 6
                CheckBox:
                    id: m6_checkbox
                    group: "checkboxes"
                    color: 1, 1, 1, 1
                    on_active: app.test(m6_id.text)
                    size_hint_x: .20
                    canvas.before:
                        Color:
                            rgb: 1,0,0
                        Ellipse:
                            pos:self.center_x-8, self.center_y-8
                            size:[16,16]
                        Color:
                            rgb: 0,0,0
                        Ellipse:
                            pos:self.center_x-7, self.center_y-7
                            size:[14,14]
                Label:
                    text: '12'
                    id: m6_id
                    color: (0,0,1,1)
                    size_hint_x: .40
                    canvas.before:
                        Color:
                            rgba: 1, 1, 1, 1
                        Rectangle:
                            pos: self.pos
                            size: self.size
                Slider:
                    id: m6_slider
                    value: 0
                    min: -180
                    max: 180
                    step: 1
                    #size_hint:(None, .5)sectiition(m2_id.text, m2_slider.value)
                TextInput:
                    id: m6_textinput
                    # size_hint_x: None
                    # width: 30
                    input_filter: 'int'
                    multiline: False
                    text: str(m6_slider.value)
                Label:
                    text: str(m6_slider.value)
                    color: (0,0,1,1)
                    canvas.before:
                        Color:
                            rgba: 1, 1, 1, 1
                        Rectangle:
                            pos: self.pos
                            size: self.size
                Slider:
                    id: velo6_slider
                    value: 750
                    min: 1
                    max: 1500
                    step: 1
                    #size_hint:(None, .5)
                    #width: 200    
                TextInput:
                    id: velo6_textinput
                    # size_hint_x: None
                    # width: 30
                    input_filter: 'int'
                    multiline: False
                    text: str(velo6_slider.value)                           
                Button:
                    id: pub6
                    text: 'Pub'
                    on_press:app.send_position(m6_id.text, m6_slider.value, velo6_slider.value, "pub6")

                # Motor 7
                CheckBox:
                    id: m7_checkbox
                    group: "checkboxes"
                    color: 1, 1, 1, 1
                    on_active: app.test(m7_id.text)
                    size_hint_x: .20
                    canvas.before:
                        Color:
                            rgb: 1,0,0
                        Ellipse:
                            pos:self.center_x-8, self.center_y-8
                            size:[16,16]
                        Color:
                            rgb: 0,0,0
                        Ellipse:
                            pos:self.center_x-7, self.center_y-7
                            size:[14,14]
                Label:
                    text: '14'
                    id: m7_id
                    color: (0,0,1,1)
                    canvas.before:
                        Color:
                            rgba: 1, 1, 1, 1
                        Rectangle:
                            pos: self.pos
                            size: self.size
                Slider:
                    id: m7_slider
                    value: 0
                    min: -180
                    max: 180
                    step: 1
                    #size_hint:(None, .5)sectiition(m2_id.text, m2_slider.value)
                TextInput:
                    id: m7_textinput
                    # size_hint_x: None
                    # width: 30
                    input_filter: 'int'
                    multiline: False
                    text: str(m7_slider.value)
                Label:
                    text: str(m7_slider.value)
                    color: (0,0,1,1)
                    canvas.before:
                        Color:
                            rgba: 1, 1, 1, 1
                        Rectangle:
                            pos: self.pos
                            size: self.size
                Slider:
                    id: velo7_slider
                    value: 750
                    min: 1
                    max: 1500
                    step: 1
                    #size_hint:(None, .5)
                    #width: 200    
                TextInput:
                    id: velo7_textinput
                    # size_hint_x: None
                    # width: 30
                    input_filter: 'int'
                    multiline: False
                    text: str(velo7_slider.value)                           
                Button:
                    id: pub7
                    text: 'Pub'
                    on_press:app.send_position(m7_id.text, m7_slider.value, velo7_slider.value, "pub7")

                #hand motors
                CheckBox:
                    id: m8_checkbox
                    group: "checkboxes"
                    color: 1, 1, 1, 1
                    on_active: app.test(m8_id.text)
                    size_hint_x: .20
                    canvas.before:
                        Color:
                            rgb: 1,0,0
                        Ellipse:
                            pos:self.center_x-8, self.center_y-8
                            size:[16,16]
                        Color:
                            rgb: 0,0,0
                        Ellipse:
                            pos:self.center_x-7, self.center_y-7
                            size:[14,14]
                Label:
                    text: '32'
                    id: m8_id
                    color: (0,0,1,1)
                    canvas.before:
                        Color:
                            rgba: 1, 1, 1, 1
                        Rectangle:
                            pos: self.pos
                            size: self.size
                Slider:
                    id: m8_slider
                    value: 0
                    min: 0
                    max: 5
                    step: 1
                    #size_hint:(None, .5)sectiition(m2_id.text, m2_slider.value)
                TextInput:
                    id: m8_textinput
                    # size_hint_x: None
                    # width: 30
                    input_filter: 'int'
                    multiline: False
                    text: str(m8_slider.value)
                Label:
                    text: str(m8_slider.value)
                    color: (0,0,1,1)
                    canvas.before:
                        Color:
                            rgba: 1, 1, 1, 1
                        Rectangle:
                            pos: self.pos
                            size: self.size
                Slider:
                    id: velo8_slider
                    value: 150
                    min: 1
                    max: 380
                    step: 1
                    #size_hint:(None, .5)
                    #width: 200    
                TextInput:
                    id: velo8_textinput
                    # size_hint_x: None
                    # width: 30
                    input_filter: 'int'
                    multiline: False
                    text: str(velo8_slider.value)                          
                Button:
                    id: pub8
                    text: 'Pub'
                    on_press:app.send_position(m8_id.text, m8_slider.value, velo8_slider.value, "pub8")
                
                CheckBox:
                    id: m9_checkbox
                    group: "checkboxes"
                    color: 1, 1, 1, 1
                    on_active: app.test(m9_id.text)
                    size_hint_x: .20
                    canvas.before:
                        Color:
                            rgb: 1,0,0
                        Ellipse:
                            pos:self.center_x-8, self.center_y-8
                            size:[16,16]
                        Color:
                            rgb: 0,0,0
                        Ellipse:
                            pos:self.center_x-7, self.center_y-7
                            size:[14,14]
                Label:
                    text: '34'
                    id: m9_id
                    color: (0,0,1,1)
                    canvas.before:
                        Color:
                            rgba: 1, 1, 1, 1
                        Rectangle:
                            pos: self.pos
                            size: self.size
                Slider:
                    id: m9_slider
                    value: 0
                    min: 0
                    max: 5
                    step: 1
                    #size_hint:(None, .5)sectiition(m2_id.text, m2_slider.value)
                TextInput:
                    id: m9_textinput
                    # size_hint_x: None
                    # width: 30
                    input_filter: 'int'
                    multiline: False
                    text: str(m9_slider.value)
                Label:
                    text: str(m9_slider.value)
                    color: (0,0,1,1)
                    canvas.before:
                        Color:
                            rgba: 1, 1, 1, 1
                        Rectangle:
                            pos: self.pos
                            size: self.size
                Slider:
                    id: velo9_slider
                    value: 150
                    min: 1
                    max: 380
                    step: 1
                    #size_hint:(None, .5)
                    #width: 200    
                TextInput:
                    id: velo9_textinput
                    # size_hint_x: None
                    # width: 30
                    input_filter: 'int'
                    multiline: False
                    text: str(velo9_slider.value)                          
                Button:
                    id: pub9
                    text: 'Pub'
                    on_press:app.send_position(m9_id.text, m9_slider.value, velo9_slider.value, "pub9")

                
                CheckBox:
                    id: m10_checkbox
                    group: "checkboxes"
                    color: 1, 1, 1, 1
                    on_active: app.test(m10_id.text)
                    size_hint_x: .20
                    canvas.before:
                        Color:
                            rgb: 1,0,0
                        Ellipse:
                            pos:self.center_x-8, self.center_y-8
                            size:[16,16]
                        Color:
                            rgb: 0,0,0
                        Ellipse:
                            pos:self.center_x-7, self.center_y-7
                            size:[14,14]
                Label:
                    text: '36'
                    id: m10_id
                    color: (0,0,1,1)
                    canvas.before:
                        Color:
                            rgba: 1, 1, 1, 1
                        Rectangle:
                            pos: self.pos
                            size: self.size
                Slider:
                    id: m10_slider
                    value: 0
                    min: 0
                    max: 5
                    step: 1
                    #size_hint:(None, .5)sectiition(m2_id.text, m2_slider.value)
                TextInput:
                    id: m10_textinput
                    # size_hint_x: None
                    # width: 30
                    input_filter: 'int'
                    multiline: False
                    text: str(m10_slider.value)
                Label:
                    text: str(m10_slider.value)
                    color: (0,0,1,1)
                    canvas.before:
                        Color:
                            rgba: 1, 1, 1, 1
                        Rectangle:
                            pos: self.pos
                            size: self.size
                Slider:
                    id: velo10_slider
                    value: 150
                    min: 1
                    max: 380
                    step: 1
                    #size_hint:(None, .5)
                    #width: 200    
                TextInput:
                    id: velo10_textinput
                    # size_hint_x: None
                    # width: 30
                    input_filter: 'int'
                    multiline: False
                    text: str(velo10_slider.value)                          
                Button:
                    id: pub10
                    text: 'Pub'
                    on_press:app.send_position(m10_id.text, m10_slider.value, velo10_slider.value, "pub10")
            
        MDTabs:
            text_color_active: [0, 0, 0, 1]
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

        Button:
            id: refresh_btn
            text: "Refresh"
            halign: "center"
            size_hint: 0.15, 0.1
            pos_hint: {"center_x": .5, "center_y": 0.5}
            on_press: root.change_image("new_robotImage.png")

        Button:
            id: run_btn
            text: "Run"
            halign: "center"
            size_hint: 0.15, 0.1
            pos_hint: {"center_x": .1, "center_y": 0.2}
        
        Button:
            id: emergency
            text: "EM-STOP"
            halign: "center"
            background_color: 1,0,0,1
            size_hint: 0.15, 0.1
            pos_hint: {"center_x": .35, "center_y": 0.2}
            on_press: app.emergency_stop()

        Button:
            id: btn 
            text: "Load Demo:"
            halign: "center"
            size_hint: 0.25, 0.1
            pos_hint: {"center_x": .65, "center_y": 0.8}
            on_parent: drop_content.dismiss()
            on_release: drop_content.open(self)

        DropDown:
            id: drop_content
            on_select: btn.text = '{}'.format(args[1])

            Button:
                id: btn1
                text: 'First Item'
                size_hint_y: None
                height: 35
                on_release: drop_content.select('First Item')
            
            Button:
                id: btn2
                text: 'Second Item'
                size_hint_y: None
                height: 35
                on_release: drop_content.select('Second Item')

        FloatLayout:
            #pos_hint: {"right": 1}
            # size_hint: 0.5, 0.5
            GridLayout:
                pos_hint: {"center_x": 0.5, "center_y": 0.4}
                col_default_width: '56dp'
                size_hint_x: None
                cols: 8
                row_default_height: '32dp'
                size_hint_y: None
                height: self.minimum_height
                Label:
                    text: 'SENSOR'
                    color: (0,0,0,1)
                Label:
                    text: 'ID'
                    color: (0,0,0,1)
                Label:
                    text: 'POS CONTROL' 
                    color: (0,0,0,1)           
                Label:
                    text: 'GOAL POS'  
                    color: (0,0,0,1) 
                Label:
                    text: 'CUR POS'
                    color: (0,0,0,1)
                Label:
                    text: 'VEL CONTROL'
                    color: (0,0,0,1) 
                Label:
                    text: 'GOAL VEL' 
                    color: (0,0,0,1)
                Label:
                    text: 'SEND POS'
                    color: (0,0,0,1)
                # Motor 1
                CheckBox:
                    id: m1_checkbox
                    group: "checkboxes"
                    color: 1, 1, 1, 1
                    on_active: app.test(m1_id.text)
                    size_hint_x: .20
                    canvas.before:
                        Color:
                            rgb: 1,0,0
                        Ellipse:
                            pos:self.center_x-8, self.center_y-8
                            size:[16,16]
                        Color:
                            rgb: 0,0,0
                        Ellipse:
                            pos:self.center_x-7, self.center_y-7
                            size:[14,14]
                Label:
                    text: '1'
                    id: m1_id
                    color: (0,0,1,1)
                    canvas.before:
                        Color:
                            rgba: 1, 1, 1, 1
                        Rectangle:
                            pos: self.pos
                            size: self.size
                Slider:
                    id: m1_slider
                    value: 0
                    min: -180
                    max: 180
                    step: 1
                    #size_hint:(None, .5)
                    #width: 200
                TextInput:
                    id: m1_textinput
                    # size_hint_x: None
                    # width: 30
                    input_filter: 'int'
                    multiline: False
                    text: str(m1_slider.value)
                Label:
                    text: str(m1_slider.value)
                    color: (0,0,1,1)
                    canvas.before:
                        Color:
                            rgba: 1, 1, 1, 1
                        Rectangle:
                            pos: self.pos
                            size: self.size
                Slider:
                    id: velo1_slider
                    value: 750
                    min: 1
                    max: 1500
                    step: 1
                    #size_hint:(None, .5)
                    #width: 200    
                TextInput:
                    id: velo1_textinput
                    # size_hint_x: None
                    # width: 30
                    input_filter: 'int'
                    multiline: False
                    text: str(velo1_slider.value)
                Button:
                    id: pub1
                    text: 'Pub'
                    on_press:app.send_position(m1_id.text, m1_slider.value, "pub1")
                # Motor 2
                CheckBox:
                    id: m2_checkbox
                    group: "checkboxes"
                    color: 1, 1, 1, 1
                    on_active: app.test(m2_id.text)
                    size_hint_x: .20
                    canvas.before:
                        Color:
                            rgb: 1,0,0
                        Ellipse:
                            pos:self.center_x-8, self.center_y-8
                            size:[16,16]
                        Color:
                            rgb: 0,0,0
                        Ellipse:
                            pos:self.center_x-7, self.center_y-7
                            size:[14,14]
                Label:
                    text: '3'
                    id: m2_id
                    color: (0,0,1,1)
                    canvas.before:
                        Color:
                            rgba: 1, 1, 1, 1
                        Rectangle:
                            pos: self.pos
                            size: self.size
                Slider:
                    id: m2_slider
                    value: 0
                    min: -180
                    max: 180
                    step: 1
                    #size_hint:(None, .5)sectiition(m2_id.text, m2_slider.value)
                TextInput:
                    id: m2_textinput
                    # size_hint_x: None
                    # width: 30
                    input_filter: 'int'
                    multiline: False
                    text: str(m2_slider.value)
                Label:
                    text: str(m2_slider.value)
                    color: (0,0,1,1)
                    canvas.before:
                        Color:
                            rgba: 1, 1, 1, 1
                        Rectangle:
                            pos: self.pos
                            size: self.size
                Slider:
                    id: velo2_slider
                    value: 750
                    min: 1
                    max: 1500
                    step: 1
                    #size_hint:(None, .5)
                    #width: 200    
                TextInput:
                    id: velo2_textinput
                    # size_hint_x: None
                    # width: 30
                    input_filter: 'int'
                    multiline: False
                    text: str(velo2_slider.value)
                Button:
                    id: pub2
                    text: 'Pub'
                    on_press:app.send_position(m2_id.text, m2_slider.value, "pub2")

                # Motor 3
                CheckBox:
                    id: m3_checkbox
                    group: "checkboxes"
                    color: 1, 1, 1, 1
                    on_active: app.test(m3_id.text)
                    size_hint_x: .20
                    canvas.before:
                        Color:
                            rgb: 1,0,0
                        Ellipse:
                            pos:self.center_x-8, self.center_y-8
                            size:[16,16]
                        Color:
                            rgb: 0,0,0
                        Ellipse:
                            pos:self.center_x-7, self.center_y-7
                            size:[14,14]
                Label:
                    text: '5'
                    id: m3_id
                    color: (0,0,1,1)
                    canvas.before:
                        Color:
                            rgba: 1, 1, 1, 1
                        Rectangle:
                            pos: self.pos
                            size: self.size
                Slider:
                    id: m3_slider
                    value: 0
                    min: -180
                    max: 180
                    step: 1
                    #size_hint:(None, .5)sectiition(m2_id.text, m2_slider.value)
                TextInput:
                    id: m3_textinput
                    # size_hint_x: None
                    # width: 30
                    input_filter: 'int'
                    multiline: False
                    text: str(m3_slider.value)
                Label:
                    text: str(m3_slider.value)
                    color: (0,0,1,1)
                    canvas.before:
                        Color:
                            rgba: 1, 1, 1, 1
                        Rectangle:
                            pos: self.pos
                            size: self.size
                Slider:
                    id: velo3_slider
                    value: 750
                    min: 1
                    max: 1500
                    step: 1
                    #size_hint:(None, .5)
                    #width: 200    
                TextInput:
                    id: velo3_textinput
                    # size_hint_x: None
                    # width: 30
                    input_filter: 'int'
                    multiline: False
                    text: str(velo3_slider.value)
                Button:
                    id: pub3
                    text: 'Pub'
                    on_press:app.send_position(m3_id.text, m3_slider.value, "pub3")

                # Motor 4
                CheckBox:
                    id: m4_checkbox
                    group: "checkboxes"
                    color: 1, 1, 1, 1
                    on_active: app.test(m4_id.text)
                    size_hint_x: .20
                    canvas.before:
                        Color:
                            rgb: 1,0,0
                        Ellipse:
                            pos:self.center_x-8, self.center_y-8
                            size:[16,16]
                        Color:
                            rgb: 0,0,0
                        Ellipse:
                            pos:self.center_x-7, self.center_y-7
                            size:[14,14]
                Label:
                    text: '7'
                    id: m4_id
                    color: (0,0,1,1)
                    canvas.before:
                        Color:
                            rgba: 1, 1, 1, 1
                        Rectangle:
                            pos: self.pos
                            size: self.size
                Slider:
                    id: m4_slider
                    value: 0
                    min: -180
                    max: 180
                    step: 1
                    #size_hint:(None, .5)sectiition(m2_id.text, m2_slider.value)
                TextInput:
                    id: m4_textinput
                    # size_hint_x: None
                    # width: 30
                    input_filter: 'int'
                    multiline: False
                    text: str(m4_slider.value)
                Label:
                    text: str(m4_slider.value)
                    color: (0,0,1,1)
                    canvas.before:
                        Color:
                            rgba: 1, 1, 1, 1
                        Rectangle:
                            pos: self.pos
                            size: self.size
                Slider:
                    id: velo4_slider
                    value: 750
                    min: 1
                    max: 1500
                    step: 1
                    #size_hint:(None, .5)
                    #width: 200    
                TextInput:
                    id: velo4_textinput
                    # size_hint_x: None
                    # width: 30
                    input_filter: 'int'
                    multiline: False
                    text: str(velo4_slider.value)
                Button:
                    id: pub4
                    text: 'Pub'
                    on_press:app.send_position(m4_id.text, m4_slider.value, "pub4")

                # Motor 5
                CheckBox:
                    id: m5_checkbox
                    group: "checkboxes"
                    color: 1, 1, 1, 1
                    on_active: app.test(m5_id.text)
                    size_hint_x: .20
                    canvas.before:
                        Color:
                            rgb: 1,0,0
                        Ellipse:
                            pos:self.center_x-8, self.center_y-8
                            size:[16,16]
                        Color:
                            rgb: 0,0,0
                        Ellipse:
                            pos:self.center_x-7, self.center_y-7
                            size:[14,14]
                Label:
                    text: '9'
                    id: m5_id
                    color: (0,0,1,1)
                    canvas.before:
                        Color:
                            rgba: 1, 1, 1, 1
                        Rectangle:
                            pos: self.pos
                            size: self.size
                Slider:
                    id: m5_slider
                    value: 0
                    min: -180
                    max: 180
                    step: 1
                    #size_hint:(None, .5)sectiition(m2_id.text, m2_slider.value)
                TextInput:
                    id: m5_textinput
                    # size_hint_x: None
                    # width: 30
                    input_filter: 'int'
                    multiline: False
                    text: str(m5_slider.value)
                Label:
                    text: str(m5_slider.value)
                    color: (0,0,1,1)
                    canvas.before:
                        Color:
                            rgba: 1, 1, 1, 1
                        Rectangle:
                            pos: self.pos
                            size: self.size
                Slider:
                    id: velo5_slider
                    value: 750
                    min: 1
                    max: 1500
                    step: 1
                    #size_hint:(None, .5)
                    #width: 200    
                TextInput:
                    id: velo5_textinput
                    # size_hint_x: None
                    # width: 30
                    input_filter: 'int'
                    multiline: False
                    text: str(velo5_slider.value)
                Button:
                    id: pub5
                    text: 'Pub'
                    on_press:app.send_position(m5_id.text, m5_slider.value, "pub5")

                # Motor 6
                CheckBox:
                    id: m6_checkbox
                    group: "checkboxes"
                    color: 1, 1, 1, 1
                    on_active: app.test(m6_id.text)
                    size_hint_x: .20
                    canvas.before:
                        Color:
                            rgb: 1,0,0
                        Ellipse:
                            pos:self.center_x-8, self.center_y-8
                            size:[16,16]
                        Color:
                            rgb: 0,0,0
                        Ellipse:
                            pos:self.center_x-7, self.center_y-7
                            size:[14,14]
                Label:
                    text: '11'
                    id: m6_id
                    color: (0,0,1,1)
                    size_hint_x: .40
                    canvas.before:
                        Color:
                            rgba: 1, 1, 1, 1
                        Rectangle:
                            pos: self.pos
                            size: self.size
                Slider:
                    id: m6_slider
                    value: 0
                    min: -180
                    max: 180
                    step: 1
                    #size_hint:(None, .5)sectiition(m2_id.text, m2_slider.value)
                TextInput:
                    id: m6_textinput
                    # size_hint_x: None
                    # width: 30
                    input_filter: 'int'
                    multiline: False
                    text: str(m6_slider.value)
                Label:
                    text: str(m6_slider.value)
                    color: (0,0,1,1)
                    canvas.before:
                        Color:
                            rgba: 1, 1, 1, 1
                        Rectangle:
                            pos: self.pos
                            size: self.size
                Slider:
                    id: velo6_slider
                    value: 750
                    min: 1
                    max: 1500
                    step: 1
                    #size_hint:(None, .5)
                    #width: 200    
                TextInput:
                    id: velo6_textinput
                    # size_hint_x: None
                    # width: 30
                    input_filter: 'int'
                    multiline: False
                    text: str(velo6_slider.value)
                Button:
                    id: pub6
                    text: 'Pub'
                    on_press:app.send_position(m6_id.text, m6_slider.value, "pub6")

                # Motor 7
                CheckBox:
                    id: m7_checkbox
                    group: "checkboxes"
                    color: 1, 1, 1, 1
                    on_active: app.test(m7_id.text)
                    size_hint_x: .20
                    canvas.before:
                        Color:
                            rgb: 1,0,0
                        Ellipse:
                            pos:self.center_x-8, self.center_y-8
                            size:[16,16]
                        Color:
                            rgb: 0,0,0
                        Ellipse:
                            pos:self.center_x-7, self.center_y-7
                            size:[14,14]
                Label:
                    text: '13'
                    id: m7_id
                    color: (0,0,1,1)
                    canvas.before:
                        Color:
                            rgba: 1, 1, 1, 1
                        Rectangle:
                            pos: self.pos
                            size: self.size
                Slider:
                    id: m7_slider
                    value: 0
                    min: -180
                    max: 180
                    step: 1
                    #size_hint:(None, .5)sectiition(m2_id.text, m2_slider.value)
                TextInput:
                    id: m7_textinput
                    # size_hint_x: None
                    # width: 30
                    input_filter: 'int'
                    multiline: False
                    text: str(m7_slider.value)
                Label:
                    text: str(m7_slider.value)
                    color: (0,0,1,1)
                    canvas.before:
                        Color:
                            rgba: 1, 1, 1, 1
                        Rectangle:
                            pos: self.pos
                            size: self.size
                Slider:
                    id: velo7_slider
                    value: 750
                    min: 1
                    max: 1500
                    step: 1
                    #size_hint:(None, .5)
                    #width: 200    
                TextInput:
                    id: velo7_textinput
                    # size_hint_x: None
                    # width: 30
                    input_filter: 'int'
                    multiline: False
                    text: str(velo7_slider.value)
                Button:
                    id: pub7
                    text: 'Pub'
                    on_press:app.send_position(m7_id.text, m7_slider.value, "pub7")
                
                #hand motors
                CheckBox:
                    id: m8_checkbox
                    group: "checkboxes"
                    color: 1, 1, 1, 1
                    on_active: app.test(m8_id.text)
                    size_hint_x: .20
                    canvas.before:
                        Color:
                            rgb: 1,0,0
                        Ellipse:
                            pos:self.center_x-8, self.center_y-8
                            size:[16,16]
                        Color:
                            rgb: 0,0,0
                        Ellipse:
                            pos:self.center_x-7, self.center_y-7
                            size:[14,14]
                Label:
                    text: '31'
                    id: m8_id
                    color: (0,0,1,1)
                    canvas.before:
                        Color:
                            rgba: 1, 1, 1, 1
                        Rectangle:
                            pos: self.pos
                            size: self.size
                            
                Slider:
                    id: m8_slider
                    value: 0
                    min: 0
                    max: 5
                    step: 1
                    #size_hint:(None, .5)sectiition(m2_id.text, m2_slider.value)
                TextInput:
                    id: m8_textinput
                    # size_hint_x: None
                    # width: 30
                    input_filter: 'int'
                    multiline: False
                    text: str(m8_slider.value)
                Label:
                    text: str(m8_slider.value)
                    color: (0,0,1,1)
                    canvas.before:
                        Color:
                            rgba: 1, 1, 1, 1
                        Rectangle:
                            pos: self.pos
                            size: self.size
                Slider:
                    id: velo8_slider
                    value: 750
                    min: 1
                    max: 1500
                    step: 1
                    #size_hint:(None, .5)
                    #width: 200    
                TextInput:
                    id: velo8_textinput
                    # size_hint_x: None
                    # width: 30
                    input_filter: 'int'
                    multiline: False
                    text: str(velo8_slider.value)
                Button:
                    id: pub8
                    text: 'Pub'
                    on_press:app.send_position(m8_id.text, m8_slider.value, "pub8")
                

                
                CheckBox:
                    id: m9_checkbox
                    group: "checkboxes"
                    color: 1, 1, 1, 1
                    on_active: app.test(m9_id.text)
                    size_hint_x: .20
                    canvas.before:
                        Color:
                            rgb: 1,0,0
                        Ellipse:
                            pos:self.center_x-8, self.center_y-8
                            size:[16,16]
                        Color:
                            rgb: 0,0,0
                        Ellipse:
                            pos:self.center_x-7, self.center_y-7
                            size:[14,14]
                Label:
                    text: '33'
                    id: m9_id
                    color: (0,0,1,1)
                    canvas.before:
                        Color:
                            rgba: 1, 1, 1, 1
                        Rectangle:
                            pos: self.pos
                            size: self.size
                Slider:
                    id: m9_slider
                    value: 0
                    min: 0
                    max: 5
                    step: 1
                    #size_hint:(None, .5)sectiition(m2_id.text, m2_slider.value)
                TextInput:
                    id: m9_textinput
                    # size_hint_x: None
                    # width: 30
                    input_filter: 'int'
                    multiline: False
                    text: str(m9_slider.value)
                Label:
                    text: str(m9_slider.value)
                    color: (0,0,1,1)
                    canvas.before:
                        Color:
                            rgba: 1, 1, 1, 1
                        Rectangle:
                            pos: self.pos
                            size: self.size
                Slider:
                    id: velo9_slider
                    value: 750
                    min: 1
                    max: 1500
                    step: 1
                    #size_hint:(None, .5)
                    #width: 200    
                TextInput:
                    id: velo9_textinput
                    # size_hint_x: None
                    # width: 30
                    input_filter: 'int'
                    multiline: False
                    text: str(velo9_slider.value)
                Button:
                    id: pub9
                    text: 'Pub'
                    on_press:app.send_position(m9_id.text, m9_slider.value, "pub9")

                
                
                CheckBox:
                    id: m10_checkbox
                    group: "checkboxes"
                    color: 1, 1, 1, 1
                    on_active: app.test(m10_id.text)
                    size_hint_x: .20
                    canvas.before:
                        Color:
                            rgb: 1,0,0
                        Ellipse:
                            pos:self.center_x-8, self.center_y-8
                            size:[16,16]
                        Color:
                            rgb: 0,0,0
                        Ellipse:
                            pos:self.center_x-7, self.center_y-7
                            size:[14,14]
                Label:
                    text: '35'
                    id: m10_id
                    color: (0,0,1,1)
                    canvas.before:
                        Color:
                            rgba: 1, 1, 1, 1
                        Rectangle:
                            pos: self.pos
                            size: self.size
                Slider:
                    id: m10_slider
                    value: 0
                    min: 0
                    max: 5
                    step: 1
                    #size_hint:(None, .5)sectiition(m2_id.text, m2_slider.value)
                TextInput:
                    id: m10_textinput
                    # size_hint_x: None
                    # width: 30
                    input_filter: 'int'
                    multiline: False
                    text: str(m10_slider.value)
                Label:
                    text: str(m10_slider.value)
                    color: (0,0,1,1)
                    canvas.before:
                        Color:
                            rgba: 1, 1, 1, 1
                        Rectangle:
                            pos: self.pos
                            size: self.size
                Slider:
                    id: velo10_slider
                    value: 750
                    min: 1
                    max: 1500
                    step: 1
                    #size_hint:(None, .5)
                    #width: 200    
                TextInput:
                    id: velo10_textinput
                    # size_hint_x: None
                    # width: 30
                    input_filter: 'int'
                    multiline: False
                    text: str(velo10_slider.value)
                Button:
                    id: pub10
                    text: 'Pub'
                    on_press:app.send_position(m10_id.text, m10_slider.value, "pub10")
            
        MDTabs:
            text_color_active: [0, 0, 0, 1]
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

        Button:
            id: refresh_btn
            text: "Refresh"
            halign: "center"
            size_hint: 0.15, 0.1
            pos_hint: {"center_x": .5, "center_y": 0.5}
            on_press: root.change_image("new_robotImage.png")

        Button:
            id: run_btn
            text: "Run"
            halign: "center"
            size_hint: 0.15, 0.1
            pos_hint: {"center_x": .1, "center_y": 0.2}
        
        Button:
            id: emergency
            text: "EM-STOP"
            halign: "center"
            background_color: 1,0,0,1
            size_hint: 0.15, 0.1
            pos_hint: {"center_x": .35, "center_y": 0.2}
            on_press: app.emergency_stop()

        Button:
            id: btn 
            text: "Load Demo:"
            halign: "center"
            size_hint: 0.25, 0.1
            pos_hint: {"center_x": .65, "center_y": 0.8}
            on_parent: drop_content.dismiss()
            on_release: drop_content.open(self)

        DropDown:
            id: drop_content
            on_select: btn.text = '{}'.format(args[1])

            Button:
                id: btn1
                text: 'First Item'
                size_hint_y: None
                height: 35
                on_release: drop_content.select('First Item')
            
            Button:
                id: btn2
                text: 'Second Item'
                size_hint_y: None
                height: 35
                on_release: drop_content.select('Second Item')

        FloatLayout:
            #pos_hint: {"right": 1}
            # size_hint: 0.5, 0.5
            GridLayout:
                pos_hint: {"center_x": 0.5, "center_y": 0.4}
                col_default_width: '56dp'
                size_hint_x: None
                cols: 8
                row_default_height: '32dp'
                size_hint_y: None
                height: self.minimum_height
                Label:
                    text: 'SENSOR'
                    color: (0,0,0,1)
                Label:
                    text: 'ID'
                    color: (0,0,0,1)
                Label:
                    text: 'POS CONTROL' 
                    color: (0,0,0,1)              
                Label:
                    text: 'GOAL POS'  
                    color: (0,0,0,1) 
                Label:
                    text: 'CUR POS'
                    color: (0,0,0,1)
                Label:
                    text: 'VEL CONTROL'
                    color: (0,0,0,1) 
                Label:
                    text: 'GOAL VEL' 
                    color: (0,0,0,1)
                Label:
                    text: 'SEND POS'
                    color: (0,0,0,1)
                # Motor 1
                CheckBox:
                    id: m1_checkbox
                    group: "checkboxes"
                    color: 1, 1, 1, 1
                    on_active: app.test(m1_id.text)
                    size_hint_x: .20
                    canvas.before:
                        Color:
                            rgb: 1,0,0
                        Ellipse:
                            pos:self.center_x-8, self.center_y-8
                            size:[16,16]
                        Color:
                            rgb: 0,0,0
                        Ellipse:
                            pos:self.center_x-7, self.center_y-7
                            size:[14,14]
                Label:
                    text: '41'
                    id: m1_id
                    color: (0,0,1,1)
                    canvas.before:
                        Color:
                            rgba: 1, 1, 1, 1
                        Rectangle:
                            pos: self.pos
                            size: self.size
                Slider:
                    id: m1_slider
                    value: 0
                    min: -180
                    max: 180
                    step: 1
                    #size_hint:(None, .5)
                    #width: 200
                TextInput:
                    id: m1_textinput
                    # size_hint_x: None
                    # width: 30
                    input_filter: 'int'
                    multiline: False
                    text: str(m1_slider.value)
                Label:
                    text: str(m1_slider.value)
                    color: (0,0,1,1)
                    canvas.before:
                        Color:
                            rgba: 1, 1, 1, 1
                        Rectangle:
                            pos: self.pos
                            size: self.size
                Slider:
                    id: velo1_slider
                    value: 750
                    min: 1
                    max: 1500
                    step: 1
                    #size_hint:(None, .5)
                    #width: 200    
                TextInput:
                    id: velo1_textinput
                    # size_hint_x: None
                    # width: 30
                    input_filter: 'int'
                    multiline: False
                    text: str(velo1_slider.value)
                Button:
                    id: pub1
                    text: 'Pub'
                    on_press:app.send_position(m1_id.text, m1_slider.value, "pub1")
                # Motor 2
                CheckBox:
                    id: m2_checkbox
                    group: "checkboxes"
                    color: 1, 1, 1, 1
                    on_active: app.test(m2_id.text)
                    size_hint_x: .20
                    canvas.before:
                        Color:
                            rgb: 1,0,0
                        Ellipse:
                            pos:self.center_x-8, self.center_y-8
                            size:[16,16]
                        Color:
                            rgb: 0,0,0
                        Ellipse:
                            pos:self.center_x-7, self.center_y-7
                            size:[14,14]
                Label:
                    text: '42'
                    id: m2_id
                    color: (0,0,1,1)
                    canvas.before:
                        Color:
                            rgba: 1, 1, 1, 1
                        Rectangle:
                            pos: self.pos
                            size: self.size
                Slider:
                    id: m2_slider
                    value: 0
                    min: -180
                    max: 180
                    step: 1
                    #size_hint:(None, .5)sectiition(m2_id.text, m2_slider.value)
                TextInput:
                    id: m2_textinput
                    # size_hint_x: None
                    # width: 30
                    input_filter: 'int'
                    multiline: False
                    text: str(m2_slider.value)
                Label:
                    text: str(m2_slider.value)
                    color: (0,0,1,1)
                    canvas.before:
                        Color:
                            rgba: 1, 1, 1, 1
                        Rectangle:
                            pos: self.pos
                            size: self.size
                Slider:
                    id: velo2_slider
                    value: 750
                    min: 1
                    max: 1500
                    step: 1
                    #size_hint:(None, .5)
                    #width: 200    
                TextInput:
                    id: velo2_textinput
                    # size_hint_x: None
                    # width: 30
                    input_filter: 'int'
                    multiline: False
                    text: str(velo2_slider.value)    
                Button:
                    id: pub2
                    text: 'Pub'
                    on_press:app.send_position(m2_id.text, m2_slider.value, "pub2")
            
        MDTabs:
            text_color_active: [0, 0, 0, 1]
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

        Button:
            id: refresh_btn
            text: "Refresh"
            halign: "center"
            size_hint: 0.15, 0.1
            pos_hint: {"center_x": .5, "center_y": 0.5}
            on_press: root.change_image("new_robotImage.png")

        Button:
            id: run_btn
            text: "Run"
            halign: "center"
            size_hint: 0.15, 0.1
            pos_hint: {"center_x": .1, "center_y": 0.2}
        
        Button:
            id: emergency
            text: "EM-STOP"
            halign: "center"
            background_color: 1,0,0,1
            size_hint: 0.15, 0.1
            pos_hint: {"center_x": .35, "center_y": 0.2}
            on_press: app.emergency_stop()

        Button:
            id: btn 
            text: "Load Demo:"
            halign: "center"
            size_hint: 0.25, 0.1
            pos_hint: {"center_x": .65, "center_y": 0.8}
            on_parent: drop_content.dismiss()
            on_release: drop_content.open(self)

        DropDown:
            id: drop_content
            on_select: btn.text = '{}'.format(args[1])

            Button:
                id: btn1
                text: 'First Item'
                size_hint_y: None
                height: 35
                on_release: drop_content.select('First Item')
            
            Button:
                id: btn2
                text: 'Second Item'
                size_hint_y: None
                height: 35
                on_release: drop_content.select('Second Item')

        FloatLayout:
            #pos_hint: {"right": 1}
            # size_hint: 0.5, 0.5
            GridLayout:
                pos_hint: {"center_x": 0.5, "center_y": 0.4}
                col_default_width: '56dp'
                size_hint_x: None
                cols: 8
                row_default_height: '32dp'
                size_hint_y: None
                height: self.minimum_height
                Label:
                    text: 'SENSOR'
                    color: (0,0,0,1)
                Label:
                    text: 'ID'
                    color: (0,0,0,1)
                Label:
                    text: 'POS CONTROL' 
                    color: (0,0,0,1)              
                Label:
                    text: 'GOAL POS'  
                    color: (0,0,0,1) 
                Label:
                    text: 'CUR POS'
                    color: (0,0,0,1)
                Label:
                    text: 'VEL CONTROL'
                    color: (0,0,0,1) 
                Label:
                    text: 'GOAL VEL' 
                    color: (0,0,0,1)
                Label:
                    text: 'SEND POS'
                    color: (0,0,0,1)
                # Motor 1
                CheckBox:
                    id: m1_checkbox
                    group: "checkboxes"
                    color: 1, 1, 1, 1
                    on_active: app.test(m1_id.text)
                    size_hint_x: .20
                    canvas.before:
                        Color:
                            rgb: 1,0,0
                        Ellipse:
                            pos:self.center_x-8, self.center_y-8
                            size:[16,16]
                        Color:
                            rgb: 0,0,0
                        Ellipse:
                            pos:self.center_x-7, self.center_y-7
                            size:[14,14]
                Label:
                    text: '27'
                    id: m1_id
                    color: (0,0,1,1)
                    canvas.before:
                        Color:
                            rgba: 1, 1, 1, 1
                        Rectangle:
                            pos: self.pos
                            size: self.size
                Slider:
                    id: m1_slider
                    value: 0
                    min: -180
                    max: 180
                    step: 1
                    #size_hint:(None, .5)
                    #width: 200
                TextInput:
                    id: m1_textinput
                    # size_hint_x: None
                    # width: 30
                    input_filter: 'int'
                    multiline: False
                    text: str(m1_slider.value)
                Label:
                    text: str(m1_slider.value)
                    color: (0,0,1,1)
                    canvas.before:
                        Color:
                            rgba: 1, 1, 1, 1
                        Rectangle:
                            pos: self.pos
                            size: self.size
                Slider:
                    id: velo1_slider
                    value: 750
                    min: 1
                    max: 1500
                    step: 1
                    #size_hint:(None, .5)
                    #width: 200    
                TextInput:
                    id: velo1_textinput
                    # size_hint_x: None
                    # width: 30
                    input_filter: 'int'
                    multiline: False
                    text: str(velo1_slider.value) 
                Button:
                    id: pub1
                    text: 'Pub'
                    on_press:app.send_position(m1_id.text, m1_slider.value, "pub1")
                # Motor 2
                CheckBox:
                    id: m2_checkbox
                    group: "checkboxes"
                    color: 1, 1, 1, 1
                    on_active: app.test(m2_id.text)
                    size_hint_x: .20
                    canvas.before:
                        Color:
                            rgb: 1,0,0
                        Ellipse:
                            pos:self.center_x-8, self.center_y-8
                            size:[16,16]
                        Color:
                            rgb: 0,0,0
                        Ellipse:
                            pos:self.center_x-7, self.center_y-7
                            size:[14,14]
                Label:
                    text: '28'
                    id: m2_id
                    color: (0,0,1,1)
                    canvas.before:
                        Color:
                            rgba: 1, 1, 1, 1
                        Rectangle:
                            pos: self.pos
                            size: self.size
                Slider:
                    id: m2_slider
                    value: 0
                    min: -180
                    max: 180
                    step: 1
                    #size_hint:(None, .5)sectiition(m2_id.text, m2_slider.value)
                TextInput:
                    id: m2_textinput
                    # size_hint_x: None
                    # width: 30
                    input_filter: 'int'
                    multiline: False
                    text: str(m2_slider.value)
                Label:
                    text: str(m2_slider.value)
                    color: (0,0,1,1)
                    canvas.before:
                        Color:
                            rgba: 1, 1, 1, 1
                        Rectangle:
                            pos: self.pos
                            size: self.size
                Slider:
                    id: velo2_slider
                    value: 750
                    min: 1
                    max: 1500
                    step: 1
                    #size_hint:(None, .5)
                    #width: 200    
                TextInput:
                    id: velo2_textinput
                    # size_hint_x: None
                    # width: 30
                    input_filter: 'int'
                    multiline: False
                    text: str(velo2_slider.value) 
                Button:
                    id: pub2
                    text: 'Pub'
                    on_press:app.send_position(m2_id.text, m2_slider.value, "pub2")

                # Motor 3
                CheckBox:
                    id: m3_checkbox
                    group: "checkboxes"
                    color: 1, 1, 1, 1
                    on_active: app.test(m3_id.text)
                    size_hint_x: .20
                    canvas.before:
                        Color:
                            rgb: 1,0,0
                        Ellipse:
                            pos:self.center_x-8, self.center_y-8
                            size:[16,16]
                        Color:
                            rgb: 0,0,0
                        Ellipse:
                            pos:self.center_x-7, self.center_y-7
                            size:[14,14]
                Label:
                    text: '29'
                    id: m3_id
                    color: (0,0,1,1)
                    canvas.before:
                        Color:
                            rgba: 1, 1, 1, 1
                        Rectangle:
                            pos: self.pos
                            size: self.size
                Slider:
                    id: m3_slider
                    value: 0
                    min: -180
                    max: 180
                    step: 1
                    #size_hint:(None, .5)sectiition(m2_id.text, m2_slider.value)
                TextInput:
                    id: m3_textinput
                    # size_hint_x: None
                    # width: 30
                    input_filter: 'int'
                    multiline: False
                    text: str(m3_slider.value)
                Label:
                    text: str(m3_slider.value)
                    color: (0,0,1,1)
                    canvas.before:
                        Color:
                            rgba: 1, 1, 1, 1
                        Rectangle:
                            pos: self.pos
                            size: self.size
                Slider:
                    id: velo3_slider
                    value: 750
                    min: 1
                    max: 1500
                    step: 1
                    #size_hint:(None, .5)
                    #width: 200    
                TextInput:
                    id: velo3_textinput
                    # size_hint_x: None
                    # width: 30
                    input_filter: 'int'
                    multiline: False
                    text: str(velo3_slider.value) 
                Button:
                    id: pub3
                    text: 'Pub'
                    on_press:app.send_position(m3_id.text, m3_slider.value, "pub3")
            
        MDTabs:
            text_color_active: [0, 0, 0, 1]
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

        Button:
            id: refresh_btn
            text: "Refresh"
            halign: "center"
            size_hint: 0.15, 0.1
            pos_hint: {"center_x": .5, "center_y": 0.5}
            on_press: root.change_image("new_robotImage.png")

        Button:
            id: run_btn
            text: "Run"
            halign: "center"
            size_hint: 0.15, 0.1
            pos_hint: {"center_x": .1, "center_y": 0.2}
        
        Button:
            id: emergency
            text: "EM-STOP"
            halign: "center"
            background_color: 1,0,0,1
            size_hint: 0.15, 0.1
            pos_hint: {"center_x": .35, "center_y": 0.2}
            on_press: app.emergency_stop()

        Button:
            id: btn 
            text: "Load Demo:"
            halign: "center"
            size_hint: 0.25, 0.1
            pos_hint: {"center_x": .65, "center_y": 0.8}
            on_parent: drop_content.dismiss()
            on_release: drop_content.open(self)

        DropDown:
            id: drop_content
            on_select: btn.text = '{}'.format(args[1])

            Button:
                id: btn1
                text: 'First Item'
                size_hint_y: None
                height: 35
                on_release: drop_content.select('First Item')
            
            Button:
                id: btn2
                text: 'Second Item'
                size_hint_y: None
                height: 35
                on_release: drop_content.select('Second Item')

        FloatLayout:
            #pos_hint: {"right": 1}
            # size_hint: 0.5, 0.5
            GridLayout:
                pos_hint: {"center_x": 0.5, "center_y": 0.4}
                col_default_width: '56dp'
                size_hint_x: None
                cols: 8
                row_default_height: '32dp'
                size_hint_y: None
                height: self.minimum_height
                Label:
                    text: 'SENSOR'
                    color: (0,0,0,1)
                Label:
                    text: 'ID'
                    color: (0,0,0,1)
                Label:
                    text: 'POS CONTROL' 
                    color: (0,0,0,1)              
                Label:
                    text: 'GOAL POS'  
                    color: (0,0,0,1) 
                Label:
                    text: 'CUR POS'
                    color: (0,0,0,1)
                Label:
                    text: 'VEL CONTROL'
                    color: (0,0,0,1) 
                Label:
                    text: 'GOAL VEL' 
                    color: (0,0,0,1)
                Label:
                    text: 'SEND POS'
                    color: (0,0,0,1)
                # Motor 1
                CheckBox:
                    id: m1_checkbox
                    group: "checkboxes"
                    color: 1, 1, 1, 1
                    on_active: app.test(m1_id.text)
                    size_hint_x: .20
                    canvas.before:
                        Color:
                            rgb: 1,0,0
                        Ellipse:
                            pos:self.center_x-8, self.center_y-8
                            size:[16,16]
                        Color:
                            rgb: 0,0,0
                        Ellipse:
                            pos:self.center_x-7, self.center_y-7
                            size:[14,14]
                Label:
                    text: '16'
                    id: m1_id
                    color: (0,0,1,1)
                    canvas.before:
                        Color:
                            rgba: 1, 1, 1, 1
                        Rectangle:
                            pos: self.pos
                            size: self.size
                Slider:
                    id: m1_slider
                    value: 0
                    min: -180
                    max: 180
                    step: 1
                    #size_hint:(None, .5)
                    #width: 200
                TextInput:
                    id: m1_textinput
                    # size_hint_x: None
                    # width: 30
                    input_filter: 'int'
                    multiline: False
                    text: str(m1_slider.value)
                Label:
                    text: str(m1_slider.value)
                    color: (0,0,1,1)
                    canvas.before:
                        Color:
                            rgba: 1, 1, 1, 1
                        Rectangle:
                            pos: self.pos
                            size: self.size
                Slider:
                    id: velo1_slider
                    value: 750
                    min: 1
                    max: 1500
                    step: 1
                    #size_hint:(None, .5)
                    #width: 200    
                TextInput:
                    id: velo1_textinput
                    # size_hint_x: None
                    # width: 30
                    input_filter: 'int'
                    multiline: False
                    text: str(velo1_slider.value) 
                Button:
                    id: pub1
                    text: 'Pub'
                    on_press:app.send_position(m1_id.text, m1_slider.value,"pub1")
                    disable: False
                # Motor 2
                CheckBox:
                    id: m2_checkbox
                    group: "checkboxes"
                    color: 1, 1, 1, 1
                    on_active: app.test(m2_id.text)
                    size_hint_x: .20
                    canvas.before:
                        Color:
                            rgb: 1,0,0
                        Ellipse:
                            pos:self.center_x-8, self.center_y-8
                            size:[16,16]
                        Color:
                            rgb: 0,0,0
                        Ellipse:
                            pos:self.center_x-7, self.center_y-7
                            size:[14,14]
                Label:
                    text: '18'
                    id: m2_id
                    color: (0,0,1,1)
                    canvas.before:
                        Color:
                            rgba: 1, 1, 1, 1
                        Rectangle:
                            pos: self.pos
                            size: self.size
                Slider:
                    id: m2_slider
                    value: 0
                    min: -180
                    max: 180
                    step: 1
                    #size_hint:(None, .5)sectiition(m2_id.text, m2_slider.value)
                TextInput:
                    id: m2_textinput
                    # size_hint_x: None
                    # width: 30
                    input_filter: 'int'
                    multiline: False
                    text: str(m2_slider.value)
                Label:
                    text: str(m2_slider.value)
                    color: (0,0,1,1)
                    canvas.before:
                        Color:
                            rgba: 1, 1, 1, 1
                        Rectangle:
                            pos: self.pos
                            size: self.size
                Slider:
                    id: velo2_slider
                    value: 750
                    min: 1
                    max: 1500
                    step: 1
                    #size_hint:(None, .5)
                    #width: 200    
                TextInput:
                    id: velo2_textinput
                    # size_hint_x: None
                    # width: 30
                    input_filter: 'int'
                    multiline: False
                    text: str(velo2_slider.value) 
                Button:
                    id: pub2
                    text: 'Pub'
                    on_press:app.send_position(m2_id.text, m2_slider.value, "pub2")

                # Motor 3
                CheckBox:
                    id: m3_checkbox
                    group: "checkboxes"
                    color: 1, 1, 1, 1
                    on_active: app.test(m3_id.text)
                    size_hint_x: .20
                    canvas.before:
                        Color:
                            rgb: 1,0,0
                        Ellipse:
                            pos:self.center_x-8, self.center_y-8
                            size:[16,16]
                        Color:
                            rgb: 0,0,0
                        Ellipse:
                            pos:self.center_x-7, self.center_y-7
                            size:[14,14]
                Label:
                    text: '20'
                    id: m3_id
                    color: (0,0,1,1)
                    canvas.before:
                        Color:
                            rgba: 1, 1, 1, 1
                        Rectangle:
                            pos: self.pos
                            size: self.size
                Slider:
                    id: m3_slider
                    value: 0
                    min: -180
                    max: 180
                    step: 1
                    #size_hint:(None, .5)sectiition(m2_id.text, m2_slider.value)
                TextInput:
                    id: m3_textinput
                    # size_hint_x: None
                    # width: 30
                    input_filter: 'int'
                    multiline: False
                    text: str(m3_slider.value)
                Label:
                    text: str(m3_slider.value)
                    color: (0,0,1,1)
                    canvas.before:
                        Color:
                            rgba: 1, 1, 1, 1
                        Rectangle:
                            pos: self.pos
                            size: self.size
                Slider:
                    id: velo3_slider
                    value: 750
                    min: 1
                    max: 1500
                    step: 1
                    #size_hint:(None, .5)
                    #width: 200    
                TextInput:
                    id: velo3_textinput
                    # size_hint_x: None
                    # width: 30
                    input_filter: 'int'
                    multiline: False
                    text: str(velo3_slider.value) 
                Button:
                    id: pub3
                    text: 'Pub'
                    on_press:app.send_position(m3_id.text, m3_slider.value, "pub3")

                # Motor 4
                CheckBox:
                    id: m4_checkbox
                    group: "checkboxes"
                    color: 1, 1, 1, 1
                    on_active: app.test(m4_id.text)
                    size_hint_x: .20
                    canvas.before:
                        Color:
                            rgb: 1,0,0
                        Ellipse:
                            pos:self.center_x-8, self.center_y-8
                            size:[16,16]
                        Color:
                            rgb: 0,0,0
                        Ellipse:
                            pos:self.center_x-7, self.center_y-7
                            size:[14,14]
                Label:
                    text: '22'
                    id: m4_id
                    color: (0,0,1,1)
                    canvas.before:
                        Color:
                            rgba: 1, 1, 1, 1
                        Rectangle:
                            pos: self.pos
                            size: self.size
                Slider:
                    id: m4_slider
                    value: 0
                    min: -180
                    max: 180
                    step: 1
                    #size_hint:(None, .5)sectiition(m2_id.text, m2_slider.value)
                TextInput:
                    id: m4_textinput
                    # size_hint_x: None
                    # width: 30
                    input_filter: 'int'
                    multiline: False
                    text: str(m4_slider.value)
                Label:
                    text: str(m4_slider.value)
                    color: (0,0,1,1)
                    canvas.before:
                        Color:
                            rgba: 1, 1, 1, 1
                        Rectangle:
                            pos: self.pos
                            size: self.size
                Slider:
                    id: velo4_slider
                    value: 750
                    min: 1
                    max: 1500
                    step: 1
                    #size_hint:(None, .5)
                    #width: 200    
                TextInput:
                    id: velo4_textinput
                    # size_hint_x: None
                    # width: 30
                    input_filter: 'int'
                    multiline: False
                    text: str(velo4_slider.value) 
                Button:
                    id: pub4
                    text: 'Pub'
                    on_press:app.send_position(m4_id.text, m4_slider.value, "pub4")

                # Motor 5
                CheckBox:
                    id: m5_checkbox
                    group: "checkboxes"
                    color: 1, 1, 1, 1
                    on_active: app.test(m5_id.text)
                    size_hint_x: .20
                    canvas.before:
                        Color:
                            rgb: 1,0,0
                        Ellipse:
                            pos:self.center_x-8, self.center_y-8
                            size:[16,16]
                        Color:
                            rgb: 0,0,0
                        Ellipse:
                            pos:self.center_x-7, self.center_y-7
                            size:[14,14]
                Label:
                    text: '24'
                    id: m5_id
                    color: (0,0,1,1)
                    canvas.before:
                        Color:
                            rgba: 1, 1, 1, 1
                        Rectangle:
                            pos: self.pos
                            size: self.size
                Slider:
                    id: m5_slider
                    value: 0
                    min: -180
                    max: 180
                    step: 1
                    #size_hint:(None, .5)sectiition(m2_id.text, m2_slider.value)
                TextInput:
                    id: m5_textinput
                    # size_hint_x: None
                    # width: 30
                    input_filter: 'int'
                    multiline: False
                    text: str(m5_slider.value)
                Label:
                    text: str(m5_slider.value)
                    color: (0,0,1,1)
                    canvas.before:
                        Color:
                            rgba: 1, 1, 1, 1
                        Rectangle:
                            pos: self.pos
                            size: self.size
                Slider:
                    id: velo5_slider
                    value: 750
                    min: 1
                    max: 1500
                    step: 1
                    #size_hint:(None, .5)
                    #width: 200    
                TextInput:
                    id: velo5_textinput
                    # size_hint_x: None
                    # width: 30
                    input_filter: 'int'
                    multiline: False
                    text: str(velo5_slider.value) 
                Button:
                    id: pub5
                    text: 'Pub'
                    on_press:app.send_position(m5_id.text, m5_slider.value, "pub5")

                # Motor 6
                CheckBox:
                    id: m6_checkbox
                    group: "checkboxes"
                    color: 1, 1, 1, 1
                    on_active: app.test(m6_id.text)
                    size_hint_x: .20
                    canvas.before:
                        Color:
                            rgb: 1,0,0
                        Ellipse:
                            pos:self.center_x-8, self.center_y-8
                            size:[16,16]
                        Color:
                            rgb: 0,0,0
                        Ellipse:
                            pos:self.center_x-7, self.center_y-7
                            size:[14,14]
                Label:
                    text: '26'
                    id: m6_id
                    color: (0,0,1,1)
                    size_hint_x: .40
                    canvas.before:
                        Color:
                            rgba: 1, 1, 1, 1
                        Rectangle:
                            pos: self.pos
                            size: self.size
                Slider:
                    id: m6_slider
                    value: 0
                    min: -180
                    max: 180
                    step: 1
                    #size_hint:(None, .5)sectiition(m2_id.text, m2_slider.value)
                TextInput:
                    id: m6_textinput
                    # size_hint_x: None
                    # width: 30
                    input_filter: 'int'
                    multiline: False
                    text: str(m6_slider.value)
                Label:
                    text: str(m6_slider.value)
                    color: (0,0,1,1)
                    canvas.before:
                        Color:
                            rgba: 1, 1, 1, 1
                        Rectangle:
                            pos: self.pos
                            size: self.size
                Slider:
                    id: velo6_slider
                    value: 750
                    min: 1
                    max: 1500
                    step: 1
                    #size_hint:(None, .5)
                    #width: 200    
                TextInput:
                    id: velo6_textinput
                    # size_hint_x: None
                    # width: 30
                    input_filter: 'int'
                    multiline: False
                    text: str(velo6_slider.value) 
                Button:
                    id: pub6
                    text: 'Pub'
                    on_press:app.send_position(m6_id.text, m6_slider.value, "pub6")
            
        MDTabs:
            text_color_active: [0, 0, 0, 1]
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

        Button:
            id: refresh_btn
            text: "Refresh"
            halign: "center"
            size_hint: 0.15, 0.1
            pos_hint: {"center_x": .5, "center_y": 0.5}
            on_press: root.change_image("new_robotImage.png")

        Button:
            id: run_btn
            text: "Run"
            halign: "center"
            size_hint: 0.15, 0.1
            pos_hint: {"center_x": .1, "center_y": 0.2}
        
        Button:
            id: emergency
            text: "EM-STOP"
            halign: "center"
            background_color: 1,0,0,1
            size_hint: 0.15, 0.1
            pos_hint: {"center_x": .35, "center_y": 0.2}
            on_press: app.emergency_stop()

        Button:
            id: btn 
            text: "Load Demo:"
            halign: "center"
            size_hint: 0.25, 0.1
            pos_hint: {"center_x": .65, "center_y": 0.8}
            on_parent: drop_content.dismiss()
            on_release: drop_content.open(self)

        DropDown:
            id: drop_content
            on_select: btn.text = '{}'.format(args[1])

            Button:
                id: btn1
                text: 'First Item'
                size_hint_y: None
                height: 35
                on_release: drop_content.select('First Item')
            
            Button:
                id: btn2
                text: 'Second Item'
                size_hint_y: None
                height: 35
                on_release: drop_content.select('Second Item')

        FloatLayout:
            #pos_hint: {"right": 1}
            # size_hint: 0.5, 0.5
            GridLayout:
                pos_hint: {"center_x": 0.5, "center_y": 0.4}
                col_default_width: '56dp'
                size_hint_x: None
                cols: 8
                row_default_height: '32dp'
                size_hint_y: None
                height: self.minimum_height
                Label:
                    text: 'SENSOR'
                    color: (0,0,0,1)
                Label:
                    text: 'ID'
                    color: (0,0,0,1)
                Label:
                    text: 'POS CONTROL' 
                    color: (0,0,0,1)              
                Label:
                    text: 'GOAL POS'  
                    color: (0,0,0,1) 
                Label:
                    text: 'CUR POS'
                    color: (0,0,0,1)
                Label:
                    text: 'VEL CONTROL'
                    color: (0,0,0,1) 
                Label:
                    text: 'GOAL VEL' 
                    color: (0,0,0,1)
                Label:
                    text: 'SEND POS'
                    color: (0,0,0,1)
                # Motor 1
                CheckBox:
                    id: m1_checkbox
                    group: "checkboxes"
                    color: 1, 1, 1, 1
                    on_active: app.test(m1_id.text)
                    size_hint_x: .20
                    canvas.before:
                        Color:
                            rgb: 1,0,0
                        Ellipse:
                            pos:self.center_x-8, self.center_y-8
                            size:[16,16]
                        Color:
                            rgb: 0,0,0
                        Ellipse:
                            pos:self.center_x-7, self.center_y-7
                            size:[14,14]
                Label:
                    text: '15'
                    id: m1_id
                    color: (0,0,1,1)
                    canvas.before:
                        Color:
                            rgba: 1, 1, 1, 1
                        Rectangle:
                            pos: self.pos
                            size: self.size
                Slider:
                    id: m1_slider
                    value: 0
                    min: -180
                    max: 180
                    step: 1
                    #size_hint:(None, .5)
                    #width: 200
                TextInput:
                    id: m1_textinput
                    # size_hint_x: None
                    # width: 30
                    input_filter: 'int'
                    multiline: False
                    text: str(m1_slider.value)
                Label:
                    text: str(m1_slider.value)
                    color: (0,0,1,1)
                    canvas.before:
                        Color:
                            rgba: 1, 1, 1, 1
                        Rectangle:
                            pos: self.pos
                            size: self.size
                Slider:
                    id: velo1_slider
                    value: 750
                    min: 1
                    max: 1500
                    step: 1
                    #size_hint:(None, .5)
                    #width: 200    
                TextInput:
                    id: velo1_textinput
                    # size_hint_x: None
                    # width: 30
                    input_filter: 'int'
                    multiline: False
                    text: str(velo1_slider.value) 
                Button:
                    id: pub1
                    text: 'Pub'
                    on_press:app.send_position(m1_id.text, m1_slider.value,"pub1")
                    disable: False
                # Motor 2
                CheckBox:
                    id: m2_checkbox
                    group: "checkboxes"
                    color: 1, 1, 1, 1
                    on_active: app.test(m2_id.text)
                    size_hint_x: .20
                    canvas.before:
                        Color:
                            rgb: 1,0,0
                        Ellipse:
                            pos:self.center_x-8, self.center_y-8
                            size:[16,16]
                        Color:
                            rgb: 0,0,0
                        Ellipse:
                            pos:self.center_x-7, self.center_y-7
                            size:[14,14]
                Label:
                    text: '17'
                    id: m2_id
                    color: (0,0,1,1)
                    canvas.before:
                        Color:
                            rgba: 1, 1, 1, 1
                        Rectangle:
                            pos: self.pos
                            size: self.size
                Slider:
                    id: m2_slider
                    value: 0
                    min: -180
                    max: 180
                    step: 1
                    #size_hint:(None, .5)sectiition(m2_id.text, m2_slider.value)
                TextInput:
                    id: m2_textinput
                    # size_hint_x: None
                    # width: 30
                    input_filter: 'int'
                    multiline: False
                    text: str(m2_slider.value)
                Label:
                    text: str(m2_slider.value)
                    color: (0,0,1,1)
                    canvas.before:
                        Color:
                            rgba: 1, 1, 1, 1
                        Rectangle:
                            pos: self.pos
                            size: self.size
                Slider:
                    id: velo2_slider
                    value: 750
                    min: 1
                    max: 1500
                    step: 1
                    #size_hint:(None, .5)
                    #width: 200    
                TextInput:
                    id: velo2_textinput
                    # size_hint_x: None
                    # width: 30
                    input_filter: 'int'
                    multiline: False
                    text: str(velo2_slider.value) 
                Button:
                    id: pub2
                    text: 'Pub'
                    on_press:app.send_position(m2_id.text, m2_slider.value, "pub2")

                # Motor 3
                CheckBox:
                    id: m3_checkbox
                    group: "checkboxes"
                    color: 1, 1, 1, 1
                    on_active: app.test(m3_id.text)
                    size_hint_x: .20
                    canvas.before:
                        Color:
                            rgb: 1,0,0
                        Ellipse:
                            pos:self.center_x-8, self.center_y-8
                            size:[16,16]
                        Color:
                            rgb: 0,0,0
                        Ellipse:
                            pos:self.center_x-7, self.center_y-7
                            size:[14,14]
                Label:
                    text: '19'
                    id: m3_id
                    color: (0,0,1,1)
                    canvas.before:
                        Color:
                            rgba: 1, 1, 1, 1
                        Rectangle:
                            pos: self.pos
                            size: self.size
                Slider:
                    id: m3_slider
                    value: 0
                    min: -180
                    max: 180
                    step: 1
                    #size_hint:(None, .5)sectiition(m2_id.text, m2_slider.value)
                TextInput:
                    id: m3_textinput
                    # size_hint_x: None
                    # width: 30
                    input_filter: 'int'
                    multiline: False
                    text: str(m3_slider.value)
                Label:
                    text: str(m3_slider.value)
                    color: (0,0,1,1)
                    canvas.before:
                        Color:
                            rgba: 1, 1, 1, 1
                        Rectangle:
                            pos: self.pos
                            size: self.size
                Slider:
                    id: velo3_slider
                    value: 750
                    min: 1
                    max: 1500
                    step: 1
                    #size_hint:(None, .5)
                    #width: 200    
                TextInput:
                    id: velo3_textinput
                    # size_hint_x: None
                    # width: 30
                    input_filter: 'int'
                    multiline: False
                    text: str(velo3_slider.value) 
                Button:
                    id: pub3
                    text: 'Pub'
                    on_press:app.send_position(m3_id.text, m3_slider.value, "pub3")

                # Motor 4
                CheckBox:
                    id: m4_checkbox
                    group: "checkboxes"
                    color: 1, 1, 1, 1
                    on_active: app.test(m4_id.text)
                    size_hint_x: .20
                    canvas.before:
                        Color:
                            rgb: 1,0,0
                        Ellipse:
                            pos:self.center_x-8, self.center_y-8
                            size:[16,16]
                        Color:
                            rgb: 0,0,0
                        Ellipse:
                            pos:self.center_x-7, self.center_y-7
                            size:[14,14]
                Label:
                    text: '21'
                    id: m4_id
                    color: (0,0,1,1)
                    canvas.before:
                        Color:
                            rgba: 1, 1, 1, 1
                        Rectangle:
                            pos: self.pos
                            size: self.size
                Slider:
                    id: m4_slider
                    value: 0
                    min: -180
                    max: 180
                    step: 1
                    #size_hint:(None, .5)sectiition(m2_id.text, m2_slider.value)
                TextInput:
                    id: m4_textinput
                    # size_hint_x: None
                    # width: 30
                    input_filter: 'int'
                    multiline: False
                    text: str(m4_slider.value)
                Label:
                    text: str(m4_slider.value)
                    color: (0,0,1,1)
                    canvas.before:
                        Color:
                            rgba: 1, 1, 1, 1
                        Rectangle:
                            pos: self.pos
                            size: self.size
                Slider:
                    id: velo4_slider
                    value: 750
                    min: 1
                    max: 1500
                    step: 1
                    #size_hint:(None, .5)
                    #width: 200    
                TextInput:
                    id: velo4_textinput
                    # size_hint_x: None
                    # width: 30
                    input_filter: 'int'
                    multiline: False
                    text: str(velo4_slider.value) 
                Button:
                    id: pub4
                    text: 'Pub'
                    on_press:app.send_position(m4_id.text, m4_slider.value, "pub4")

                # Motor 5
                CheckBox:
                    id: m5_checkbox
                    group: "checkboxes"
                    color: 1, 1, 1, 1
                    on_active: app.test(m5_id.text)
                    size_hint_x: .20
                    canvas.before:
                        Color:
                            rgb: 1,0,0
                        Ellipse:
                            pos:self.center_x-8, self.center_y-8
                            size:[16,16]
                        Color:
                            rgb: 0,0,0
                        Ellipse:
                            pos:self.center_x-7, self.center_y-7
                            size:[14,14]
                Label:
                    text: '23'
                    id: m5_id
                    color: (0,0,1,1)
                    canvas.before:
                        Color:
                            rgba: 1, 1, 1, 1
                        Rectangle:
                            pos: self.pos
                            size: self.size
                Slider:
                    id: m5_slider
                    value: 0
                    min: -180
                    max: 180
                    step: 1
                    #size_hint:(None, .5)sectiition(m2_id.text, m2_slider.value)
                TextInput:
                    id: m5_textinput
                    # size_hint_x: None
                    # width: 30
                    input_filter: 'int'
                    multiline: False
                    text: str(m5_slider.value)
                Label:
                    text: str(m5_slider.value)
                    color: (0,0,1,1)
                    canvas.before:
                        Color:
                            rgba: 1, 1, 1, 1
                        Rectangle:
                            pos: self.pos
                            size: self.size
                Slider:
                    id: velo5_slider
                    value: 750
                    min: 1
                    max: 1500
                    step: 1
                    #size_hint:(None, .5)
                    #width: 200    
                TextInput:
                    id: velo5_textinput
                    # size_hint_x: None
                    # width: 30
                    input_filter: 'int'
                    multiline: False
                    text: str(velo5_slider.value) 
                Button:
                    id: pub5
                    text: 'Pub'
                    on_press:app.send_position(m5_id.text, m5_slider.value, "pub5")

                # Motor 6
                CheckBox:
                    id: m6_checkbox
                    group: "checkboxes"
                    color: 1, 1, 1, 1
                    on_active: app.test(m6_id.text)
                    size_hint_x: .20
                    canvas.before:
                        Color:
                            rgb: 1,0,0
                        Ellipse:
                            pos:self.center_x-8, self.center_y-8
                            size:[16,16]
                        Color:
                            rgb: 0,0,0
                        Ellipse:
                            pos:self.center_x-7, self.center_y-7
                            size:[14,14]
                Label:
                    text: '25'
                    id: m6_id
                    color: (0,0,1,1)
                    size_hint_x: .40
                    canvas.before:
                        Color:
                            rgba: 1, 1, 1, 1
                        Rectangle:
                            pos: self.pos
                            size: self.size
                Slider:
                    id: m6_slider
                    value: 0
                    min: -180
                    max: 180
                    step: 1
                    #size_hint:(None, .5)sectiition(m2_id.text, m2_slider.value)
                TextInput:
                    id: m6_textinput
                    # size_hint_x: None
                    # width: 30
                    input_filter: 'int'
                    multiline: False
                    text: str(m6_slider.value)
                Label:
                    text: str(m6_slider.value)
                    color: (0,0,1,1)
                    canvas.before:
                        Color:
                            rgba: 1, 1, 1, 1
                        Rectangle:
                            pos: self.pos
                            size: self.size
                Slider:
                    id: velo6_slider
                    value: 750
                    min: 1
                    max: 1500
                    step: 1
                    #size_hint:(None, .5)
                    #width: 200    
                TextInput:
                    id: velo6_textinput
                    # size_hint_x: None
                    # width: 30
                    input_filter: 'int'
                    multiline: False
                    text: str(velo6_slider.value) 
                Button:
                    id: pub6
                    text: 'Pub'
                    on_press:app.send_position(m6_id.text, m6_slider.value, "pub6")
            
        MDTabs:
            text_color_active: [0, 0, 0, 1]
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
            right_action_items: [["robot-confused-outline", lambda x: app.callback_1()], ["power", lambda x: app.callback_2()]]
            size_hint: 1, .1
            pos_hint: {"center_x": .5, "center_y": 0.95}
        
        MDLabel:
            text: "LiDAR RVIZ"
            halign: "center"
  
        MDTabs:
            text_color_active: [0, 0, 0, 1]
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
            right_action_items: [["robot-confused-outline", lambda x: app.callback_1()], ["power", lambda x: app.callback_2()]]
            size_hint: 1, .1
            pos_hint: {"center_x": .5, "center_y": 0.95}

        MDLabel:
            text: "Realsense RVIZ"
            halign: "center"
  
        MDTabs:
            text_color_active: [0, 0, 0, 1]
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
            right_action_items: [["robot-confused-outline", lambda x: app.callback_1()], ["power", lambda x: app.callback_2()]]
            size_hint: 1, .1
            pos_hint: {"center_x": .5, "center_y": 0.95}
        
        MDLabel:
            text: "IMU Log/readings"
            halign: "center"
  
        MDTabs:
            text_color_active: [0, 0, 0, 1]
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
            right_action_items: [["robot-confused-outline", lambda x: app.callback_1()], ["power", lambda x: app.callback_2()]]
            size_hint: 1, .1
            pos_hint: {"center_x": .5, "center_y": 0.95}
        
        MDLabel:
            text: "Force Torque Data Log/readings"
            halign: "center"
  
        MDTabs:
            text_color_active: [0, 0, 0, 1]
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
    pass

class MenuTorsoScreen(Screen):
    pass

class MenuLeftArmScreen(Screen):
    pass

class MenuRightArmScreen(Screen):
    pass

class MenuLeftLegScreen(Screen):
    pass

class MenuRightLegScreen(Screen):
    pass

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

#initializing the motors using a seperate thread
#subprocess open up the python file which starts the launch file
def init_motors():
    print("initing motors")
    Thread(target=lambda *largs: subprocess.run([sys.executable, os.getcwd() + "/src/gui_tutorials/src/read_write_node_launch.py"])).start()

#initializing motor sensors 
#subprocess open up the python file which starts the launch file
def init_motor_sensors():
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
    
    for i in left_hand:
        dxl_comm_result, dxl_error = packetHandler2.write2ByteTxRx(portHandler1, i, ADDR_VEL_POSITION_HANDS, SPEED_HANDS)
        dxl_comm_result, dxl_error = packetHandler2.write1ByteTxRx(portHandler1, i, ADDR_TORQUE_ENABLE_HANDS, TORQUE_ENABLE)
        dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler1, i, ADDR_ACC, ACC)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % packetHandler2.getTxRxResult(dxl_comm_result))
            print("WARNING MOTOR ", i, " NOT CONNECTED!!!")
            # getch()
        elif dxl_error != 0:
            print("%s" % packetHandler2.getRxPacketError(dxl_error))
            print("WARNING MOTOR ", i, " NOT CONNECTED!!!")
            # getch()
        else:
            print("DYNAMIXEL has been successfully connected")
    
    for i in right_hand:
        dxl_comm_result, dxl_error = packetHandler2.write2ByteTxRx(portHandler2, i, ADDR_VEL_POSITION_HANDS, SPEED_HANDS)
        dxl_comm_result, dxl_error = packetHandler2.write1ByteTxRx(portHandler2, i, ADDR_TORQUE_ENABLE_HANDS, TORQUE_ENABLE)
        dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler1, i, ADDR_ACC, ACC)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % packetHandler2.getTxRxResult(dxl_comm_result))
            print("WARNING MOTOR ", i, " NOT CONNECTED!!!")
            # getch()
        elif dxl_error != 0:
            print("%s" % packetHandler2.getRxPacketError(dxl_error))
            print("WARNING MOTOR ", i, " NOT CONNECTED!!!")
            # getch()
        else:
            print("DYNAMIXEL has been successfully connected")
    
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
        
    # #thread function which greys out the publish button for 3 seconds when it is pressed
    # #prevents the rest of the gui from sleeping when time.sleep is called
    def send_position_callback(self, id, position, velocity, buttonid):
    	pub.publish(int(id), int(position), int(velocity))
    	self.screen.ids[buttonid].disabled = True #disables the buttons
    	time.sleep(10) #disbaled for 3 seconds
    	self.screen.ids[buttonid].disabled = False

    # #function which converts degrees to the appropriate position values for the motors based on their types
    # def send_position(self, id, position, buttonid):
    # 	print("Button pressed")
    # 	id_i = int(id) #casting the id to an int 
    # 	if ((id_i > 0 and id_i < 9) or (id_i==15) or (id_i == 16) or (id_i == 27)): #converts degrees to appropriate position value
    # 		position = ((int(position)*(501433*2))/360) #initial position *2/360
    # 	elif ((id_i > 8 and id_i < 15) or (id_i==28) or (id_i == 29)):
    # 		position = ((int(position)*(303454*2))/360) #initial position *2/360
    # 	print("position = ", position)
    # 	thread1 = Thread(target=self.send_position_callback, args=(id, position,buttonid)) #starts the thread which controls the button functionality
    # 	thread1.setDaemon(True)
    # 	thread1.start()
    # 	# thread2 = Thread(target=self.send_position_callback, args=(id, position,buttonid))
    # 	# thread2.start()

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

    # HumanoidApp().run() #running the gui
    # command = "rosnode kill --all" #once the gui has exited, then kill all rosnodes
    # p = subprocess.run(command, shell=True) 
    # roscore.terminate() #close roscore
    
    ########################################
    #init_urdf()
    rospy.init_node('simple_gui', anonymous=True)
    # Runs KIVY HumanoidApp Class
    HumanoidApp().run() 
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

# class TutorialApp(MDApp):

#     #thread function which contains the sequence to move the right arm in a wave
#     def wave_right_arm_callback(self, buttonid):
#         print("waving right arm")
#         pub.publish(int(right_arm[1]), int((20*(501433*2))/360)) #moving the motors to hardcoded value, converting the angle from sliders to a positional value for the motors
#         rospy.sleep(1)
#         pub.publish(int(right_arm[3]), int((-77*(501433*2))/360))
#         rospy.sleep(1)
#         pub.publish(int(right_arm[4]), int((-1*(303454*2))/360))
#         rospy.sleep(1)
#         pub.publish(int(right_arm[5]), int((-71*(303454*2))/360))
#         rospy.sleep(2)
#         pub.publish(int(right_arm[4]), int((29*(303454*2))/360))
#         rospy.sleep(1)
#         pub.publish(int(right_arm[4]), int((-29*(303454*2))/360))
#         rospy.sleep(1)
#         pub.publish(int(right_arm[4]), int((29*(303454*2))/360))
#         rospy.sleep(1)
#         pub.publish(int(right_arm[1]), int((20*(501433*2))/360))
#         rospy.sleep(1)
#         pub.publish(int(right_arm[3]), int((-16*(501433*2))/360))
#         rospy.sleep(1)
#         pub.publish(int(right_arm[4]), int((0*(303454*2))/360))
#         rospy.sleep(1)
#         pub.publish(int(right_arm[5]), int((0*(303454*2))/360))
#         self.screen.ids[buttonid].disabled = True  #disables the button to keep users from repeatedly pushing while the movement sequence is taking place
#         time.sleep(10) #keep disabled for 10 seconds
#         self.screen.ids[buttonid].disabled = False
    
#     #demo right arm wave through a thread to keep main window from freezing while demo takes place
#     def wave_right_arm(self, buttonid):
#         thread1 = Thread(target=self.wave_right_arm_callback, args=(buttonid,)) #starts the thread which controls the button functionality
#         thread1.start()

#     #thread function which contains the sequence to move the left arm in a wave
#     def wave_left_arm_callback(self, buttonid):
#         print("waving left arm")
#         pub.publish(int(left_arm[1]), int((-90*(501433*2))/360))
#         rospy.sleep(2)
#         pub.publish(int(left_arm[2]), int((-90*(501433*2))/360))
#         rospy.sleep(2)
#         pub.publish(int(left_arm[3]), int((67*(501433*2))/360))
#         rospy.sleep(2)
#         pub.publish(int(left_arm[3]), int((51*(501433*2))/360))
#         rospy.sleep(1)
#         pub.publish(int(left_arm[3]), int((67*(501433*2))/360))
#         rospy.sleep(1)
#         pub.publish(int(left_arm[3]), int((51*(501433*2))/360))
#         rospy.sleep(1)
#         pub.publish(int(left_arm[3]), int((67*(501433*2))/360))
#         rospy.sleep(1)
#         pub.publish(int(left_arm[3]), int((51*(501433*2))/360))
#         rospy.sleep(1)
#         pub.publish(int(left_arm[1]), int((-20*(501433*2))/360))
#         rospy.sleep(1)
#         pub.publish(int(left_arm[2]), int((0*(501433*2))/360))
#         rospy.sleep(1)
#         pub.publish(int(left_arm[3]), int((0*(501433*2))/360))
#         self.screen.ids[buttonid].disabled = True  #disables the button to keep users from repeatedly pushing while the movement sequence is taking place
#         time.sleep(10) #keep disabled for 10 seconds
#         self.screen.ids[buttonid].disabled = False

#     # # demo left arm wave through a thread to keep main window from freezing while demo takes place
#     def wave_left_arm(self, buttonid):
#         thread1 = Thread(target=self.wave_left_arm_callback, args=(buttonid,)) #starts the thread which controls the button functionality
#         thread1.start()

#     #launches the seperate the popup windows for the motor sliders with different threads
#     @staticmethod
#     def motor_section_show(id):
#         print("New window open")
#         if str(id) == 'Head': #if the button pressed on main window is 'Head' pass robot_section.py 'Head' as argument
#             Thread(target=lambda *largs: subprocess.run([sys.executable, os.getcwd() + "/src/gui_tutorials/src/robot_section.py"], input=b"Head")).start() #starting up the code for each different section in a seperate thread
#         elif str(id) == 'Left Arm': #if the button pressed on main window is 'Left Arm' pass robot_section.py 'Left Arm' as argument
#             Thread(target=lambda *largs: subprocess.run([sys.executable, os.getcwd() + "/src/gui_tutorials/src/robot_section.py"], input=b"Left Arm")).start()
#         elif str(id) == 'Right Arm': #if button pressed on main window is 'Right Arm' pass robot_section.py 'Right Arm' as argumeny
#             Thread(target=lambda *largs: subprocess.run([sys.executable, os.getcwd() + "/src/gui_tutorials/src/robot_section.py"], input=b"Right Arm")).start()
#         elif str(id) == 'Torso': #if button pressed on main window is 'Torso' pass robot_section.py 'Torso' as argument
#             Thread(target=lambda *largs: subprocess.run([sys.executable, os.getcwd() + "/src/gui_tutorials/src/robot_section.py"], input=b"Torso")).start()
#         elif str(id) == 'Left Leg': #if button pressed on main window is 'Left Leg' launch the left leg popup pass robot_section.py 'Left Leg' as argument
#             Thread(target=lambda *largs: subprocess.run([sys.executable, os.getcwd() + "/src/gui_tutorials/src/robot_section.py"], input=b"Left Leg")).start()
#         elif str(id) == 'Right Leg': #if the button pressed on main window is 'Right Leg' pass robot_section.py 'Right Leg' as argument
#             Thread(target=lambda *largs: subprocess.run([sys.executable, os.getcwd() + "/src/gui_tutorials/src/robot_section.py"], input=b"Right Leg")).start()
#         elif str(id) == 'IMU': #if the button pressed on main window is 'Left Arm' pass robot_section.py 'Left Arm' as argument
#             Thread(target=lambda *largs: subprocess.run([sys.executable, os.getcwd() + "/src/gui_tutorials/src/robot_section.py"], input=b"IMU")).start()
#             threatStart = Thread(target = launch_button, args=())
#             threatStart.start()
#             Thread(target=lambda *largs: subprocess.run([sys.executable, os.getcwd() + "/src/gui_tutorials/src/imu_subscriber.py"])).start()
