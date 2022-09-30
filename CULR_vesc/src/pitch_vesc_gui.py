#!/usr/bin/env python

import sympy as sp
import numpy as np
import math
import time
import rospy
import os
from std_msgs.msg import Float64, Float64MultiArray, String
from gazebo_msgs.srv import GetModelState, GetLinkState
from sensor_msgs.msg import Imu
from geometry_msgs.msg import *
import pylab as pl
import matplotlib.pyplot as plt
import state_equation_roll as ser
import WIP_utils as utils
from math import nan
from tkinter.constants import NONE
import PySimpleGUI as sg
import ctypes
import platform
from PySimpleGUI.PySimpleGUI import T
import serial
import time
import matplotlib.pyplot as plt
import numpy as np
from openrobot_vesc_pyserial import vesc_pyserial as vs
from threading import Timer
from tkinter.font import ROMAN
from itertools import count
import mpc_pitch
import Cal_joint as cj

######################### Class Init #########################
# default serial

selected_ser_name = ''
selected_ser_class = ''
ser_ind = 0
baud = ('115200', '460800', '921600')
height = ('0.889','0.969','1.069','1.146')
tout = 0.5

# default about joint
joint_list = ['Wheel', 'Ankle_Pitch', 'Ankle_Roll', 'Knee', 'Hip_Pitch', 'Hip_Roll']
# joint_list = ['F2']

joint_original_tuple = tuple(joint_list)
vesc_joint_match = {3:'Wheel', 22:'Ankle_Pitch', 33:'Ankle_Roll', 44:'Knee', 55:'Hip_Pitch', 66:'Hip_Roll'} # dictionary type, ID:'Joint Number'
# vesc_joint_match = {4:'F2'} # dictionary type, ID:'Joint Number'
RAD2DEG = 180/np.pi 


global vesc_id_data
refresh_list_flag = False
vesc_serial_list = []
roll_list = []
index = count()

x_index = []
y = []

connect_flag = 0
on_flag = 0
balancing_flag = 0
gimbal_flag = 0
pos_flag = 0
joint_table_flag = 0
joint_state_flag = 0

st_data = []
Gr_mode = False
tor_mode = False
balancing_state = 0


def get_serial_port_name_from_class(s_class):
    return s_class.serial_name.port

def get_serial_class_from_port_name(port_name):
    vesc_serial_class_selected = None
    for class_instance in vesc_serial_list:
        if get_serial_port_name_from_class(class_instance) == port_name:
            vesc_serial_class_selected = class_instance
            #print("selected vesc serial class instance:",vesc_serial_class_selected)
    return vesc_serial_class_selected    

def del_serial_class(s_class):
    i = 0
    for class_instance in vesc_serial_list:
        if class_instance == s_class:
            del vesc_serial_list[i]
            #print("selected vesc serial class instance:",vesc_serial_class_selected)
        i += 1

def del_vesc_list(port_name):
    i = 0
    for i in range(len(vesc_id_data)-1, -1, -1):
        vesc_obj = vesc_id_data[i]
        if vesc_obj[0] == port_name:
            vesc_id_data.remove(vesc_obj)

def get_vesc_id_from_joint_number(joint_num):
    print(joint_num)

def refresh_joint_list_after_delete_seleced_joint_number(joint_num):
    for i in range(len(joint_list)-1, -1, -1):
        if joint_list[i] == joint_num:
            joint_list.remove(joint_num)
    joint_list.sort()    

def refresh_joint_list_after_replace_seleced_joint_number(prev_joint_num, selected_joint_num):
    for i in range(len(joint_list)-1, -1, -1):
        if joint_list[i] == selected_joint_num:
            joint_list.remove(selected_joint_num)
    joint_list.append(prev_joint_num)
    joint_list.sort()    

def reset_joint_list():
    global joint_list
    joint_list = list(joint_original_tuple)
    #print("joint_list",joint_list)

######################### Common Functions #########################
# For 4K monitor(HiDPI)
def make_dpi_aware():
    if platform.system() == "Windows":
        if int(platform.release()) == "7":
            ctypes.windll.user32.SetProcessDPIAware()
        elif int(platform.release()) >= 8:
            ctypes.windll.shcore.SetProcessDpiAwareness(True)

# 시리얼 포트 스캔
def scan_serial_ports():
    p_name, p_desc, p_hwid, p_num = vs.list_serial()
    #print(p_name)
    #print(p_desc)
    #print(p_hwid)
    if p_num == 0: p_data = [["No Device"],[""]]
    else:
        p_data = [] 
        for i in range(p_num):
            p_data = p_data + [[p_name[i], p_hwid[i]]]
    return p_data

def refresh_vesc_list():
    vesc_id_data.clear()
    for ser_class in vesc_serial_list:
        if ser_class is not None:
            ids = ser_class.get_controller_id_list()
            
            if len(ids) != 0:
                for i in range(len(ids)):
                    try:
                        if i == 0:
                            if vesc_joint_match[ids[i]] is not None: 
                                vesc_id_data.append([get_serial_port_name_from_class(ser_class), ids[i], "Local", vesc_joint_match[ids[i]]])
                        else:
                            if vesc_joint_match[ids[i]] is not None: 
                                vesc_id_data.append([get_serial_port_name_from_class(ser_class), ids[i], "CAN", vesc_joint_match[ids[i]]])
                    except:
                        if i == 0:
                            vesc_id_data.append([get_serial_port_name_from_class(ser_class), ids[i], "Local", joint_list[i]])
                            print("Joint number of vesc id:", ids[i], "is not pre-defined")
                        else:
                            vesc_id_data.append([get_serial_port_name_from_class(ser_class), ids[i], "CAN", joint_list[i]])
                            print("Joint number of vesc id:", ids[i], "is not pre-defined")
                #print(vesc_id_data)
        else:
            print("No devices, SCAN VESC first")
    window.Element('-VESC_TABLE-').Update(values=vesc_id_data)
    refresh_list_flag = False

def set_servo_control(vesc_target_id, value):
    # prepare data
    comm_set_cmd = vs.COMM_PACKET_ID['COMM_CUSTOM_APP_DATA']
    custom_cmd = vs.COMM_PACKET_ID_OPENROBOT['COMM_SET_SERVO']

    custom_data = [custom_cmd, vesc_target_id, value]
    send_data = vs.packet_encoding(comm_set_cmd, custom_data)
    return send_data

def set_dps_control(vesc_target_id, value):
    # prepare data
    comm_set_cmd = vs.COMM_PACKET_ID['COMM_CUSTOM_APP_DATA']
    custom_cmd = vs.COMM_PACKET_ID_OPENROBOT['COMM_SET_DPS']

    custom_data = [custom_cmd, vesc_target_id, value]
    send_data = vs.packet_encoding(comm_set_cmd, custom_data)
    return send_data

def set_duty_control(vesc_target_id, value):
    # prepare data
    comm_set_cmd = vs.COMM_PACKET_ID['COMM_CUSTOM_APP_DATA']
    custom_cmd = vs.COMM_PACKET_ID['COMM_SET_DUTY']

    custom_data = [custom_cmd, vesc_target_id, value]
    send_data = vs.packet_encoding(comm_set_cmd, custom_data)
    return send_data

def set_current_control(vesc_target_id, value):
    # prepare data
    comm_set_cmd = vs.COMM_PACKET_ID['COMM_CUSTOM_APP_DATA']
    custom_cmd = vs.COMM_PACKET_ID['COMM_SET_CURRENT']

    custom_data = [custom_cmd, vesc_target_id, value]
    send_data = vs.packet_encoding(comm_set_cmd, custom_data)
    return send_data

def current_brake(vesc_target_id, value):
    # prepare data
    comm_set_cmd = vs.COMM_PACKET_ID['COMM_CUSTOM_APP_DATA']
    custom_cmd = vs.COMM_PACKET_ID['COMM_SET_CURRENT_BRAKE']

    custom_data = [custom_cmd, vesc_target_id, value]
    send_data = vs.packet_encoding(comm_set_cmd, custom_data)
    return send_data

def set_request(vesc_target_id):
    # prepare data
    comm_set_cmd = vs.COMM_PACKET_ID['COMM_CUSTOM_APP_DATA']
    custom_data = [None, vesc_target_id]

    send_data = vs.packet_encoding(comm_set_cmd, custom_data)
    return send_data

def set_rcservo_pos_control(vesc_target_id, value):
    # prepare data
    comm_set_cmd = vs.COMM_PACKET_ID['COMM_SET_SERVO_POS']
    send_data = vs.packet_encoding(comm_set_cmd, value)
    return send_data

def set_release(vesc_target_id):
    # prepare data
    comm_set_cmd = vs.COMM_PACKET_ID['COMM_CUSTOM_APP_DATA']
    custom_cmd = vs.COMM_PACKET_ID_OPENROBOT['COMM_SET_RELEASE']

    custom_data = [custom_cmd, vesc_target_id, 0]
    send_data = vs.packet_encoding(comm_set_cmd, custom_data)
    return send_data

def send_cmd(joint_number, cmd, value=0):
    s_class = None
    for i in range(len(vesc_id_data)):
        if len(vesc_id_data[i]) == 4 and vesc_id_data[i][3] == joint_number:
            s_class = get_serial_class_from_port_name(vesc_id_data[i][0])
            vesc_id = vesc_id_data[i][1]
            break

    if s_class is not None and s_class.usb_connection_flag:
        if vesc_id_data[i][2] == 'Local':
                vesc_id = 0xFF
        if cmd == "release":
            send_data = set_release(vesc_id)
            s_class.serial_write(send_data)
            print(joint_number, "Released")
        elif cmd == "servo":
            send_data = set_servo_control(vesc_id, value)
            s_class.serial_write(send_data)
            return send_data
        elif cmd == "dps":
            send_data = set_dps_control(vesc_id, value)
            s_class.serial_write(send_data)
        elif cmd == "duty":
            send_data = set_duty_control(vesc_id, value)
            s_class.serial_write(send_data)
        elif cmd == "current":
            send_data = set_current_control(vesc_id, value)
            s_class.serial_write(send_data)
        elif cmd == "current_brake":
            send_data = current_brake(vesc_id, value)
            s_class.serial_write(send_data)
        elif cmd == "request":
            send_data = set_request(vesc_id)
            s_class.serial_write(send_data)
        elif cmd == "rcservo":
            send_data = set_rcservo_pos_control(vesc_id, value)
            s_class.serial_write(send_data)
        elif cmd == "terminal":
            # terminal command using CAN_FORWARD
            if vesc_id ==  0xFF:
                comm_set_cmd = vs.COMM_PACKET_ID['COMM_TERMINAL_CMD']
                send_data = vs.packet_encoding(comm_set_cmd, value)
                s_class.serial_write(send_data)
            else:
                comm_set_cmd = vs.COMM_PACKET_ID['COMM_FORWARD_CAN']
                send_data = vs.packet_encoding(comm_set_cmd, [vesc_id, vs.COMM_PACKET_ID['COMM_TERMINAL_CMD'], value])
                s_class.serial_write(send_data)
    else:
        print(joint_number, "is Not Connected")
        
def call_wheel_ecd_data():
    send_cmd('Wheel','request')
    pos_w_data = selected_ser_class.get_ecd_value()
    rps_w_data = selected_ser_class.get_rps_value()
    return pos_w_data, rps_w_data

def call_ankle_pitch_ecd_data():
    send_cmd('Ankle_Pitch','request')
    pos_ap_data = selected_ser_class.get_ecd_value()
    rps_ap_data = selected_ser_class.get_rps_value()
    return pos_ap_data, rps_ap_data

def call_ankle_roll_ecd_data():
    send_cmd('Ankle_Roll','request')
    pos_ar_data = selected_ser_class.get_ecd_value()
    rps_ar_data = selected_ser_class.get_rps_value()
    return pos_ar_data, rps_ar_data

def call_knee_ecd_data():
    send_cmd('Knee','request')
    pos_kn_data = selected_ser_class.get_ecd_value()
    rps_kn_data = selected_ser_class.get_rps_value()
    return pos_kn_data, rps_kn_data
def call_hip_pitch_ecd_data():
    send_cmd('Hip_Pitch','request')
    pos_hp_data = selected_ser_class.get_ecd_value()
    rps_hp_data = selected_ser_class.get_rps_value()
    return pos_hp_data, rps_hp_data

def call_hip_roll_ecd_data():
    send_cmd('Hip_Roll','request')
    pos_hr_data = selected_ser_class.get_ecd_value()
    rps_hr_data = selected_ser_class.get_rps_value()
    return pos_hr_data, rps_hr_data
# def adjust_joint_limit(joint_range_i, joint_range_j, keyhere):
#     layout_subwin2_1 = [ [sg.Text(keyhere)],
#                          [sg.InputText(joint_range[joint_range_i][joint_range_j], key=keyhere), sg.Button('Ok', bind_return_key=True)]
#     ]
#     window_subwin2_1 = sg.Window('Joint Limit Reset', layout_subwin2_1)
#     events_subwin2_1, values_subwin2_1 = window_subwin2_1.Read()

#     if events_subwin2_1 == 'Ok':
#         try:
#             value = int(values_subwin2_1[keyhere])
#             #print(type(value), value)
#             joint_range[joint_range_i][joint_range_j] = value
#             window.Element(event).Update(values_subwin2_1[keyhere])
#             window.Refresh()
#         except:
#             print("Please input number")
#     window_subwin2_1.close()

######################### GUI #########################
# usb port table data 
ports_title = ["Port Name", "VID:PID"]
ports_data = scan_serial_ports()
# vesc id table data
vesc_id_data = []
vesc_id_headings = ["Port","ID","COMM", "Joint"]
time_data = []
time_heading = ["Time Step"]
layout_col1 = [ [sg.Text('<VESC USB>', font=("Tahoma", 16))],
                [sg.Table(values=ports_data, headings=ports_title, max_col_width=100,
                                col_widths=[12,11],
                                background_color='black',
                                auto_size_columns=False,
                                display_row_numbers=True,
                                justification='center',
                                num_rows=2,
                                alternating_row_color='black',
                                enable_events=True,
                                key='-USB_TABLE-',
                                row_height=30)],
                [sg.Text('Baudrate', font=("Tahoma", 12)), sg.Combo(values=baud, default_value='921600', readonly=True, k='-BAUD-')],
                [sg.Button('SCAN'), sg.Button('CONNECT'), sg.Button('DISCONNECT')],
                [sg.HorizontalSeparator()],
                [sg.Text('Connected VESC IDs')],
                [sg.Table(values=vesc_id_data, headings=vesc_id_headings, max_col_width=100,
                                col_widths=[10,5,7,6],
                                background_color='black',
                                bind_return_key=True,
                                auto_size_columns=False,
                                display_row_numbers=False,
                                justification='center',
                                num_rows=3,
                                alternating_row_color='black',
                                key='-VESC_TABLE-',
                                row_height=50)],
                [sg.Button('SCAN VESC'), sg.Button('Refresh List')],
                [sg.Table(values=time_data,headings=time_heading, max_col_width=1000,
                                                            background_color='black',
                                                            auto_size_columns=False,
                                                            display_row_numbers=False,
                                                            justification='center',
                                                            num_rows=1,
                                                            alternating_row_color='black',
                                                            key='-TIME_TABLE-',
                                                            row_height=30)],
                #[sg.HorizontalSeparator()],
]

logging_layout = [ [sg.Button('Clear'),
                    sg.Button('DEBUG PRINT ON'), 
                    sg.Button('DEBUG PRINT OFF')],
                   [sg.Output(size=(45,16), font='Courier 11', key='-OUTPUT-')],
]

command_layout = [ [sg.Text("Terminal Command to"), 
                    sg.Combo(values=joint_list, default_value=joint_list[0], readonly=True, k='-TARGET_JOINT-'), 
                    sg.Input(size=(53,1), focus=True, key='-TERMINAL_INPUT-'), sg.Button('Terminal SEND', bind_return_key=True)],

]

joint_range = [[-6584, 6376], [-3344, 3136], [-4643, 4717], [-7632, 6768], [-7027, 7373], [-7286, 7114]]
task_range = (-900, 900)
angle_range = (-180, 180)
default_pos = [-130, -104, -78, -86, -288, -201] #[-104, -26, -75, -86, -288, -201]
defualt_task = [0, 0, 803, 0, 0, 0]
size_bar_joint = (21.5,20)
size_bar_task = (30,20)

joint_data = []
joint_heading = ["", "Ankle", "Knee", "Hip"]
size_input = (10,None)
layout_col2 = [ [sg.Text('<Balancing Control Setup>', font=("Tahoma", 16))],
                [sg.Text('Height', size=(8,1)), sg.Combo(size=(5,1), values=height, default_value='0.969', readonly=True, k='-HEIGHT-'), sg.Button('Initial Position', size=(15,1))],
                [sg.Text('<Desired Joint Angle from Height [deg]>', font=("Tahoma", 16))],
                [sg.Table(values=joint_data, headings=joint_heading, max_col_width=130,
                                background_color='black',
                                col_widths=[10,15,15, 15],
                                auto_size_columns=False,
                                display_row_numbers=False,
                                justification='center',
                                num_rows=2,
                                alternating_row_color='black',
                                key='-JOINT_TABLE-',
                                row_height=30)],
                [sg.HorizontalSeparator()],
                [sg.Button('Balancing Start', size=(15,1)), sg.Button('Balancing Stop', size=(15,1), key='-Stop-'), sg.Button('All Release', size=(15,1))],
                [sg.Text('<Joint Servo Control [deg]>', font=("Tahoma", 16))],
                [sg.Text(joint_list[1]), sg.Text(" "), 
                 sg.Text(joint_range[0][0], enable_events=True, key='-JOINT1_MIN-', size=(8,1)), 
                 sg.Slider(range = [joint_range[0][0], joint_range[0][1]], default_value = default_pos[0], size = size_bar_joint, orientation='h', enable_events=True, key='-JOINT1-'), 
                 sg.Text(joint_range[0][1], enable_events=True, key='-JOINT1_MAX-', size=(8,1)), sg.Button('Ankle Pitch Release', size=(18,1))],
                [sg.Text(joint_list[2]), sg.Text("  "), 
                 sg.Text(joint_range[1][0], enable_events=True, key='-JOINT2_MIN-', size=(8,1)), 
                 sg.Slider(range = [joint_range[1][0], joint_range[1][1]], default_value = default_pos[1], size = size_bar_joint, orientation='h', enable_events=True, key='-JOINT2-'), 
                 sg.Text(joint_range[1][1], enable_events=True, key='-JOINT2_MAX-', size=(8,1)), sg.Button('Ankle Roll Release', size=(18,1))],
                [sg.Text(joint_list[3]), sg.Text("        "), 
                 sg.Text(joint_range[2][0], enable_events=True, key='-JOINT3_MIN-', size=(8,1)), 
                 sg.Slider(range = [joint_range[2][0], joint_range[2][1]], default_value = default_pos[2], size = size_bar_joint, orientation='h', enable_events=True, key='-JOINT3-'), 
                 sg.Text(joint_range[2][1], enable_events=True, key='-JOINT3_MAX-', size=(8,1)), sg.Button('Knee Release', size=(18,1))],
                [sg.Text(joint_list[4]), sg.Text("   "), 
                 sg.Text(joint_range[3][0], enable_events=True, key='-JOINT4_MIN-', size=(8,1)), 
                 sg.Slider(range = [joint_range[3][0], joint_range[3][1]], default_value = default_pos[3], size = size_bar_joint, orientation='h', enable_events=True, key='-JOINT4-'), 
                 sg.Text(joint_range[3][1], enable_events=True, key='-JOINT4_MAX-', size=(8,1)), sg.Button('Hip Pitch Release', size=(18,1))],
                [sg.Text(joint_list[5]), sg.Text("    "), 
                 sg.Text(joint_range[4][0], enable_events=True, key='-JOINT5_MIN-', size=(8,1)), 
                 sg.Slider(range = [joint_range[4][0], joint_range[4][1]], default_value = default_pos[4], size = size_bar_joint, orientation='h', enable_events=True, key='-JOINT5-'), 
                 sg.Text(joint_range[4][1], enable_events=True, key='-JOINT5_MAX-', size=(8,1)), sg.Button('Hip Roll Release', size=(18,1))],
                
                
]

status_data = []
status_heading = ["ID", "rps", "curr_A", "motor_temp"]
imu_data = []
imu_heading = ["theta_1", "theta_b"]
imu_status_data = []
layout_col3= [ [sg.Text('<Status>', font=("Tahoma", 15)), sg.Button('ON'), sg.Button('OFF')],
                [sg.Table(values=status_data, headings=status_heading, max_col_width=130,
                                background_color='black',
                                col_widths=[6,12,12,13],
                                auto_size_columns=False,
                                display_row_numbers=False,
                                justification='center',
                                num_rows=8,
                                alternating_row_color='black',
                                key='-STATUS_TABLE-',
                                row_height=20)],
                [sg.HorizontalSeparator()],
                [sg.Text('<Wheel-Ankle Pos>', font=("Tahoma", 15)), sg.Button('Pos ON'), sg.Button('Pos OFF')],
                [sg.Table(values=imu_status_data, headings=imu_heading, max_col_width=130,
                                background_color='black',
                                col_widths=[15,15],
                                auto_size_columns=False,
                                display_row_numbers=False,
                                justification='center',
                                num_rows=8,
                                alternating_row_color='black',
                                key='-POS_TABLE-',
                                row_height=10)],
]


layout_main = [ [sg.Column(layout_col1), 
                 sg.VSeparator(),
		         sg.Column(layout_col2),
                 sg.VSeparator(),
                 sg.Column(layout_col3),
                ],
		[sg.HorizontalSeparator()],
		[sg.Column(command_layout)],
]

# 4k hidpi solution
make_dpi_aware()
#print(platform.system())
#print(platform.architecture()[1])
# Create the Window
if platform.system() == "Linux" and platform.architecture()[1] == "ELF":
    window = sg.Window('CULR Pitch Balancing Control Gui', layout_main, finalize=True, location=(0,0), size=(800,600), keep_on_top=True)
else:
    window = sg.Window('CULR Pitch Balancing Control Gui', layout_main, finalize=True)
    
def Imucallback(msg):
    global roll
    global roll_vel
    
    roll, pitch, yaw = utils.quaternion_to_euler_angle(msg.orientation) 
    roll_vel = msg.angular_velocity.x
    
    # roll_list.append(roll)

    # x_index.append(next(index))

    # print(x_acc_list)
    # print(y_acc_list)
    # print(z_acc_list)
    # print(msg.orientation.y)
    
i = 1
j = 1
#################################################################################################
cur_time = time.time()  
sec_time = time.time() 

while True:
    event, values = window.read(timeout=1)
    
    last_time = cur_time
    cur_time = time.time()
    time_data = [cur_time-last_time]
    window.Element('-TIME_TABLE-').Update(values=time_data)
    

    if event == sg.WIN_CLOSED or event == 'Cancel': # if user closes window or clicks cancel
        for class_instance in vesc_serial_list:
            class_instance.exitThread_usb = True
            class_instance.usb_connection_flag = False
            # close serial
            if class_instance.serial_name is not None:
                if class_instance.serial_name.is_open:
                    class_instance.serial_name.close()
        break

    if event == "-USB_TABLE-":
        row = values['-USB_TABLE-']
        if len(row) != 0:
            ser_ind = row[0]
            selected_ser_name = ports_data[ser_ind][0]
            print("USB Port Selected - row:{0}, port:{1}".format(ser_ind, selected_ser_name))

    if event == "SCAN":
        print("Scan valid usb ports...")
        ports_data = scan_serial_ports()
        window.Element('-USB_TABLE-').Update(values=ports_data)
        
    if event == "Refresh List":
        refresh_vesc_list()

    if event == "CONNECT":
        baudrate_sel = values['-BAUD-']
        if selected_ser_name == 'No Device':
            print("Select valid devices")
        elif selected_ser_name != '':
            if get_serial_class_from_port_name(selected_ser_name) is None:
                while True:
                    try:
                        # create serial connection class
                        selected_ser_class = vs.VESC_USB(serial.Serial(selected_ser_name, baudrate_sel, timeout=tout), ser_ind)
                        vesc_serial_list.append(selected_ser_class)
                    except serial.SerialException:
                        continue
                    else:
                        print("VESC USB Connected at {}, {}bps".format(get_serial_port_name_from_class(selected_ser_class), baudrate_sel))
                        break

                if selected_ser_class.usb_connection_flag:
                    #종료 시그널 등록
                    vs.signal.signal(vs.signal.SIGINT, selected_ser_class.handler_usb)

                    #시리얼 읽을 쓰레드 생성
                    thread_usb_rx = vs.threading.Thread(target=selected_ser_class.readThread)
                    #시작!
                    thread_usb_rx.start()
                    connect_flag = 1
                    print("VESC USB RX Thread Started", selected_ser_class)

                    # Specify Next event, automatically scan vesc IDs
                    event = "SCAN VESC"
            else:
                print("Select USB Port is already Opened")
        else:
            print("Please select USB Port first")
        
    if event == "DISCONNECT":
        if selected_ser_name == 'No Device':
            print("Select valid devices")
        elif selected_ser_name != '':
            selected_ser_class = get_serial_class_from_port_name(selected_ser_name)
            
            #print(len(vesc_id_data))

            if selected_ser_class is not None:
                #print(vesc_id_data)
                # delete connected vesc list 
                del_vesc_list(get_serial_port_name_from_class(selected_ser_class))
                selected_ser_class.reset_controller_id_list()
                #print(vesc_id_data)
                window.Element('-VESC_TABLE-').Update(values=vesc_id_data)

                # serial class disconnection
                selected_ser_class.exitThread_usb = True
                selected_ser_class.serial_name.close()
                del_serial_class(selected_ser_class)
                connect_flag = 0
                on_flag = 0
                st_data = []
                print("VESC USB Disconnected")
            else:
                print("VESC Not Connected")

            reset_joint_list()
        else:
            print("Please select USB Port first")
 
    if event == "SCAN VESC":
        selected_ser_class = get_serial_class_from_port_name(selected_ser_name)

        if selected_ser_class is not None:
            if selected_ser_class.usb_connection_flag:
                selected_ser_class.reset_controller_id_list()
                send_data = vs.packet_encoding(vs.COMM_PACKET_ID['COMM_GET_VALUES'])
                #print(send_data)
                selected_ser_class.serial_write(send_data)
                time.sleep(0.1)
                send_data = vs.packet_encoding(vs.COMM_PACKET_ID['COMM_PING_CAN'])
                #print(send_data)
                selected_ser_class.serial_write(send_data)
                
                # 5초후에 REPRESH LIST event 실행하기
                if refresh_list_flag == False:
                    Timer(5, refresh_vesc_list).start()           
            else:
                print("VESC Not Connected")
        else:
            print("VESC Not Connected")

    if event == "Clear":
        window['-OUTPUT-'].update(value='')

    if event == "DEBUG PRINT ON":
        selected_ser_class = get_serial_class_from_port_name(selected_ser_name)
        selected_ser_class.debug_print_get_value_return = True
        selected_ser_class.debug_print_custom_return = True
        print("DEBUG PRINT ON")

    if event == "DEBUG PRINT OFF":
        selected_ser_class = get_serial_class_from_port_name(selected_ser_name)
        selected_ser_class.debug_print_get_value_return = False
        selected_ser_class.debug_print_custom_return = False
        print("DEBUG PRINT OFF")

    if event == "Terminal SEND":
        send_cmd(values['-TARGET_JOINT-'], 'terminal', values['-TERMINAL_INPUT-'])

    if event == "-VESC_TABLE-":
        if len(joint_list) != 0:
            layout_subwin1 = [ [sg.Text('Select Joint Number')],
                               [sg.Combo(values=joint_list, default_value=joint_list[0], readonly=True, k='-JOINT_COMBO-'), sg.Button('Ok')]
            ]
            window_subwin1 = sg.Window('Joint Selection', layout_subwin1)
            events_subwin1, values_subwin1 = window_subwin1.Read()

            if events_subwin1 == 'Ok':
                if values_subwin1['-JOINT_COMBO-']:    # if something is highlighted in the list
                    joint_number = values_subwin1['-JOINT_COMBO-']
                    if len(vesc_id_data[values['-VESC_TABLE-'][0]]) == 3:
                        vesc_id_data[values['-VESC_TABLE-'][0]].append(joint_number)
                        refresh_joint_list_after_delete_seleced_joint_number(joint_number)
                    elif len(vesc_id_data[values['-VESC_TABLE-'][0]]) == 4 and vesc_id_data[values['-VESC_TABLE-'][0]][3] != joint_number:
                        last_joint_number = vesc_id_data[values['-VESC_TABLE-'][0]][3]
                        vesc_id_data[values['-VESC_TABLE-'][0]][3] = joint_number
                        refresh_joint_list_after_replace_seleced_joint_number(last_joint_number, joint_number)

            window_subwin1.close()

            print(vesc_id_data[values['-VESC_TABLE-'][0]])
            window.Element('-VESC_TABLE-').Update(values=vesc_id_data)
        else:
            sg.Popup("All Joint selected")
        
    if event == "ON":
        if connect_flag == 1:
            on_flag = 1
            print("VESC Status data ON")
        
        else:
            print("Please select USB Port first")
    
    if on_flag ==1:
        
        send_cmd('Wheel','request')
        st_data = selected_ser_class.get_status_data()
        # print(st_data)
        window.Element('-STATUS_TABLE-').Update(values=st_data)
    
    if event == "OFF":
        if connect_flag == 1:
            on_flag = 0
            print("VESC Status data OFF")
        else:
            print("Please select USB Port first")

#########################################################################

    if event == "Initial Position":
        if balancing_state == 0:
            balancing_flag = 2
            joint_table_flag = 1
            joint_state_flag = 1
            if j == 1:
                h = np.round(float(values['-HEIGHT-'])-0.069,1)
                thetalistd = cj.get_init_pos(h)
            else:
                pass
        j+=1
              
    if joint_table_flag == 1:
        # thetalist_1 = [["Current", 1, 2, 3]]
        thetalist = [["Desired", float(thetalistd[0])*RAD2DEG, float(thetalistd[1])*RAD2DEG, float(thetalistd[2])*RAD2DEG],
                     ["Current", 1, 2, 3]]
        window.Element('-JOINT_TABLE-').Update(values=thetalist)
        
    # if balancing_flag == 2:
    #     send_cmd('Ankle_Pitch', 'servo', thetalistd[1])
    #     send_cmd('Ankle_Roll', 'servo', 0)
    #     send_cmd('Knee', 'servo', thetalistd[2])
    #     send_cmd('Hip_Pitch', 'servo', thetalistd[3])
    #     send_cmd('Hip_Roll', 'servo', 0)
    
    # if joint_state_flag == 1:
    #     ankle_pitch_pos, ankle_pitch_vel = call_ankle_pitch_ecd_data()
    #     ankle_roll_pos, ankle_prollvel = call_ankle_roll_ecd_data()
    #     knee_pos, knee_vel = call_knee_ecd_data()
    #     hip_pitch_pos, hip_pitch_vel = call_hip_pitch_ecd_data()
    #     hip_roll_pos, hip_roll_vel = call_hip_roll_ecd_data()
        
    #     window.Element("-JOINT1-").Update(ankle_pitch_pos[0][0])
    #     window.Element("-JOINT2-").Update(ankle_roll_pos[0][0])
    #     window.Element("-JOINT3-").Update(knee_pos[0][0])
    #     window.Element("-JOINT4-").Update(hip_pitch_pos[0][0])
    #     window.Element("-JOINT5-").Update(hip_roll_pos[0][0])
        
        
#########################################################################
    
    if event == "Balancing Start":
        if balancing_state == 0:
            balancing_flag = 1 
            print("Balancing started")
        else :
            print("Balancing already started")
    
    if balancing_flag == 1:
        rospy.init_node('Imu_read', anonymous=True)
        imu_sub = rospy.Subscriber('/imu/data_raw', Imu, Imucallback)
        wheel_pos, wheel_vel = call_wheel_ecd_data()
        ankle_pos, ankle_vel = call_ankle_pitch_ecd_data()
        if i < 2:
            mpc_model, estimator, u = mpc_pitch.pitch_mpc_init(roll, roll_vel, wheel_vel, ankle_pos, ankle_vel)
            send_cmd('Wheel', 'servo', u[0])
            send_cmd('Ankle_Pitch', 'servo', u[1])
        else:
            u = mpc_pitch.pitch_mpc(roll, roll_vel, wheel_vel, ankle_pos, ankle_vel, mpc_model, estimator)
            send_cmd('Wheel', 'servo', u[0])
            send_cmd('Ankle_Pitch', 'servo', u[1])
        i += 1

    if event == "-Stop-":
        if balancing_state == 0:
                balancing_flag = 0 
                print("Balancing stopped")
        else :
            print("Balancing already stopped")
            
    if event == "All Release":
        balancing_flag = 0 
        send_cmd('Wheel', 'release')
        send_cmd('Ankle_Pitch', 'release')
        send_cmd('Ankle_Roll', 'release')
        send_cmd('Knee', 'release')
        send_cmd('Hip_Pitch', 'release')
        send_cmd('Hip_Roll', 'release')
        
    if event == "Pos ON":
        if connect_flag ==1:
            pos_flag = 1
        
    if pos_flag == 1:
        theta_pos = [roll, roll + ankle_pos]
        window.Element("-POS_TABLE-").Update(theta_pos)
        
    if event == "Pos OFF":
        if connect_flag ==1:
            pos_flag = 0
            
    if event == "Wheel Release":
        balancing_flag = 0 
        send_cmd('Wheel', 'release')

    if event == "Ankle_Pitch Release":
        balancing_flag = 0 
        send_cmd('Ankle_Pitch', 'release')
        
    if event == "Ankle_Roll Release":
        balancing_flag = 0 
        send_cmd('Ankle_Roll', 'release')
        
    if event == "Knee Release":
        balancing_flag = 0 
        send_cmd('Knee', 'release')
        
    if event == "Hip_Pitch Release":
        balancing_flag = 0 
        send_cmd('Hip_Pitch', 'release')
        
    if event == "Hip_Roll Release":
        balancing_flag = 0 
        send_cmd('Hip_Roll', 'release')
        
    
            
#########################################################################

        



