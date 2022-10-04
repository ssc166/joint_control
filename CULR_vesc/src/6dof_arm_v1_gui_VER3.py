from tkinter.constants import NONE, X
import PySimpleGUI as sg
import ctypes
import platform
from PySimpleGUI.PySimpleGUI import T, TRANSPARENT_BUTTON
from numpy.core.defchararray import join
import serial
import time
import modern_robotics as mr
import numpy as np
import math
import concurrent.futures as cf
from openrobot_vesc_pyserial import vesc_pyserial as vs
from openrobot_vesc_pyserial import vesc_pcan as vp
from threading import Timer

######################### Class Init #########################
# pcan

# default serial
selected_ser_name = ''
selected_ser_class = ''
ser_ind = 0
baud = ('115200', '460800', '921600')
tout = 0.5

# default about joint
joint_list = ['J1', 'J2', 'J3', 'J4', 'J5', 'J6']
joint_original_tuple = tuple(joint_list)
vesc_joint_match = {1:'J1', 2:'J2', 3:'J3', 4:'J4', 5:'J5', 6:'J6'} # dictionary type, ID:'Joint Number'
global vesc_id_data
refresh_list_flag = False
vesc_serial_list = []
duty_mode = False
ecd_data = []
connect_flag = 0
on_flag = 0

# Link length : unit [m]
l = np.array([0.165, 0.198, 0.196, 0.116, 0.116, 0.116]) #np.array([0.175, 0.1985, 0.196, 0.116, 0.116, 0.116])

# Homogenous Transformation at Zero position
M = np.array([[1,0,0,0],[0,1,0,0],[0,0,1,l[0]+l[1]+l[2]+l[4]],[0,0,0,1]]) 

## Blist ##
Blist = np.array([[0, 0, 1, 0, 0, 0],
                [0, 1, 0, l[1]+l[2]+l[4], 0, 0],
                [0, 1, 0, l[2]+l[4], 0, 0],
                [0, 1, 0, l[4], 0, 0],
                [0, 0, 1, l[5], 0, 0],
                [0, 1, 0, 0, 0, 0]]).T

T67 = np.array([[1,0,0,0],[0,1,0,0],[0,0,1,0.128],[0,0,0,1]])

joint_home = np.array([0.,0.,0.,0.,0.,0.])
T_current = np.dot(mr.FKinBody(M,Blist,joint_home), T67)

thetalist_current = np.array([0.,0.,0.,0.,0.,0.])
x_current, y_current, z_current, R_current, P_current, Y_current = T_current[0][3], T_current[1][3], T_current[2][3], 0., 0., 0.
# thetalist_current = np.array([100,100,100,100,100,100])
# x_current, y_current, z_current, R_current, P_current, Y_current = 100, 100, 100, 100, 100, 100
via_cnt = 0
v_data = []
v_data_display = []
home_flag = False
gripper_state = 0 # [0 : gripper close, 1 : gripper open]

# gui theme select
sg.theme('Python')   # Add a touch of color

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

def set_traj_control(vesc_target_id, value):
    # prepare data
    comm_set_cmd = vs.COMM_PACKET_ID['COMM_CUSTOM_APP_DATA']
    custom_cmd = vs.COMM_PACKET_ID_OPENROBOT['COMM_SET_TRAJ']

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
        elif cmd == "traj":
            send_data = set_traj_control(vesc_id, value)
            s_class.serial_write(send_data)
        elif cmd == "duty":
            send_data = set_duty_control(vesc_id, value)
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
    # else:
    #     print(joint_number, "is Not Connected")

def adjust_joint_limit(joint_range_i, joint_range_j, keyhere):
    layout_subwin2_1 = [ [sg.Text(keyhere)],
                         [sg.InputText(joint_range[joint_range_i][joint_range_j], key=keyhere), sg.Button('Ok', bind_return_key=True)]
    ]
    window_subwin2_1 = sg.Window('Joint Limit Reset', layout_subwin2_1)
    events_subwin2_1, values_subwin2_1 = window_subwin2_1.Read()

    if events_subwin2_1 == 'Ok':
        try:
            value = int(values_subwin2_1[keyhere])
            #print(type(value), value)
            joint_range[joint_range_i][joint_range_j] = value
            window.Element(event).Update(values_subwin2_1[keyhere])
            window.Refresh()
        except:
            print("Please input number")
    window_subwin2_1.close()

####################### Kinematics #########################

def joint_angle_limit(joint_angle):

    for i in range(len(joint_angle)):
        if i == 0:
            if joint_angle[i] > np.pi:
                joint_angle[i] = np.pi
            if joint_angle[i] < -np.pi:
                joint_angle[i] = -np.pi
        elif i == 1 :
            if joint_angle[i] > 70* np.pi/180: #np.pi/3:
                joint_angle[i] = 70* np.pi/180
            if joint_angle[i] < -70* np.pi/180:
                joint_angle[i] = -70* np.pi/180
        elif i > 1:
            if joint_angle[i] > np.pi/2:
                joint_angle[i] = np.pi/2
            if joint_angle[i] < -np.pi/2:
                joint_angle[i] = -np.pi/2

    return joint_angle
    
def Roll(ang_r):
    return np.array([[1,0,0],[0,np.cos(ang_r),-np.sin(ang_r)],[0,np.sin(ang_r),np.cos(ang_r)]])

def Pitch(ang_p): 
    return np.array([[np.cos(ang_p),0,np.sin(ang_p)],[0,1,0],[-np.sin(ang_p),0,np.cos(ang_p)]])

def Yaw(ang_y):
    return np.array([[np.cos(ang_y),-np.sin(ang_y),0],[np.sin(ang_y),np.cos(ang_y),0],[0,0,1]])

def get_X_d(x, y, z, roll, pitch, yaw):

    R = np.dot(np.dot(Roll(roll), Pitch(pitch)), Yaw(yaw))
    xyz = np.array([x, y, z])
    
    T_d = np.vstack([np.c_[R, xyz], np.array([0,0,0,1])])

    return T_d

def rotationMatrixToEulerAngles(R) :
    """
    경로 생성에서 얻은 Td에서 회전 각도를 구하는 함수.
    End-effector의 Rot_x,y,z를 얻기위해 사용.
    """
    # assert(isRotationMatrix(R))
    sy = math.sqrt(R[0,0] * R[0,0] +  R[1,0] * R[1,0])
    singular = sy < 1e-6

    if  not singular :
        x = math.atan2(R[2,1] , R[2,2])
        y = math.atan2(-R[2,0], sy)
        z = math.atan2(R[1,0], R[0,0])
    else :
        x = math.atan2(-R[1,2], R[1,1])
        y = math.atan2(-R[2,0], sy)
        z = 0
    return np.array([x, y, z])

def convert_radian_from_npi_to_pi(thetalist):
    
    if type(thetalist) is not np.ndarray:
        if(-3.14 > thetalist):
            thetalist = thetalist + np.pi*2
        elif(thetalist > 3.14):
            thetalist = thetalist - np.pi*2
    else:
        for i in range(len(thetalist)):
            while(-3.14 > thetalist[i]):
                thetalist[i] = thetalist[i] + np.pi*2
            while(thetalist[i] > 3.14):
                thetalist[i] = thetalist[i] - np.pi*2
    
    return thetalist

def FK(thetalist):

    global T_current, x_current, y_current, z_current, R_current, P_current, Y_current, thetalist_current, M, Blist, T67
    
    Xnow = np.dot(mr.FKinBody(M,Blist,thetalist), T67)

    val_x = Xnow[0][3]*1000
    val_y = Xnow[1][3]*1000
    val_z = Xnow[2][3]*1000

    val_roll,val_pitch,val_yaw = convert_radian_from_npi_to_pi(rotationMatrixToEulerAngles(Xnow))

    T_current, x_current, y_current, z_current, R_current, P_current, Y_current = Xnow, val_x, val_y, val_z, val_roll, val_pitch, val_yaw
    thetalist_current = thetalist

    # print(Xnow, val_x, val_y, val_z, val_roll, val_pitch, val_yaw)
    # print(thetalist)

    # print("Done !!")

    return Xnow, val_x, val_y, val_z, val_roll, val_pitch, val_yaw

def IK(thetalist, T_desired):

    global M, Blist, T67
    
    theta = thetalist

    T = np.dot(mr.FKinBody(M,Blist,theta),T67)
    T_d = T_desired
    err_v, err_w = 1e-10, 1e-10
    
    i = 0
    max_iterations = 30

    Vb = mr.se3ToVec(mr.MatrixLog6(np.dot(mr.TransInv(T), T_d)))
    err = np.linalg.norm([Vb[0], Vb[1], Vb[2]]) > err_w or np.linalg.norm([Vb[3], Vb[4], Vb[5]]) > err_v
    
    while err and i < max_iterations:

        theta = theta + np.dot(np.linalg.pinv(mr.JacobianBody(Blist, theta)), Vb)
        # theta = joint_angle_limit(theta)
        T = np.dot(mr.FKinBody(M,Blist,theta),T67)
        Vb = mr.se3ToVec(mr.MatrixLog6(np.dot(mr.TransInv(T), T_d)))
        err = np.linalg.norm([Vb[0], Vb[1], Vb[2]]) > err_w or np.linalg.norm([Vb[3], Vb[4], Vb[5]]) > err_v

        i = i + 1

    return theta, not err

def get_via_T(v_data):
    
    global M, Blist, T67
    
    via_thetalist = []
    via_task = []
    via_T = []
    
    for i in range(len(v_data)):
        array_via_theta = np.array([v_data[i][1]])
        array_via_task = np.array([v_data[i][2]])
        via_thetalist.append(array_via_theta)
        via_task.append(array_via_task)
        
        for j in range(1):
            via_T.append(np.dot(mr.FKinBody(M,Blist,via_thetalist[i][0]), T67))

    return via_T, via_thetalist

# intelligent robotics lib #
def Trapezoidal_Traj_Gen_Given_Amax_and_T(amax,T,dt):
    a = amax
    
    if a*math.pow(T,2) >= 4:     
        v = 0.5*(a*T - math.pow(a,0.5)*math.pow((a*math.pow(T,2)-4),0.5))
    else:
        return False, False            
    
    time = 0
    t_save = time
    traj_save = np.array([0,0,0])
    while T >= time:
        if time >= 0 and time <= (v/a):
            sddot = a
            sdot = a*time
            s = 0.5*a*math.pow(time,2)
            
        if time > (v/a) and time <= (T-v/a):
            sddot = 0
            sdot = v 
            s = v*time - 0.5*math.pow(v,2)/a
            
        if time > (T-v/a) and time <= T:
            sddot = -a
            sdot = a*(T-time)
            s = (2*a*v*T - 2*math.pow(v,2) - math.pow(a,2)*math.pow((time-T),2))/(2*a)
        
        t_save = np.vstack((t_save, time))
        traj_save = np.vstack((traj_save, np.array([s,sdot,sddot])))
        time += dt

    return t_save, traj_save

def Path_Gen(start,goal,traj):
    path = start + traj*(goal-start)
    return path

def call_ecd_data():
    send_cmd('J1','request')
    data = selected_ser_class.get_ecd_value()
    return data

def check_ecd_data(currunt_ecd_data, target_pos):
    n = len(target_pos)
    # print(n)
    ecd = np.array([np.nan]*n)
    check_data = np.array([0]*n)
    check = 0
    
    for i in range(n):
        ecd[i] = currunt_ecd_data[i][1]
    # print(ecd)
    for j in range(n):
        if j == 0:
            check_data[j] = 1
        else:
            target_c1 = target_pos[j] + 1
            target_c2 = target_pos[j] - 1
            print(target_c1, target_c2)
            if (ecd[j] == target_pos[j]) or (ecd[j] == target_c1) or (ecd[j] == target_c2): # 엔코더 값이 목표 값과 같지 않을 수도 있어서 범위를 +- 1 로 함.
                # print("joint",j,"check")
                check_data[j] = 1                   
            else:
                check_data[j] = 0
            # print(check_data)
    
    for k in range(n):
        check += check_data[k]
        # print(check)
    
    if check == n:
        return True
    else:
        return False

######################### GUI #########################
# usb port table data 
ports_title = ["Port Name", "VID:PID"]
ports_data = scan_serial_ports()
# vesc id table data
vesc_id_data = []
vesc_id_headings = ["Port","ID","COMM", "Joint"]
layout_col1 = [ [sg.Text('<VESC USB>', font=("Tahoma", 20))],
                [sg.Table(values=ports_data, headings=ports_title, max_col_width=100,
                                col_widths=[12,11],
                                background_color='black',
                                auto_size_columns=False,
                                display_row_numbers=True,
                                justification='center',
                                num_rows=3,
                                alternating_row_color='black',
                                enable_events=True,
                                key='-USB_TABLE-',
                                row_height=40)],
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
                                num_rows=8,
                                alternating_row_color='black',
                                key='-VESC_TABLE-',
                                row_height=40)],
                [sg.Button('SCAN VESC'), sg.Button('Refresh List')],
                [sg.HorizontalSeparator()],
                [sg.Text('<PCAN>', font=("Tahoma", 20))],
                [sg.Button('PCAN OPEN'), sg.Button('PCAN CLOSE')],
                [sg.Text('PCAN disconnected')],
]

# joint_max = 1800
# joint_min = -1800
# joint_range = [[joint_min, joint_max], [joint_min, joint_max], [joint_min, joint_max], [joint_min, joint_max], [joint_min, joint_max], [joint_min, joint_max]]
# joint_range = [[-6480, 6480], [-3240, 3240], [-4680, 4680], [-3600, 3600], [-3600, 3600], [-3600, 3600]]
joint_range = [[-6584, 6376], [-3344, 3136], [-4643, 4717], [-7632, 6768], [-7027, 7373], [-7286, 7114]]
task_range = (-900, 900)
angle_range = (-180, 180)
default_pos = [-130, -104, -78, -86, -288, -201] #[-104, -26, -75, -86, -288, -201]
defualt_task = [0, 0, 803, 0, 0, 0]
size_bar_joint = (21.5,20)
size_bar_task = (30,20)
layout_col2 = [ [sg.Text('<Joint Space Control [deg]>', font=("Tahoma", 17))],
                [sg.Text(joint_list[0]), sg.Text(" "), 
                 sg.Text(joint_range[0][0], enable_events=True, key='-JOINT1_MIN-', size=(8,1)), 
                 sg.Slider(range = [joint_range[0][0], joint_range[0][1]], default_value = default_pos[0], size = size_bar_joint, orientation='h', enable_events=True, key='-JOINT1-'), 
                 sg.Text(joint_range[0][1], enable_events=True, key='-JOINT1_MAX-', size=(8,1)), sg.Button('J1 Release')],
                [sg.Text(joint_list[1]), sg.Text(" "), 
                 sg.Text(joint_range[1][0], enable_events=True, key='-JOINT2_MIN-', size=(8,1)), 
                 sg.Slider(range = [joint_range[1][0], joint_range[1][1]], default_value = default_pos[1], size = size_bar_joint, orientation='h', enable_events=True, key='-JOINT2-'), 
                 sg.Text(joint_range[1][1], enable_events=True, key='-JOINT2_MAX-', size=(8,1)), sg.Button('J2 Release')],
                [sg.Text(joint_list[2]), sg.Text(" "), 
                 sg.Text(joint_range[2][0], enable_events=True, key='-JOINT3_MIN-', size=(8,1)), 
                 sg.Slider(range = [joint_range[2][0], joint_range[2][1]], default_value = default_pos[2], size = size_bar_joint, orientation='h', enable_events=True, key='-JOINT3-'), 
                 sg.Text(joint_range[2][1], enable_events=True, key='-JOINT3_MAX-', size=(8,1)), sg.Button('J3 Release')],
                [sg.Text(joint_list[3]), sg.Text(" "), 
                 sg.Text(joint_range[3][0], enable_events=True, key='-JOINT4_MIN-', size=(8,1)), 
                 sg.Slider(range = [joint_range[3][0], joint_range[3][1]], default_value = default_pos[3], size = size_bar_joint, orientation='h', enable_events=True, key='-JOINT4-'), 
                 sg.Text(joint_range[3][1], enable_events=True, key='-JOINT4_MAX-', size=(8,1)), sg.Button('J4 Release')],
                [sg.Text(joint_list[4]), sg.Text(" "), 
                 sg.Text(joint_range[4][0], enable_events=True, key='-JOINT5_MIN-', size=(8,1)), 
                 sg.Slider(range = [joint_range[4][0], joint_range[4][1]], default_value = default_pos[4], size = size_bar_joint, orientation='h', enable_events=True, key='-JOINT5-'), 
                 sg.Text(joint_range[4][1], enable_events=True, key='-JOINT5_MAX-', size=(8,1)), sg.Button('J5 Release')],
                [sg.Text(joint_list[5]), sg.Text(" "), 
                 sg.Text(joint_range[5][0], enable_events=True, key='-JOINT6_MIN-', size=(8,1)), 
                 sg.Slider(range = [joint_range[5][0], joint_range[5][1]], default_value = default_pos[5], size = size_bar_joint, orientation='h', enable_events=True, key='-JOINT6-'), 
                 sg.Text(joint_range[5][1], enable_events=True, key='-JOINT6_MAX-', size=(8,1)), sg.Button('J6 Release')],
                [sg.Button('IK ON'), sg.Button('IK OFF'), sg.Button('Gripper OPEN'), sg.Button('Gripper CLOSE'), sg.Button('All Release')],
                [sg.HorizontalSeparator()],
                [sg.Text('<Task Space Control [mm, deg]>', font=("Tahoma", 17))],
                [sg.Button('-x'), sg.Text('[X]'), sg.Text(task_range[0]), sg.Slider(range = task_range, default_value = defualt_task[0], size = size_bar_task, orientation='h', enable_events=True, key='-X-'), sg.Text(task_range[1]), sg.Button('+x')],
                [sg.Button('-y'), sg.Text('[Y]'), sg.Text(task_range[0]), sg.Slider(range = task_range, default_value = defualt_task[1], size = size_bar_task, orientation='h', enable_events=True, key='-Y-'), sg.Text(task_range[1]), sg.Button('+y')],
                [sg.Button('-z'), sg.Text('[Z]'), sg.Text(task_range[0]), sg.Slider(range = task_range, default_value = defualt_task[2], size = size_bar_task, orientation='h', enable_events=True, key='-Z-'), sg.Text(task_range[1]), sg.Button('+z')],
                [sg.Button('-R'), sg.Text('[R]'), sg.Text(angle_range[0]), sg.Slider(range = angle_range, default_value = defualt_task[3], size = size_bar_task, orientation='h', enable_events=True, key='-ROLL-'), sg.Text(angle_range[1]), sg.Button('+R')],
                [sg.Button('-P'), sg.Text('[P]'), sg.Text(angle_range[0]), sg.Slider(range = angle_range, default_value = defualt_task[4], size = size_bar_task, orientation='h', enable_events=True, key='-PITCH-'), sg.Text(angle_range[1]), sg.Button('+P')],
                [sg.Button('-Y'), sg.Text('[Y]'), sg.Text(angle_range[0]), sg.Slider(range = angle_range, default_value = defualt_task[5], size = size_bar_task, orientation='h', enable_events=True, key='-YAW-'), sg.Text(angle_range[1]), sg.Button('+Y')],
]

status_data = []
status_heading = ["ID", "msec", "pos_rad", "vel_rps", "curr_A", "motor_temp"]
size_input = (10,None)
# via_data = [["Via1", "(10, 20, 30, 12, 23, 31)", "(10, 20, 30, 12, 23, 31)"], ["Via2", "(14, 23, 13, 23, 32, 13)", "(14, 23, 13, 23, 32, 13)"]]
via_data = []
# via_headings = ["Via Point", "Joint Data", "Task Data"]
via_headings = ["Num", "Joint Data", "Task Data"]
layout_col3 = [ [sg.Text('<Status>', font=("Tahoma", 17)), sg.Button('ON'), sg.Button('OFF')],
                [sg.Table(values=status_data, headings=status_heading, max_col_width=100,
                                background_color='black',
                                col_widths=[6,7,10,10,10,11],
                                auto_size_columns=False,
                                display_row_numbers=False,
                                justification='center',
                                num_rows=8,
                                alternating_row_color='black',
                                key='-STATUS_TABLE-',
                                row_height=40)],
                [sg.HorizontalSeparator()],
                [sg.Text('<Control>', font=("Tahoma", 17))],
                [sg.Button('Finding Home'), sg.Button('Set Current as Home'), sg.Button('Go Home'), sg.Button('Remote Control')],
                [sg.Button('Clear Via Point'), sg.Button('Add Via Point'), sg.Button('Play Via Point')],
                [sg.Table(values=via_data, headings=via_headings, max_col_width=150,
                                background_color='black',
                                col_widths=[4,25,25],
                                auto_size_columns=False,
                                display_row_numbers=True,
                                justification='right',
                                num_rows=10,
                                alternating_row_color='black',
                                key='-TABLE_VIA-',
                                row_height=40)]
]

logging_layout = [ [sg.Text("Anything printed will display here!"), 
                    sg.Button('Clear'),
                    sg.Button('DEBUG PRINT ON'), 
                    sg.Button('DEBUG PRINT OFF'),
                    sg.Button('NON Status CAN MSG Print ON'),
                    sg.Button('NON Status CAN MSG Print OFF')],
                   #[sg.Output(size=(162,20), font='Courier 8', key='-OUTPUT-')],
                   [sg.Text("Terminal Command to"), 
                    sg.Combo(values=joint_list, default_value=joint_list[0], readonly=True, k='-TARGET_JOINT-'), 
                    sg.Input(size=(109,1), focus=True, key='-TERMINAL_INPUT-'), sg.Button('Terminal SEND', bind_return_key=True)],
]
layout_main = [ [sg.Column(layout_col1), 
                 sg.VSeparator(), 
                 sg.Column(layout_col2), 
                 sg.VSeparator(), 
                 sg.Column(layout_col3),
                ],
                [sg.HSeparator()],
                [sg.Column(logging_layout)],
]

# 4k hidpi solution
make_dpi_aware()
#print(platform.architecture())
# Create the Window
window = sg.Window('6-DOF Manipulator Control GUI Ver2', layout_main)

######################### GUI Event Loop #########################
# Event Loop to process "events" and get the "values" of the inputs
while True:
    event, values = window.read(timeout=100)

    DEG2RAD = lambda deg: deg*np.pi/180
    RAD2DEG = lambda rad: rad*180/np.pi

    if event == sg.WIN_CLOSED or event == 'Cancel': # if user closes window or clicks cancel
        for class_instance in vesc_serial_list:
            class_instance.exitThread_usb = True
            class_instance.usb_connection_flag = False
            # close serial
            if class_instance.serial_name is not None:
                if class_instance.serial_name.is_open:
                    class_instance.serial_name.close()



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
        
    if event == "ON":
        if connect_flag == 1:
            pool = cf.ThreadPoolExecutor(max_workers=2)
            
            # pool.submit(send_cmd,'J3','request')
            
            on_flag = 1
            print("VESC Encoder RX Thread Started")
        
        else:
            print("Please select USB Port first")
    
    if on_flag ==1:
        
        # future = pool.submit(call_ecd_data)
        # ecd_data = future.result()
        send_cmd('J1','request')
        st_data = selected_ser_class.get_status_data()
        window.Element('-STATUS_TABLE-').Update(values=st_data)
        # print(future.running())
        # send_cmd('J1','request')
        # ecd_data = selected_ser_class.get_ecd_value()
        # print(ecd_data)
        # print(ecd_data[0][1], ecd_data[1][1], ecd_data[2][1], ecd_data[3][1], ecd_data[4][1], ecd_data[5][1])

    if event == "OFF":
        if connect_flag == 1:
            # selected_ser_class.exitThread_ecd_data = True
            on_flag = 0
            # pool.shutdown()
            print("VESC Encoder RX Thread Ended")
        else:
            print("Please select USB Port first")

    if event == "IK ON":
        if connect_flag == 1:
            send_cmd('J1','request')
            pos = selected_ser_class.get_ecd_value()
            rps = selected_ser_class.get_rps_value()
            
            print(pos, rps)
        else:
            print("Please select USB Port first")
    
    # if event == "IK OFF":

    #     send_cmd('J1','duty',0.1)

    if event == "Gripper OPEN":
        if gripper_state == 0:
            t_end = time.time() + 1.8

            while(time.time() < t_end):
                # send_cmd('J6', 'rcservo', 30500)
                send_cmd('J6', 'rcservo', 37500)

            send_cmd('J6', 'rcservo', 35000)
        else :
            print("Gripper already opened")
    
    if event == "Gripper CLOSE":
        if gripper_state == 1:
            t_end = time.time() + 2

            while(time.time() < t_end):
                send_cmd('J6', 'rcservo', 30500)
                # send_cmd('J1', 'rcservo', 37500)

            send_cmd('J6', 'rcservo', 35000)
        else:
            print("Gripper already closed")

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
                ecd_data = []
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
                # testa = vs.packet_encoding(vs.COMM_PACKET_ID['COMM_CUSTOM_APP_DATA'])
                print("~~~~~~~")
                print(send_data)
                print("~~~~~~~")
                selected_ser_class.serial_write(send_data)
                time.sleep(0.1)
                send_data = vs.packet_encoding(vs.COMM_PACKET_ID['COMM_PING_CAN'])
                print("~~~~~~~")
                print(send_data)
                print("~~~~~~~")
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
        
    if event == "-JOINT1-":
        a = None       
        val = values[event]
        # thetalist = np.array([ -0.33015812, -0.51875064, -0.65315591, -0.57334066,  1.57079633,  0.29714231])
        thetalist = DEG2RAD(np.array([(val-default_pos[0])/36, (values["-JOINT2-"]-default_pos[1])/50, (values["-JOINT3-"]-default_pos[2])/52, (values["-JOINT4-"]-default_pos[3])/40, (values["-JOINT5-"]-default_pos[4])/40, (values["-JOINT6-"]-default_pos[5])/40]))
        val_x, val_y, val_z, val_roll, val_pitch, val_yaw = FK(thetalist)[1:]


        window.Element("-X-").Update(val_x)
        window.Element("-Y-").Update(val_y)
        window.Element("-Z-").Update(val_z)
        window.Element("-ROLL-").Update(RAD2DEG(val_roll))
        window.Element("-PITCH-").Update(RAD2DEG(val_pitch))
        window.Element("-YAW-").Update(RAD2DEG(val_yaw))
        send_cmd('J1', 'servo', val)

    if event == "-JOINT1_MIN-":
        joint_index = 0
        adjust_joint_limit(joint_index,0,'JOINT1 MIN VALUE:')
        window.Element('-JOINT1-').Update(range = [joint_range[joint_index][0], joint_range[joint_index][1]])
        print("Joint", joint_index+1, "is Limit adjusted as", joint_range[joint_index][0], "to", joint_range[joint_index][1])
    if event == "-JOINT1_MAX-":
        joint_index = 0
        adjust_joint_limit(joint_index,1,'JOINT1 MAX VALUE:')
        window.Element('-JOINT1-').Update(range = [joint_range[joint_index][0], joint_range[joint_index][1]])
        print("Joint", joint_index+1, "is Limit adjusted as", joint_range[joint_index][0], "to", joint_range[joint_index][1])

    if event == "-JOINT2-":
        val = values[event]

        # thetalist = np.array([DEG2RAD(values["-JOINT1-"]/36), DEG2RAD(val/36), DEG2RAD(values["-JOINT3-"]/52), DEG2RAD(values["-JOINT4-"]/40), DEG2RAD(values["-JOINT5-"]/40), DEG2RAD(values["-JOINT6-"]/40)])
        thetalist = DEG2RAD(np.array([(values["-JOINT1-"]-default_pos[0])/36, (val-default_pos[1])/36, (values["-JOINT3-"]-default_pos[2])/52, (values["-JOINT4-"]-default_pos[3])/40, (values["-JOINT5-"]-default_pos[4])/40, (values["-JOINT6-"]-default_pos[5])/40]))
        val_x, val_y, val_z, val_roll, val_pitch, val_yaw = FK(thetalist)[1:]

        window.Element("-X-").Update(val_x)
        window.Element("-Y-").Update(val_y)
        window.Element("-Z-").Update(val_z)
        window.Element("-ROLL-").Update(RAD2DEG(val_roll))
        window.Element("-PITCH-").Update(RAD2DEG(val_pitch))
        window.Element("-YAW-").Update(RAD2DEG(val_yaw))

        send_cmd('J2', 'servo', val)
    if event == "-JOINT2_MIN-":
        joint_index = 1
        adjust_joint_limit(joint_index,0,'JOINT2 MIN VALUE:')
        window.Element('-JOINT2-').Update(range = [joint_range[joint_index][0], joint_range[joint_index][1]])
        print("Joint", joint_index+1, "is Limit adjusted as", joint_range[joint_index][0], "to", joint_range[joint_index][1])
    if event == "-JOINT2_MAX-":
        joint_index = 1
        adjust_joint_limit(joint_index,1,'JOINT2 MAX VALUE:')
        window.Element('-JOINT2-').Update(range = [joint_range[joint_index][0], joint_range[joint_index][1]])
        print("Joint", joint_index+1, "is Limit adjusted as", joint_range[joint_index][0], "to", joint_range[joint_index][1])

    if event == "-JOINT3-":
        val = values[event]

        # thetalist = np.array([DEG2RAD(values["-JOINT1-"]/36),DEG2RAD(values["-JOINT2-"]/50), DEG2RAD(val/52), DEG2RAD(values["-JOINT4-"]/40), DEG2RAD(values["-JOINT5-"]/40), DEG2RAD(values["-JOINT6-"]/40)])
        thetalist = DEG2RAD(np.array([(values["-JOINT1-"]-default_pos[0])/36, (values["-JOINT2-"]-default_pos[1])/50, (val-default_pos[2])/52, (values["-JOINT4-"]-default_pos[3])/40, (values["-JOINT5-"]-default_pos[4])/40, (values["-JOINT6-"]-default_pos[5])/40]))
        
        val_x, val_y, val_z, val_roll, val_pitch, val_yaw = FK(thetalist)[1:]

        window.Element("-X-").Update(val_x)
        window.Element("-Y-").Update(val_y)
        window.Element("-Z-").Update(val_z)
        window.Element("-ROLL-").Update(RAD2DEG(val_roll))
        window.Element("-PITCH-").Update(RAD2DEG(val_pitch))
        window.Element("-YAW-").Update(RAD2DEG(val_yaw))

        send_cmd('J3', 'servo', val)
    if event == "-JOINT3_MIN-":
        joint_index = 2
        adjust_joint_limit(joint_index,0,'JOINT3 MIN VALUE:')
        window.Element('-JOINT3-').Update(range = [joint_range[joint_index][0], joint_range[joint_index][1]])
        print("Joint", joint_index+1, "is Limit adjusted as", joint_range[joint_index][0], "to", joint_range[joint_index][1])
    if event == "-JOINT3_MAX-":
        joint_index = 2
        adjust_joint_limit(joint_index,1,'JOINT3 MAX VALUE:')
        window.Element('-JOINT3-').Update(range = [joint_range[joint_index][0], joint_range[joint_index][1]])
        print("Joint", joint_index+1, "is Limit adjusted as", joint_range[joint_index][0], "to", joint_range[joint_index][1])

    if event == "-JOINT4-":
        val = values[event]

        # thetalist = np.array([DEG2RAD(values["-JOINT1-"]/36),DEG2RAD(values["-JOINT2-"]/50), DEG2RAD(values["-JOINT3-"]/52), DEG2RAD(val/40), DEG2RAD(values["-JOINT5-"]/40), DEG2RAD(values["-JOINT6-"]/40)])
        thetalist = DEG2RAD(np.array([(values["-JOINT1-"]-default_pos[0])/36, (values["-JOINT2-"]-default_pos[1])/50, (values["-JOINT3-"]-default_pos[2])/52, (val-default_pos[3])/40, (values["-JOINT5-"]-default_pos[4])/40, (values["-JOINT6-"]-default_pos[5])/40]))
        val_x, val_y, val_z, val_roll, val_pitch, val_yaw = FK(thetalist)[1:]

        window.Element("-X-").Update(val_x)
        window.Element("-Y-").Update(val_y)
        window.Element("-Z-").Update(val_z)
        window.Element("-ROLL-").Update(RAD2DEG(val_roll))
        window.Element("-PITCH-").Update(RAD2DEG(val_pitch))
        window.Element("-YAW-").Update(RAD2DEG(val_yaw))

        send_cmd('J4', 'servo', val)
    if event == "-JOINT4_MIN-":
        joint_index = 3
        adjust_joint_limit(joint_index,0,'JOINT4 MIN VALUE:')
        window.Element('-JOINT4-').Update(range = [joint_range[joint_index][0], joint_range[joint_index][1]])
        print("Joint", joint_index+1, "is Limit adjusted as", joint_range[joint_index][0], "to", joint_range[joint_index][1])
    if event == "-JOINT4_MAX-":
        joint_index = 3
        adjust_joint_limit(joint_index,1,'JOINT4 MAX VALUE:')
        window.Element('-JOINT4-').Update(range = [joint_range[joint_index][0], joint_range[joint_index][1]])
        print("Joint", joint_index+1, "is Limit adjusted as", joint_range[joint_index][0], "to", joint_range[joint_index][1])

    if event == "-JOINT5-":
        val = values[event]

        # thetalist = np.array([DEG2RAD(values["-JOINT1-"]/36),DEG2RAD(values["-JOINT2-"]/50), DEG2RAD(values["-JOINT3-"]/52), DEG2RAD(values["-JOINT4-"]/40), DEG2RAD(val/40), DEG2RAD(values["-JOINT6-"]/40)])
        thetalist = DEG2RAD(np.array([(values["-JOINT1-"]-default_pos[0])/36, (values["-JOINT2-"]-default_pos[1])/50, (values["-JOINT3-"]-default_pos[2])/52, (values["-JOINT4-"]-default_pos[3])/40, (val-default_pos[4])/40, (values["-JOINT6-"]-default_pos[5])/40]))
        val_x, val_y, val_z, val_roll, val_pitch, val_yaw = FK(thetalist)[1:]

        window.Element("-X-").Update(val_x)
        window.Element("-Y-").Update(val_y)
        window.Element("-Z-").Update(val_z)
        window.Element("-ROLL-").Update(RAD2DEG(val_roll))
        window.Element("-PITCH-").Update(RAD2DEG(val_pitch))
        window.Element("-YAW-").Update(RAD2DEG(val_yaw))

        send_cmd('J5', 'servo', val)
    if event == "-JOINT5_MIN-":
        joint_index = 4
        adjust_joint_limit(joint_index,0,'JOINT5 MIN VALUE:')
        window.Element('-JOINT5-').Update(range = [joint_range[joint_index][0], joint_range[joint_index][1]])
        print("Joint", joint_index+1, "is Limit adjusted as", joint_range[joint_index][0], "to", joint_range[joint_index][1])
    if event == "-JOINT5_MAX-":
        joint_index = 4
        adjust_joint_limit(joint_index,1,'JOINT5 MAX VALUE:')
        window.Element('-JOINT5-').Update(range = [joint_range[joint_index][0], joint_range[joint_index][1]])
        print("Joint", joint_index+1, "is Limit adjusted as", joint_range[joint_index][0], "to", joint_range[joint_index][1])

    if event == "-JOINT6-":
        val = values[event]

        # thetalist = np.array([DEG2RAD(values["-JOINT1-"]/36),DEG2RAD(values["-JOINT2-"]/50), DEG2RAD(values["-JOINT3-"]/52), DEG2RAD(values["-JOINT4-"]/40), DEG2RAD(values["-JOINT5-"]/40), DEG2RAD(val/40)])
        thetalist = DEG2RAD(np.array([(values["-JOINT1-"]-default_pos[0])/36, (values["-JOINT2-"]-default_pos[1])/50, (values["-JOINT3-"]-default_pos[2])/52, (values["-JOINT4-"]-default_pos[3])/40, (values["-JOINT5-"]-default_pos[4])/40, (val-default_pos[5])/40]))
        val_x, val_y, val_z, val_roll, val_pitch, val_yaw = FK(thetalist)[1:]

        window.Element("-X-").Update(val_x)
        window.Element("-Y-").Update(val_y)
        window.Element("-Z-").Update(val_z)
        window.Element("-ROLL-").Update(RAD2DEG(val_roll))
        window.Element("-PITCH-").Update(RAD2DEG(val_pitch))
        window.Element("-YAW-").Update(RAD2DEG(val_yaw))
        
        send_cmd('J6', 'servo', val) 
    if event == "-JOINT6_MIN-":
        joint_index = 5
        adjust_joint_limit(joint_index,0,'JOINT6 MIN VALUE:')
        window.Element('-JOINT6-').Update(range = [joint_range[joint_index][0], joint_range[joint_index][1]])
        print("Joint", joint_index+1, "is Limit adjusted as", joint_range[joint_index][0], "to", joint_range[joint_index][1])
    if event == "-JOINT6_MAX-":
        joint_index = 5
        adjust_joint_limit(joint_index,1,'JOINT6 MAX VALUE:')
        window.Element('-JOINT6-').Update(range = [joint_range[joint_index][0], joint_range[joint_index][1]])
        print("Joint", joint_index+1, "is Limit adjusted as", joint_range[joint_index][0], "to", joint_range[joint_index][1])

    if event == "All Release":
        for joint in joint_list:
            send_cmd(joint, 'release')
        
        duty_mode = False

    if event == "J1 Release":
        send_cmd('J1', 'release')

    if event == "J2 Release":
        send_cmd('J2', 'release')

    if event == "J3 Release":
        send_cmd('J3', 'release')

    if event == "J4 Release":
        send_cmd('J4', 'release')

    if event == "J5 Release":
        send_cmd('J5', 'release')

    if event == "J6 Release":
        send_cmd('J6', 'release')

    if event == "+x":

        val = values["-X-"]
        val_up = val+1
        window.Element("-X-").Update(val_up)
        val_x = val_up*0.001
        T_current[0][3] = val_x

        thetalist = thetalist_current
        
        T_sd = T_current
        print(T_sd)

        theta, success = IK(thetalist, T_sd)
        theta = convert_radian_from_npi_to_pi(theta)
    
        if success is True:
            Xnow, val_x, val_y, val_z, val_roll, val_pitch, val_yaw = FK(theta)

            val_roll = RAD2DEG(val_roll)
            val_pitch = RAD2DEG(val_pitch)
            val_yaw = RAD2DEG(val_yaw)

            window.Element("-Y-").Update(val_y)
            window.Element("-Z-").Update(val_z)
            window.Element("-ROLL-").Update(val_roll)
            window.Element("-PITCH-").Update(val_pitch)
            window.Element("-YAW-").Update(val_yaw)

            window.Element("-JOINT1-").Update((RAD2DEG(theta[0])*36)+default_pos[0])
            window.Element("-JOINT2-").Update((RAD2DEG(theta[1])*50)+default_pos[1])
            window.Element("-JOINT3-").Update((RAD2DEG(theta[2])*52)+default_pos[2])
            window.Element("-JOINT4-").Update((RAD2DEG(theta[3])*40)+default_pos[3])
            window.Element("-JOINT5-").Update((RAD2DEG(theta[4])*40)+default_pos[4])
            window.Element("-JOINT6-").Update((RAD2DEG(theta[5])*40)+default_pos[5])

            send_cmd('J1', 'servo', (RAD2DEG(theta[0])*36)+default_pos[0])
            send_cmd('J2', 'servo', (RAD2DEG(theta[1])*50)+default_pos[1])
            send_cmd('J3', 'servo', (RAD2DEG(theta[2])*52)+default_pos[2])
            send_cmd('J4', 'servo', (RAD2DEG(theta[3])*40)+default_pos[3])
            send_cmd('J5', 'servo', (RAD2DEG(theta[4])*40)+default_pos[4])
            send_cmd('J6', 'servo', (RAD2DEG(theta[5])*40)+default_pos[5])
        
        elif not success:
            print("Inverse kinematic fail")
            window.Element("-X-").Update(val)
            T_current[0][3] = val
            print(T_sd)

        
    if event == "-x":

        val = values["-X-"]
        val_up = val-1
        window.Element("-X-").Update(val_up)
        val_x = val_up*0.001
        T_current[0][3] = val_x

        thetalist = thetalist_current
        # thetalist = np.array([DEG2RAD(values["-JOINT1-"]/36),DEG2RAD(values["-JOINT2-"]/50), DEG2RAD(values["-JOINT3-"]/52), DEG2RAD(values["-JOINT4-"]/40), DEG2RAD(values["-JOINT5-"]/40), DEG2RAD(values["-JOINT6-"]/40)])
        
        T_sd = T_current
        print(T_sd)

        theta, success = IK(thetalist, T_sd)
        theta = convert_radian_from_npi_to_pi(theta)
    
        if success is True:
            Xnow, val_x, val_y, val_z, val_roll, val_pitch, val_yaw = FK(theta)

            val_roll = RAD2DEG(val_roll)
            val_pitch = RAD2DEG(val_pitch)
            val_yaw = RAD2DEG(val_yaw)

            window.Element("-Y-").Update(val_y)
            window.Element("-Z-").Update(val_z)
            window.Element("-ROLL-").Update(val_roll)
            window.Element("-PITCH-").Update(val_pitch)
            window.Element("-YAW-").Update(val_yaw)

            window.Element("-JOINT1-").Update((RAD2DEG(theta[0])*36)+default_pos[0])
            window.Element("-JOINT2-").Update((RAD2DEG(theta[1])*50)+default_pos[1])
            window.Element("-JOINT3-").Update((RAD2DEG(theta[2])*52)+default_pos[2])
            window.Element("-JOINT4-").Update((RAD2DEG(theta[3])*40)+default_pos[3])
            window.Element("-JOINT5-").Update((RAD2DEG(theta[4])*40)+default_pos[4])
            window.Element("-JOINT6-").Update((RAD2DEG(theta[5])*40)+default_pos[5])

            send_cmd('J1', 'servo', (RAD2DEG(theta[0])*36)+default_pos[0])
            send_cmd('J2', 'servo', (RAD2DEG(theta[1])*50)+default_pos[1])
            send_cmd('J3', 'servo', (RAD2DEG(theta[2])*52)+default_pos[2])
            send_cmd('J4', 'servo', (RAD2DEG(theta[3])*40)+default_pos[3])
            send_cmd('J5', 'servo', (RAD2DEG(theta[4])*40)+default_pos[4])
            send_cmd('J6', 'servo', (RAD2DEG(theta[5])*40)+default_pos[5])

        elif not success:
            print("Inverse kinematic fail")
            window.Element("-X-").Update(val)
            T_current[0][3] = val
            print(T_sd)
    
    if event == "+y":

        val = values["-Y-"]
        val_up = val+1
        window.Element("-Y-").Update(val_up)
        val_y = val_up*0.001
        T_current[1][3] = val_y

        thetalist = thetalist_current
        # thetalist = np.array([DEG2RAD(values["-JOINT1-"]/36),DEG2RAD(values["-JOINT2-"]/50), DEG2RAD(values["-JOINT3-"]/52), DEG2RAD(values["-JOINT4-"]/40), DEG2RAD(values["-JOINT5-"]/40), DEG2RAD(values["-JOINT6-"]/40)])
        
        T_sd = T_current
        print(T_sd)

        theta, success = IK(thetalist, T_sd)
        theta = convert_radian_from_npi_to_pi(theta)
    
        if success is True:
            Xnow, val_x, val_y, val_z, val_roll, val_pitch, val_yaw = FK(theta)

            val_roll = RAD2DEG(val_roll)
            val_pitch = RAD2DEG(val_pitch)
            val_yaw = RAD2DEG(val_yaw)

            window.Element("-X-").Update(val_x)
            window.Element("-Z-").Update(val_z)
            window.Element("-ROLL-").Update(val_roll)
            window.Element("-PITCH-").Update(val_pitch)
            window.Element("-YAW-").Update(val_yaw)

            window.Element("-JOINT1-").Update((RAD2DEG(theta[0])*36)+default_pos[0])
            window.Element("-JOINT2-").Update((RAD2DEG(theta[1])*50)+default_pos[1])
            window.Element("-JOINT3-").Update((RAD2DEG(theta[2])*52)+default_pos[2])
            window.Element("-JOINT4-").Update((RAD2DEG(theta[3])*40)+default_pos[3])
            window.Element("-JOINT5-").Update((RAD2DEG(theta[4])*40)+default_pos[4])
            window.Element("-JOINT6-").Update((RAD2DEG(theta[5])*40)+default_pos[5])

            send_cmd('J1', 'servo', (RAD2DEG(theta[0])*36)+default_pos[0])
            send_cmd('J2', 'servo', (RAD2DEG(theta[1])*50)+default_pos[1])
            send_cmd('J3', 'servo', (RAD2DEG(theta[2])*52)+default_pos[2])
            send_cmd('J4', 'servo', (RAD2DEG(theta[3])*40)+default_pos[3])
            send_cmd('J5', 'servo', (RAD2DEG(theta[4])*40)+default_pos[4])
            send_cmd('J6', 'servo', (RAD2DEG(theta[5])*40)+default_pos[5])
        
        elif not success:
            print("Inverse kinematic fail")
            window.Element("-Y-").Update(val)
            T_current[1][3] = val
            print(T_sd)

        
    if event == "-y":

        val = values["-Y-"]
        val_up = val-1
        window.Element("-Y-").Update(val_up)
        val_y = val_up*0.001
        T_current[1][3] = val_y

        thetalist = thetalist_current
        # thetalist = np.array([DEG2RAD(values["-JOINT1-"]/36),DEG2RAD(values["-JOINT2-"]/50), DEG2RAD(values["-JOINT3-"]/52), DEG2RAD(values["-JOINT4-"]/40), DEG2RAD(values["-JOINT5-"]/40), DEG2RAD(values["-JOINT6-"]/40)])
        
        T_sd = T_current
        print(T_sd)

        theta, success = IK(thetalist, T_sd)
        theta = convert_radian_from_npi_to_pi(theta)
    
        if success is True:
            Xnow, val_x, val_y, val_z, val_roll, val_pitch, val_yaw = FK(theta)

            val_roll = RAD2DEG(val_roll)
            val_pitch = RAD2DEG(val_pitch)
            val_yaw = RAD2DEG(val_yaw)

            window.Element("-X-").Update(val_x)
            window.Element("-Z-").Update(val_z)
            window.Element("-ROLL-").Update(val_roll)
            window.Element("-PITCH-").Update(val_pitch)
            window.Element("-YAW-").Update(val_yaw)

            window.Element("-JOINT1-").Update((RAD2DEG(theta[0])*36)+default_pos[0])
            window.Element("-JOINT2-").Update((RAD2DEG(theta[1])*50)+default_pos[1])
            window.Element("-JOINT3-").Update((RAD2DEG(theta[2])*52)+default_pos[2])
            window.Element("-JOINT4-").Update((RAD2DEG(theta[3])*40)+default_pos[3])
            window.Element("-JOINT5-").Update((RAD2DEG(theta[4])*40)+default_pos[4])
            window.Element("-JOINT6-").Update((RAD2DEG(theta[5])*40)+default_pos[5])

            send_cmd('J1', 'servo', (RAD2DEG(theta[0])*36)+default_pos[0])
            send_cmd('J2', 'servo', (RAD2DEG(theta[1])*50)+default_pos[1])
            send_cmd('J3', 'servo', (RAD2DEG(theta[2])*52)+default_pos[2])
            send_cmd('J4', 'servo', (RAD2DEG(theta[3])*40)+default_pos[3])
            send_cmd('J5', 'servo', (RAD2DEG(theta[4])*40)+default_pos[4])
            send_cmd('J6', 'servo', (RAD2DEG(theta[5])*40)+default_pos[5])

        elif not success:
            print("Inverse kinematic fail")
            window.Element("-Y-").Update(val)
            T_current[1][3] = val
            print(T_sd)
    
    if event == "+z":

        val = values["-Z-"]
        val_up = val+1
        window.Element("-Z-").Update(val_up)
        val_z = val_up*0.001
        T_current[2][3] = val_z

        thetalist = thetalist_current
        # thetalist = np.array([DEG2RAD(values["-JOINT1-"]/36),DEG2RAD(values["-JOINT2-"]/50), DEG2RAD(values["-JOINT3-"]/52), DEG2RAD(values["-JOINT4-"]/40), DEG2RAD(values["-JOINT5-"]/40), DEG2RAD(values["-JOINT6-"]/40)])
        
        T_sd = T_current
        print(T_sd)

        theta, success = IK(thetalist, T_sd)
        theta = convert_radian_from_npi_to_pi(theta)
    
        if success is True:
            Xnow, val_x, val_y, val_z, val_roll, val_pitch, val_yaw = FK(theta)

            val_roll = RAD2DEG(val_roll)
            val_pitch = RAD2DEG(val_pitch)
            val_yaw = RAD2DEG(val_yaw)

            window.Element("-X-").Update(val_x)
            window.Element("-Y-").Update(val_y)
            window.Element("-ROLL-").Update(val_roll)
            window.Element("-PITCH-").Update(val_pitch)
            window.Element("-YAW-").Update(val_yaw)

            window.Element("-JOINT1-").Update((RAD2DEG(theta[0])*36)+default_pos[0])
            window.Element("-JOINT2-").Update((RAD2DEG(theta[1])*50)+default_pos[1])
            window.Element("-JOINT3-").Update((RAD2DEG(theta[2])*52)+default_pos[2])
            window.Element("-JOINT4-").Update((RAD2DEG(theta[3])*40)+default_pos[3])
            window.Element("-JOINT5-").Update((RAD2DEG(theta[4])*40)+default_pos[4])
            window.Element("-JOINT6-").Update((RAD2DEG(theta[5])*40)+default_pos[5])

            send_cmd('J1', 'servo', (RAD2DEG(theta[0])*36)+default_pos[0])
            send_cmd('J2', 'servo', (RAD2DEG(theta[1])*50)+default_pos[1])
            send_cmd('J3', 'servo', (RAD2DEG(theta[2])*52)+default_pos[2])
            send_cmd('J4', 'servo', (RAD2DEG(theta[3])*40)+default_pos[3])
            send_cmd('J5', 'servo', (RAD2DEG(theta[4])*40)+default_pos[4])
            send_cmd('J6', 'servo', (RAD2DEG(theta[5])*40)+default_pos[5])
        
        elif not success:
            print("Inverse kinematic fail")
            window.Element("-Z-").Update(val)
            T_current[2][3] = val
            print(T_sd)

        
    if event == "-z":

        val = values["-Z-"]
        val_up = val-1
        window.Element("-Z-").Update(val_up)
        val_z = val_up*0.001
        T_current[2][3] = val_z

        thetalist = thetalist_current
        # thetalist = np.array([DEG2RAD(values["-JOINT1-"]/36),DEG2RAD(values["-JOINT2-"]/50), DEG2RAD(values["-JOINT3-"]/52), DEG2RAD(values["-JOINT4-"]/40), DEG2RAD(values["-JOINT5-"]/40), DEG2RAD(values["-JOINT6-"]/40)])
        
        T_sd = T_current
        print(T_sd)

        theta, success = IK(thetalist, T_sd)
        theta = convert_radian_from_npi_to_pi(theta)
    
        if success is True:
            Xnow, val_x, val_y, val_z, val_roll, val_pitch, val_yaw = FK(theta)

            val_roll = RAD2DEG(val_roll)
            val_pitch = RAD2DEG(val_pitch)
            val_yaw = RAD2DEG(val_yaw)

            window.Element("-X-").Update(val_x)
            window.Element("-Y-").Update(val_y)
            window.Element("-ROLL-").Update(val_roll)
            window.Element("-PITCH-").Update(val_pitch)
            window.Element("-YAW-").Update(val_yaw)

            window.Element("-JOINT1-").Update((RAD2DEG(theta[0])*36)+default_pos[0])
            window.Element("-JOINT2-").Update((RAD2DEG(theta[1])*50)+default_pos[1])
            window.Element("-JOINT3-").Update((RAD2DEG(theta[2])*52)+default_pos[2])
            window.Element("-JOINT4-").Update((RAD2DEG(theta[3])*40)+default_pos[3])
            window.Element("-JOINT5-").Update((RAD2DEG(theta[4])*40)+default_pos[4])
            window.Element("-JOINT6-").Update((RAD2DEG(theta[5])*40)+default_pos[5])

            send_cmd('J1', 'servo', (RAD2DEG(theta[0])*36)+default_pos[0])
            send_cmd('J2', 'servo', (RAD2DEG(theta[1])*50)+default_pos[1])
            send_cmd('J3', 'servo', (RAD2DEG(theta[2])*52)+default_pos[2])
            send_cmd('J4', 'servo', (RAD2DEG(theta[3])*40)+default_pos[3])
            send_cmd('J5', 'servo', (RAD2DEG(theta[4])*40)+default_pos[4])
            send_cmd('J6', 'servo', (RAD2DEG(theta[5])*40)+default_pos[5])
        
        elif not success:
            print("Inverse kinematic fail")
            window.Element("-Z-").Update(val)
            T_current[2][3] = val
            print(T_sd)
    
    if event == "+R":

        val = values["-ROLL-"]
        val_up = val+1
        val_up_rad = convert_radian_from_npi_to_pi(DEG2RAD(val_up))
        window.Element("-ROLL-").Update(val_up)
        T_sd = get_X_d(x_current,y_current,z_current,val_up_rad,P_current, Y_current)

        thetalist = thetalist_current
        # thetalist = np.array([DEG2RAD(values["-JOINT1-"]/36),DEG2RAD(values["-JOINT2-"]/50), DEG2RAD(values["-JOINT3-"]/52), DEG2RAD(values["-JOINT4-"]/40), DEG2RAD(values["-JOINT5-"]/40), DEG2RAD(values["-JOINT6-"]/40)])
        
        # print(T_sd)

        theta, success = IK(thetalist, T_sd)
        theta = convert_radian_from_npi_to_pi(theta)
    
        if success is True:
            Xnow, val_x, val_y, val_z, val_roll, val_pitch, val_yaw = FK(theta)

            val_roll = RAD2DEG(val_roll)
            val_pitch = RAD2DEG(val_pitch)
            val_yaw = RAD2DEG(val_yaw)

            window.Element("-X-").Update(val_x)
            window.Element("-Y-").Update(val_y)
            window.Element("-PITCH-").Update(val_pitch)
            window.Element("-YAW-").Update(val_yaw)

            window.Element("-JOINT1-").Update((RAD2DEG(theta[0])*36)+default_pos[0])
            window.Element("-JOINT2-").Update((RAD2DEG(theta[1])*50)+default_pos[1])
            window.Element("-JOINT3-").Update((RAD2DEG(theta[2])*52)+default_pos[2])
            window.Element("-JOINT4-").Update((RAD2DEG(theta[3])*40)+default_pos[3])
            window.Element("-JOINT5-").Update((RAD2DEG(theta[4])*40)+default_pos[4])
            window.Element("-JOINT6-").Update((RAD2DEG(theta[5])*40)+default_pos[5])

            # send_cmd('J1', 'servo', (RAD2DEG(theta[0])*36)+default_pos[0])
            # send_cmd('J2', 'servo', (RAD2DEG(theta[1])*50)+default_pos[1])
            # send_cmd('J3', 'servo', (RAD2DEG(theta[2])*52)+default_pos[2])
            # send_cmd('J4', 'servo', (RAD2DEG(theta[3])*40)+default_pos[3])
            # send_cmd('J5', 'servo', (RAD2DEG(theta[4])*40)+default_pos[4])
            # send_cmd('J6', 'servo', (RAD2DEG(theta[5])*40)+default_pos[5])
        
        elif not success:
            print("Inverse kinematic fail")
            window.Element("-ROLL-").Update(val)

    if event == "-R":

        val = values["-ROLL-"]
        val_up = val-1
        val_up_rad = convert_radian_from_npi_to_pi(DEG2RAD(val_up))
        window.Element("-ROLL-").Update(val_up)
        T_sd = get_X_d(x_current,y_current,z_current,val_up_rad,P_current, Y_current)

        thetalist = thetalist_current
        # thetalist = np.array([DEG2RAD(values["-JOINT1-"]/36),DEG2RAD(values["-JOINT2-"]/50), DEG2RAD(values["-JOINT3-"]/52), DEG2RAD(values["-JOINT4-"]/40), DEG2RAD(values["-JOINT5-"]/40), DEG2RAD(values["-JOINT6-"]/40)])
        
        theta, success = IK(thetalist, T_sd)
        theta = convert_radian_from_npi_to_pi(theta)
    
        if success is True:
            Xnow, val_x, val_y, val_z, val_roll, val_pitch, val_yaw = FK(theta)

            val_roll = RAD2DEG(val_roll)
            val_pitch = RAD2DEG(val_pitch)
            val_yaw = RAD2DEG(val_yaw)

            window.Element("-X-").Update(val_x)
            window.Element("-Y-").Update(val_y)
            window.Element("-PITCH-").Update(val_pitch)
            window.Element("-YAW-").Update(val_yaw)

            window.Element("-JOINT1-").Update((RAD2DEG(theta[0])*36)+default_pos[0])
            window.Element("-JOINT2-").Update((RAD2DEG(theta[1])*50)+default_pos[1])
            window.Element("-JOINT3-").Update((RAD2DEG(theta[2])*52)+default_pos[2])
            window.Element("-JOINT4-").Update((RAD2DEG(theta[3])*40)+default_pos[3])
            window.Element("-JOINT5-").Update((RAD2DEG(theta[4])*40)+default_pos[4])
            window.Element("-JOINT6-").Update((RAD2DEG(theta[5])*40)+default_pos[5])

            # send_cmd('J1', 'servo', (RAD2DEG(theta[0])*36)+default_pos[0])
            # send_cmd('J2', 'servo', (RAD2DEG(theta[1])*50)+default_pos[1])
            # send_cmd('J3', 'servo', (RAD2DEG(theta[2])*52)+default_pos[2])
            # send_cmd('J4', 'servo', (RAD2DEG(theta[3])*40)+default_pos[3])
            # send_cmd('J5', 'servo', (RAD2DEG(theta[4])*40)+default_pos[4])
            # send_cmd('J6', 'servo', (RAD2DEG(theta[5])*40)+default_pos[5])
        
        elif not success:
            print("Inverse kinematic fail")
            window.Element("-ROLL-").Update(val)

    if event == "+P":

        val = values["-PITCH-"]
        val_up = val+1
        window.Element("-PITCH-").Update(val_up)
        val_up_rad = convert_radian_from_npi_to_pi(DEG2RAD(val_up))
        T_sd = get_X_d(x_current,y_current,z_current,R_current, val_up_rad, Y_current)
        # print(T_sd)

        thetalist = thetalist_current
        # thetalist = np.array([DEG2RAD(values["-JOINT1-"]/36),DEG2RAD(values["-JOINT2-"]/50), DEG2RAD(values["-JOINT3-"]/52), DEG2RAD(values["-JOINT4-"]/40), DEG2RAD(values["-JOINT5-"]/40), DEG2RAD(values["-JOINT6-"]/40)])
        
        # print(T_sd)

        theta, success = IK(thetalist, T_sd)
        theta = convert_radian_from_npi_to_pi(theta)
    
        if success is True:
            Xnow, val_x, val_y, val_z, val_roll, val_pitch, val_yaw = FK(theta)

            val_roll = RAD2DEG(val_roll)
            val_pitch = RAD2DEG(val_pitch)
            val_yaw = RAD2DEG(val_yaw)

            window.Element("-X-").Update(val_x)
            window.Element("-Y-").Update(val_y)
            window.Element("-ROLL-").Update(val_roll)
            window.Element("-YAW-").Update(val_yaw)

            window.Element("-JOINT1-").Update((RAD2DEG(theta[0])*36)+default_pos[0])
            window.Element("-JOINT2-").Update((RAD2DEG(theta[1])*50)+default_pos[1])
            window.Element("-JOINT3-").Update((RAD2DEG(theta[2])*52)+default_pos[2])
            window.Element("-JOINT4-").Update((RAD2DEG(theta[3])*40)+default_pos[3])
            window.Element("-JOINT5-").Update((RAD2DEG(theta[4])*40)+default_pos[4])
            window.Element("-JOINT6-").Update((RAD2DEG(theta[5])*40)+default_pos[5])

            # send_cmd('J1', 'servo', (RAD2DEG(theta[0])*36)+default_pos[0])
            # send_cmd('J2', 'servo', (RAD2DEG(theta[1])*50)+default_pos[1])
            # send_cmd('J3', 'servo', (RAD2DEG(theta[2])*52)+default_pos[2])
            # send_cmd('J4', 'servo', (RAD2DEG(theta[3])*40)+default_pos[3])
            # send_cmd('J5', 'servo', (RAD2DEG(theta[4])*40)+default_pos[4])
            # send_cmd('J6', 'servo', (RAD2DEG(theta[5])*40)+default_pos[5])

            # if home_flag is True:
            #     joint_1.publish(np.float64(theta[0]))
            #     joint_2.publish(np.float64(theta[1]))
            #     joint_3.publish(np.float64(theta[2]))
            #     joint_4.publish(np.float64(theta[3]))
            #     joint_5.publish(np.float64(theta[4]))
            #     joint_6.publish(np.float64(theta[5]))
        
        elif not success:
            print("Inverse kinematic fail")
            window.Element("-PITCH-").Update(val)

    if event == "-P":

        val = values["-PITCH-"]
        val_up = val-1
        window.Element("-PITCH-").Update(val_up)
        val_up_rad = DEG2RAD(val_up)
        T_sd = get_X_d(x_current,y_current,z_current,R_current, val_up_rad, Y_current)

        thetalist = thetalist_current
        # thetalist = np.array([DEG2RAD(values["-JOINT1-"]/36),DEG2RAD(values["-JOINT2-"]/50), DEG2RAD(values["-JOINT3-"]/52), DEG2RAD(values["-JOINT4-"]/40), DEG2RAD(values["-JOINT5-"]/40), DEG2RAD(values["-JOINT6-"]/40)])
        
        # print(T_sd)

        theta, success = IK(thetalist, T_sd)
        theta = convert_radian_from_npi_to_pi(theta)
    
        if success is True:
            print("")
            Xnow, val_x, val_y, val_z, val_roll, val_pitch, val_yaw = FK(theta)

            val_roll = RAD2DEG(val_roll)
            val_pitch = RAD2DEG(val_pitch)
            val_yaw = RAD2DEG(val_yaw)

            window.Element("-X-").Update(val_x)
            window.Element("-Y-").Update(val_y)
            window.Element("-ROLL-").Update(val_roll)
            window.Element("-YAW-").Update(val_yaw)

            window.Element("-JOINT1-").Update((RAD2DEG(theta[0])*36)+default_pos[0])
            window.Element("-JOINT2-").Update((RAD2DEG(theta[1])*50)+default_pos[1])
            window.Element("-JOINT3-").Update((RAD2DEG(theta[2])*52)+default_pos[2])
            window.Element("-JOINT4-").Update((RAD2DEG(theta[3])*40)+default_pos[3])
            window.Element("-JOINT5-").Update((RAD2DEG(theta[4])*40)+default_pos[4])
            window.Element("-JOINT6-").Update((RAD2DEG(theta[5])*40)+default_pos[5])

            # send_cmd('J1', 'servo', (RAD2DEG(theta[0])*36)+default_pos[0])
            # send_cmd('J2', 'servo', (RAD2DEG(theta[1])*50)+default_pos[1])
            # send_cmd('J3', 'servo', (RAD2DEG(theta[2])*52)+default_pos[2])
            # send_cmd('J4', 'servo', (RAD2DEG(theta[3])*40)+default_pos[3])
            # send_cmd('J5', 'servo', (RAD2DEG(theta[4])*40)+default_pos[4])
            # send_cmd('J6', 'servo', (RAD2DEG(theta[5])*40)+default_pos[5])
        
        elif not success:
            print("Inverse kinematic fail")
            window.Element("-PITCH-").Update(val)

    if event == "+Y":

        val = values["-YAW-"]
        val_up = val+1
        window.Element("-YAW-").Update(val_up)
        val_up_rad = convert_radian_from_npi_to_pi(DEG2RAD(val_up))
        T_sd = get_X_d(x_current,y_current,z_current,R_current, P_current, val_up_rad)

        thetalist = thetalist_current
        # thetalist = np.array([DEG2RAD(values["-JOINT1-"]/36),DEG2RAD(values["-JOINT2-"]/50), DEG2RAD(values["-JOINT3-"]/52), DEG2RAD(values["-JOINT4-"]/40), DEG2RAD(values["-JOINT5-"]/40), DEG2RAD(values["-JOINT6-"]/40)])
        
        # print(T_sd)

        theta, success = IK(thetalist, T_sd)
        theta = convert_radian_from_npi_to_pi(theta)
    
        if success is True:
            Xnow, val_x, val_y, val_z, val_roll, val_pitch, val_yaw = FK(theta)

            val_roll = RAD2DEG(val_roll)
            val_pitch = RAD2DEG(val_pitch)
            val_yaw = RAD2DEG(val_yaw)

            window.Element("-X-").Update(val_x)
            window.Element("-Y-").Update(val_y)
            window.Element("-ROLL-").Update(val_roll)
            window.Element("-PITCH-").Update(val_pitch)
            
            window.Element("-JOINT1-").Update((RAD2DEG(theta[0])*36)+default_pos[0])
            window.Element("-JOINT2-").Update((RAD2DEG(theta[1])*50)+default_pos[1])
            window.Element("-JOINT3-").Update((RAD2DEG(theta[2])*52)+default_pos[2])
            window.Element("-JOINT4-").Update((RAD2DEG(theta[3])*40)+default_pos[3])
            window.Element("-JOINT5-").Update((RAD2DEG(theta[4])*40)+default_pos[4])
            window.Element("-JOINT6-").Update((RAD2DEG(theta[5])*40)+default_pos[5])

            # send_cmd('J1', 'servo', (RAD2DEG(theta[0])*36)+default_pos[0])
            # send_cmd('J2', 'servo', (RAD2DEG(theta[1])*50)+default_pos[1])
            # send_cmd('J3', 'servo', (RAD2DEG(theta[2])*52)+default_pos[2])
            # send_cmd('J4', 'servo', (RAD2DEG(theta[3])*40)+default_pos[3])
            # send_cmd('J5', 'servo', (RAD2DEG(theta[4])*40)+default_pos[4])
            # send_cmd('J6', 'servo', (RAD2DEG(theta[5])*40)+default_pos[5])
        
        elif not success:
            print("Inverse kinematic fail")
            window.Element("-YAW-").Update(val)

    if event == "-Y":

        val = values["-YAW-"]
        val_up = val-1
        window.Element("-YAW-").Update(val_up)
        val_up_rad = convert_radian_from_npi_to_pi(DEG2RAD(val_up))
        T_sd = get_X_d(x_current,y_current,z_current,R_current, P_current, val_up_rad)

        thetalist = thetalist_current
        # thetalist = np.array([DEG2RAD(values["-JOINT1-"]/36),DEG2RAD(values["-JOINT2-"]/50), DEG2RAD(values["-JOINT3-"]/52), DEG2RAD(values["-JOINT4-"]/40), DEG2RAD(values["-JOINT5-"]/40), DEG2RAD(values["-JOINT6-"]/40)])

        theta, success = IK(thetalist, T_sd)
        theta = convert_radian_from_npi_to_pi(theta)
    
        if success is True:
            Xnow, val_x, val_y, val_z, val_roll, val_pitch, val_yaw = FK(theta)

            val_roll = RAD2DEG(val_roll)
            val_pitch = RAD2DEG(val_pitch)
            val_yaw = RAD2DEG(val_yaw)

            window.Element("-X-").Update(val_x)
            window.Element("-Y-").Update(val_y)
            window.Element("-ROLL-").Update(val_roll)
            window.Element("-PITCH-").Update((val_pitch))
            
            window.Element("-JOINT1-").Update((RAD2DEG(theta[0])*36)+default_pos[0])
            window.Element("-JOINT2-").Update((RAD2DEG(theta[1])*50)+default_pos[1])
            window.Element("-JOINT3-").Update((RAD2DEG(theta[2])*52)+default_pos[2])
            window.Element("-JOINT4-").Update((RAD2DEG(theta[3])*40)+default_pos[3])
            window.Element("-JOINT5-").Update((RAD2DEG(theta[4])*40)+default_pos[4])
            window.Element("-JOINT6-").Update((RAD2DEG(theta[5])*40)+default_pos[5])

            # send_cmd('J1', 'servo', (RAD2DEG(theta[0])*36)+default_pos[0])
            # send_cmd('J2', 'servo', (RAD2DEG(theta[1])*50)+default_pos[1])
            # send_cmd('J3', 'servo', (RAD2DEG(theta[2])*52)+default_pos[2])
            # send_cmd('J4', 'servo', (RAD2DEG(theta[3])*40)+default_pos[3])
            # send_cmd('J5', 'servo', (RAD2DEG(theta[4])*40)+default_pos[4])
            # send_cmd('J6', 'servo', (RAD2DEG(theta[5])*40)+default_pos[5])

        elif not success:
            print("Inverse kinematic fail")
            window.Element("-YAW-").Update(val)

    if event == "Clear Via Point":
        v_data = []
        v_data_display = []
        window.Element("-TABLE_VIA-").Update(v_data)
        via_cnt = 0
        print("Clear via point")

    if event =="Add Via Point":
    
        print("Add Via Point")
        if via_cnt == 0:   
            pos_data = [x_current, y_current, z_current, R_current, P_current, Y_current]
            pos_data_display = np.array([values["-X-"], values["-Y-"], values["-Z-"], values["-ROLL-"], values["-PITCH-"], values["-YAW-"]])

            thetalist = np.array([(values["-JOINT1-"]-default_pos[0])/36, (values["-JOINT2-"]-default_pos[1])/50, (values["-JOINT3-"]-default_pos[2])/52, (values["-JOINT4-"]-default_pos[3])/40, (values["-JOINT5-"]-default_pos[4])/40, (values["-JOINT6-"]-default_pos[5])/40])
            theta_data = [thetalist[0], thetalist[1], thetalist[2], thetalist[3], thetalist[4], thetalist[5]]
            via_cnt += 1

            v_point_display = [via_cnt,  np.round(theta_data), np.round([pos_data_display[0], pos_data_display[1], pos_data_display[2], pos_data_display[3], pos_data_display[4], pos_data_display[5]])]
            v_point_real = [via_cnt, thetalist_current, pos_data]
            v_data_display.append(v_point_display)
            v_data.append(v_point_real)
            window.Element("-TABLE_VIA-").Update(values=v_data_display)
            
            print("##################################################################################")
            print(via_cnt)
            print(v_data)
            print("##################################################################################")
        
        elif via_cnt > 0 :
            pos_data = [x_current, y_current, z_current, R_current, P_current, Y_current]
            pos_data_display = np.array([values["-X-"], values["-Y-"], values["-Z-"], values["-ROLL-"], values["-PITCH-"], values["-YAW-"]])
            
            thetalist = np.array([(values["-JOINT1-"]-default_pos[0])/36, (values["-JOINT2-"]-default_pos[1])/50, (values["-JOINT3-"]-default_pos[2])/52, (values["-JOINT4-"]-default_pos[3])/40, (values["-JOINT5-"]-default_pos[4])/40, (values["-JOINT6-"]-default_pos[5])/40])
            theta_data = [thetalist[0], thetalist[1], thetalist[2], thetalist[3], thetalist[4], thetalist[5]]
            via_cnt += 1

            v_point_display = [via_cnt,  np.round(theta_data), np.round([pos_data_display[0], pos_data_display[1], pos_data_display[2], pos_data_display[3], pos_data_display[4], pos_data_display[5]])]
            v_point_real = [via_cnt, thetalist_current, pos_data]
            v_data_display.append(v_point_display)
            v_data.append(v_point_real)
            window.Element("-TABLE_VIA-").Update(values=v_data_display)
            
            print("##################################################################################")
            print(via_cnt)
            print(v_data)
            print("##################################################################################")

    if event =="Play Via Point":
        if via_cnt == 0:
            print("No Via point")
        
        elif via_cnt > 0:
            via_cnt = 0
            traj_cnt = 0
            theta_cnt = 0
            # theta_npi = []

            via_T, via_thetalist = get_via_T(v_data)

            t, traj = Trapezoidal_Traj_Gen_Given_Amax_and_T(1.5, 2, 0.01)

            print("Play Via Point")
            print("##################################################################################")
            print(T_current)
            print("----------------------------------------------------------------------------------")
            for j in range(len(via_T)):
                print(via_T[j])
                print("----------------------------------------------------------------------------------")
                # print(via_thetalist[j][0])
            print("##################################################################################")

            j_count_max = len(t)

            for i in range (len(via_thetalist)):
                # print(traj_cnt)
                # print("??????????????????????????????????????????????????????????????????????????")
                if traj_cnt == 0:
                    j_count = 0
                    ecd_data = call_ecd_data()

                    th1_traj = Path_Gen(thetalist_current[0], via_thetalist[0][0][0], traj[:,0])
                    th2_traj = Path_Gen(thetalist_current[1], via_thetalist[0][0][1], traj[:,0])
                    th3_traj = Path_Gen(thetalist_current[2], via_thetalist[0][0][2], traj[:,0])
                    th4_traj = Path_Gen(thetalist_current[3], via_thetalist[0][0][3], traj[:,0])
                    th5_traj = Path_Gen(thetalist_current[4], via_thetalist[0][0][4], traj[:,0])
                    th6_traj = Path_Gen(thetalist_current[5], via_thetalist[0][0][5], traj[:,0])
                    # th7_traj = Path_Gen(thetalist_current[2], via_thetalist[0][0][5], traj[:,0])
                    traj_cnt += 1
                    gr = np.array([36, 50, 52, 40, 40, 40])
                    # gr = np.array([36, 50, 52])
                    target_rad = np.array([th1_traj[-1], th2_traj[-1], th3_traj[-1], th4_traj[-1], th5_traj[-1], th6_traj[-1]])
                    target_deg = RAD2DEG(target_rad)
                    target_deg = (target_deg * gr)+default_pos
                    print(np.round(target_deg))
                    while check_ecd_data(ecd_data, np.round(target_deg)) is not True:

                        send_cmd('J1', 'servo', (RAD2DEG(th1_traj[j_count])*36)+default_pos[0])
                        send_cmd('J2', 'servo', (RAD2DEG(th2_traj[j_count])*50)+default_pos[1])
                        send_cmd('J3', 'servo', (RAD2DEG(th3_traj[j_count])*52)+default_pos[2])
                        send_cmd('J4', 'servo', (RAD2DEG(th4_traj[j_count])*40)+default_pos[3])
                        send_cmd('J5', 'servo', (RAD2DEG(th5_traj[j_count])*40)+default_pos[4])
                        send_cmd('J6', 'servo', (RAD2DEG(th6_traj[j_count])*40)+default_pos[5])
                        ecd_data = selected_ser_class.get_ecd_value()

                        time.sleep(0.01)

                        j_count += 1
                        if j_count >= j_count_max:
                            j_count = j_count_max-1

                    Xnow, val_x, val_y, val_z, val_roll, val_pitch, val_yaw = FK(target_rad)

                    val_roll = RAD2DEG(val_roll)
                    val_pitch = RAD2DEG(val_pitch)
                    val_yaw = RAD2DEG(val_yaw)

                    window.Element("-X-").Update(val_x)
                    window.Element("-Y-").Update(val_y)
                    window.Element("-Z-").Update(val_z)
                    window.Element("-ROLL-").Update(val_roll)
                    window.Element("-PITCH-").Update(val_pitch)
                    window.Element("-YAW-").Update(val_yaw)

                    window.Element("-JOINT1-").Update((RAD2DEG(th1_traj[j_count])*36)+default_pos[0])
                    window.Element("-JOINT2-").Update((RAD2DEG(th2_traj[j_count])*50)+default_pos[1])
                    window.Element("-JOINT3-").Update((RAD2DEG(th3_traj[j_count])*52)+default_pos[2])
                    window.Element("-JOINT4-").Update((RAD2DEG(th4_traj[j_count])*40)+default_pos[3])
                    window.Element("-JOINT5-").Update((RAD2DEG(th5_traj[j_count])*40)+default_pos[4])
                    window.Element("-JOINT6-").Update((RAD2DEG(th6_traj[j_count])*40)+default_pos[5])

                    print("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!")

                elif traj_cnt > 0:
                    j_count = 0
                    ecd_data = call_ecd_data()

                    th1_traj = Path_Gen(via_thetalist[i-1][0][0], via_thetalist[i][0][0], traj[:,0])
                    th2_traj = Path_Gen(via_thetalist[i-1][0][1], via_thetalist[i][0][1], traj[:,0])
                    th3_traj = Path_Gen(via_thetalist[i-1][0][2], via_thetalist[i][0][2], traj[:,0])
                    th4_traj = Path_Gen(via_thetalist[i-1][0][3], via_thetalist[i][0][3], traj[:,0])
                    th5_traj = Path_Gen(via_thetalist[i-1][0][4], via_thetalist[i][0][4], traj[:,0])
                    th6_traj = Path_Gen(via_thetalist[i-1][0][5], via_thetalist[i][0][5], traj[:,0])
                    traj_cnt += 1
                    via_cnt += 1
                    gr = np.array([36, 50, 52, 40, 40, 40])
                    target_rad = np.array([th1_traj[-1], th2_traj[-1], th3_traj[-1], th4_traj[-1], th5_traj[-1], th6_traj[-1]])
                    target_deg = RAD2DEG(target_rad)
                    target_deg = (target_deg * gr)+default_pos
                    print(np.round(target_deg))
                    
                    while check_ecd_data(ecd_data, np.round(target_deg)) is not True:

                        send_cmd('J1', 'servo', (RAD2DEG(th1_traj[j_count])*36)+default_pos[0])
                        send_cmd('J2', 'servo', (RAD2DEG(th2_traj[j_count])*50)+default_pos[1])
                        send_cmd('J3', 'servo', (RAD2DEG(th3_traj[j_count])*52)+default_pos[2])
                        send_cmd('J4', 'servo', (RAD2DEG(th4_traj[j_count])*40)+default_pos[3])
                        send_cmd('J5', 'servo', (RAD2DEG(th5_traj[j_count])*40)+default_pos[4])
                        send_cmd('J6', 'servo', (RAD2DEG(th6_traj[j_count])*40)+default_pos[5])
                        ecd_data = selected_ser_class.get_ecd_value()

                        time.sleep(0.01)

                        j_count += 1
                        if j_count >= j_count_max:
                            j_count = j_count_max-1

                    Xnow, val_x, val_y, val_z, val_roll, val_pitch, val_yaw = FK(target_rad)

                    val_roll = RAD2DEG(val_roll)
                    val_pitch = RAD2DEG(val_pitch)
                    val_yaw = RAD2DEG(val_yaw)

                    window.Element("-X-").Update(val_x)
                    window.Element("-Y-").Update(val_y)
                    window.Element("-Z-").Update(val_z)
                    window.Element("-ROLL-").Update(val_roll)
                    window.Element("-PITCH-").Update(val_pitch)
                    window.Element("-YAW-").Update(val_yaw)

                    window.Element("-JOINT1-").Update((RAD2DEG(th1_traj[j_count])*36)+default_pos[0])
                    window.Element("-JOINT2-").Update((RAD2DEG(th2_traj[j_count])*50)+default_pos[1])
                    window.Element("-JOINT3-").Update((RAD2DEG(th3_traj[j_count])*52)+default_pos[2])
                    window.Element("-JOINT4-").Update((RAD2DEG(th4_traj[j_count])*40)+default_pos[3])
                    window.Element("-JOINT5-").Update((RAD2DEG(th5_traj[j_count])*40)+default_pos[4])
                    window.Element("-JOINT6-").Update((RAD2DEG(th6_traj[j_count])*40)+default_pos[5])

                    print("~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~")

                print(traj_cnt)
        traj_cnt = 0

        print("Done")
        print(T_current)
        print("##################################################################################")


    
    ## Home posision switch ##
    if event == "Set Current as Home":
        if not home_flag:
            joint_default = np.array([-130,-104,-78,-86,-288,-201]) #-104

            J1_range = (joint_default[0]-(180*36), joint_default[0]+(180*36))
            J2_range = (joint_default[1]-(90*50), joint_default[1]+(90*50))
            J3_range = (joint_default[2]-(90*52), joint_default[2]+(90*52))
            J4_range = (joint_default[3]-(90*40), joint_default[3]+(90*40))
            J5_range = (joint_default[4]-(90*40), joint_default[4]+(90*40))
            J6_range = (joint_default[5]-(90*40), joint_default[5]+(90*40))
        
        elif home_flag is True:
            # joint_default = np.array([-130,-130,-78,-86,-288,-201]) #-104
            default_pos = np.array([values["-JOINT1-"], values["-JOINT2-"], values["-JOINT3-"], values["-JOINT4-"], values["-JOINT5-"], values["-JOINT6-"]])
            joint_default = default_pos

            J1_range = (joint_default[0]-(180*36), joint_default[0]+(180*36))
            J2_range = (joint_default[1]-(90*50), joint_default[1]+(90*50))
            J3_range = (joint_default[2]-(90*52), joint_default[2]+(90*52))
            J4_range = (joint_default[3]-(90*40), joint_default[3]+(90*40))
            J5_range = (joint_default[4]-(90*40), joint_default[4]+(90*40))
            J6_range = (joint_default[5]-(90*40), joint_default[5]+(90*40))
            
        window.Element("-JOINT1-").Update(range=J1_range)
        window.Element("-JOINT2-").Update(range=J2_range)
        window.Element("-JOINT3-").Update(range=J3_range)
        window.Element("-JOINT4-").Update(range=J4_range)
        window.Element("-JOINT5-").Update(range=J5_range)
        window.Element("-JOINT6-").Update(range=J6_range)

        window.Element("-JOINT1_MIN-").Update(J1_range[0])
        window.Element("-JOINT2_MIN-").Update(J2_range[0])
        window.Element("-JOINT3_MIN-").Update(J3_range[0])
        window.Element("-JOINT4_MIN-").Update(J4_range[0])
        window.Element("-JOINT5_MIN-").Update(J5_range[0])
        window.Element("-JOINT6_MIN-").Update(J6_range[0])
        
        window.Element("-JOINT1_MAX-").Update(J1_range[1])
        window.Element("-JOINT2_MAX-").Update(J2_range[1])
        window.Element("-JOINT3_MAX-").Update(J3_range[1])
        window.Element("-JOINT4_MAX-").Update(J4_range[1])
        window.Element("-JOINT5_MAX-").Update(J5_range[1])
        window.Element("-JOINT6_MAX-").Update(J6_range[1])

        home_flag = True
        print("Set Home")
        print(home_flag)
        print(joint_home)
        print("##################################################################################")
       
    
    if event =="Go Home":
        
        if not home_flag:
            print("Home position does not exit")

        elif home_flag is True:

            print("Go Home")

            joint_home = np.array([0., 0., 0., 0., 0., 0.])
            t, traj = Trapezoidal_Traj_Gen_Given_Amax_and_T(1.5, 2, 0.01)
            th1_traj = Path_Gen(thetalist_current[0], joint_home[0], traj[:,0])
            th2_traj = Path_Gen(thetalist_current[1], joint_home[1], traj[:,0])
            th3_traj = Path_Gen(thetalist_current[2], joint_home[2], traj[:,0])
            th4_traj = Path_Gen(thetalist_current[3], joint_home[3], traj[:,0])
            th5_traj = Path_Gen(thetalist_current[4], joint_home[4], traj[:,0])
            th6_traj = Path_Gen(thetalist_current[5], joint_home[5], traj[:,0])

            for i in range(len(t)):
                theta = np.array([th1_traj[i], th2_traj[i], th3_traj[i], th4_traj[i], th5_traj[i], th6_traj[i]])
                Xnow, val_x, val_y, val_z, val_roll, val_pitch, val_yaw = FK(theta)

                val_roll = RAD2DEG(val_roll)
                val_pitch = RAD2DEG(val_pitch)
                val_yaw = RAD2DEG(val_yaw)

                window.Element("-X-").Update(val_x)
                window.Element("-Y-").Update(val_y)
                window.Element("-Z-").Update(val_z)
                window.Element("-ROLL-").Update(val_roll)
                window.Element("-PITCH-").Update(val_pitch)
                window.Element("-YAW-").Update(val_yaw)
                
                window.Element("-JOINT1-").Update((RAD2DEG(theta[0])*36)+default_pos[0])
                window.Element("-JOINT2-").Update((RAD2DEG(theta[1])*50)+default_pos[1])
                window.Element("-JOINT3-").Update((RAD2DEG(theta[2])*52)+default_pos[2])
                window.Element("-JOINT4-").Update((RAD2DEG(theta[3])*40)+default_pos[3])
                window.Element("-JOINT5-").Update((RAD2DEG(theta[4])*40)+default_pos[4])
                window.Element("-JOINT6-").Update((RAD2DEG(theta[5])*40)+default_pos[5])

                send_cmd('J1', 'servo', (RAD2DEG(theta[0])*36)+default_pos[0])
                send_cmd('J2', 'servo', (RAD2DEG(theta[1])*50)+default_pos[1])
                send_cmd('J3', 'servo', (RAD2DEG(theta[2])*52)+default_pos[2])
                send_cmd('J4', 'servo', (RAD2DEG(theta[3])*40)+default_pos[3])
                send_cmd('J5', 'servo', (RAD2DEG(theta[4])*40)+default_pos[4])
                send_cmd('J6', 'servo', (RAD2DEG(theta[5])*40)+default_pos[5])

                time.sleep(0.01)

