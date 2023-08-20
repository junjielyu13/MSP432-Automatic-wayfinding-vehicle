import os
from dotenv import load_dotenv
import serial
import tkinter as tk
from tkinter import *
from tkinter import ttk
from tkinter import messagebox
import threading
import queue
import time
import serial.tools.list_ports
import numpy as np
import math
import subprocess
from multiprocessing.connection import Client
from timeit import default_timer as getTime
import Pmw
import platform
import sys


root = tk.Tk()
Pmw.initialise(root)
plataforma = platform.system()
print("Platform: ", plataforma)
Comando_python = sys.executable
print("Python command: ", Comando_python)


load_dotenv()
if plataforma == "Linux":
    Default_port_com = '/dev/ttyACM0'
elif plataforma == "Darwin":
    Default_port_com = '/dev/cu.usbmodemM43210051'
else:  # Windows
    Default_port_com = os.getenv('PORT')

INITIAL_POS_X = int(os.getenv('INITIAL_POS_X'))
INITIAL_POS_Y = int(os.getenv('INITIAL_POS_Y'))
INITIAL_POS_THETA = math.pi / 2
MAX_SIM_STEPS = 100000
SIM_STEP_MS_TIME = 100

Comando_plot = os.getenv('PLOT_FILE')
fichero_habitacion = os.getenv('ROOM_FILE')
OUTPUT_FILE_NAME = os.getenv('LOG_FILE')

fichero_log = open(OUTPUT_FILE_NAME, "w")
DEMO = 0
delay_Simul = 0.001
delay_Puerto = 0.001
DELTA_T = SIM_STEP_MS_TIME / 1000.0
CNTS_2_MM = 1000.0 * DELTA_T / 1023
L_AXIS = 1.0
t_last_upd = 0.0
ORIENT_L = 1
ORIENT_R = -1
V_inicial_demo_L = 0
V_inicial_demo_R = 0

DEBUG_trama_check = IntVar()
DEBUG_Moduls_check = IntVar()
DEBUG_Consola_check = IntVar()
SIMUL_check = IntVar()
SIMUL_Save_check = IntVar()
DEBUG_trama_check.set(0)
DEBUG_Moduls_check.set(0)
DEBUG_Consola_check.set(0)
SIMUL_Save_check.set(0)
DEBUG_trama = DEBUG_trama_check.get()
DEBUG_Moduls = DEBUG_Moduls_check.get()
DEBUG_Consola = DEBUG_Consola_check.get()
SIMUL_On_Off = SIMUL_check.get()
SIMUL_Save = SIMUL_Save_check.get()
texto_trama = StringVar()
texto_motor_left = StringVar()
texto_motor_right = StringVar()
valor_barra_izq = IntVar()
valor_barra_der = IntVar()
valor_barra_cent = IntVar()
Led_motor_left = IntVar()
Led_motor_right = IntVar()
estados = {0: "OFF", 1: "ON "}

port_com = Default_port_com
baud_rate = 115200
baud_rates = [9600, 19200, 38400, 57600, 115200,
              230400, 460800, 500000, 921600, 1000000]
timeout = 0.002
lectura = 0
seguir = 1
ser = serial.Serial()

INSTR_IDLE = 0x00
INSTR_END = 0xFF
INSTR_STOP_THREAD = 0xEE
INSTR_STOP_SIMUL = 0xCC
INSTR_SIMUL_ENDED = 0xDD

INSTR_PING = 0x01
INSTR_READ = 0x02
INSTR_WRITE = 0x03
INSTR_REG_WR = 0x04
INSTR_ACTION = 0x05
INSTR_RESET = 0x06
INSTR_SYNC_WRT = 0x83

DYN_REG__CW_ANGLE_LIMIT_L = 0x06
DYN_REG__CW_ANGLE_LIMIT_H = 0x07
DYN_REG__CCW_ANGLE_LIMIT_L = 0x08
DYN_REG__CCW_ANGLE_LIMIT_H = 0x09
DYN_REG__LED = 0x19,
DYN_REG__IR_LEFT = 0x1A
DYN_REG__IR_CENTER = 0x1B
DYN_REG__IR_RIGHT = 0x1C
DYN_REG__OBS_DETECT_VALUE = 0x14
DYN_REG__OBS_DETECT_FLAG = 0x20
DYN_REG__GOAL_SPEED_L = 0x20
DYN_REG__GOAL_SPEED_H = 0x21
DYN_REG__MOV_SPEED_L = 0x20
DYN_REG__MOV_SPEED_H = 0x21
DYN_REG__LEFT_IR_SENSOR = 0x1A
DYN_REG__CENTER_IR_SENSOR = 0x1B
DYN_REG__RIGHT_IR_SENSOR = 0x1C

MOTOR_ID_L = 0x01
MOTOR_ID_R = 0x02
SENSOR_ID = 0x03
BROADCAST_ID = 0xFE
lista_ID_Modulos = [MOTOR_ID_L, MOTOR_ID_R, SENSOR_ID, BROADCAST_ID]

CCW = 0x00
CW = 0x04
AX12_moving_L = ""
AX12_moving_R = ""
ESTAT_ROBOT = ""
n_moduls = 3
ID = 0
trama = b''

simulacio = INSTR_IDLE
simulando = 0
actualizar_graf = 0

instruction_set = {
    0x00: "IDLE",
    0x01: "PING",
    0x02: "READ",
    0x03: "WRITE",
    0x04: "REG_WRITE",
    0x05: "ACTION",
    0x06: "RESET",
    0x83: "SYNC_WRITE",
    0xFF: "END"
}

AX12_memory = {
    0x00: "Model Number(L)",
    0x01: "Model Number(H)",
    0x02: "Firmware Version",
    0x03: "ID",
    0x04: "Baud Rate",
    0x05: "Return Delay Time",
    0x06: "CW Angle Limit(L)",
    0x07: "CW Angle Limit(H)",
    0x08: "CCW Angle Limit(L)",
    0x09: "CCW Angle Limit(H)",
    0x0A: "Reserved",
    0x0B: "High Temp. Limit",
    0x0C: "Low Voltage Limit",
    0x0D: "High Voltage Limit",
    0x0E: "Max Torque(L)",
    0x0F: "Max Torque(H)",
    0x10: "Status Return Level",
    0x11: "Alarm LED",
    0x12: "Alarm Shoutdown",
    0x13: "Reserved",
    0x14: "Down Calibration(L)",
    0x15: "Down Calibration(H)",
    0x16: "Up Calibration(L)",
    0x17: "Up Calibration(H)",
    0x18: "Torque Enable",
    0x19: "LED",
    0x1A: "Left IR",
    0x1B: "Center IR",
    0x1C: "Right IR",
    0x1D: "CCW Compliance Slope",
    0x1E: "Goal Position(L)",
    0x1F: "Goal Position(H)",
    0x20: "Moving Speed(L)",
    0x21: "Moving Speed(H)",
    0x22: "Torque Limit(L)",
    0x23: "Torque Limit(H)",
    0x24: "Present Position(L)",
    0x25: "Present Position(H)",
    0x26: "Present Speed(L)",
    0x27: "Present Speed(H)",
    0x28: "Present Load(L)",
    0x29: "Present Load(H)",
    0x2A: "Present Voltage",
    0x2B: "Present Temperature",
    0x2C: "Registered Instruction",
    0x2D: "ADC_value",
    0x2E: "Moving",
    0x2F: "Lock",
    0x30: "Punch(L)",
    0x31: "Punch(H)"
}


def puertos_serie():
    lista_puertos = []
    puertos = serial.tools.list_ports.comports()
    for puerto in puertos:
        lista_puertos.append(str(puerto[0]))
    lista_puertos.append('None')
    lista_puertos.sort()
    print(lista_puertos)
    return lista_puertos


def refrescar_puertos(cb_lista: ttk.Combobox):
    global port_com
    lista_puertos = puertos_serie()
    cb_lista.config(values=lista_puertos)
    indice = 0
    for puerto in lista_puertos:
        if puerto == Default_port_com:
            cb_lista.current(indice)
        indice += 1
    port_com = cb_lista.get()


def f_led(ID):
    if ID == MOTOR_ID_L:
        Led_motor_left.set(AX12[ID - 1][0x19])
    elif ID == MOTOR_ID_R:
        Led_motor_right.set(AX12[ID - 1][0x19])
    else:
        if DEBUG_Consola == 1:
            print("El modul amb ID ", ID, " no poseix cap LED")
        return
    if DEBUG_Consola == 1:
        if AX12[ID - 1][0x19] == 1:
            print("LED motor", ID, "ON")
        else:
            print("LED motor", ID, "OFF")


def f_moving_speed(ID):
    global AX12_moving_L
    global AX12_moving_R
    if ID != MOTOR_ID_L and ID != MOTOR_ID_R:
        if DEBUG_Consola == 1:
            print("El modul amb ID ", ID, " no es un motor")
        return
    velocitat = AX12[ID - 1][0x20] + (AX12[ID - 1][0x21] & 0x03) * 256
    if DEBUG_Consola == 1:
        print("Velocitat  motor", ID, "=", velocitat)
    text_motor = "v=" + str(velocitat)
    sentit = AX12[ID - 1][0x21] & 0x04
    if sentit == 0:
        if DEBUG_Consola == 1:
            print("Sentit gir motor", ID, "= CCW")
        if ID == 1:
            AX12_moving_L = text_motor + " CCW"
        else:
            AX12_moving_R = text_motor + " CCW"
    else:
        if DEBUG_Consola == 1:
            print("Sentit gir motor", ID, "= CW")
        if ID == 1:
            AX12_moving_L = text_motor + " CW"
        else:
            AX12_moving_R = text_motor + " CW"
    return


def f_angle_limit(ID):
    if ID != MOTOR_ID_L and ID != MOTOR_ID_R:
        if DEBUG_Consola == 1:
            print("El modul amb ID ", ID, " no es un motor")
        return
    cw_angle = AX12[ID - 1][0x06] + (AX12[ID - 1][0x07] & 0x03) * 256
    ccw_angle = AX12[ID - 1][0x08] + (AX12[ID - 1][0x09] & 0x03) * 256
    if DEBUG_Consola == 1:
        if cw_angle == 0:
            print("Motor", ID, "gir continu en sentit horari")
        else:
            print("Motor", ID, "angle limit en sentit horari:", cw_angle)
        if ccw_angle == 0:
            print("Motor", ID, "gir continu en sentit anti-horari")
        else:
            print("Motor", ID, "angle limit en sentit anti-horari:", ccw_angle)
    return


def AX12_func(argument, id_modulo):
    if id_modulo not in lista_ID_Modulos:
        return
    switcher = {
        # 0x00: "Model Number(L)",
        # 0x01: "Model NUmber(H)",
        # 0x02: "Firmware Version",
        # 0x03:  "ID",
        # 0x04: "Baud Rate",
        # 0x05: "Return Delay Time",
        0x06: f_angle_limit,
        0x07: f_angle_limit,
        0x08: f_angle_limit,
        0x09: f_angle_limit,
        # 0x0A: "Reserved",
        # 0x0B: "High Temp. Limit",
        # 0x0C: "Low Voltage Limit",
        # 0x0D: "High Voltage Limit",
        # 0x0E: "Max Torque(L)",
        # 0x0F: "Max Torque(H)",
        # 0x10: "Status Return Level",
        # 0x11: "Alarm LED",
        # 0x12: "Alarm Shoutdown",
        # 0x13: "Reserved",
        # 0x14: "Down Calibration(L)",
        # 0x15: "Down Calibration(H)",
        # 0x16: "Up Calibration(L)",
        # 0x17: "Up Calibration(H)",
        # 0x18: "Torque Enable",
        0x19: f_led,
        # 0x1A: f_Left_IR,
        # 0x1B: f_Center_IR,
        # 0x1C: f_Right_IR,
        # 0x1D: "CCW Compliance Slope",
        # 0x1E: "Goal Position(L)",
        # 0x1F: "Goal Position(H)",
        0x20: f_moving_speed,
        0x21: f_moving_speed,
        # 0x22: "Torque Limit(L)",
        # 0x23: "Torque Limit(H)",
        # 0x24: "Present Position(L)",
        # 0x25: "Present Position(H)",
        # 0x26: "Present Speed(L)",
        # 0x27: "Present Speed(H)",
        # 0x28: "Present Load(L)",
        # 0x29: "Present Load(H)",
        # 0x2A: "Present Voltage",
        # 0x2B: "Present Temperature",
        # 0x2C: "Registered Instruction",
        # 0x2E: "Moving",
        # 0x2F: "Lock",
        # 0x30: "Punch(L)",
        # 0x31: "Punch(H)"
    }
    cadena_error = "Funció " + format(argument, '#04x') + " no implementada"
    func = switcher.get(argument, lambda id_modulo: print(cadena_error))
    func(id_modulo)


def reset_modul_AX12(id_modul):
    global AX12
    AX12[id_modul - 1][0x00] = 0x0C  # Model Number(L)
    AX12[id_modul - 1][0x02] = 0x01  # Model Number(H)
    AX12[id_modul - 1][0x03] = id_modul  # ID
    AX12[id_modul - 1][0x04] = 0x01  # Baud Rate
    AX12[id_modul - 1][0x05] = 0xFA  # Return Delay Time
    AX12[id_modul - 1][0x06] = 0x00  # CW Angle Limit(L)
    AX12[id_modul - 1][0x07] = 0x00  # CW Angle Limit(H)
    AX12[id_modul - 1][0x08] = 0xFF  # CCW Angle Limit(L)
    AX12[id_modul - 1][0x09] = 0x03  # CW Angle Limit(H)
    AX12[id_modul - 1][0x0B] = 0x55  # Highest Limit Temperature
    AX12[id_modul - 1][0x0C] = 0x3C  # Lowest Limit Voltage
    AX12[id_modul - 1][0x0D] = 0xBE  # Highest Limit Voltage
    AX12[id_modul - 1][0x0E] = 0xFF  # Max Torque(L)
    AX12[id_modul - 1][0x0F] = 0x03  # Max Torque(H)
    AX12[id_modul - 1][0x10] = 0x02  # Status Return Level
    AX12[id_modul - 1][0x11] = 0x04  # Alarm LED
    AX12[id_modul - 1][0x12] = 0x04  # Alarm Shutdown
    # For AX_S1: Obstacle Detected Compare Value (default = 32)
    AX12[id_modul - 1][0x14] = 0x20
    AX12[id_modul - 1][0x19] = 0x00  # Led Shutdown
    AX12[id_modul - 1][0x1A] = 0x00  # Left IR
    AX12[id_modul - 1][0x1B] = 0x00  # Center IR
    AX12[id_modul - 1][0x1C] = 0x00  # Left IR
    AX12[id_modul - 1][0x1D] = 0x00  # Valor inicial ADC para la Demo
    # Velocidad inicial (LO) nula. For AX_S1: Obstacle Detection flag
    AX12[id_modul - 1][0x20] = 0x00
    AX12[id_modul - 1][0x21] = 0x00  # Velocidad inicial (HI) nula, sentido 0
    AX12[id_modul - 1][0x22] = AX12[id_modul - 1][0x0E]  # Torque Limit(L)
    AX12[id_modul - 1][0x23] = AX12[id_modul - 1][0x0F]  # Torque Limit(H)
    return


def comprova_checksum(frame):
    len_trama = len(frame)
    chk_sum = 0
    for index in range(2, (len_trama - 1)):
        chk_sum = chk_sum + frame[index]
    chk_sum = chk_sum & 0xFF
    if (chk_sum | frame[len_trama - 1]) == 0xFF:
        if DEBUG_trama:
            print('Checksum correcte')
        return 0x00
    else:
        print('Error de Checksum')
        return 0x10


def comprova_instr(instruccio):
    if (instruccio < 0x07) or (instruccio == 0xFF) or (instruccio == INSTR_SYNC_WRT):
        if DEBUG_Consola == 1:
            print("Instrucció:", instruction_set[instruccio])
        return 0x00
    else:
        print("Error d'instruccio")
        return 0x70


def send_status_packet(modul_id, error_code):
    status_frame = Status_NoError[:]
    status_frame[2] = modul_id
    status_frame[4] = error_code
    len_trama = len(status_frame)
    l_chksum = 0
    for index in range(2, (len_trama - 1)):
        l_chksum = l_chksum + status_frame[index]
    l_chksum = (~l_chksum & 0xFF)
    status_frame[len_trama - 1] = l_chksum
    string = ''.join(['0x%02X ' % b for b in status_frame])
    if DEBUG_trama:
        print("status packet in hex:", string)
        print("status packet in dec:", status_frame)
    ser.write(status_frame)
    return


def generate_read_packet(modul_id, address, num_param):
    global simulando
    status_frame = Status_NoError[:]
    status_frame[2] = modul_id
    status_frame[3] = num_param + 2
    for index in range(0, num_param):
        status_frame.insert(index + 5, AX12[modul_id - 1][address + index])
    len_trama = len(status_frame)
    l_chksum = 0
    for index in range(2, (len_trama - 1)):
        l_chksum = l_chksum + status_frame[index]
    l_chksum = (~l_chksum & 0xFF)
    status_frame[len_trama - 1] = l_chksum
    string = ''.join(['0x%02X ' % b for b in status_frame])
    if DEBUG_trama:
        print("status packet in hex:", string)
        print("status packet in dec:", status_frame)
    ser.write(status_frame)
    return


def generate_status_packet(id_modul, instruc, code_error, trama):
    if instruc != 2:
        send_status_packet(id_modul, code_error)
    elif instruc == INSTR_READ:
        address = trama[5]
        num_param = trama[6]
        generate_read_packet(id_modul, address, num_param)
    return


def robot_status():
    global ESTAT_ROBOT
    if DEBUG_Consola == 1:
        print("----------- ESTAT DEL ROBOT -----------------------")
    v_left = AX12[MOTOR_ID_L - 1][0x20] + \
        (AX12[MOTOR_ID_L - 1][0x21] & 0x03) * 256
    sentit_left = AX12[MOTOR_ID_L - 1][0x21] & 0x04
    v_right = AX12[MOTOR_ID_R - 1][0x20] + \
        (AX12[MOTOR_ID_R - 1][0x21] & 0x03) * 256
    sentit_right = AX12[MOTOR_ID_R - 1][0x21] & 0x04
    if (v_left == 0) and (v_right == 0):
        if DEBUG_Consola == 1:
            print("Robot Parat")
        ESTAT_ROBOT = "Robot Parat"
    elif sentit_left == sentit_right:
        if sentit_left == CW:
            if DEBUG_Consola == 1:
                print("Robot Gira Esquerra")
            ESTAT_ROBOT = "Robot Gira Esquerra"
        else:
            if DEBUG_Consola == 1:
                print("Robot Gira Dreta")
            ESTAT_ROBOT = "Robot Gira Dreta"
    elif abs(v_left - v_right) < 1:
        if sentit_left == CW:
            if DEBUG_Consola == 1:
                print("Robot Marxa Enrere")
            ESTAT_ROBOT = "Robot Marxa Enrere"
        else:
            if DEBUG_Consola == 1:
                print("Robot Marxa Endavant")
            ESTAT_ROBOT = "Robot Marxa Endavant"
    elif v_left > v_right:
        if sentit_left == CW:
            if DEBUG_Consola == 1:
                print("Robot Gira Esquerra")
            ESTAT_ROBOT = "Robot Gira Esquerra"
        else:
            if DEBUG_Consola == 1:
                print("Robot Gira Dreta")
            ESTAT_ROBOT = "Robot Gira Dreta"
    elif sentit_left == CW:
        if DEBUG_Consola == 1:
            print("Robot Gira Dreta")
        ESTAT_ROBOT = "Robot Gira Dreta"
    else:
        if DEBUG_Consola == 1:
            print("Robot Gira Esquerra")
        ESTAT_ROBOT = "Robot Gira Esquerra"
    return


def print_AX_MemoryMap():
    print("-----------------------------------------------------------------")
    print("= @ =|===== FUNCIO ===== MODUL => [", MOTOR_ID_L,
          "] ====== [", MOTOR_ID_R, "] ====== [", SENSOR_ID, "]")
    for i in range(50):
        mot1 = ''.join(['0x%02X ' % (AX12[MOTOR_ID_L - 1][i])])
        mot2 = ''.join(['0x%02X ' % (AX12[MOTOR_ID_R - 1][i])])
        sens = ''.join(['0x%02X ' % (AX12[SENSOR_ID - 1][i])])
        print('0x{:02X} | {:-<24}'.format(i,
              AX12_memory[i]), ">", mot1, "      ", mot2, "      ", sens)
    print("#################################################################")
    return


def Actualitza_AX_Memory(id_modul, adressa, nparametres, values):
    for index in range(nparametres):
        AX12[id_modul - 1][adressa + index] = values[index]
    return


def print_trama():
    print(" ")
    print("####################################################")
    print("Received Instruction Frame:", trama)
    for index in range(len(trama)):
        print("Received Instruction Frame[",
              index, "]:", '0x%02X ' % trama[index])
    print("####################################################")
    return


def print_tipus_Instruccio(instruccio, comandament):
    error_de_instr = comprova_instr(instruccio)
    if DEBUG_Consola == 1:
        if error_de_instr == 0:
            if instruccio == INSTR_ACTION:
                comandament = 0x2C
            print("----------- INSTRUCCIÓ i COMANDAMENT --------------")
            print("Command:", AX12_memory[comandament])
    return error_de_instr


AX12 = [[0] * 50 for i in range(n_moduls)]
reset_modul_AX12(MOTOR_ID_L)
reset_modul_AX12(MOTOR_ID_R)
reset_modul_AX12(SENSOR_ID)
Status_NoError = [0xff, 0xff, 0x01, 0x02, 0x00, 0xfc]
instruccio = INSTR_IDLE

if DEBUG_Moduls:
    print_AX_MemoryMap()


class Hilo(threading.Thread):
    def __init__(self, nombre, cola, cola_hilo):
        threading.Thread.__init__(self)
        self.cola = cola
        self.cola_hilo = cola_hilo

    def leer_puerto(self, la_cola, la_cola_hilo):
        global instruccio
        global trama
        global ID
        global AX12_moving_L
        global AX12_moving_R
        global lectura
        Lista_acciones = []
        AX12_moving_L = "PARAT"
        AX12_moving_R = "PARAT"
        print("Lectura puerto iniciada...", instruccio)
        while instruccio != INSTR_END:
            if not la_cola.empty():
                mensaje = la_cola.get()
                if mensaje == INSTR_END:
                    instruccio = mensaje
                    if DEBUG_Consola == 1:
                        print("instruccio rebuda: ", instruccio)
                if DEBUG_Consola == 1:
                    print("mensaje recibido en leer_puerto: ", mensaje)
            elif lectura == 1:
                trama = b''
                if ser.is_open and ser.in_waiting >= 4:
                    inicio_trama = ser.read(4)
                    if inicio_trama[0] == 0xFF and inicio_trama[1] == 0xFF:
                        resto_trama = ser.read(inicio_trama[3])
                        trama = b''.join([inicio_trama, resto_trama])
                    else:
                        trama = b''
                if trama != b'':
                    items_array = len(trama)
                    if DEBUG_Consola == 1:
                        print("Número de items en el array:", items_array)
                    if items_array >= 6:
                        if DEBUG_Consola == 1:
                            print("")
                            print(
                                "****************************************************")
                        instruccio = trama[4]

                        if DEBUG_trama == 1:
                            print_trama()

                        instr_error = print_tipus_Instruccio(
                            instruccio, trama[5])
                        chk_sum_error = comprova_checksum(
                            trama[0:trama[3] + 4])
                        error = (chk_sum_error | instr_error)
                        ID = trama[2]
                        if ID != BROADCAST_ID:
                            generate_status_packet(
                                ID, instruccio, error, trama)
                        else:
                            if DEBUG_Consola == 1:
                                print("Broadcasting ID Instruction Packet")
                        if ID not in lista_ID_Modulos:
                            print("Error: la ID ", format(
                                ID, '#04x'), " no es vàlida")
                        elif error == 0:
                            if instruccio == INSTR_WRITE:
                                if ID == BROADCAST_ID:
                                    print("Instrucció ", format(
                                        instruccio, '#04x'), " incompatible amb ID ", format(ID, '#04x'))
                                else:
                                    n_parametres = trama[3] - 3
                                    address = trama[5]
                                    values = trama[6:]
                                    Actualitza_AX_Memory(
                                        ID, address, n_parametres, values)
                                    AX12_func(address, ID)
                                    if DEBUG_Moduls:
                                        print_AX_MemoryMap()
                                    robot_status()
                                    texto_trama.set(ESTAT_ROBOT)
                                    texto_motor_left.set(AX12_moving_L)
                                    texto_motor_right.set(AX12_moving_R)

                            elif instruccio == INSTR_REG_WR:
                                if ID == BROADCAST_ID:
                                    print("Instrucció ", format(
                                        instruccio, '#04x'), " incompatible amb ID Broadcast ", format(ID, '#04x'))
                                else:
                                    ID = trama[2]
                                    direccion = trama[5]
                                    num_param = trama[3] - 3
                                    parametros = []
                                    for param in range(num_param):
                                        parametros.append(trama[6 + param])
                                    Lista_acciones.append(
                                        [ID, direccion, num_param, parametros])
                                    if DEBUG_Consola == 1:
                                        print(Lista_acciones)
                            elif instruccio == INSTR_ACTION:
                                if not Lista_acciones:
                                    print("No hay acciones pendientes...")
                                else:
                                    for index in range(len(Lista_acciones)):
                                        if DEBUG_Consola == 1:
                                            print("Accion pendiente:",
                                                  Lista_acciones[index])
                                        ID = Lista_acciones[index][0]
                                        direccion = Lista_acciones[index][1]
                                        num_param = Lista_acciones[index][2]
                                        for param in range(num_param):
                                            parametro = Lista_acciones[index][3][param]
                                            AX12[ID - 1][direccion +
                                                         param] = parametro
                                        AX12_func(direccion, ID)
                                    robot_status()
                                    texto_trama.set(ESTAT_ROBOT)
                                    texto_motor_left.set(AX12_moving_L)
                                    texto_motor_right.set(AX12_moving_R)
                                    if DEBUG_Moduls:
                                        print_AX_MemoryMap()
                                    Lista_acciones = []
                            elif instruccio == INSTR_SYNC_WRT:
                                if ID != BROADCAST_ID:
                                    print("Instrucció", format(instruccio, '#04x'), "incompatible amb ID", format(ID, '#04x'),
                                          ", ID Broadcast", format(BROADCAST_ID, '#04x'), "requerida.")
                                    continue
                                total_length = trama[3]
                                address = trama[5]
                                length = trama[6]
                                n_modules = (total_length - 4)/(length + 1)
                                if n_modules != int(n_modules):
                                    print(
                                        "Error en la longitud total del paquet. La longitud rebuda ha estat ", total_length)
                                    continue
                                for i in range(int(n_modules)):
                                    id = trama[7+(length+1)*i]
                                    values = trama[8+(length+1)
                                                   * i: 8+(length+1)*(i+1)]
                                    Actualitza_AX_Memory(
                                        id, address, length, values)
                                    AX12_func(address, id)
                                if DEBUG_Moduls:
                                    print_AX_MemoryMap()
                                robot_status()

                                texto_trama.set(ESTAT_ROBOT)
                                texto_motor_left.set(AX12_moving_L)
                                texto_motor_right.set(AX12_moving_R)
                        else:
                            Lista_acciones = []
                            ser.reset_input_buffer()

        la_cola_hilo.put(INSTR_STOP_THREAD)
        if DEBUG_Consola == 1:
            print("lectura parada")
        lectura = 0

    def run(self):
        self.leer_puerto(self.cola, self.cola_hilo)
        if DEBUG_Consola == 1:
            print("funcion run terminada")


WORLD__N_BYTES = 4
WORLD__N_BITS = 32
WORLD__MAX_2POW = 5


class _habitacion_t:
    def __init__(self, fichero_habitacion):
        print("Datos habitacion:", fichero_habitacion)
        parametros = np.genfromtxt(fichero_habitacion, dtype="int",
                                   delimiter=',', skip_header=2,
                                   max_rows=1, deletechars="\n")

        print(parametros)

        self.ancho = parametros.data[0]
        self.alto = parametros.data[1]
        print("Ancho = ", self.ancho, " Alto = ", self.alto)

        parametros = np.genfromtxt(fichero_habitacion, delimiter=',', dtype="int",
                                   skip_header=4, max_rows=1, deletechars="\n")

        num_obstaculos = parametros * 1
        print("Hay", num_obstaculos, "obstaculos.")

        self.obstaculos = np.genfromtxt(fichero_habitacion, dtype="int",
                                        delimiter=',', skip_header=6,
                                        max_rows=num_obstaculos)

        self.x0s = self.obstaculos[0:, 0]
        self.y0s = self.obstaculos[0:, 1]
        self.anchos = self.obstaculos[0:, 2]
        self.altos = self.obstaculos[0:, 3]
        print("Obstaculos:")
        print("n: \tx \ty \tancho \talto")
        for i in range(0, num_obstaculos - 1):
            print(i, ":", "\t", self.x0s[i], "\t", self.y0s[i],
                  "\t", self.anchos[i], "\t", self.altos[i])
        X_LEN = (self.ancho >> WORLD__MAX_2POW)
        self.datos = np.genfromtxt(fichero_habitacion, delimiter=',', dtype="int",
                                   skip_header=13 + num_obstaculos, skip_footer=1, deletechars="\n",
                                   usecols=np.arange(0, X_LEN))


habitacion = _habitacion_t(fichero_habitacion)
X_LEN = (habitacion.ancho >> WORLD__MAX_2POW)


class _robot_pos_t:
    def __init__(self, INITIAL_POS_X, INITIAL_POS_Y, theta, mundo):
        self.x = INITIAL_POS_X
        self.y = INITIAL_POS_Y
        self.l_axis = L_AXIS
        self.theta = theta
        self.iv_l = 0
        self.iv_r = 0
        self.v_l = 0.0
        self.v_r = 0.0
        self.r = 0.0
        self.w = 0.0
        self.icc_x = 0.0
        self.icc_y = 0.0
        self.x_p = 0.0
        self.x_p = self.x
        self.y_p = 0.0
        self.y_p = self.y
        self.sim_step = 0
        self.world = mundo


robot_pos_str = _robot_pos_t(
    INITIAL_POS_X, INITIAL_POS_Y, INITIAL_POS_THETA, habitacion.datos)
print("Robot en (", robot_pos_str.x, ",",
      robot_pos_str.y, "), angulo ", robot_pos_str.theta)


def obstaculo(x, y, robot_pos: _robot_pos_t):
    mundo = robot_pos.world
    p_offset = x >> WORLD__MAX_2POW
    p_bit = (WORLD__N_BITS - 1) - (x - (p_offset << WORLD__MAX_2POW))

    if mundo[y, p_offset] & (1 << p_bit):
        return True
    return False


def sensor_distance(x0, y0, theta, robot_pos: _robot_pos_t):
    modulo = 0.0
    x = 0.0
    y = 0.0
    indice = 0
    u8_mod = 0
    dx = math.cos(theta)
    dy = math.sin(theta)

    while (modulo < 255) and not (obstaculo(round(x0 + x), round(y0 + y), robot_pos)):
        x += dx
        y += dy
        modulo = math.sqrt(x * x + y * y)
        indice += 1

    if modulo > 255:
        u8_mod = 255
    else:
        u8_mod = round(modulo)

    return u8_mod


def distance(robot_pos: _robot_pos_t):
    x0 = robot_pos.x
    y0 = robot_pos.y
    theta = robot_pos.theta
    theta_r = theta - math.pi / 2
    theta_l = theta + math.pi / 2

    centro = sensor_distance(x0, y0, theta, robot_pos)
    izq = sensor_distance(x0, y0, theta_l, robot_pos)
    der = sensor_distance(x0, y0, theta_r, robot_pos)

    return izq, centro, der


def elapsed_time(t1, milliseconds):
    t2 = time.time()
    true_elapsed_time = (t2 - t1) * 1000
    if true_elapsed_time > milliseconds:
        return True, true_elapsed_time
    else:
        return False, true_elapsed_time


def check_colision(robot_pos: _robot_pos_t):
    if obstaculo(robot_pos.x, robot_pos.y, robot_pos):
        print("***** COLLISION DETECTED AT", robot_pos_str.x, robot_pos_str.y, "simulator step",
              robot_pos_str.sim_step, "\n")
        return True
    return False


def check_out_of_bounds(ANCHO, ALTO):
    if robot_pos_str.x > ANCHO - 1 or robot_pos_str.y > ALTO - 1 or robot_pos_str.x < 0 or robot_pos_str.y < 0:
        print("***** WARNING, LEAVING ROOM... \n")
        return True
    return False


def check_simulation_end():
    if MAX_SIM_STEPS != 0 and robot_pos_str.sim_step >= MAX_SIM_STEPS:
        print("***** SIMULATION END REACHED. STOPPING SIMULATOR\n")
        return True
    return False


def end_simulator():
    return


def _speed_dyn_2_speed_int(motor_id):
    v = AX12[motor_id - 1][DYN_REG__GOAL_SPEED_L]
    v |= ((AX12[motor_id - 1][DYN_REG__GOAL_SPEED_H] & 0x03) << 8)
    if AX12[motor_id - 1][DYN_REG__GOAL_SPEED_H] & 0x04:
        v *= -1
    return v


def read_speed(robot_pos_str: _robot_pos_t):
    robot_pos_str.iv_l = _speed_dyn_2_speed_int(MOTOR_ID_L) * ORIENT_L
    robot_pos_str.iv_r = _speed_dyn_2_speed_int(MOTOR_ID_R) * ORIENT_R
    return


def calculate_new_position(robot_pos_str: _robot_pos_t):
    read_speed(robot_pos_str)
    robot_pos_str.v_l = CNTS_2_MM * robot_pos_str.iv_l
    robot_pos_str.v_r = CNTS_2_MM * robot_pos_str.iv_r

    if robot_pos_str.iv_l == robot_pos_str.iv_r:
        robot_pos_str.x_p += robot_pos_str.v_l * \
            DELTA_T * math.cos(robot_pos_str.theta)
        robot_pos_str.y_p += robot_pos_str.v_r * \
            DELTA_T * math.sin(robot_pos_str.theta)
    else:
        robot_pos_str.r = (robot_pos_str.l_axis / 2) * (robot_pos_str.v_l + robot_pos_str.v_r) \
            / (robot_pos_str.v_r - robot_pos_str.v_l)
        robot_pos_str.w = (robot_pos_str.v_r -
                           robot_pos_str.v_l) / robot_pos_str.l_axis
        robot_pos_str.icc_x = robot_pos_str.x_p \
            - robot_pos_str.r * math.sin(robot_pos_str.theta)
        robot_pos_str.icc_y = robot_pos_str.y_p \
            + robot_pos_str.r * math.cos(robot_pos_str.theta)
        robot_pos_str.x_p = math.cos(robot_pos_str.w * DELTA_T) * (robot_pos_str.x_p - robot_pos_str.icc_x) \
            - math.sin(robot_pos_str.w * DELTA_T) * (robot_pos_str.y_p - robot_pos_str.icc_y) \
            + robot_pos_str.icc_x
        robot_pos_str.y_p = math.sin(robot_pos_str.w * DELTA_T) * (robot_pos_str.x_p - robot_pos_str.icc_x) \
            + math.cos(robot_pos_str.w * DELTA_T) * (robot_pos_str.y_p - robot_pos_str.icc_y) \
            + robot_pos_str.icc_y

        robot_pos_str.theta += robot_pos_str.w * DELTA_T
        if robot_pos_str.theta < -math.pi:
            robot_pos_str.theta += 2 * math.pi
        elif robot_pos_str.theta > math.pi:
            robot_pos_str.theta -= 2 * math.pi

    robot_pos_str.x = round(robot_pos_str.x_p)
    robot_pos_str.y = round(robot_pos_str.y_p)

    return


def update_sensor_data(robot_pos: _robot_pos_t):
    distancia_left, distancia_center, distancia_right = distance(robot_pos)
    AX12[SENSOR_ID - 1][DYN_REG__IR_LEFT] = distancia_left
    AX12[SENSOR_ID - 1][DYN_REG__CENTER_IR_SENSOR] = distancia_center
    AX12[SENSOR_ID - 1][DYN_REG__IR_RIGHT] = distancia_right

    if DYN_REG__OBS_DETECT_VALUE > 0:
        right_flag = (distancia_right > DYN_REG__OBS_DETECT_VALUE) << 2
        center_flag = (distancia_center > DYN_REG__OBS_DETECT_VALUE) << 1
        left_flag = (distancia_left > DYN_REG__OBS_DETECT_VALUE)
        AX12[SENSOR_ID - 1][DYN_REG__OBS_DETECT_FLAG] = right_flag | center_flag | left_flag

    return


def calcular_distancias_demo():
    distancia_left = AX12[SENSOR_ID - 1][DYN_REG__IR_LEFT]
    distancia_right = AX12[SENSOR_ID - 1][DYN_REG__IR_RIGHT]
    distancia_center = AX12[SENSOR_ID - 1][DYN_REG__CENTER_IR_SENSOR]
    creciendo = AX12[SENSOR_ID - 1][0x1D]

    if distancia_left < 1:
        distancia_left = 256
    distancia_left -= 1

    if distancia_right > 255:
        distancia_right = 0
    distancia_right += 1

    if creciendo:
        if distancia_center < 255:
            distancia_center += 1
        else:
            creciendo = 0
    else:
        if distancia_center > 0:
            distancia_center -= 1
        else:
            creciendo = 1
    AX12[SENSOR_ID - 1][0x1D] = creciendo
    AX12[SENSOR_ID - 1][DYN_REG__IR_LEFT] = distancia_left
    AX12[SENSOR_ID - 1][DYN_REG__CENTER_IR_SENSOR] = distancia_center
    AX12[SENSOR_ID - 1][DYN_REG__IR_RIGHT] = distancia_right


def update_movement_simulator_values():
    global t_last_upd, simulando, fichero_log
    objective_delay = SIM_STEP_MS_TIME
    elapsed, true_elapsed_time = elapsed_time(t_last_upd, objective_delay)
    if DEMO:
        calcular_distancias_demo()
        return False
    if elapsed:
        objective_delay -= (true_elapsed_time - SIM_STEP_MS_TIME)
        t_last_upd = time.time()
        robot_pos_str.sim_step += 1
        if MAX_SIM_STEPS != 0 and robot_pos_str.sim_step >= MAX_SIM_STEPS:
            print("***** SIMULATION END REACHED. STOPPING SIMULATOR\n")
            simulando = 0
            return False
        calculate_new_position(robot_pos_str)
        if not check_out_of_bounds(habitacion.ancho, habitacion.alto):
            update_sensor_data(robot_pos_str)
            if check_colision(robot_pos_str):
                return False
            conn.send("%.2f, %.2f\n" % (robot_pos_str.x_p, robot_pos_str.y_p))

            if SIMUL_Save:
                if fichero_log.closed:
                    fichero_log = open(OUTPUT_FILE_NAME, "a")
                fichero_log.write("%.2f, %.2f, %.3f, %.2f, %.2f\n" % (robot_pos_str.x_p, robot_pos_str.y_p,
                                                                      robot_pos_str.theta, robot_pos_str.v_l,
                                                                      robot_pos_str.v_r))
            return True

    return False


def reset_robot_pos(robot_pos: _robot_pos_t):
    robot_pos.x = INITIAL_POS_X
    robot_pos.y = INITIAL_POS_Y
    robot_pos.theta = INITIAL_POS_THETA
    robot_pos.iv_l = 0
    robot_pos.iv_r = 0
    robot_pos.v_l = 0.0
    robot_pos.v_r = 0.0
    robot_pos.r = 0.0
    robot_pos.w = 0.0
    robot_pos.icc_x = 0.0
    robot_pos.icc_y = 0.0
    robot_pos.x_p = 0.0
    robot_pos.x_p = robot_pos.x
    robot_pos.y_p = 0.0
    robot_pos.y_p = robot_pos.y
    robot_pos.sim_step = 0
    update_sensor_data(robot_pos)


class Simul(threading.Thread):
    def __init__(self, nombre, cola, cola_simul):
        threading.Thread.__init__(self)
        self.cola = cola
        self.cola_simul = cola_simul

    def movement(self, la_cola, la_cola_simul):
        global simulacio, simulando, actualizar_graf
        print("Hilo simulacion movimiento iniciado, simulación en pausa...")
        # Pondremos una velocidad inicial, para las pruebas y/o la demo:
        AX12[MOTOR_ID_L - 1][DYN_REG__GOAL_SPEED_L] = V_inicial_demo_L & 0xFF
        AX12[MOTOR_ID_L -
             1][DYN_REG__GOAL_SPEED_H] = (V_inicial_demo_L >> 8) & 0x07
        AX12[MOTOR_ID_R - 1][DYN_REG__GOAL_SPEED_L] = V_inicial_demo_R & 0xFF
        AX12[MOTOR_ID_R -
             1][DYN_REG__GOAL_SPEED_H] = (V_inicial_demo_R >> 8) & 0x07

        t_anterior = 0
        t0 = 0
        while simulacio != INSTR_STOP_SIMUL:
            t_anterior = t0
            t0 = getTime()
            t_iteracion = (t0 - t_anterior) * 1000  # en ms
            if t_iteracion > SIM_STEP_MS_TIME:
                print("Warning: simulation too slow ", t_iteracion,
                      " ms instead of ", SIM_STEP_MS_TIME, " ms!")
            if simulando == 1:
                if update_movement_simulator_values():
                    # Actualizamos las barras graficas:
                    valor_barra_izq.set(AX12[SENSOR_ID - 1][DYN_REG__IR_LEFT])
                    valor_barra_der.set(AX12[SENSOR_ID - 1][DYN_REG__IR_RIGHT])
                    valor_barra_cent.set(
                        AX12[SENSOR_ID - 1][DYN_REG__CENTER_IR_SENSOR])
            time.sleep(delay_Simul)

        if DEBUG_Consola == 1:
            print("simulacion parada")
        simulando = 0
        simulacio = INSTR_SIMUL_ENDED

    def run(self):
        self.movement(self.cola, self.cola_simul)
        if DEBUG_Consola == 1:
            print("funcion run terminada")


class Application(tk.Frame):
    counter = 0
    contador = 0
    puntos = []

    def __init__(self, master=None):
        super().__init__(master)
        self.master = master
        self.grid()
        self.create_widgets()
        self.set_tooltips()
        self.crear_hilo()
        self.crear_simul()
        self.check_queue()

    def create_widgets(self):
        global port_com
        global baud_rate
        self.spacer_up = Label(self, width=5, height=1)
        self.spacer_up.grid(row=0, column=0)
        self.spacer_center = Label(self, width=5, height=1)
        self.spacer_center.grid(row=20, column=0)
        self.spacer_bottom = Label(self, width=5, height=1)
        self.spacer_bottom.grid(row=40, column=10)

        self.Label_Debug = Label(self, text="DEBUG:")
        self.Label_Debug.grid(row=1, column=1, sticky=W)

        self.Debug_frame = Checkbutton(self, variable=DEBUG_trama_check)
        self.Debug_frame["text"] = "Debug Trames"
        self.Debug_frame["fg"] = "blue"
        self.Debug_frame["command"] = self.set_debug_frames
        self.Debug_frame.grid(row=1, column=2, sticky=W)

        self.Debug_module = Checkbutton(self, variable=DEBUG_Moduls_check)
        self.Debug_module["text"] = "Debug Moduls"
        self.Debug_module["fg"] = "blue"
        self.Debug_module["command"] = self.set_debug_moduls
        self.Debug_module.grid(row=1, column=3, sticky=W)

        self.Debug_consola = Checkbutton(self, variable=DEBUG_Consola_check)
        self.Debug_consola["text"] = "Verbose"
        self.Debug_consola["fg"] = "blue"
        self.Debug_consola["command"] = self.set_debug_consola
        self.Debug_consola.grid(row=1, column=4, sticky=E)

        self.Print_Memoria_AX = Button(
            self, text="Imprimir Memoria Mòduls", fg="blue", command=self.imprimir_AX_memory)
        self.Print_Memoria_AX.grid(row=2, column=2, columnspan=2, sticky=W)

        self.Label_Simul = Label(self, text="SIMULACIO:")
        self.Label_Simul.grid(row=3, column=1, sticky=W)

        try:
            self.play_btn_img = PhotoImage(file="img/resume_co.png")
            self.pause_btn_img = PhotoImage(file="img/suspend_co.png")
            self.Simul_On_Off = Checkbutton(self, text="Simul On/Off", fg="blue",
                                            image=self.play_btn_img, selectimage=self.pause_btn_img,
                                            compound='left', indicatoron=False,
                                            variable=SIMUL_check, command=self.set_Simul_On_Off)
        except tk.TclError:
            self.Simul_On_Off = Checkbutton(self, text="Simul On/Off", fg="blue",
                                            variable=SIMUL_check, command=self.set_Simul_On_Off)
        self.Simul_On_Off.grid(row=3, column=2, sticky=W)

        self.Simul_grabar = Checkbutton(
            self, fg="blue", variable=SIMUL_Save_check)
        self.Simul_grabar["text"] = "Desar"
        self.Simul_grabar["command"] = self.grabar_Simul_OnOff
        self.Simul_grabar.grid(row=3, column=3)

        try:
            self.reset_btn_img = PhotoImage(file="img/terminate_co.png")
            self.Simul_reset = Button(self, image=self.reset_btn_img, text="Reset",
                                      compound='left', fg="blue", command=self.reset_simul)
        except tk.TclError:
            self.Simul_reset = Button(
                self, text="Reset", compound='left', fg="blue", command=self.reset_simul)
        self.Simul_reset.grid(row=3, column=4, sticky=W)

        self.label_robot = Label(self, text="ESTAT ROBOT:")
        self.label_robot.grid(row=4, column=1, sticky=W)
        self.label_trama = Label(self, textvariable=texto_trama)
        texto_trama.set("ROBOT PARAT")
        self.label_trama.grid(row=4, column=2, columnspan=2, sticky=W)

        self.label_AX12_L = Label(self, text="MOTOR Esq.:")
        self.label_AX12_L.grid(row=5, column=1)
        self.label_motor_left = Label(self, textvariable=texto_motor_left)
        texto_motor_left.set("PARAT")
        self.label_motor_left.grid(row=5, column=2)
        self.label_AX12_R = Label(self, text="MOTOR Dret:")
        self.label_AX12_R.grid(row=6, column=1)
        self.label_motor_right = Label(self, textvariable=texto_motor_right)
        texto_motor_right.set("PARAT")
        self.label_motor_right.grid(row=6, column=2)

        self.led_motor_left = Radiobutton(self, text="Led Esq.", value=1,
                                          variable=Led_motor_left, state=DISABLED)
        self.led_motor_left.grid(row=5, column=3)
        self.led_motor_right = Radiobutton(self, text="Led Dret", value=1,
                                           variable=Led_motor_right, state=DISABLED)
        self.led_motor_right.grid(row=6, column=3)

        self.label_izq = Label(self, text="IR Esq.")
        self.label_izq.grid(row=7, column=1)
        self.progress_bar_izq = ttk.Progressbar(self, orient="horizontal",
                                                length=255, maximum=255,
                                                mode="determinate", variable=valor_barra_izq)
        self.progress_bar_izq["value"] = 0
        self.progress_bar_izq.place(
            relx=0.5, rely=0.5, relwidth=0.80, anchor=tk.CENTER)
        self.progress_bar_izq.grid(row=7, column=2, columnspan=3)
        self.label_valor_izq = Label(self, textvariable=valor_barra_izq)
        self.label_valor_izq.grid(row=7, column=5, sticky=W)

        self.label_der = Label(self, text="IR Dret")
        self.label_der.grid(row=8, column=1)
        self.progress_bar_der = ttk.Progressbar(self, orient="horizontal",
                                                length=255, maximum=255,
                                                mode="determinate", variable=valor_barra_der)
        self.progress_bar_der["value"] = 200
        self.progress_bar_der.place(
            relx=0.5, rely=0.5, relwidth=0.80, anchor=tk.CENTER)
        self.progress_bar_der.grid(row=8, column=2, columnspan=3)
        self.label_valor_der = Label(self, textvariable=valor_barra_der)
        self.label_valor_der.grid(row=8, column=5, sticky=W)

        self.label_centro = Label(self, text="IR Centre")
        self.label_centro.grid(row=9, column=1)
        self.progress_bar_centro = ttk.Progressbar(self, orient="horizontal",
                                                   length=255, maximum=255,
                                                   mode="determinate", variable=valor_barra_cent)
        self.progress_bar_centro["value"] = 200
        self.progress_bar_centro.place(
            relx=0.5, rely=0.5, relwidth=0.80, anchor=tk.CENTER)
        self.progress_bar_centro.grid(row=9, column=2, columnspan=3)
        self.label_valor_centro = Label(self, textvariable=valor_barra_cent)
        self.label_valor_centro.grid(row=9, column=5, sticky=W)

        self.quit = tk.Button(self, text="SORTIR",
                              fg="red", command=self.salir)
        self.quit.grid(row=21, column=4)

        self.cb = ttk.Combobox(self, values=lista_puertos, width=10)
        self.cb.current(0)
        self.cb.grid(row=21, column=1)
        self.cb.bind('<<ComboboxSelected>>', self.on_select)
        indice = 0
        for puerto in lista_puertos:
            if puerto == Default_port_com:
                self.cb.current(indice)
            indice += 1
        port_com = self.cb.get()
        print("Puerto seleccionado: ", port_com)

        self.cb_rate = ttk.Combobox(self, state="readonly", width=12,
                                    values=baud_rates)
        self.cb_rate.current(4)
        self.cb_rate.grid(row=21, column=2)
        self.cb_rate.bind('<<ComboboxSelected>>', self.on_select_rate)
        baud_rate = self.cb_rate.get()
        print("Velocidad: ", baud_rate)

        self.refrescar = tk.Button(
            self, command=lambda: refrescar_puertos(self.cb))
        self.refrescar["text"] = "Refrescar"
        self.refrescar["fg"] = "blue"
        self.refrescar.grid(row=21, column=3, sticky=W)

    def set_tooltips(self):
        self.tooltip_Debug_frame = Pmw.Balloon(root)
        self.tooltip_Debug_frame.bind(self.Debug_frame,
                                      'Activa/Desactiva impressió a consola de la trama rebuda \ni de la resposta enviada (Status Packet)')

        self.tooltip_Debug_module = Pmw.Balloon(root)
        self.tooltip_Debug_module.bind(self.Debug_module,
                                       'Activa/Desactiva impressió a consola del mapa de memòria dels mòduls \nAX12 (motors) i AX-S1 (sensors) al rebre una comanda vàlida')

        self.tooltip_Debug_consola = Pmw.Balloon(root)
        self.tooltip_Debug_consola.bind(self.Debug_consola,
                                        'Activa/Desactiva impressió a consola de \nmissatges addicional dels estats del robot')

        self.tooltip_Print_Memoria_AX = Pmw.Balloon(root)
        self.tooltip_Print_Memoria_AX.bind(self.Print_Memoria_AX,
                                           'Imprimir a la consola el mapa de memòria \ndels mòduls AX12 (motors) i AX-S1 (sensors)')

        self.tooltip_Simul_On_Off = Pmw.Balloon(root)
        self.tooltip_Simul_On_Off.bind(self.Simul_On_Off,
                                       'Iniciar/Pausar la simulació')

        self.tooltip_Simul_grabar = Pmw.Balloon(root)
        el_tip = "Activa/Desactiva l’emmagatzematge al fitxer \"" + \
            OUTPUT_FILE_NAME + "\" \nde les dades que es van simulant"
        self.tooltip_Simul_grabar.bind(self.Simul_grabar, el_tip)

        self.tooltip_Simul_reset = Pmw.Balloon(root)
        el_tip = "Reinicia tots els paràmetres de la simulació, tant l’estat del robot \n" \
                 "com la seva posició, així com el fitxer de dades. \n" \
                 "El robot s’atura, i es torna a ficar a la posició inicial definida per defecte, \n" \
                 "descartant tots els punts simulats fins ara. \n" \
                 "El fitxer de dades es buida."
        self.tooltip_Simul_reset.bind(self.Simul_reset, el_tip)

        self.tooltip_cb = Pmw.Balloon(root)
        el_tip = "Selecciona el port USB on es troba la UART de  l’MSP"
        self.tooltip_cb.bind(self.cb, el_tip)

        self.tooltip_cb_rate = Pmw.Balloon(root)
        el_tip = "Configura la velocitat de bits (baud rate) en bps, \nper adequar-se a la que s’hagi programat a l’MSP"
        self.tooltip_cb_rate.bind(self.cb_rate, el_tip)

        self.tooltip_refrescar = Pmw.Balloon(root)
        el_tip = "Torna a buscar els ports disponibles"
        self.tooltip_refrescar.bind(self.refrescar, el_tip)

        self.tooltip_quit = Pmw.Balloon(root)
        el_tip = "surt de l’emulador, tancant totes les finestres, i terminant tots els fils i processos. \n(Nota: la creueta de la part superior dreta de la finestra no surt de forma neta)"
        self.tooltip_quit.bind(self.quit, el_tip)

    def on_select(self, event=None):
        global lectura
        global ser
        global port_com
        lectura = 0
        seleccion = self.cb.get()
        port_com = seleccion
        print("Nuevo puerto seleccionado:", port_com)
        try:
            if ser.is_open:
                ser.close()
            ser.port = port_com
            ser.baudrate = baud_rate
            ser.timeout = timeout
            ser.open()
            lectura = 1
        except serial.SerialException as e:
            mensaje_error = "No se puede abrir el puerto\n\t" + port_com
            messagebox.showerror("Error COM", mensaje_error)
            print("No se puede abrir el puerto", port_com, ", error ", e)
            lectura = 0

    def on_select_rate(self, event=None):
        global baud_rate
        baud_rate = self.cb_rate.get()
        ser.baudrate = baud_rate
        print("Baud rate seleccionado: ", baud_rate, " bps")

    def say_hi(self):
        print("J. Bosch & C. Serre,")
        print("UB, 2020-2021.")
        print("versió", version)
        mensaje = "J. Bosch & C. Serre\nUB, 2020-2021.\nVer. " + version
        messagebox.showinfo("Autors", mensaje)

    def set_debug_frames(self):
        global DEBUG_trama
        DEBUG_trama = DEBUG_trama_check.get()

    def set_debug_moduls(self):
        global DEBUG_Moduls
        DEBUG_Moduls = DEBUG_Moduls_check.get()

    def set_debug_consola(self):
        global DEBUG_Consola
        DEBUG_Consola = DEBUG_Consola_check.get()

    def imprimir_AX_memory(self):
        print_AX_MemoryMap()

    def imprimir_IR(self):
        valores_sensores = AX12[SENSOR_ID -
                                1][DYN_REG__IR_LEFT:DYN_REG__IR_LEFT + 3]
        print("Sensors Esq., Centre, Dret: ", valores_sensores)

    def set_Simul_On_Off(self):
        global simulando, SIMUL_On_Off
        SIMUL_On_Off = SIMUL_check.get()
        simulando = SIMUL_On_Off
        print("Simulacion ", estados[SIMUL_On_Off])

    def grabar_Simul_OnOff(self):
        global SIMUL_Save, fichero_log
        if fichero_log.closed:
            fichero_log = open(OUTPUT_FILE_NAME, "w")
        SIMUL_Save = SIMUL_Save_check.get()
        return

    def crear_hilo(self):
        self.cola = queue.Queue()
        self.cola_hilo = queue.Queue()
        if DEBUG_Consola == 1:
            print(self.cola, self.cola_hilo)

        hilo = Hilo("puerto", self.cola, self.cola_hilo)
        hilo.start()

    def crear_simul(self):
        self.cola_simul = queue.Queue()
        if DEBUG_Consola == 1:
            print(self.cola, self.cola_simul)

        simul = Simul("Simul", self.cola, self.cola_simul)
        update_sensor_data(robot_pos_str)
        print("Robot @:", robot_pos_str.x, robot_pos_str.y)
        print("Distancia: Izq.=", AX12[SENSOR_ID - 1][DYN_REG__IR_LEFT],
              ", Centro =", AX12[SENSOR_ID - 1][DYN_REG__IR_CENTER],
              ", Der. =", AX12[SENSOR_ID - 1][DYN_REG__IR_RIGHT])

        valor_barra_izq.set(AX12[SENSOR_ID - 1][DYN_REG__IR_LEFT])
        valor_barra_der.set(AX12[SENSOR_ID - 1][DYN_REG__IR_RIGHT])
        valor_barra_cent.set(AX12[SENSOR_ID - 1][DYN_REG__CENTER_IR_SENSOR])

        simul.start()

    def reset_simul(self):
        global fichero_log
        fichero_log.close()
        print("Simulacio reiniciada")
        SIMUL_check.set(0)
        self.set_Simul_On_Off()

        reset_modul_AX12(MOTOR_ID_R)
        reset_modul_AX12(MOTOR_ID_L)
        reset_robot_pos(robot_pos_str)
        f_moving_speed(MOTOR_ID_R)
        f_moving_speed(MOTOR_ID_L)
        f_led(MOTOR_ID_R)
        f_led(MOTOR_ID_L)
        robot_status()
        texto_trama.set(ESTAT_ROBOT)
        texto_motor_left.set(AX12_moving_L)
        texto_motor_right.set(AX12_moving_R)

        valor_barra_izq.set(AX12[SENSOR_ID - 1][DYN_REG__IR_LEFT])
        valor_barra_der.set(AX12[SENSOR_ID - 1][DYN_REG__IR_RIGHT])
        valor_barra_cent.set(AX12[SENSOR_ID - 1][DYN_REG__CENTER_IR_SENSOR])
        conn.send('reset')
        conn.send("%.2f, %.2f\n" % (INITIAL_POS_X, INITIAL_POS_Y))
        return

    def check_queue(self):
        global instruccio, simulacio, simulando
        if not self.cola_hilo.empty():
            instruccio = self.cola_hilo.get()
            if DEBUG_Consola == 1:
                print("get text from read queue:", instruccio)
        if instruccio == INSTR_STOP_THREAD:
            simulando = 0
            simulacio = INSTR_STOP_SIMUL
            return INSTR_STOP_THREAD
        if not self.cola_simul.empty():
            mensaje = self.cola_simul.get()

        root.after(100, self.check_queue)

    def salir(self):
        self.cola.put(INSTR_END)
        if (instruccio == INSTR_STOP_THREAD) & (simulacio == INSTR_SIMUL_ENDED):
            conn.send('close')
            if DEBUG_Consola == 1:
                print("Hilos finalizados")
            self.master.destroy()
        else:
            root.after(200, self.salir)


lista_puertos = puertos_serie()
root.title("EMULADOR ROBOT PAE")
app = Application(master=root)

try:
    if ser.is_open:
        ser.close()
    ser.port = port_com
    ser.baudrate = baud_rate
    ser.timeout = timeout
    ser.open()
    lectura = 1
    print(ser.get_settings())
except serial.SerialException as e:
    mensaje = "No se puede abrir el puerto\n\t" + port_com
    messagebox.showerror("Error COM", mensaje)
    print("No se puede abrir el puerto", port_com, ", error ", e)

grafica = None
try:
    grafica = subprocess.Popen(
        [Comando_python, Comando_plot], stdin=subprocess.PIPE, text=True)
except OSError as e:
    print(e)
except TypeError:
    grafica = subprocess.Popen(
        [Comando_python, Comando_plot], stdin=subprocess.PIPE, universal_newlines=True)
try:
    if grafica is not None:
        grafica.communicate(
            "Ventana plot, creada desde el Emulador PAE!", 0.01)
except subprocess.TimeoutExpired:
    pass

address = ('localhost', 6000)

n_retries = 0
MAX_RETRIES = 20
connected = False
conn = None
while not connected:
    try:
        conn = Client(address, authkey=b'secret password')
        connected = True
    except ConnectionRefusedError:
        if n_retries > MAX_RETRIES:
            raise
        n_retries += 1
        time.sleep(0.1)


if conn is not None:
    conn.send("conexion mediante un socket!")

root.lift()
root.after(1, lambda: root.lift())
root.after(2, lambda: root.focus_force())


app.mainloop()

ser.close()
grafica.terminate()
conn.close()
