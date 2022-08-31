#%%
import tkinter as tk
from tkinter import *
import threading
import time
import serial
import math
import numpy as np
import matplotlib.pyplot as plt
from scipy.optimize import curve_fit
#%% Iniciar controlador serie

def iniciar():
    ser = serial.Serial(port='COM4', baudrate=115200, bytesize=8, parity='N', stopbits=1, timeout=0.005, xonxoff=0, rtscts=0) 
    ser.close() 
    ser.open()
    return ser

#reset controlador
def reset(ser):
    ser.write(bytes('X','utf-8')) 
    time.sleep(0.01)
    ser.flushInput()

#escribo voltaje, pregunto posicion y velocidad
def write_and_ask(ser):
    str = 'V0\n\r'
    ser.write(bytes(str,'utf-8'))
    time.sleep(0.002)
    s = ser.readline(25) #Lee 25 bytes y devuelve 25 bytes 
    print(s)
    
# Para iniciar el controlador ejecutar las siguientes lineas:
    
ser = iniciar() #Inicia el controlador.
reset(ser) # Luego lo resetea.

#%% 
"""   ------------------------ Funciones de Comunicación con el controlador ------------------------"""

def setVoltageGetData(puerto,voltaje):
    puerto.flushInput()
    str = 'V%f\n\r' % (voltaje)
    puerto.write(bytes(str,'utf-8'))
    time.sleep(0.002)
    s = puerto.readline(25)
    pos = float(s[0:9])
    vel = float(s[10:23])  
    return pos,vel

def setVoltage(puerto,voltaje):
    puerto.flushInput()
    str = 'V%f\n\r' % (voltaje)
    puerto.write(bytes(str,'utf-8'))
    time.sleep(0.002)
    s = puerto.readline(25)
    pos = float(s[0:9])
    vel = float(s[10:23])  
    return pos,vel

def GetData(puerto):
    time.sleep(0.001)
    s = puerto.readline(25)
    pos = float(s[0:9])
    vel = float(s[10:23])  
    return pos,vel

def resetControlador(puerto):
    puerto.write(bytes('X','utf-8')) 
    time.sleep(0.01)
    puerto.flushInput()

def voltajeCorregido(voltaje):
    voltpos = 2
    voltneg = 2 
    maxvolt = 12
    if(voltaje > 0 ):        
        voltaje *= maxvolt/(maxvolt+voltpos)
        voltaje += voltpos
    else:        
        voltaje *= maxvolt/(maxvolt+voltneg)
        voltaje -= voltneg
    return voltaje


  #%%
"""     Cálculo de la señal de control ( u(t) ) y  de los términos PID

        Proporcional = kp*e
        
        Integral = ki*integral(e)
        
        derivativo = kd*de/dt     """
        
def error(setpoint,variable):
    return (setpoint - variable)
        
def P(kp,error):
    return kp*error

def I(ki,error):
    return ki*error

def D(kd,error,error_prev,dt):
    return kd*(error - error_prev)/dt

def señal_control(P,I,D):
    return P+I+D  
#%%
"""   ------------------------    Controlador PID      ------------------------""" 

posicion = [] ; velocidad =[] ; tiempo = np.linspace(0,50,num=50)

kp=2 ; ki = 2 ; kd = 2 ; setpoint = 10 ; dt =0.1

#Primer paso
pos, vel = setVoltageGetData(ser, 0) #Le mando 0 volts al motor. Entonces pos,vel=0
posicion.append(pos)
velocidad.append(vel)

# Calculo error inicial. En este caso el error es +10.
e_0 = error(setpoint,pos) 

#Errores para en el termino integral.
e_int_0 = 0 
e_int = e_int_0 + e_0

#Errores para el termino derivativo.
e_prev_0 = 0 
e_prev = e_prev_0 + e_0  

u = señal_control(P(kp,e_0), 0 ,D(kd,e,e_0,dt))


for t in tiempo:
    pos, vel = setVoltageGetData(ser, m*u)
    posicion.append(pos)
    velocidad.append(vel)
    
    e = error(setpoint, pos)
    e_prev = e_prev + e
    e_int = e_int + e
    u = señal_control(P(kp,e),I(ki,e_int) , D(kd,e,e_prev,dt))
    
#%%
"""   ------------------------    Medición continua      ------------------------""" 

# #"Defino" el puerto serie. 
# ser = serial.Serial(port='COM4', baudrate=115200, bytesize=8, parity='N', stopbits=1, timeout=0.005, xonxoff=0, rtscts=0) 
# v = 0 #Voltaje inicial. 

# #Primer paso
# pos, vel = setVoltageGetData(ser, 0) #Le mando 0 volts al motor. Entonces pos,vel=0
# posicion.append(pos)
# velocidad.append(vel)

# Calculo error inicial. En este caso el error es +10..
pos = 0
e_0 = error(setpoint,pos) 

#Errores para en el termino integral.
e_int_0 = 0 
e_int = e_int_0 + e_0

#Errores para el termino derivativo.
e_prev_0 = 0 
e_prev = e_prev_0 + e_0  

class GUI(Frame):
    def __init__(self):
        self.CrearBoton()
        pass
    def CrearBoton(self):
        # self.boton = Tk()
        self.boton = Button(self, text="Parar",command=self.parar)
        self.label=Label(self.boton,text="")
        self.label.grid(row=0,column=1)
        self.boton.grid(row=1,column=0)
        
        hilolabel = threading.Thread(target = Medir)
        hilolabel.run()
            
    def parar():
        ser.close()
        self.boton.destroy()

def Medir():
    global v, e_int, e_prev
    print("hola")
    #pos, vel = setVoltageGetData(ser, v)
    # e = error(setpoint,pos)
    # e_int = e_int + e
    # e_prev = e_prev + e
    
    # u = señal_control(P(kp,e),I(ki,e_int),D(kd,e,e_prev,dt))
    # v = 2*u
    # Medir()
     
# def comenzar():
#     Bot.CrearBoton()

Bot = Tk()
Bot = GUI()
Bot.mainloop()

# hiloGUI = threading.Thread(target = comenzar)
# hiloGUI.run()

#%%
""" --------  --------"""
import random  as rnd
import tkinter as tk
from tkinter import ttk
import threading
import time
import serial
import math
import numpy as np
# 2 hilos y un while 

flag = True
alfa = 80

def corregir (e):
    e_prev = e_prev + e
    e_int = e_int + e
    u = señal_control(P(kp,e), I(ki,e_int), D(kd,e,e_prev,dt))
    setVoltage(ser, m*u)
    
def loop_medir():
    while flag:
        if error() > alfa:
            corregir()
            print("corregir")
            
        time.sleep(1)
    print("paró")
    sys.exit()
    
def error():
    pos, vel = GetData(ser)
    e = setpoint - pos
    print(e)
    return e
    
def parar():
    global flag
    flag = False
    
def crearboton():
    root = tk.Tk()
    root.config(width=300, height=200)
    root.title("Stopper PID")
    boton = ttk.Button(text="Parar",command = parar)
    boton.place(x=50, y=50)
    root.mainloop()
    

hilo = threading.Thread(target = loop_medir)
hilo.start()

crearboton()

#%%


