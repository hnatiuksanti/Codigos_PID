# -*- coding: utf-8 -*-
"""
Created on Wed May  5 01:56:43 2021
@author: damian
Comandos implementados en la placa:
V###.### ó v###.###:    establece la tensión de salida del controlador.
                        valor usado por el controlador para fijar la modulación por ancho de pulso: PWM = 0,5 (1+Vsalida/Vmaxima).
R####### ó R#######:    fija la resolución del encoder.
                        solo se usa para transformar los puloss a grados pero no esta implementado.
M###.### ó m###.###:    establece la máxima tesion soportada por el motor.
                        valor usado por el controlador para fijar la modulación por ancho de pulso: PWM = 0,5 (1+Vsalida/Vmaxima).
X ó x:                  reinicia la placa.
"""

import serial
import time 
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
"""------------------------ Respuesta a un pulso de voltaje y gráfico ------------------------"""

def respuesta_pulso(volt):
    #reseteo el controlador    
    resetControlador(ser)
    time.sleep(0.2)
    
    #inicializo variables. Todas con el mismo len y todas vacias, por eso el "np.nan"
    voltajes = np.concatenate((0*np.ones(10) ,volt*np.ones(85), 0*np.ones(10)))
    
    N = len(voltajes)
    posiciones = np.zeros(N)
    posiciones[:] = np.nan
    velocidades=np.zeros(N)
    velocidades[:] = np.nan
    tiempos=np.zeros(N)
    tiempos[:] =np.nan
    
    #loop poniendo voltaje y midiendo posicion y velocidad
    toc = time.time()
    for i in range(N):  
        time.sleep(.04)
        pos,vel = setVoltageGetData(ser,voltajes[i])
        time.sleep(.04)
        pos,vel = setVoltageGetData(ser,voltajes[i])
        
        posiciones[i] = pos
        velocidades[i] = vel
        tiempos[i] = time.time()-toc
    
    
    return [tiempos,voltajes,posiciones,velocidades]

lista_variables = respuesta_pulso(5) # Mando un pulso de 5 volts, y guardo en una lista 
                                     # las variables que luego voy a graficar.
#plot de la salida
plt.close('all')
fig, axs = plt.subplots(3, sharex=True)
axs[0].plot(lista_variables[0], lista_variables[1],'.-')
axs[0].set(ylabel = 'Voltaje')

axs[1].plot(lista_variables[0], lista_variables[2],'.-')
axs[1].set(ylabel = 'Posición')

v2 = np.diff(lista_variables[2]) / np.diff(lista_variables[0]) /4000
axs[2].plot(lista_variables[0][:-1], v2,'.-')
axs[2].plot(lista_variables[0], lista_variables[3],'.-')
axs[2].set(ylabel = 'Velocidad')

plt.legend(('Medida por la PC','Pedida por la placa'))
plt.xlabel('Tiempo [s]')

#%%
"""------------------------ Respuesta a muchos pulsos de voltaje ------------------------"""

# Calcula el promedio de la volcidad para cada pulso de voltaje distinto. 
def muchos_pulsos(voltajes):
    vmedios=[]
    
    for voltaje in voltajes: 
        resultados = respuesta_pulso(voltaje)
        vel = resultados[3][30:80]
        prom = sum(vel)/len(vel)
        vmedios.append(prom)
    
    return vmedios

voltajes = np.linspace(-24,24,num=48,endpoint=False,retstep=False, dtype=None )
vmedios = muchos_pulsos(voltajes)
plt.plot(vmedios, voltajes,'.-')

#%%
"""------------------------ Caracterización del motor ------------------------"""
#Quiero calcular la pendiente entre la velocidad y el voltaje. 
"VAMOS HACER UN AJUSTE LINEAL!!!!"

def fit_func(x, m, b):
    return m*x + b

#Seleccionamos un conjunto de puntos donde el gráfico se ve lineal. 
v = voltajes[5:20]
ve = vmedios[5:20]

params = curve_fit(fit_func, v, ve)
[m,b] = params[0]

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
"""   ------------------------    Controlador P      ------------------------"""

"""Posicion Vs Tiempo para distintos valores de Kp"""

posicion = [] ; velocidad =[] ; kp_lista=["Poner varlores de Kp"] ; tiempo = np.linspace(0,50,num=50)
setpoint = 10 
i=0 #Variable que voy a usar para crear gráficos.
for kp in kp_lista:
    #Primer paso
    pos, vel = setVoltageGetData(ser, 0) #Le mando 0 volts al motor. Entonces pos,vel=0
    posicion.append(pos)
    velocidad.append(vel)
    
    e_0 = error(setpoint,pos) #Error Inicial.
    u = señal_control(P(kp,e_0), 0, 0)
        
    for t in tiempo:
        pos, vel = setVoltageGetData(ser, m*u)
        posicion.append(pos)
        velocidad.append(vel)
        
        e = error(setpoint, pos)
        u = señal_control(P(kp,e), 0, 0)
    
    plt.plot(tiempo, posicion,'--', label = f"Kp={kp}")
    i=i+1
    time.sleep(0.02)

plt.ylabel('Posición', size=15)
plt.xlabel('Tiempo', size=15)
plt.grid()
plt.axhline(y=setpoint, linestyle=':', c='k', label='Setpoint')
plt.legend(loc=1)
plt.show()



#%%
"""------------------------    Controlador PI      ------------------------"""

#Hace lo mismo que control_P solo que le agrega el termino integral
"""Posicion Vs Tiempo para distintos valores de Ki"""

posicion = [] ; velocidad =[] ; ki_lista=["Poner varlores de Ki"]; tiempo = np.linspace(0,50,num=50)

kp=2 ; setpoint = 10 ; Ti= 1
i=0
for ki in ki_lista:
    #Primer paso
    pos, vel = setVoltageGetData(ser, 0) #Le mando 0 volts al motor. Entonces pos,vel=0
    posicion.append(pos)
    velocidad.append(vel)
    
    #Error inical. 
    e_0 = error(setpoint,pos) 
    
    #Errores para el termino integral.
    e_int_0 = 0 
    e_int = (e_int_0 + e_0)*Ti
    
    u = señal_control(P(kp,e_0), I(ki,e_int),0)
        
    for t in tiempo:
        pos, vel = setVoltageGetData(ser, m*u)
        posicion.append(pos)
        
        e = error(setpoint, pos)
        e_int = (e_int + e)*Ti
        u = señal_control(P(kp,e), I(ki,e_int), 0)

    plt.plot(tiempo, posicion,'--', label = f"Ki={ki}")
    i=i+1
    
plt.ylabel('Posición', size=15)
plt.xlabel('Tiempo', size=15)
plt.axhline(y=setpoint, linestyle=':', c='k', label='Setpoint')
plt.legend(loc=1)
plt.show()

#%%
"""   ------------------------    Controlador PD     ------------------------""" 

#Hace lo mismo que control_P solo que le agrega el termino derivativo

"""Posicion Vs Tiempo para distintos valores de Kd"""

posicion = [] ; velocidad =[] ; kd_lista=["Poner varlores de Kd"] ;tiempo = np.linspace(0,50,num=50)

kp=2 ; ki = 0 ; kd = 2 ; setpoint = 10 ; dt =1

for kd in kd_lista:
    #Primer paso
    pos, vel = setVoltageGetData(ser, 0) #Le mando 0 volts al motor. Entonces pos,vel=0
    posicion.append(pos)
    velocidad.append(vel)
    
    #Error incial.
    e_0 = error(setpoint,pos) 
    
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
        u = señal_control(P(kp,e),0 , D(kd,e,e_prev,dt))

plt.plot(tiempo, posicion, '--', c='tab:red')
plt.ylabel('Posición', size=15)
plt.xlabel('Tiempo', size=15)
plt.axhline(y=setpoint, linestyle=':', c='k', label='Setpoint')
plt.legend()
plt.show()

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

plt.plot(tiempo, posicion, '--', c='tab:red')
plt.ylabel('Posición', size=15)
plt.xlabel('Tiempo', size=15)
plt.axhline(y=setpoint, linestyle=':', c='k', label='Setpoint')
plt.legend()
plt.show()


#%%
"""   ------------------------    Método Ziegler Nichols      ------------------------""" 
volt =  5
respuesta = respuesta_pulso(volt)

tiempo = respuesta[0]; posicion = [2]

def sigmoid (x, A, h, slope):
    return 1 / (1 + np.exp ((h - x) / slope)) *A

p, _ = curve_fit(sigmoid, tiempo, posicion)

A = p[0] ; xmedio = p[1] ;  slope = p[2] 
# Show parameters for the fit
# print(p)




#%%
"""   ------------------------    Setpoint fijo      ------------------------""" 
setpoint = [10,100,1000,10000]



#%%
"""   ------------------------    Medición a Lazo Cerrado      ------------------------""" 

def controlador(voltaje):
    pos,vel = setVoltageGetData(ser, voltaje)
    controlador()
controlador()


    
    









    
    