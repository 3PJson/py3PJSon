# -*- coding: utf-8 -*-
"""
Created on Thu Apr  8 09:38:06 2021

@author: Timothe
"""

import serial
import serial.tools.list_ports
import time
import struct
import numpy as np

import os, sys
sys.path.append(r"C:/Users/Timothe/NasgoyaveOC/Professionnel/TheseUNIC/DevScripts/Python/__packages__")
import pyserialprotocol as pyPyrot

with serial.Serial(port='COM11', baudrate=250000, timeout=0.1, writeTimeout=0.1) as port_serie:
    number_of_messages = 0
    for i in range(50):
        port_serie.write("{get_varvalue:230}".encode()) #300000
        while True :
            valeur = pyPyrot.CheckMessage(port_serie,0.001)
            if not valeur.isNone() :
                if valeur.type == 'bins':
                    print(valeur.content)
                number_of_messages = number_of_messages + 1
            else :
                break
    print(number_of_messages)


