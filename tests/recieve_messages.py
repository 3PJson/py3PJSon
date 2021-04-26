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
import pyPPJson as protocol

from collections import deque
import threading
import sys



class ComThread(threading.Thread):

    def __init__(self, queue, table, port = 'COM11'):

        self.port = port
        self.table = table
        self.queue = queue
        super(MyThread, self).__init__()

    def run(self):

        with serial.Serial(port=self.port, baudrate=250000, timeout=0.1, writeTimeout=0.1) as port_serie:

            reciever = protocol.MessageBuilder(port_serie)
            reciever.setTable(self.table)
            sender = protocol.MessageSender(port_serie)
            sender.setTable(self.table)

            while True :
                sender.poll()
                reciever.checkByte()
                if not reciever.running :
                    if reciever.available is not None :

                        if reciever.available :
                            if reciever.type == 'b' :
                                print(reciever.content.value)

                            else :
                                print(reciever.content)
                        else :
                            sender.connect_aknowledge(reciever.content)
                            #print(sender.transfer_duration())
                        reciever.clear()

                if not sender.running and len(self.queue):
                    sentence = self.queue.pop()
                    sender.send(sentence)

                if reciever.elapsed() > 1 :
                    print("exiting")
                    break

###############################


# types = protocol.TypesTable([10,20,230,158],[int,int,float,int],[4,4,4,8],[True,True,None,False])
# message_queue = deque()

# thread = MyThread(message_queue, types, 'COM11')
# thread.start()

# thread.queue.append( "{get_vartype:230}".encode() )
# thread.queue.append( [230, 1.0044] )
# thread.queue.append( "{get_varvalue:230}".encode() )


from numba import jit
import numba

#@jit(nopython=True)
# @numba.njit
# def appendBytes(queue : deque, byteslist : bytes):
#     for byte in byteslist:
#         queue.append(byte)

def appendBytesNOJIT(queue,byteslist):
    for byte in byteslist:
        queue.append(bytes([byte]))

def appendBytesToDeque(queue,byteslist):# TODO : write this in cython
    for byteid in range(len(byteslist)):
        queue.append(byteslist[byteid:byteid+1])

# cdef int add (int x, int y):
#  cdef int result
#  result = x + y
#  return result

################################
from collections import deque

message_queue = deque()
lista = []

test = b'\xd3\xFF'
for _ in range(1000000):

    #message_queue.append( 20 )
    # message_queue.appendleft( 23 )
    # message_queue.appendleft( 4 )
    # message_queue.pop()
    # message_queue.popleft()

    #appendBytes(message_queue,test)


    appendBytesNOJIT(message_queue,test)
    appendBytesToDeque(message_queue,test)

    if len(message_queue)>0:

        message_queue.pop()
        message_queue.popleft()
        message_queue.pop()
        message_queue.popleft()

    #lista.append(34)
    #lista.pop()

    # val = time.perf_counter()
    # val = time.perf_counter() - val

    # val = time.process_time()
    # val = time.process_time() - val
