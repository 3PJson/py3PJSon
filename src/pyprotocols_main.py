# -*- coding: utf-8 -*-
"""
Created on Thu Apr  8 09:04:06 2021

@author: Timothe
"""

import serial
import serial.tools.list_ports
import relaxed_json as relaxed
import time
import struct
import threading
from collections import deque

from numba import jit

class bytesarray(bytearray) :

    def append(self,item):
        if isinstance(item,bytes) :
            super().append(int.from_bytes(item,"big"))
            #super().append([item])
        elif isinstance(item,bytearray) :
            for it in item :
                super().append(it)
        elif isinstance(item,list) :
            for it in item :
                super().append(it)
        else :
            super().append(item)



class PPJsonMessage():

    def __init__(self, content , **kwargs):

        self.content = content
        self.origin = kwargs.get("origin")
        self.send_time = kwargs.get("send_time")
        self.recieve_time = kwargs.get("recieve_time", [])
        self.aknowledge_state = kwargs.get("aknowledge_state")
        self.aknowledge_time = kwargs.get("aknowledge_time")

        self.ready = None

    def setSend_time(self,origin):
        self.origin = origin

    def setSend_time(self,send_time):
        self.send_time = send_time

    def appendRecieve_time(self,recieve_time):
        self.recieve_time.append(recieve_time)

    def setAknowledge_state(self,aknowledge_state):
        self.aknowledge_state = aknowledge_state

    def setAknowledge_time(self,aknowledge_time):
        self.aknowledge_time = aknowledge_time

    def __str__(self):
        return "<PPJsonMessage>\n\tType: " + str(type(self.content)) + "\n\tContent: " + self.content.__str__() + "\n\tSend Time: " + str(self.send_time) + "\n\tRecieve Time: " + str(self.recieve_time) + "\n\tAknowledge Time: " + str(self.aknowledge_time) + "\n\tAknowledge State: " + str(self.aknowledge_state) + "\n\tRead State: " + str(self.ready)

    def __repr__(self):
        return self.__str__()

class AknowledgeContent():
    def __init__(self, ak_type, ak_value, ak_origin = None):
        self.type = ak_type
        self.ak_value = ak_value
        self.ak_origin = ak_origin

class JsonContent(dict) :
    def __init__(self,dictionnary):
        super().__init__(dictionnary)

class BinaryContent() :
    def __init__(self, bt_array, table):

        self.bytearray = bt_array
        self.id = self.bytearray[0]
        self.vartype = table.getType(self.id)
        self.bytesize = table.getSize(self.id)
        self.signed = table.getSign(self.id)
        if self.vartype is int:
            self.value = self.vartype.from_bytes(self.bytearray[1:-2], byteorder='big', signed=self.signed)
        elif self.vartype is float :
            if self.bytesize == 4 :
                self.value = struct.unpack('f',self.bytearray[1:-2])[0]
            elif self.bytesize == 8 :
                self.value = struct.unpack('d',self.bytearray[1:-2])[0]
        else :
            print("Uh oh... Data type not supported")
        self.get_crc = int.from_bytes(self.bytearray[-2:], byteorder='big', signed=False)

    def calc_crc(self) :
        return crc16( self.bytearray[1:-2], self.bytesize)

    def __str__(self):
        return "<BinaryContent>\n   ID: " + str(self.id) + "\n   Value: " + str(self.value) + "\n   crc_got: " + str(self.get_crc) + "\n   crc_cal: " + str(self.calc_crc()) + "<>"

    def __repr__(self):
        return self.__str__()

    def valid(self):
        return self.calc_crc() == self.get_crc

#@jit(nopython=True)
def crc16(data : bytearray, length, offset = 0):
    if data is None or offset < 0 or offset > len(data)- 1 and offset+length > len(data):
        return 0
    crc = 0xFFFF
    for i in range(0, length):
        crc ^= data[offset + i] << 8
        for j in range(0,8):
            if (crc & 0x8000) > 0:
                crc =(crc << 1) ^ 0x1021
            else:
                crc = crc << 1
    return crc & 0xFFFF

def appendBytesToDeque(queue,byteslist): # TODO : write this in cython

    for byte in byteslist:
        queue.append(bytes([byte]))

class MessageBuilder():

    def setTable(self, table):
        self.typetable = table

    def __init__(self,serialport):

        self.port = serialport

        self.type = None
        self.content = None
        self.available = None # True for content useable , False for content related to answers (answer recieved or not recieved), None for corrupted message.
        self.init = 0
        self.running = True
        self.last_byte_capture_time = 0
        self.buffer = deque()

    def clear(self):

        self.type = None
        self.content = None
        self.available = None
        self.last_byte_capture_time = 0
        self.running = True #used to say that the process of making up a message is over, sucessfull or not, regardless of content

    def elapsed(self):
        if self.last_byte_capture_time == 0:
            self.last_byte_capture_time = time.perf_counter()
        return time.perf_counter() - self.last_byte_capture_time

    def checkByte(self, max_time = 0.0015, mode = 0):

        if mode == 1 :
            if self.port.in_waiting :
                appendBytesToDeque( self.buffer, self.port.read(self.port.in_waiting) )

        if self.running :
            timeover = self.elapsed() > max_time
            item = None
            if mode == 0 :
                if self.port.in_waiting :
                    item = self.port.read()
            else :
                if len(self.buffer) > 0 :
                    item = self.buffer.popleft()

            if not timeover and item is not None :

                self.last_byte_capture_time = time.perf_counter()
                self._addContent(item)

            if timeover and self.type is not None:
                print("timeover")
                self._terminate()

    def _addContent(self,byte,force_stop = False):
        if self.type == None :
            self._firstByte(byte)
        else :
            self._ulteriorBytes(byte)

    def _firstByte(self,byte):
            if byte == bytes([0x23]) : # #HEXINC
                self.type = 'b'     # TODO : ADD A CHECK FOR LINESIZE DEPENDING ON INDEX SET IN MEMORY OR CONFIGFILE(CONFIGFILE BETTER)
                self.content = PPJsonMessage(bytesarray())
                self.content.appendRecieve_time(time.perf_counter())
                self.counter = 1
            elif byte == bytes([0x7B]) : # {JSONINC
                self.type = 'j'
                self.content = PPJsonMessage('{')
                self.content.appendRecieve_time(time.perf_counter())
                self.counter = 1
            elif byte == bytes([0x40]) : # @AKINC
                self.type = 'a'
                self.content = "incomplete_aknowledge"
            else :
                self.type = 'n'
                try :
                    self.content = byte.decode()
                except :
                    print('passing')
                    pass

            #print("message_rcv_start_at ",end = '')
            #print(time.perf_counter())

    def _ulteriorBytes(self,byte):

        if self.type == 'b' :
            self.counter = self.counter + 1
            self.content.content.append(byte)
            if self.counter == 2 :
                self.size = self.typetable.getSize(self.content.content[0])
            if self.counter >= self.size + 4: #4 bytes : start byte, identifier, and 2 bytes CRC
                self._stopBin()
                return

        elif self.type == 'j' :
            try :
                byte = byte.decode()
            except :
                print(byte)
                return
            self.content.content = self.content.content + byte
            if byte == '}' :
                if self.counter == 1 :
                    self._stopJson()
                    return
                self.counter = self.counter - 1
            elif byte is '{' :
                self.counter = self.counter + 1

        elif self.type == 'a' :
            if byte ==  bytes([0x41]) :
                self.content = AknowledgeContent('b',True)#'binary_aknowledge'
                self._stopAk()
                return
            elif byte ==  bytes([0x6E]) :
                self.content = AknowledgeContent('b',False, 'crc')#'binary_not_aknowledge'
                self._stopAk()
                return
            else :
                self.content = AknowledgeContent('b',False, 'inv')#'invalid_aknowledge'
                self._stopAk()
                return

        elif self.type == 'n' :
            try :
                decoded_byte = byte.decode()
            except :
                return
            if decoded_byte == '\n':
                self._stopUndef()
                return
            else :
                self.content = self.content + decoded_byte
            if len(self.content)>40:
                self._stopUndef()
                return

    def _stopBin(self):
        self.content.appendRecieve_time(time.perf_counter())
        self.content.content = BinaryContent(self.content.content,self.typetable)
        valid = self.content.content.valid()
        aktime = AknowledgeBin(valid,self.port)
        self.content.setAknowledge_time(aktime)
        self.available = valid
        self._terminate()

    def _stopJson(self):
        self.content.appendRecieve_time(time.perf_counter())
        try :
            self.content.content = JsonContent(relaxed.rjson.parse(self.content.content))
        except :
            aktime = AknowledgeJson(False,self.port)
            self.content = None
            self.content.setAknowledge_time(aktime)
            self.available = False
            self._terminate()
            return

        if self._JsonPrechecker() :
            self.available = False #Message was just a Json aknowledge # Do something with the message sender
            self._terminate()

        else :
            aktime = AknowledgeJson(True,self.port) #Have to check if key is expected # TODO
            self.content.setAknowledge_time(aktime)
            self.available = True
            self._terminate()

    def _stopAk(self):
        self.available = False
        self._terminate()

    def _stopUndef(self):
        self.available = True
        self._terminate()

    def _terminate(self):
        self.running = False

    def _JsonPrechecker(self):
        if self.content.content.get("Ak") is not None :
            self.content = AknowledgeContent('j',True)
            self.type = 'a'
            return 1
        elif self.content.content.get("Na") is not None :
            self.content = AknowledgeContent('j',False, self.content.content.get("Na"))
            self.type = 'a'
            return 1
        return 0

def AknowledgeBin(valid,port):
    msg = bytesarray([0x40])
    if valid :
        msg.append(0x41)
    else :
        msg.append(0x6E)
    port.write(msg)

    #if valid :
        #print("Just Sent Binary Aknowledge at : ", end = '')
    #else :
        #print("Just Sent Binary NOT-Aknowledge at :", end = '')
    aktime = time.perf_counter()
    #print(aktime)
    return aktime

def AknowledgeJson(valid,port,optional_info = "1"):
    if valid :
        msg = "{Ak:1}"
    else :
        msg = "{Na:" + optional_info + "}"
    port.write(msg.encode())

    #if valid :
        #print("Just Sent Json Aknowledge at :", end = '')
    #else :
        #print("Just Sent Json NOT-Aknowledge at : ", end = '')
    aktime = time.perf_counter()
    #print(aktime)
    return aktime

class MessageSender():

    def __init__(self, port, timeout = 0.01, max_attempts = 3):

        self.port = port
        self.mailcopy = None
        self.max_time = timeout
        self.attempts = 0
        self.running = 0
        self.max_attempts = max_attempts

    def setTable(self,table):

        self.typesTable = table

    def clear(self):

        self.running = 0
        self.attempts = 0
        self.mailcopy = None

    def send( self, mail ):

        if isinstance( mail, list) :
            self.mailcopy = self.typesTable.asBytesArray(mail[0], mail[1])
        else :
            self.mailcopy = mail
        self.running = 1
        self._post()
        #self.initial_post_time = time.perf_counter()

    def poll(self):

        if self.running :
            if self.elapsed() > self.max_time :
                self._post()

    def connect_aknowledge(self, reference ):

        if reference.ak_value == True :
            self.clear()
            print("Just Recieved Aknowledge", end = '')
            if reference.type == 'b' :
                print(" Binary", end = '')
            else :
                print(" Json", end = '')
            #self.response_time = time.perf_counter()
            print()
        else :
            self._post()
            print("Just Recieved NOT-Aknowledge", end = '')
            if reference.type == 'b' :
                print(" Binary from ;", end = '')
            else :
                print(" Json from :", end = '')
            print(reference.ak_origin)

    def elapsed(self):

        return time.perf_counter() - self.posttime

    def transfer_duration(self):
        return self.response_time - self.initial_post_time

    def _post(self):
        if self.running :
            if self.attempts >= self.max_attempts :
                self.clear()
            else :
                print("Sending : ",self.mailcopy)
                print("Attempts : ",self.attempts)
                self.port.write(self.mailcopy)
                self.posttime = time.perf_counter()
                self.attempts = self.attempts + 1


class ExpectedAnswersTable(dict):

    def __init__(self, dictionnary = None):
        super().__init__(dictionnary)

    def expectAnswer(self,message_key):
        return message_key in self.keys()

    def answers(self,message_key):
        return self.get(message_key)

class TypesTable():
    def __init__(self,*items):
        self.types = [None] * 255
        self.sizes = [None] * 255
        self.signs = [None] * 255
        if items is not None :
            self.add(*items)

    def add(self, ids, types, sizes, signs):
        if isinstance(ids,list):
            for index, id in enumerate(ids) :
                self.types[id] = types[index]
                self.sizes[id] = sizes[index]
                self.signs[id] = signs[index]
        else :
            self.types[ids] = types
            self.sizes[ids] = sizes
            self.signs[ids] = signs

    def getType(self,id):
        return self.types[id]

    def getSize(self,id):
        return self.sizes[id]

    def getSign(self,id):
        return self.signs[id]

    def asBytesArray(self,id,value):
        if self.types[id] is None :
            raise(ValueError(f"No value has been declared for ID {id}"))

        arr_bytes = bytesarray([0x23,id])
        if self.types[id] is int :
            value = int(value)
            if not self.sizes[id] and value < 0 :
                value = abs(value)
            valarray = bytearray(value.to_bytes(self.sizes[id], byteorder='little',signed = self.signs[id] ))

        elif self.types[id] is float :
            if self.sizes[id] == 4 :
                valarray = bytearray(struct.pack("f", value)) # TODO : check behavior for doubles : if doubles values in arduinoPPJson match the ones sent here

            elif self.sizes[id] == 8 :
                valarray = bytearray(struct.pack("d", value)) # TODO : check behavior for doubles : if doubles values in arduinoPPJson match the ones sent here

            else :
                raise(ValueError(f"Float bytesize can only be 4 or 8 but was set to {self.sizes[id]} for ID {id}"))
                # TODO : Check that at definition time instead of at runtime to avoid raisong an error with latency for the user

        else :
            raise(TypeError(f"Type of ID {id} is not supported : {self.types[id]}"))

        crc = crc16(valarray,self.sizes[id])
        print(crc)
        arr_bytes.append( valarray )
        arr_bytes.append( bytesarray(crc.to_bytes(2, byteorder='big' ) ) )
        return arr_bytes


class RecievingThread(threading.Thread):

    def __init__(self, port, reciever):
        threading.Thread.__init__(self)
        self.serialport = port
        self.reciever = reciever
        self.result = None

    def run(self):

        while True :
            self.reciever.checkByte()
            if not reciever.running :
                if reciever.available is not None :
                    if reciever.available :
                        self.result = PPJsonMessage(reciever.content )
                    else :
                        self.result = reciever








from PyQt5.QtCore import QThread, pyqtSignal

class ComThread(QThread):

    PacketRecieved = pyqtSignal()
    def __init__(self, queue, table, port = 'COM11', baudrate = 250000):

        self.port = port
        self.table = table
        self.queue = queue
        self.baudrate = baudrate
        super(ComThread, self).__init__()

        self.data = None
        self.type = None

    def run(self):

        with serial.Serial(port = self.port, baudrate = self.baudrate, timeout=0.1, writeTimeout=0.1) as port_serie:

            reciever = MessageBuilder(port_serie)
            reciever.setTable(self.table)
            sender = MessageSender(port_serie)
            sender.setTable(self.table)

            while True :
                sender.poll()
                reciever.checkByte()
                if not reciever.running :
                    if reciever.available is not None :
                        if reciever.available :

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

    def placeholder(self):
        self.PacketRecieved = pyqtSignal()
        self.blockSignals(True)
        return self.data

import copy



# def ComsWrapper(port = 'COM11', queue, typestable ):

#     with serial.Serial(port=port, baudrate=250000, timeout=0.1, writeTimeout=0.1) as port_serie:

#         #typestable = TypesTable([10,20,230,158],[int,int,float,int],[4,4,4,8],[True,True,None,False])

#         reciever = MessageBuilder(port_serie)
#         reciever.setTable(typestable)
#         sender = MessageSender(port_serie)
#         sender.setTable(typestable)

#         sentence = message_queue.pop()
#         sender.send(sentence)

#         reciever.clear()

#         while True :
#             sender.poll()
#             reciever.checkByte()
#             if not reciever.running :
#                 if reciever.available is not None :
#                     messages = messages + 1
#                     if reciever.available :
#                         if reciever.type == 'b' :
#                             print(reciever.content.value)
#                             bmessages = bmessages + 1
#                         else :
#                             print(reciever.content)
#                     else :
#                         sender.connect_aknowledge(reciever.content)
#                         #print(sender.transfer_duration())
#                     reciever.clear()

#             if not sender.running and len(message_queue):
#                 sentence = message_queue.pop()
#                 sender.send(sentence)

#             if reciever.elapsed() > 0.02 :
#                 print("exiting")
#                 break


if __name__ == '__main__':


    # types = TypesTable([10,20,230,158],[int,int,float,int],[4,4,4,8],[True,True,None,False])
    # message_queue = deque()

    # thread = ComThread(message_queue, types, 'COM11')

    # #thread.PacketRecieved.connect(getThreadData)
    # #thread.PacketRecieved.connect(HandlePacket)

    # thread.queue.append( "{get_vartype:230}".encode() )
    # thread.queue.append( [230, 1.0044] )
    # thread.queue.append( "{get_varvalue:230}".encode() )
    # thread.start()

    # while True : #thread.isRunning() :
    #     #if thread.PacketRecieved :
    #     pass
    #     #print("ak")
    #         #data = thread.placeholder()
    #         #print(data)

####################


    with serial.Serial(port='COM11', baudrate=250000, timeout=0.1, writeTimeout=0.1) as port_serie:
        #arr_bytes = bytearray([0x23,0x0A,0xef,0x7b,0xb5,0xef,0x11,0x04]) # 4017862127
        #arr_bytes = bytearray([0x23,0x0A,0x1f,0x2c,0xb5,0x3f,0x9E,0x6A]) # 523023679
        #arr_bytes = bytearray([0x23,0xE6,0x1f,0x2c,0xb5,0x3f,0x9E,0x6A]) # 523023679


        types = TypesTable([10,20,30,230,158],[int,int,int,float,int],[4,4,4,4,8],[True,True,True,None,False])

        reciever = MessageBuilder(port_serie)
        reciever.setTable(types)
        sender = MessageSender(port_serie)
        sender.setTable(types)

        message_queue = deque()

        results = []

        for _ in range(1) :

            #message_queue.append( "{get_vartype:230}".encode() )
            #message_queue.append( [10, 1000] )
            #message_queue.append( "{get_varvalue:10}".encode() )
            #message_queue.append( "{go:9871}".encode() )
            message_queue.append( "{lock:0}".encode() )
            #message_queue.append( "{go:0}".encode() )
            message_queue.append( [10, 0] )
            message_queue.append( [20, 0] )

            #message_queue.append( "{lock:0}".encode() )

            sentence = message_queue.popleft()
            sender.send(sentence)

            reciever.clear()

            while True :
                sender.poll()
                reciever.checkByte()
                if not reciever.running :
                    if reciever.available is not None :
                        if isinstance(reciever.content, AknowledgeContent):
                            sender.connect_aknowledge(reciever.content)
                        else :
                            results.append(reciever.content)
                    reciever.clear()

                if not sender.running and len(message_queue):
                    sentence = message_queue.popleft()
                    sender.send(sentence)

                if reciever.elapsed() > 2 :
                    print("\nexiting\n")
                    break

                #port_serie.write(arr_bytes)#0x230A 0xef7bb5ef 0x1104

    print(results)
    # threads = MyThread()
    # threads.start()


