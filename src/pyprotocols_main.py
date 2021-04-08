# -*- coding: utf-8 -*-
"""
Created on Thu Apr  8 09:04:06 2021

@author: Timothe
"""

import serial
import serial.tools.list_ports
import time
import struct
import relaxed_json as relaxed

class bytesarray(bytearray) :
    def append(self,item):
        if isinstance(item,bytes) :
            super().append(int.from_bytes(item,"big"))
            #super().append([item])
        else :
            super().append(item)

class MessageJson(dict) :

    def __init__(self,dictionnary):
        super().__init__(dictionnary)

class MessageBin() :

    def __init__(self, bt_array, vartype = int, signed = False):

        self.bytearray = bt_array
        self.vartype = vartype
        self.signed = signed
        self.bytesize = len(self.bytearray)-3
        self.id = self.bytearray[0]
        if self.vartype is int:
            self.value = vartype.from_bytes(self.bytearray[1:-2], byteorder='big', signed=self.signed)
        elif self.vartype is float :
            self.value = struct.unpack('f',self.bytearray[1:-2])[0]
        else :
            print("Uh oh... Data type not supported")
        self.get_crc = int.from_bytes(self.bytearray[-2:], byteorder='big', signed=False)

    def calc_crc(self) :
        return crc16( self.bytearray[1:-2] , 0 , self.bytesize)

    def __str__(self):
        return "<bin_holder>\n   ID: " + str(self.id) + "\n   Value: " + str(self.value) + "\n   crc_got: " + str(self.get_crc) + "\n   crc_cal: " + str(self.calc_crc())

    def __repr__(self):
        return self.__str__()

    def valid(self):
        return self.calc_crc() == self.get_crc

def crc16(data : bytearray, offset , length):
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


class MessageBuilder():

    def __init__(self):

        self.type = None
        self.content = None
        self.size = 4
        self.pending_status = None
        self.available = None
        self.resend = None
        self.stop_resend_wait = None

    def clear(self):

        self.type = None
        self.content = None
        self.pending_status = None
        self.available = None
        self.resend = None
        self.stop_resend_wait = None

    def addContent(self,byte,port,force_stop = False):
        if self.type == None :
            self._firstByte(byte)
        else :
            self._ulteriorBytes(byte)
        self.statusResponse(port, force_stop)

    def _firstByte(self,byte):

            if byte == bytes([0x23]) : # #HEXINC
                self.type = 'bins'     # TODO : ADD A CHECK FOR LINESIZE DEPENDING ON INDEX SET IN MEMORY OR CONFIGFILE(CONFIGFILE BETTER)
                self.content = bytesarray()
                self.counter = 1
            elif byte == bytes([0x7B]) : # {JSONINC
                self.type = 'json'
                self.content = '{'
                self.counter = 1
            elif byte == bytes([0x40]) : # @AKINC
                self.type = 'aklg'
                self.content = "incomplete_aknowledge"
            else :
                self.type = 'ndef'
                self.content = byte.decode()

    def _ulteriorBytes(self,byte):

        if self.type == 'bins' :
            self.counter = self.counter + 1
            self.content.append(byte)
            if self.counter >= self.size + 4:
                self.pending_status = 'b?' #binary check ?
                return

        elif self.type == 'json' :
            try :
                byte = byte.decode()
            except :
                print(byte)
                return
            self.content = self.content + byte
            if byte == '}' :
                if self.counter == 1 :
                    try :
                        self.content = MessageJson(relaxed.rjson.parse(self.content))
                        self.pending_status = 'ja' #json ak
                    except :
                        self.pending_status = 'jn' #json nak
                    return
                self.counter = self.counter - 1
            if byte is '{' :
                self.counter = self.counter + 1

        elif self.type == 'aklg' :
            if byte ==  bytes([0x41]) :
                self.content = 'binary_aknowledge'
                self.pending_status = '--' #do nothing
                return
            elif byte ==  bytes([0x6E]) :
                self.content = 'binary_not_aknowledge'
                self.pending_status = 'rs' #do nothing
                return
            else :
                self.content = 'binary_corrupted_aknowledge'
                self.pending_status = 'rs' #repeat send request
                return

        elif self.type == 'ndef' :
            decoded_byte = byte.decode()
            if decoded_byte == '\n':
                self.pending_status = '--' #do nothing
                return
            else :
                self.content = self.content + decoded_byte
            if len(self.content)>60:
                self.pending_status = '--' #do nothing
                return

    def statusResponse(self,port,force_stop = False):
        if self.pending_status is not None or force_stop:
            valid = False
            if self.type is not None :

                if self.type == 'bins' :
                    if self.pending_status == "b?" :
                        self.content = MessageBin(self.content)
                        valid = self.content.valid()
                    AknowledgeBin(valid,port)

                elif self.type == 'json' :
                    if self.JsonPrechecker() :
                        valid = False
                    else :
                        if self.pending_status == 'ja' :
                            valid = True
                        elif self.pending_status == 'jn' :
                            valid = False
                        AknowledgeJson(valid,port)

                if self.pending_status == 'xs' :
                    self.stop_resend_wait = 1
                    #STOP RESEND PROCEDURE if AK
                if self.pending_status == 'rs' :
                    self.resend = 1

                self.available = valid

    def terminate(self,port):
        if self.available == None :
            self.statusResponse(port,True)

    def JsonPrechecker(self):
        if not isinstance(self.content,dict):
            self.pending_status = 'jn'
            return 0
        if self.content.get("Ak") is not None :
            self.content = "JsonAk_" + self.content.get("Ak")
            self.type = 'aklg'
            self.pending_status = 'xs'
            return 1
        elif self.content.get("Na") is not None :
            self.content = "JsonNaK_" + self.content.get("Na")
            self.type = 'aklg'
            self.pending_status = 'sr'
            return 1
        return 0

    def isNone(self):
        if self.type is None :
            return True
        return False

def AknowledgeBin(valid,port):
    msg = bytesarray([0x40])
    if valid :
        msg.append(0x41)
    else :
        msg.append(0x6E)
    port.write(msg)

def AknowledgeJson(valid,port,optional_info = "1"):
    if valid :
        msg = "{Ak:1}"
    else :
        msg = "{Na:" + optional_info + "}"
    port.write(msg.encode())


class AknowledgeWaiter():

    def __init__(self, mail):

        self.mailcopy =
        self.posttime = 0

    def clear():
         pass

    def posted():

        pass



class Transiter():

    def __init__(self,port):

        self.serial.port = port
        self.builder = MessageBuilder()

    def scan(self):

        if serialport.in_waiting:
            byte_capture_time = time.time()
            item = serialport.read()
            Builder.addContent(item,serialport)

            if Builder.available is not None :

    def send(self):
        pass

    def emit_reception(self):
        pass

    def _send_queue(self):

    def _output_buffer(self):

    def _handleMessage(self):

    def _aknowledge_waiter(self):

        AknowledgeWaiter()


def CheckMessage(serialport, max_time = 0.0015):
    byte_capture_time = time.time()
    elapsed_time = byte_capture_time
    Builder = MessageBuilder()

    while elapsed_time-byte_capture_time < max_time :
        elapsed_time = time.time()

        if serialport.in_waiting:
            byte_capture_time = time.time()
            item = serialport.read()
            Builder.addContent(item,serialport)

            if Builder.available is not None :
                break

    Builder.terminate(serialport)
    return Builder

if __name__ == '__main__':

    with serial.Serial(port='COM11', baudrate=250000, timeout=0.1, writeTimeout=0.1) as port_serie:
        #arr_bytes = bytearray([0x23,0x0A,0xef,0x7b,0xb5,0xef,0x11,0x04]) # 4017862127
        #arr_bytes = bytearray([0x23,0x0A,0x1f,0x2c,0xb5,0x3f,0x9E,0x6A]) # 523023679
        arr_bytes = bytearray([0x23,0xE6,0x1f,0x2c,0xb5,0x3f,0x9E,0x6A]) # 523023679


        port_serie.write("{get_vartype:230}".encode()) #300000
        while True :
            valeur = CheckMessage(port_serie)
            if not valeur.isNone() :
                print(valeur.content)

            else :
                break

        port_serie.write(arr_bytes)#0x230A 0xef7bb5ef 0x1104
        while True :
            valeur = CheckMessage(port_serie,0.01)
            if not valeur.isNone() :
                print(valeur.content)

            else :
                break

        port_serie.write("{get_varvalue:230}".encode()) #300000
        while True :
            valeur = CheckMessage(port_serie,0.01)
            if not valeur.isNone() :
                if valeur.type == 'bins' :
                    print(valeur.content)
                else :
                    print(valeur.content)
            else :
                break
