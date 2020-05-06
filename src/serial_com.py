#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Created on Mon Mar  9 15:49:25 2020

@author: linux-asd
"""

import serial
import time
import numpy

class ArduinoSerial:  #clase para la comunicacion con arduino mediante serial, se inicia diciendo puerto y velocidad
                        #y en la funcion crea un string con el formato para la lectura.
                        #en la primera ejecucion hay que abrir el puerto
    def __init__(self , port):
        #ls -l /dev | grep ACM to identify serial port of the arduino
        self.arduino = serial.Serial(port, 115200, timeout = 1)
        # Nota: provocamos un reseteo manual de la placa para leer desde
        # el principio
        self.arduino.setDTR(False)
        time.sleep(1)
        self.arduino.flushInput()
        self.arduino.setDTR(True)
        time.sleep(2)
        
    def serialSend(self, pulse):  
        comando = "<{0}#{1}#{2}#{3}#{4}#{5}#{6}#{7}#{8}#{9}#{10}#{11}>" #Input
        command=comando.format(int(pulse[0]), int(pulse[1]), int(pulse[2]), 
                               int(pulse[3]), int(pulse[4]), int(pulse[5]), 
                               int(pulse[6]), int(pulse[7]), int(pulse[8]), 
                               int(pulse[9]), int(pulse[10]), int(pulse[11]))
        self.arduino.write(bytes(command , encoding='utf8'))# Mandar un comando hacia Arduino
#        print(command)      
    

    def serialRecive(self):
        try:
            startMarker = 60
            endMarker = 62
      
            getSerialValue = bytes()
            x = "z" # any value that is not an end- or startMarker
            byteCount = -1 # to allow for the fact that the last increment will be one too many
                  
                  # wait for the start character
            while  ord(x) != startMarker: 
                x = self.arduino.read()
                  
                  # save data until the end marker is found
            while ord(x) != endMarker:
                if ord(x) != startMarker:
                    getSerialValue = getSerialValue + x 
                    byteCount += 1
                x = self.arduino.read()
                          
            loopTime , roll , pitch  = numpy.fromstring(getSerialValue.decode('ascii', errors='replace'), sep = '#' )
#            print(roll , pitch) 
                
        except ValueError:
            pass
        
        self.arduino.flushInput()    
        return loopTime , roll , pitch
            
    def close(self):
        self.arduino.close()
