# -*- coding: utf-8 -*-
#main.py
#author: Drawbot team CRI - Marion Ficher
#date : october 2019
#version : V1.0

import serial
from Conversion_maths4 import Drawbot

def ask_function():
    try:
        choice = int(input("Enter 1 for a line, 2 for a square "))
    except ValueError:
        print ("Not a digit")
        return ask_function()
    if choice < 1 or choice > 2:
        print ("Not valid")
        return ask_function()
    return choice

def ask_coordinates():
    try:
        x = int(input("Enter the x coordinates of the choosen point "))
        y = int(input("Enter the y coordinates of the choosen point "))
    except ValueError:
        print ("Not a digit")
        return ask_function()
    if choice < 1 or choice > 2:
        print ("Not valid")
        return ask_function()
    return choice

test_drawbot = Drawbot()
coordX = 0
coordY = 0

choice = ask_function()

if choice ==1:
    test_drawbot.drawStraightline(300, 300, 1000, 0)
    test_drawbot.reinit_drawing()

if choice ==2:
    test_drawbot.drawSquare (300,300,100)
    test_drawbot.reinit_drawing()

'''
a = serial.Serial("/dev/ttyUSB1",9600,timeout=1)
b = a.readline()

print (b) 
print ("Enter 1 to ON LED and 0 to OFF LED")

while 1:                                      
    input_data =input()                  
    print ("you entered", input_data)         
    
    if (input_data == '1'):                  
        a.write("1".encode())             
        print ("LED ON")
       
    
    if (input_data == '0'):                   
        a.write("0".encode())              
        print ("LED OFF")
'''