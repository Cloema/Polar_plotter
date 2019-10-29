# -*- coding: utf-8 -*-
#Conversion_maths4.py
#author: Drawbot team CRI - Marion Ficher
#date : october 2019
#version : V1.0

import math
import tkinter
import matplotlib.pyplot as plt
import time
import serial

#maybe not necessary if we want to make automation on it
LG_SURF = 1000 #mm distance between the 2 steppers
HEIGHT_SURF = 1000 #mm height surface
PERD_PEN = 500 #mm perpendicular between the motors and pen
LG_PEN = 500 #mm
delay_motor = 0.001 #seconde as delay for sending instructions to arduino

'''
                Our coordinate system is :

                (0,HEIGHT_SURF)....(LG_SURF,HEIGHT_SURF)
                .                   .
                .                   .
                .                   .
                (0,0)..............(LG_SURF,0)
                Origin point is in the left corner
                We start the program with the calibration in the middle of the surface :
                    Origin point is (x=LG_SURF/2, y=HEIGHT_SURF)
    
    class Drawbot() 
'''
class Drawbot (object):
    def __init__(self):

        '''
            Size in mm of the drawing surface
        '''
        self.mmWidth = LG_SURF
        self.mmHeight = HEIGHT_SURF

        # Number of thread by revolutions per motor
        self.motorStepsPerRev = 200 # 400 not 200
        self.mmPerRev = 8 * math.pi # = 8 mm * pi = 25.13274
        print("mmPerRev : ", self.mmPerRev)

        self.deltamin = (math.pi * 8)/ self.motorStepsPerRev
        self.deltastep = 0.05
        #self.deltastep = (self.deltamin/2)-0.01
        print("Delta min : ", self.deltastep)

        #number of mm by Steps
        self.mmPerStep = self.mmPerRev / self.motorStepsPerRev
        print("mmPerStep : ", self.mmPerStep)

        '''
            Size in steps of the drawing surface
        '''
        self.stepsWidth = self.mmWidth / self.mmPerStep
        self.stepsHeight = self.mmHeight / self.mmPerStep
        print("stepsWidth", self.stepsWidth)
        print("stepsHeight", self.stepsHeight)

        '''
            Position of the pen holder
            originX is the x coordinate for the center
            originY is the y coordinate for the center
        '''
        self.originX = self.mmWidth/2
        self.originY = self.mmHeight/2

        '''
            Position of the pen holder
            currentX is the x coordinate
            currentY is the y coordinate
        '''
        self.currentX = self.originX
        self.currentY = self.originY

        '''
            distance between the pen and the motors, length of the rubber
            A is the left side
            B is the right side
        '''
        self.lengthA = math.sqrt(math.pow((self.stepsWidth/2), 2) + math.pow((self.stepsHeight/2), 2))
        self.lengthB = math.sqrt(math.pow((self.stepsWidth/2), 2) + math.pow((self.stepsHeight/2), 2))

        print("currentA : ", self.lengthA)
        print("currentB : ", self.lengthB)

        '''
            marginX is the margin where the penholder can't go in the axe X
            marginY is the margin where the penholder can't go in the axe Y
        '''
        self.marginX = 250
        self.marginY = 250


    '''
        drawStraightLine(X1, Y1, targetX,targetY) : draw a line from (X1,Y1) to (targetX,targetY)
            X1 : x coordinate of the line origin
            Y1 : y coordinate of the line origin
            target X : x coordinate of the end of the line
            target Y : y coordinate of the end of the line
     '''
    def drawStraightline (self, X1, Y1, targetX, targetY):
        indic_drawing = False
        while indic_drawing == False:
            print("current X : ", self.currentX, " AND current Y : ", self.currentY)
            print("X1 : ", X1, " AND Y1 : ", Y1)
            print("targetX : ", targetX, " AND targetY : ", targetY)
            print()
            if (X1 != self.currentX and Y1 != self.currentY):
                #send instruction to the servomotor (motor 2) to make up the pen
                #MOVE
                print("2,0")
                print("MOTOR IS MOVING TO THE X,Y POINT TO START THE DRAWING")
                self.motors_move(self.currentX, self.currentY, X1, Y1)
                # a.write("2,0".encode())
            else:
                #send instruction to the servomotor (motor 2) to make down the pen
                #WRITE
                print("2,1")
                # a.write("2,1".encode())
                self.motors_move(X1, Y1, targetX, targetY)
                print("LINE IS PRINTED")
                indic_drawing = True

    '''
        reinit_drawing() : put the cursor at the orgin point
    '''
    def reinit_drawing (self):
        # send instruction to the servomotor (motor 2) to make up the pen
        # MOVE
        print("2,0")
        # a.write("2,0".encode())
        print("MOTOR IS MOVING TO THE ORIGIN POINT")
        self.motors_move(self.currentX, self.currentY, self.originX, self.originY)


    '''
        drawSquare(coordX, coordY, length)
            coordX = x coordinate for the origin point of the square
            coordY = y coordinate for the origin point of the square
            length of the square in mm
     '''
    def drawSquare (self, coordX, coordY, length):
        squareX = []
        squareY = []
        #coordinate 0
        squareX.append(coordX)
        squareY.append(coordY)
        # coordinate 1
        squareX.append(coordX+length)
        squareY.append(coordY)
        # coordinate 2
        squareX.append(coordX+length)
        squareY.append(coordY-length)
        # coordinate 3
        squareX.append(coordX)
        squareY.append(coordY-length)

        print("Square X : ", squareX)
        print("Square Y : ", squareY)
        cpt_points = 0

        while cpt_points < 4:
            #last point
            if cpt_points == 3:
                self.drawStraightline(squareX[cpt_points], squareY[cpt_points],  squareX[0],squareY[0])
            else :
                self.drawStraightline (squareX[cpt_points],squareY[cpt_points],squareX[cpt_points+1],squareY[cpt_points+1])
            cpt_points +=1
        print("square draw")

    '''
        convertstepsTomm (distance) : conversion of a steps distance to a distance in mm
    '''
    def convertstepsTomm (self, distance):
        return (distance * self.mmPerStep)

    '''
        convertmmTosteps (distance) : conversion of distance in a mm to a distance in steps
    '''
    def convertmmTosteps (self, distance):
        return (distance / self.mmPerStep)

    '''
        looknbStep(L1, L2) : calculate the number of revolutions to send to the motor 
        comparing the lengths L1 and L2
     '''
    def looknbStep(self, L1, L2):
        if L1>L2:
            inter= round(L1 - L2)
        else:
            inter = round(L2 - L1)
        if inter == 0:
            inter = 1
        return inter

    '''
        motors_move(X1, Y1, targetX,targetY) : make the motor moves from (X1,Y1) to (targetX,targetY)
        Inverse model : we know the target
        Always verify to that x belongs to [0 ; LG] and y belongs to [0 ; HG] in mm, we have to convert it in steps
        In a first time, we calculate angles teta1 and teta2. These are the angles between the motors and the rubber
        Second, we calculate the lengths L1 et L2 which are the final lengths that needed for the motors
        L1 et L2 must belong to [0; sqrt(pow(LG,2) + pow(HG,2)]
     '''

    def motors_move(self, X1, Y1, targetX, targetY):
        #calculation of the angles
        teta1 = math.atan2((self.mmHeight-Y1), X1)
        teta2 = math.atan2((self.mmHeight-Y1), (self.mmWidth-X1))
        print("teta 1 : ", teta1, " degré : ", math.degrees(teta1))
        print("teta 2 : ", teta2, " degré : ", math.degrees(teta2))

        #calculation of the final lengths L1 et L2
        L1 = (self.mmHeight-Y1)/math.sin(teta1)
        L2 = (L1*math.sin(teta1)) / math.sin(teta2)
        print("L1 : ", L1)
        print("L2 : ", L2)
        # calculation of the line for a targetX lower than X1
        if (X1 < targetX):
            pente = (targetY - Y1) / (targetX - X1)
            coordP = ((targetY + Y1) - pente * (targetX + X1)) / 2
            nbVector = round((targetX - X1) / self.deltastep)
        else:
            if (X1 == targetX):
                nbVector = round(abs(targetY - Y1) / self.deltastep)
                if Y1 < targetY: self.deltastep = -self.deltastep
            elif (X1 > targetX):
                pente = (targetY - Y1) / (targetX - X1)
                coordP = ((targetY + Y1) - pente * (targetX + X1)) / 2
                nbVector = abs(round((X1 - targetX) / self.deltastep))
        #print("pente : ", pente)
        #print("coordP : ", coordP)
        print("nbVector : ", nbVector)
        cptNbElement=0
        matX = []
        matY = []

        #creation of matrix matX and matY which contain all coordinates X,Y for the line
        while cptNbElement <= nbVector:
            if cptNbElement==0:
                #matX will contain X coordinates
                #it's a matrice with nbVector elements
                matX.append(X1)
                matY.append(Y1)
            elif (cptNbElement == nbVector):
                matX.append(targetX)
                matY.append(targetY)
                if self.deltastep < 0 : self.deltastep = abs(self.deltastep)
            else:
                if (X1 == targetX):
                    matX.append(targetX)
                    matY.append( matX[cptNbElement - 1] + self.deltastep)
                elif (X1 < targetX):
                    matX.append(matX[cptNbElement - 1] + self.deltastep)
                    matY.append((pente * matX[cptNbElement]) + coordP)
                else:
                    matX.append(matX[cptNbElement - 1] - self.deltastep)
                    matY.append((pente * matX[cptNbElement]) + coordP)
            cptNbElement +=1

        print("lenght matX : ", len(matX), "lenght matY : ", len(matY))
        print(matX)
        print(matY)

        matL1 = []
        matL2 = []

        cptNbElement = 0

        # creation of matrix matL1 and matL2 which contain all ruber length between the pen holder and motor 1 for matL1
        #mat2 contain all lengths between motor2 and pen holder
        if len(matX) == len(matY):
            while cptNbElement < len(matX):
                # calculation of the angles
                teta1inter = math.atan2((self.mmHeight - matY[cptNbElement]), matX[cptNbElement])
                teta2inter = math.atan2((self.mmHeight - matY[cptNbElement]), (self.mmWidth - matX[cptNbElement]))
                # calculation of the lengths
                L1inter = (self.mmHeight - matY[cptNbElement]) / math.sin(teta1inter)
                L2inter = (self.mmHeight - matY[cptNbElement]) / math.sin(teta2inter)
                matL1.append(L1inter)
                matL2.append(L2inter)
                cptNbElement += 1

        print("length matL1 : ", len(matL1), "length matL2 : ", len(matL2))

        #SEND the number of revolutions
        print(matL1)
        print(matL2)
        plt.plot(matX, matY, 'go-')
        plt.show()

        #
        #
        #algorithm for steps and sending instructions to the motors
        #
        cptNbElement = 1
        L1_real = matL1[0]
        L2_real = matL2[0]
        element_L1 = []
        element_L2 = []
        element_L1.append(L1_real)
        element_L2.append(L2_real)

        print ("mmPerStep : ",  self.mmPerStep)

        if len(matL1) == len(matL2):
            while cptNbElement < len(matL1):
                #MOTOR 1
                # test the difference between the current element and real length
                # if the difference is bigger than a revolution of a motor step, the motor has to turn
                if abs(L1_real - matL1[cptNbElement] )> self.mmPerStep :
                    time.sleep(delay_motor)
                    if L1_real > matL1[cptNbElement]:
                        L1_real -= self.looknbStep (L1_real,matL1[cptNbElement])*self.mmPerStep
                        print("0,",-self.looknbStep (L1_real,matL1[cptNbElement]))
                        #a.write("0,-self.looknbStep (L1_real,matL1[cptNbElement])".encode())
                    else:
                        L1_real += self.looknbStep (L1_real,matL1[cptNbElement])*self.mmPerStep
                        #print("DIRECTION + ELEMENT 1 ", L1_real)
                        print("0,",self.looknbStep(L1_real, matL1[cptNbElement]))
                        element_L1.append(L1_real)
                        #a.write("0,self.looknbStep (L1_real,matL1[cptNbElement])".encode())

                #MOTOR 2
                if abs(L2_real - matL2[cptNbElement] )> self.mmPerStep :
                    time.sleep(delay_motor)
                    if L2_real > matL2[cptNbElement]:
                        L2_real -= self.looknbStep (L2_real,matL2[cptNbElement])*self.mmPerStep
                        #print("DIRECTION - ELEMENT 2 ", L2_real)
                        print("1,",-self.looknbStep (L2_real,matL2[cptNbElement]))
                        #a.write("1,-self.looknbStep (L2_real,matL2[cptNbElement])".encode())
                    else:
                        L2_real += self.looknbStep (L2_real,matL2[cptNbElement])*self.mmPerStep
                        #print("DIRECTION + ELEMENT 2 ", L2_real)
                        print("1,",self.looknbStep(L2_real, matL2[cptNbElement]))
                        #a.write("1,self.looknbStep (L2_real,matL2[cptNbElement])".encode())
                    element_L2.append(L2_real)
                cptNbElement += 1

        print("element L1", element_L1)
        print("element L2", element_L2)
        self.currentX = targetX
        self.currentY = targetY
        print("current x : ", self.currentX, " and current Y : ", self.currentY)
        matX.clear()
        matY.clear()
        matL1.clear()
        matL2.clear()
        element_L1.clear()
        element_L2.clear()