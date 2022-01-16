#import numpy as np
import operator
import sys
from croblink import *
from math import *
import xml.etree.ElementTree as ET
from astar import *

CELLROWS=7
CELLCOLS=14

class MyRob(CRobLinkAngs):
    tempx = 0
    tempy = 0
    positionInitX = 0.0
    positionInitY = 0.0
    moving = False
    visited_cells = {}         #  (coordx,coordy) : "string de "o" e "c"   <- o = open , c = close
    mapping = {(28,14): 'I'} # 28,14 -> meio do mapa (27.5,13.5)
    x_for_mapping = 0
    y_for_mapping = 0
    came_from = ""
    walls_spotted = set()
    beacons_cells = {}
    path = []
    finish_m = False
    beacons_xy= []
    fake_gps_x = 0.0
    fake_gps_y = 0.0
    currentwheels = (0,0)
    lastwheels = (0,0)
    left_wheel = 0
    right_wheel = 0
    last_x = 0
    last_y = 0

    def __init__(self, rob_name, rob_id, angles, host):
        CRobLinkAngs.__init__(self, rob_name, rob_id, angles, host)

    # In this map the center of cell (i,j), (i in 0..6, j in 0..13) is mapped to labMap[i*2][j*2].
    # to know if there is a wall on top of cell(i,j) (i in 0..5), check if the value of labMap[i*2+1][j*2] is space and not
    def setMap(self, labMap):
        self.labMap = labMap

    def printMap(self):
        for l in reversed(self.labMap):
            print(''.join([str(l) for l in l]))

    def run(self):
        if self.status != 0:
            print("Connection refused or error")
            quit()

        state = 'stop'
        stopped_state = 'run'

        while True:
            self.readSensors()

            if self.measures.endLed:
                print(self.robName + " exiting")
                quit()

            if state == 'stop' and self.measures.start:
                self.tempx = self.measures.x
                self.tempy = self.measures.y
                self.positionInitY = 0.0
                self.positionInitX = 0.0
                self.moving = False
                state = stopped_state

                a = self.watch_walls()                  # para adicionar a posicao inicial as visitadas
                key = (self.x_for_mapping,self.y_for_mapping)
                espace = ""
                for i in a:
                    if i == 0:
                        espace += "o"           #o = open
                    else:
                        espace += "c"           #c = close
                if key not in self.visited_cells:
                    self.visited_cells[key] = espace

            if state != 'stop' and self.measures.stop:
                stopped_state = state
                state = 'stop'
            
            if state == 'run':
                if self.measures.visitingLed==True:
                    state='wait'
                if self.measures.ground==0:
                    self.setVisitingLed(True)
                self.wander()
            elif state=='wait':
                self.setReturningLed(True)
                if self.measures.visitingLed==True:
                    self.setVisitingLed(False)
                if self.measures.returningLed==True:
                    state='return'
                self.driveMotors(0.0,0.0)
            elif state=='return':
                if self.measures.visitingLed==True:
                    self.setVisitingLed(False)
                if self.measures.returningLed==True:
                    self.setReturningLed(False)
                self.wander()
            

    def wander(self):
        center_id = 0
        left_id = 1
        right_id = 2
        back_id = 3
        # mapa 55 colunas 27 linhas
        # andar 1 linha / coluna  = 1 diametro (temos que andar 2 diametros para ficar no centro da celula)
        # 'x' -> livre, '|' -> parede vertical, '-' -> parede horizontal, ' '-> desconhecido

        # bussola: 0 -> direita, 90 -> cima, esquerda -> 180,baixo ->-90  
        walls = self.watch_walls()
        espace = 0
        for i in walls:
            if i == 0:
                espace = espace+1

        #print(walls,"\n<<<<<<<<<<<<<<<<<<<<<")
        #print("atual mapping",(self.x_for_mapping,self.y_for_mapping))
        # print("x:",self.measures.x,"y:",self.measures.y)
        #print("objetivo x:",self.positionInitX,"objetivo y:",self.positionInitY)
        #print("path:",self.path)
        #print("beacons:",self.beacons_cells)
        #print("fake gps:",self.fake_gps_x,self.fake_gps_y)
        #print("real gps:",self.measures.x-self.tempx,self.measures.y-self.tempy)
        #print("sensores:",self.measures.irSensor[center_id],"back:",self.measures.irSensor[back_id],"left:",self.measures.irSensor[left_id],"right",self.measures.irSensor[right_id])
        #print("visited:",self.visited_cells)
        #self.correct_Pos()
        


        # nao é preciso fazer o caminho optimo mas tem q ir para o 0,0 no fim

        #print("BEACONNNNNNNNNNNNNNNNNNNNNNNNNN",self.measures.beacon,"|||||",self.nBeacons)
        #print("GROUNDDDDDDDDDDDDDDDDDDDDDDD",self.measures.ground)
        if self.measures.time == 5999: 
            self.finish_m = True
        
        if self.measures.ground > -1:
            if self.measures.ground not in self.beacons_cells.values():
                self.beacons_cells[(self.x_for_mapping,self.y_for_mapping)] = self.measures.ground

        # cima,direita,esquerda,baixo    1 -> parede 0 -> espace
        if not self.moving:
            key = (self.x_for_mapping,self.y_for_mapping)       # atual
            key1 = (self.x_for_mapping,(self.y_for_mapping +2)) # cima key1 
            key2 = ((self.x_for_mapping+2),self.y_for_mapping)  # direita key 2
            key3 = ((self.x_for_mapping-2),self.y_for_mapping)  # esquerda key3
            key4 = (self.x_for_mapping,(self.y_for_mapping-2))  # baixo key4
            if key in self.visited_cells and self.finish_m==False:
                if self.visited_cells.get(key) == "cccc" and len(self.path) == 0:
                    visited = []
                    wallis = set()
                    for x,y in self.walls_spotted:
                        chave = (x,y)
                        wallis.add(chave)
                    last_avaliable = []
                    for x,y in self.visited_cells:
                         chave = (x,y)
                         visited.append(chave)
                         for word in self.visited_cells.get(chave):
                             for letra in word:
                                 if letra == "o":
                                     last_avaliable = chave
                    if last_avaliable == []:
                        self.finish_m = True
                    #print("key:",key,"goal:",last_avaliable)
                    if last_avaliable != []:
                        self.path = astar(key,last_avaliable,visited,self.walls_spotted)
                        #print("Rato burroooooooooooooOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOO")
                        #print("------------------",self.path)
                if len(self.path) != 0:
                    next = self.path[len(self.path)-1]
                    sentidoX = self.x_for_mapping - next[0]    # +esquerda - direita
                    sentidoY = self.y_for_mapping - next[1]    # +baixo - cima

                    lastx = next[0]
                    lasty = next[1]

                    if sentidoX == 0:
                        if sentidoY > 0:
                            if self.rotateDown():
                                value = ""
                                value += self.visited_cells.get(key)[0] + self.visited_cells.get(key)[1]+ self.visited_cells.get(key)[2]+ "c" 
                                self.visited_cells[key] = value
                                self.positionInitY = self.positionInitY - sentidoY
                                self.y_for_mapping = self.y_for_mapping - 2
                                self.came_from = "down"
                                self.path.remove((lastx,lasty))
                                self.moveY()
                            else:
                                self.rotateDown()
                        if sentidoY < 0:
                            if self.rotateUp():
                                value = ""
                                value +=  "c" +self.visited_cells.get(key)[1]+ self.visited_cells.get(key)[2] + self.visited_cells.get(key)[3]
                                self.visited_cells[key] = value
                                self.positionInitY = self.positionInitY - sentidoY
                                self.y_for_mapping = self.y_for_mapping + 2
                                self.came_from = "up"
                                self.path.remove((lastx,lasty))
                                self.moveY()
                            else:
                                self.rotateUp()
                    if sentidoY == 0:
                        if sentidoX > 0:
                            if self.rotateLeft():
                                value = ""
                                value += self.visited_cells.get(key)[0] + self.visited_cells.get(key)[1]+ "c" + self.visited_cells.get(key)[3]
                                self.visited_cells[key] = value
                                self.positionInitX = self.positionInitX - sentidoX
                                self.x_for_mapping = self.x_for_mapping - 2
                                self.came_from = "left"
                                self.path.remove((lastx,lasty))
                                self.moveX()
                            else:
                                self.rotateLeft()
                        if sentidoX < 0:
                            if self.rotateRight():
                                value = ""
                                value += self.visited_cells.get(key)[0]+ "c" + self.visited_cells.get(key)[2] + self.visited_cells.get(key)[3]
                                self.visited_cells[key] = value
                                self.positionInitX = self.positionInitX - sentidoX
                                self.x_for_mapping = self.x_for_mapping + 2
                                self.came_from = "right"
                                self.path.remove((lastx,lasty))
                                self.moveX()
                            else:
                                self.rotateRight()

                elif self.visited_cells.get(key)[0] == 'o':
                        if self.rotateUp():
                            value = ""
                            value +=  "c" +self.visited_cells.get(key)[1]+ self.visited_cells.get(key)[2] + self.visited_cells.get(key)[3]
                            self.visited_cells[key] = value
                            self.positionInitY = self.positionInitY + 2
                            self.y_for_mapping = self.y_for_mapping + 2
                            self.came_from = "up"
                            self.moveY()
                        else:
                            self.rotateUp()
                elif self.visited_cells.get(key)[1] == 'o':
                        if self.rotateRight():
                            value = ""
                            value += self.visited_cells.get(key)[0]+ "c" + self.visited_cells.get(key)[2] + self.visited_cells.get(key)[3]
                            self.visited_cells[key] = value
                            self.positionInitX = self.positionInitX + 2
                            self.x_for_mapping = self.x_for_mapping + 2
                            self.came_from = "right"
                            self.moveX()
                        else:
                            self.rotateRight()
                elif self.visited_cells.get(key)[2] == 'o':
                        if self.rotateLeft():
                            value = ""
                            value += self.visited_cells.get(key)[0] + self.visited_cells.get(key)[1]+ "c" + self.visited_cells.get(key)[3]
                            self.visited_cells[key] = value
                            self.positionInitX = self.positionInitX - 2
                            self.x_for_mapping = self.x_for_mapping - 2
                            self.came_from = "left"
                            self.moveX()
                        else:
                            self.rotateLeft()
                elif self.visited_cells.get(key)[3] == 'o':
                        if self.rotateDown():
                            value = ""
                            value += self.visited_cells.get(key)[0] + self.visited_cells.get(key)[1]+ self.visited_cells.get(key)[2]+ "c" 
                            self.visited_cells[key] = value
                            self.positionInitY = self.positionInitY - 2
                            self.y_for_mapping = self.y_for_mapping - 2
                            self.came_from = "down"
                            self.moveY()
                        else:
                            self.rotateDown()
            elif key in self.visited_cells and self.finish_m == True:
                if len(self.path) == 0:
                    visited = []
                    for x,y in self.visited_cells:
                         chave = (x,y)
                         visited.append(chave)
                    start = (0,0)
                    for x,y in self.beacons_cells:
                        chave = (x,y)
                        self.beacons_xy.append(chave)
                    
                    #self.path.extend(astar(self.beacons_xy[1],self.beacons_xy[0],visited,self.walls_spotted))
                    #self.path.extend(astar(self.beacons_xy[2],self.beacons_xy[1],visited,self.walls_spotted))
                    #self.path.extend(astar(self.beacons_xy[0],self.beacons_xy[2],visited,self.walls_spotted))
                    i = 0
                    for a in (0,(len(self.beacons_xy)-1)):
                        self.path.extend(astar(self.beacons_xy[i+1],self.beacons_xy[i],visited,self.walls_spotted))
                        i= i+1
                    
                    self.path.extend(astar(self.beacons_xy[0],self.beacons_xy[i],visited,self.walls_spotted))                  
                    self.planning_output()

                    self.finish()
                    quit()
            else:
                if espace == 1:
                    if walls[0] == 0:
                        if self.rotateUp():
                            #print("watzareeeeeeee\nwatzare\nwatzare\n")
                            self.positionInitY = self.positionInitY + 2
                            self.y_for_mapping = self.y_for_mapping + 2
                            self.came_from = "up"
                            self.moveY()
                        else:
                            self.rotateUp()
                    elif walls[1] == 0:
                        if self.rotateRight():
                            self.positionInitX = self.positionInitX + 2
                            self.x_for_mapping = self.x_for_mapping + 2
                            self.came_from = "right"
                            self.moveX()
                        else:
                            self.rotateRight()
                    elif walls [2] == 0:
                        if self.rotateLeft():
                            self.positionInitX = self.positionInitX - 2
                            self.x_for_mapping = self.x_for_mapping - 2
                            self.came_from = "left"
                            self.moveX()
                        else:
                            self.rotateLeft()
                    elif walls[3] == 0:
                        if self.rotateDown():
                            self.positionInitY = self.positionInitY - 2
                            self.y_for_mapping = self.y_for_mapping - 2
                            self.came_from = "down"
                            self.moveY()
                        else:
                            self.rotateDown()
        else:
            # bussola: 0 -> direita, 90 -> cima, esquerda -> 180,baixo ->-90  
            #print("esperar q anda")
            #not self.moving and self.correct_Pos()
            if (self.measures.compass > 80 and self.measures.compass < 100) or (self.measures.compass > -100 and self.measures.compass < -80):
                self.moveY()
            else:
                self.moveX()



    # rodar ------------------
    def rotateDown(self):
        # -90 graus
        if self.measures.compass < -95.0 or self.measures.compass > -85.0:
            #print("rotate down")
            if self.measures.compass > -90 and self.measures.compass < 90:
                self.driveMotors(+0.05,-0.05)
            if self.measures.compass >= 90 or self.measures.compass <= -90:
                self.driveMotors(-0.05,+0.05)
            return False
        else:
            self.left_wheel = 0
            self.right_wheel = 0
            self.lastwheels = (0,0)
            return True
    def rotateLeft(self):
        # 180 graus e -180
        if self.measures.compass > -175 and self.measures.compass < 175.0:
            #print("rotate left")
            if self.measures.compass <= 0:
                self.driveMotors(+0.05,-0.05)
            if self.measures.compass > 0:
                self.driveMotors(-0.05,+0.05)
            return False
        else:
            self.left_wheel = 0
            self.right_wheel = 0
            self.lastwheels = (0,0)
            return True
    def rotateUp(self):
        #print("rotate up\n")
        # 90 graus
        if self.measures.compass > 95.0 or self.measures.compass < 85.0:
            #print("rotate up")
            if self.measures.compass > -90 and self.measures.compass < 90:
                self.driveMotors(-0.05,+0.05)
            if self.measures.compass >= 90 or self.measures.compass <= -90:
                self.driveMotors(+0.05,-0.05)
            return False
        else:
            self.left_wheel = 0
            self.right_wheel = 0
            self.lastwheels = (0,0)
            return True
    def rotateRight(self):
        # 0 graus
        if self.measures.compass < -5.0 or self.measures.compass > 5.0:
            #print("rotate right")
            if self.measures.compass >= 0:
                self.driveMotors(+0.05,-0.05)
            if self.measures.compass < 0:
                self.driveMotors(-0.05,+0.05)
            return False
        else:
            self.left_wheel = 0
            self.right_wheel = 0
            self.lastwheels = (0,0)
            return True


    # andar -----------------
    def moveX(self):
        if(abs(self.positionInitX-self.fake_gps_x) > 0.1):
            if self.measures.compass > -10.0 and self.measures.compass < 10:
                self.align(0.05,self.measures.compass,0.01,self.correctCompass())
            else:
                self.align(0.05,self.measures.compass,0.01,self.correctCompass())
            self.moving = True
        if(abs(self.positionInitX-self.fake_gps_x) <= 0.1):
            #print("stop moving X\n")
            self.driveMotors(0.00,0.00)
            self.correct_Pos()
            walls = self.watch_walls()
            key = (self.x_for_mapping,self.y_for_mapping)
            espace = ""
            for i in walls:
                if i == 0:
                    espace += "o"           #o = open
                else:
                    espace += "c"           #c = close
            # cima,direita,esquerda,baixo
            tmp = ""
            if self.came_from == "left":
                tmp += espace[0] + "c" + espace[2] + espace[3]
            elif self.came_from == "right":
                tmp += espace[0] + espace[1] + "c" + espace[3]
            espace = tmp
            if key not in self.visited_cells:
                self.visited_cells[key] = espace

            if walls[0] == 1:
                self.insert_mapping((28+self.x_for_mapping,14-self.y_for_mapping-1),'-')
                self.walls_spotted.add((self.x_for_mapping,self.y_for_mapping+1))
            else:
                self.insert_mapping((28+self.x_for_mapping,14-self.y_for_mapping-1),'X')
            if walls[1] == 1:
                self.insert_mapping((28+self.x_for_mapping+1,14-self.y_for_mapping),'|')
                self.walls_spotted.add((self.x_for_mapping+1,self.y_for_mapping))
            else:
                self.insert_mapping((28+self.x_for_mapping+1,14-self.y_for_mapping),'X')
            if walls[2] == 1:
                self.insert_mapping((28+self.x_for_mapping-1,14-self.y_for_mapping),'|')
                self.walls_spotted.add((self.x_for_mapping-1,self.y_for_mapping))
            else:
                self.insert_mapping((28+self.x_for_mapping-1,14-self.y_for_mapping),'X')
            if walls[3] == 1:
                self.insert_mapping((28+self.x_for_mapping,14-self.y_for_mapping+1),'-')
                self.walls_spotted.add((self.x_for_mapping,self.y_for_mapping-1))
            else:
                self.insert_mapping((28+self.x_for_mapping,14-self.y_for_mapping+1),'X')
            self.insert_mapping((28+self.x_for_mapping,14-self.y_for_mapping),'X')
            self.moving = False
            

    def moveY(self):
        # bussola: 0 -> direita, 90 -> cima, esquerda -> 180,baixo ->-90 
        if(abs(self.positionInitY-self.fake_gps_y) > 0.1):
            if self.measures.compass > 80.0 and self.measures.compass < 100.0:
                self.align(0.05,self.measures.compass,0.01,self.correctCompass())
            else:
                self.align(0.05,self.measures.compass,0.01,self.correctCompass())
            #self.driveMotors(0.10,0.10)

            self.moving = True
        if(abs(self.positionInitY-self.fake_gps_y) <= 0.1):
            self.driveMotors(0.00,0.00)
            self.correct_Pos()
            walls = self.watch_walls()
            key = (self.x_for_mapping,self.y_for_mapping)
            espace = "" 
            for i in walls:
                if i == 0:
                    espace += "o"           #o = open
                else:
                    espace += "c"           #c = close

            # cima,direita,esquerda,baixo
            tmp = ""
            if self.came_from == "up":
                tmp += espace[0] +espace[1] + espace[2] + "c"
            elif self.came_from == "down":
                tmp += "c" + espace[1] + espace[2] + espace[3] 
            espace = tmp
            if key not in self.visited_cells:
                self.visited_cells[key] = espace
            if walls[0] == 1:
                self.insert_mapping((28+self.x_for_mapping,14-self.y_for_mapping-1),'-')
                self.walls_spotted.add((self.x_for_mapping,self.y_for_mapping+1))
            else:
                self.insert_mapping((28+self.x_for_mapping,14-self.y_for_mapping-1),'X')
            if walls[1] == 1:
                self.insert_mapping((28+self.x_for_mapping+1,14-self.y_for_mapping),'|')
                self.walls_spotted.add((self.x_for_mapping+1,self.y_for_mapping))
            else:
                self.insert_mapping((28+self.x_for_mapping+1,14-self.y_for_mapping),'X')
            if walls[2] == 1:
                self.insert_mapping((28+self.x_for_mapping-1,14-self.y_for_mapping),'|')
                self.walls_spotted.add((self.x_for_mapping-1,self.y_for_mapping))
            else:
                self.insert_mapping((28+self.x_for_mapping-1,14-self.y_for_mapping),'X')
            if walls[3] == 1:
                self.insert_mapping((28+self.x_for_mapping,14-self.y_for_mapping+1),'-')
                self.walls_spotted.add((self.x_for_mapping,self.y_for_mapping-1))
            else:
                self.insert_mapping((28+self.x_for_mapping,14-self.y_for_mapping+1),'X')
            self.insert_mapping((28+self.x_for_mapping,14-self.y_for_mapping),'X')
            self.moving = False

    # -------- corrigir a posicao para o meio das celulas ---------------
    # 0.8 centro             2 de largura 0,2 parede     celula sem parede 2-0.4 = 1.6
    def correct_Pos(self):
        center_id = 0
        left_id = 1
        right_id = 2
        back_id = 3

        walls = self.watch_walls()
        # bussola: 0 -> direita, 90 -> cima, esquerda -> 180,baixo ->-90 
        # walls  [cima,direita,esquerda,baixo]

        #print("Correcting\nCorrecting\nCorrecting\nCorrecting\nCorrecting\nCorrecting\nCorrecting\nCorrecting\n")
        #print("came from :",self.came_from)
        #print("walls:",walls)

        if self.came_from == "up":
            #print("up\nup\nup")
            #explain contas com os calculos dos sensores 
            if walls[0] == 1:
                wall_pos = self.positionInitY + 1
                self.last_y = wall_pos - self.sensor_calculs(center_id)-0.5
            if walls[1] == 1:
                wall_pos = self.positionInitX + 1
                self.last_x = wall_pos - self.sensor_calculs(right_id)-0.5
            if walls[2] == 1:
                wall_pos = self.positionInitX - 1
                self.last_x = wall_pos + self.sensor_calculs(left_id)+0.5
        if self.came_from == "down":
            #print("down\ndown\ndown")
            if walls[1] == 1:
                wall_pos = self.positionInitX + 1
                self.last_x = wall_pos - self.sensor_calculs(left_id)-0.5
            if walls[2] == 1:
                wall_pos = self.positionInitX - 1
                self.last_x = wall_pos + self.sensor_calculs(right_id)+0.5
            if walls[3] == 1:
                wall_pos = self.positionInitY - 1
                self.last_y = wall_pos + self.sensor_calculs(center_id)+0.5
        if self.came_from == "right":
            #print("right\nright\nright")
            if walls[0] == 1:
                wall_pos = self.positionInitY + 1
                self.last_y = wall_pos - self.sensor_calculs(left_id)-0.5
            if walls[1] == 1:
                wall_pos = self.positionInitX + 1
                self.last_x = wall_pos - self.sensor_calculs(center_id)-0.5
            if walls[3] == 1:
                #print("entrei\nentrei\nentrei\nentrei\nentrei\nentrei\n")
                wall_pos = self.positionInitY - 1
                self.last_y = wall_pos + self.sensor_calculs(right_id)+0.5
        if self.came_from == "left":
            #print("left\nleft\nleft")
            # walls  [cima,direita,esquerda,baixo]
            if walls[0] == 1:
                wall_pos = self.positionInitY + 1
                self.last_y = wall_pos - self.sensor_calculs(right_id)-0.5
            if walls[2] == 1:
                wall_pos = self.positionInitX - 1
                self.last_x = wall_pos + self.sensor_calculs(center_id)+0.5
            if walls[3] == 1:
                wall_pos = self.positionInitY - 1
                self.last_y = wall_pos + self.sensor_calculs(left_id)+0.5

    # --------- new gps------
    def fake_gps(self, x,y):

        self.lastwheels = self.currentwheels
        self.currentwheels = ( (self.left_wheel + self.lastwheels[0]) / 2 , (self.right_wheel + self.lastwheels[1]) / 2 )

        if self.last_x == 0 and self.last_y == 0:
            lin = (self.currentwheels[0]+self.currentwheels[1])/2 /2
        else:
            lin = (self.currentwheels[0]+self.currentwheels[1])/2

        bussola = self.correctCompass()

        self.fake_gps_x = x + lin *cos(radians(bussola))
        self.fake_gps_y = y + lin *sin(radians(bussola))

        self.last_x = self.fake_gps_x
        self.last_y = self.fake_gps_y


    def correctCompass(self):
        if -15 < self.measures.compass < 15:
            return 0
        elif 75 < self.measures.compass< 105:
            return 90
        elif -105 < self.measures.compass <-75:
            return -90
        elif self.measures.compass <= -170 or self.measures.compass >= 170:  
            return 180 * self.measures.compass / abs(self.measures.compass)

        return self.measures.compass

    # identificar paredes ----------------
    def watch_walls(self):
        center_id = 0
        left_id = 1
        right_id = 2
        back_id = 3        
        positions=[0,0,0,0]   # cima,direita,esquerda,baixo
        # parede -> 1 free -> 0
        
        # bussola: 0 -> direita, 90 -> cima, esquerda -> 180,baixo ->-90  
        if self.measures.compass > -10.0 and self.measures.compass < 10:
            if  self.measures.irSensor[center_id] > 1.00:
                positions[1]= 1
            else:
                positions[1] = 0

            if  self.measures.irSensor[left_id] > 1.00:
                positions[0]= 1
            else:
                positions[0] = 0

            if  self.measures.irSensor[right_id] > 1.00:
                positions[3]= 1
            else:
                positions[3] = 0

            if  self.measures.irSensor[back_id] > 1.00:
                positions[2]= 1
            else:
                positions[2] = 0

        elif self.measures.compass > 80.0 and self.measures.compass < 100.0:
            if  self.measures.irSensor[center_id] > 1.00:
                positions[0]= 1
            else:
                positions[0] = 0

            if  self.measures.irSensor[left_id] > 1.00:
                positions[2]= 1
            else:
                positions[2] = 0

            if  self.measures.irSensor[right_id] > 1.00:
                positions[1]= 1
            else:
                positions[1] = 0

            if  self.measures.irSensor[back_id] > 1.00:
                positions[3]= 1
            else:
                positions[3] = 0
        
        elif self.measures.compass > 170.0 or self.measures.compass < -170.0:
            if  self.measures.irSensor[center_id] > 1.00:
                positions[2]= 1
            else:
                positions[2] = 0

            if  self.measures.irSensor[left_id] > 1.00:
                positions[3]= 1
            else:
                positions[3] = 0

            if  self.measures.irSensor[right_id] > 1.00:
                positions[0]= 1
            else:
                positions[0] = 0

            if  self.measures.irSensor[back_id] > 1.00:
                positions[1]= 1
            else:
                positions[1] = 0
            
        elif self.measures.compass < -80 and self.measures.compass > -100:
            if  self.measures.irSensor[center_id] > 1.00:
                positions[3]= 1
            else:
                positions[3] = 0

            if  self.measures.irSensor[left_id] > 1.00:
                positions[1]= 1
            else:
                positions[1] = 0

            if  self.measures.irSensor[right_id] > 1.00:
                positions[2]= 1
            else:
                positions[2] = 0

            if  self.measures.irSensor[back_id] > 1.00:
                positions[0]= 1
            else:
                positions[0] = 0
            
        self.mapping_output()
        return positions 


    # L = lin + rot/2
    # rot = k(m -2,17) -> parede da direita  
    def align(self, linear, m, k, ref):
        rot = k * (m-ref)

        self.right_wheel = linear - (rot/2)
        self.left_wheel = linear + (rot/2)

        self.fake_gps(self.last_x,self.last_y)
        self.driveMotors(self.left_wheel,self.right_wheel)
        
    
    def sensor_calculs(self, sensor):
        return 1/self.measures.irSensor[sensor]

    def planning_output(self):
        f = open(filep,'w')
        for item in self.path:
            string = str(item)
            string = string.replace('(','')
            string= string.replace(')','')
            string=  string.replace(',','')
            f.write(string+ "\n")
        f.write("0 0")
        f.close()

    def insert_mapping(self, key, symbol):
        if key not in self.mapping or self.mapping.get(key) == 'X':
            self.mapping[key] = str(symbol)

    def mapping_output(self):
        f = open(filem,'w')
        for x in range(1,28):
            for y in range(1,56):
                if(y,x) in self.mapping:
                    f.write(self.mapping.get((y,x)))
                else:
                    f.write(' ')
            f.write('\n')
        f.close()


class Map():
    def __init__(self, filename):
        tree = ET.parse(filename)
        root = tree.getroot()
        
        self.labMap = [[' '] * (CELLCOLS*2-1) for i in range(CELLROWS*2-1) ]
        i=1
        for child in root.iter('Row'):
           line=child.attrib['Pattern']
           row =int(child.attrib['Pos'])
           if row % 2 == 0:  # this line defines vertical lines
               for c in range(len(line)):
                   if (c+1) % 3 == 0:
                       if line[c] == '|':
                           self.labMap[row][(c+1)//3*2-1]='|'
                       else:
                           None
           else:  # this line defines horizontal lines
               for c in range(len(line)):
                   if c % 3 == 0:
                       if line[c] == '-':
                           self.labMap[row][c//3*2]='-'
                       else:
                           None
               
           i=i+1


rob_name = "pClient1"
host = "localhost"
pos = 1
mapc = None
filep = "planning.out"
filem = "mapping.out"

for i in range(1, len(sys.argv),2):
    if (sys.argv[i] == "--host" or sys.argv[i] == "-h") and i != len(sys.argv) - 1:
        host = sys.argv[i + 1]
    elif (sys.argv[i] == "--pos" or sys.argv[i] == "-p") and i != len(sys.argv) - 1:
        pos = int(sys.argv[i + 1])
    elif (sys.argv[i] == "--robname" or sys.argv[i] == "-p") and i != len(sys.argv) - 1:
        rob_name = sys.argv[i + 1]
    elif (sys.argv[i] == "--map" or sys.argv[i] == "-m") and i != len(sys.argv) - 1:
        mapc = Map(sys.argv[i + 1])
    elif (sys.argv[i] == "--filep" or sys.argv[i] == "-fp") and i != len(sys.argv) - 1:
        filep = sys.argv[i + 1]
    elif (sys.argv[i] == "--filem" or sys.argv[i] == "-fm") and i != len(sys.argv) - 1:
        filem = sys.argv[i + 1]    
    else:
        print("Unkown argument", sys.argv[i])
        quit()

if __name__ == '__main__':
    rob=MyRob(rob_name,pos,[0.0,90.0,-90.0,180.0],host)
    if mapc != None:
        rob.setMap(mapc.labMap)
        rob.printMap()
    
    rob.run()
