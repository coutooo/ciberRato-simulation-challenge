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

    #mapaC2 = np.zeros(shape=(27,55))

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
                self.positionInitY = self.measures.y
                self.positionInitX = self.measures.x
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
        # print("atual",(self.x_for_mapping,self.y_for_mapping))
        # print("x:",self.measures.x,"y:",self.measures.y)
        # print("objetivo x:",self.positionInitX,"objetivo y:",self.positionInitY)
        #print("path:",self.path)
        #print("beacons:",self.beacons_cells)


        #print("BEACONNNNNNNNNNNNNNNNNNNNNNNNNN",self.measures.beacon,"|||||",self.nBeacons)
        #print("GROUNDDDDDDDDDDDDDDDDDDDDDDD",self.measures.ground)
        
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
                    
                    self.path.extend(astar(self.beacons_xy[1],self.beacons_xy[0],visited,self.walls_spotted))
                    self.path.extend(astar(self.beacons_xy[2],self.beacons_xy[1],visited,self.walls_spotted))
                    self.path.extend(astar(self.beacons_xy[0],self.beacons_xy[2],visited,self.walls_spotted))
                    #print(self.path)
                    # for a in self.beacons_xy:
                    #     self.path.extend(astar(key,a,visited,self.walls_spotted))
                    # print(self.path)
                    self.planning_output()

                    self.finish()
                    quit()
            else:
                if espace == 1:
                    if walls[0] == 0:
                        if self.rotateUp():
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
                self.driveMotors(+0.15,-0.15)
            if self.measures.compass >= 90 or self.measures.compass <= -90:
                self.driveMotors(-0.15,+0.15)
            return False
        else:
            return True
    def rotateLeft(self):
        # 180 graus e -180
        if self.measures.compass > -175 and self.measures.compass < 175.0:
            #print("rotate left")
            if self.measures.compass <= 0:
                self.driveMotors(+0.15,-0.15)
            if self.measures.compass > 0:
                self.driveMotors(-0.15,+0.15)
            return False
        else:
            return True
    def rotateUp(self):
        # 90 graus
        if self.measures.compass > 95.0 or self.measures.compass < 85.0:
            #print("rotate up")
            if self.measures.compass > -90 and self.measures.compass < 90:
                self.driveMotors(-0.15,+0.15)
            if self.measures.compass >= 90 or self.measures.compass <= -90:
                self.driveMotors(+0.15,-0.15)
            return False
        else:
            return True
    def rotateRight(self):
        # 0 graus
        if self.measures.compass < -5.0 or self.measures.compass > 5.0:
            #print("rotate right")
            if self.measures.compass >= 0:
                self.driveMotors(+0.15,-0.15)
            if self.measures.compass < 0:
                self.driveMotors(-0.15,+0.15)
            return False
        else:
            return True


    # andar -----------------
    def moveX(self):
        if(abs(round(self.positionInitX,1)-round(self.measures.x,1)) > 0.2):
            if self.measures.compass > -10.0 and self.measures.compass < 10:
                self.align(0.15,self.measures.compass,0.05,0)
            else:
                self.align(0.15,self.measures.compass,0.05,(180 * self.measures.compass / abs(self.measures.compass)))
            self.moving = True
        if(abs(round(self.positionInitX,1)-round(self.measures.x,1)) < 0.2):
            self.driveMotors(0.00,0.00)
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
                self.walls_spotted.add((self.x_for_mapping,self.y_for_mapping+1))
            if walls[1] == 1:
                self.walls_spotted.add((self.x_for_mapping+1,self.y_for_mapping))
            if walls[2] == 1:
                self.walls_spotted.add((self.x_for_mapping-1,self.y_for_mapping))
            if walls[3] == 1:
                self.walls_spotted.add((self.x_for_mapping,self.y_for_mapping-1))
            self.moving = False

    def moveY(self):
        # bussola: 0 -> direita, 90 -> cima, esquerda -> 180,baixo ->-90 
        if(abs(round(self.positionInitY,1)-round(self.measures.y,1)) > 0.2):
            if self.measures.compass > 80.0 and self.measures.compass < 100.0:
                self.align(0.15,self.measures.compass,0.05,90)
            else:
                self.align(0.15,self.measures.compass,0.05,-90)
            #self.driveMotors(0.10,0.10)

            self.moving = True
        if(abs(round(self.positionInitY,1)-round(self.measures.y,1)) < 0.2):
            self.driveMotors(0.00,0.00)
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
                self.walls_spotted.add((self.x_for_mapping,self.y_for_mapping+1))
            if walls[1] == 1:
                self.walls_spotted.add((self.x_for_mapping+1,self.y_for_mapping))
            if walls[2] == 1:
                self.walls_spotted.add((self.x_for_mapping-1,self.y_for_mapping))
            if walls[3] == 1:
                self.walls_spotted.add((self.x_for_mapping,self.y_for_mapping-1))
            self.moving = False

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
            
        #self.mapping_output()
        return positions 


    # L = lin + rot/2
    # rot = k(m -2,17) -> parede da direita  
    def align(self, linear, m, k, ref):
        rot = k * (m-ref)

        right_wheel = linear - (rot/2)
        left_wheel = linear + (rot/2)

        self.driveMotors(left_wheel,right_wheel)
    
    def planning_output(self):
        f = open(file,'w')
        for item in self.path:
            string = str(item)
            string = string.replace('(','')
            string= string.replace(')','')
            string=  string.replace(',','')
            f.write(string+ "\n")
        f.write("0 0")
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
file = "planning.out"

for i in range(1, len(sys.argv),2):
    if (sys.argv[i] == "--host" or sys.argv[i] == "-h") and i != len(sys.argv) - 1:
        host = sys.argv[i + 1]
    elif (sys.argv[i] == "--pos" or sys.argv[i] == "-p") and i != len(sys.argv) - 1:
        pos = int(sys.argv[i + 1])
    elif (sys.argv[i] == "--robname" or sys.argv[i] == "-p") and i != len(sys.argv) - 1:
        rob_name = sys.argv[i + 1]
    elif (sys.argv[i] == "--map" or sys.argv[i] == "-m") and i != len(sys.argv) - 1:
        mapc = Map(sys.argv[i + 1])
    elif (sys.argv[i] == "--file" or sys.argv[i] == "-f") and i != len(sys.argv) - 1:
        file = sys.argv[i + 1]
    else:
        print("Unkown argument", sys.argv[i])
        quit()

if __name__ == '__main__':
    rob=MyRob(rob_name,pos,[0.0,90.0,-90.0,180.0],host)
    if mapc != None:
        rob.setMap(mapc.labMap)
        rob.printMap()
    
    rob.run()
