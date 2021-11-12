import numpy as np
import sys
from croblink import *
from math import *
import xml.etree.ElementTree as ET

CELLROWS=7
CELLCOLS=14

class MyRob(CRobLinkAngs):
    positionInitX = 0.0
    positionInitY = 0.0
    moving = False

    mapaC2 = np.zeros(shape=(27,55))

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
                print(self.rob_name + " exiting")
                quit()

            if state == 'stop' and self.measures.start:
                state = stopped_state

            if state != 'stop' and self.measures.stop:
                stopped_state = state
                state = 'stop'
            
            if state == 'run':
                if self.measures.visitingLed==True:
                    state='wait'
                if self.measures.ground==0:
                    self.setVisitingLed(True)

                self.positionInitY = self.measures.y
                self.positionInitX = self.measures.x 
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
        # L = lin + rot/2
        # rot = k(m -2,17) -> parede da direita  


        # bussola: 0 -> direita, 90 -> cima, esquerda -> 180,baixo ->-90  

        # frente,direita,esquerda,atras    1 -> parede 0 -> espace
        walls = self.watch_walls()
        print("x:",self.measures.x,"y:",self.measures.y)
        print("objetivo x:",self.positionInitX,"objetivo y:",self.positionInitY,"\n")
        print("moving",self.moving)
        print(walls,"\n<<<<<<<<<<<<<<<<<<<<<")

        # to do :
        # pegar na posicao inicial e somar a variavel e incrementar 0.2 na variavel
        # em vez de somar 0.2 a posicao q mando

        espace = 0
        for i in walls:
            if i == 0:
                espace = espace+1
        if espace == 1:
            if walls[0] == 0:
                if self.rotateUp() and not self.moving:
                    self.positionInitY = self.positionInitY + 2
                    self.moveY()
                else:
                    self.rotateUp()
            elif walls[1] == 0:
                if self.rotateRight() and not self.moving:
                    self.positionInitX = self.positionInitX + 2
                    self.moveX()
                else:
                    self.rotateRight()
            elif walls [2] == 0:
                if self.rotateLeft() and not self.moving:
                    self.positionInitX = self.positionInitX - 2
                    self.moveX()
                else:
                    self.rotateLeft()
            elif walls[3] == 0:
                if self.rotateDown() and not self.moving:
                    self.positionInitY = self.positionInitY - 2
                    self.moveY()
                else:
                    self.rotateDown()
        elif espace > 1 and not self.moveX() and not self.moving:
                    # cima,direita,esquerda,baixo    1 -> parede 0 -> espace
            if walls[0] == 0:
                if self.rotateUp() and not self.moving:
                    self.positionInitY = self.positionInitY + 2
                    self.moveY()
                else:
                    self.rotateUp()
            elif walls[0] == 1 and walls[1] == 0:
                if self.rotateRight() and not self.moving:
                    self.positionInitX = self.positionInitX + 2
                    self.moveX()
                else:
                    self.rotateRight()
            elif walls[0] == 1 and walls[1] == 1 and walls[2] == 0:
                if self.rotateLeft() and not self.moving:
                    self.positionInitX = self.positionInitX - 2
                    self.moveX()
                else:
                    self.rotateLeft()
        else:
            print("esperar q anda")


    # rodar ------------------
    def rotateDown(self):
        # -90 graus
        if self.measures.compass < -95.0 or self.measures.compass > -85.0:
            self.driveMotors(-0.01,+0.01)
            return False
        else:
            return True
    def rotateLeft(self):
        # 180 graus
        if self.measures.compass > 185.0 or self.measures.compass < 175.0:
            self.driveMotors(-0.01,+0.01)
            return False
        else:
            return True
    def rotateUp(self):
        # 90 graus
        if self.measures.compass > 95.0 or self.measures.compass < 85.0:
            self.driveMotors(-0.01,+0.01)
            return False
        else:
            return True
    def rotateRight(self):
        # 0 graus
        if self.measures.compass < -5.0 or self.measures.compass > 5.0:
            self.driveMotors(-0.01,+0.01)
            return False
        else:
            return True


    # andar -----------------
    def moveX(self):
        if("{:.1f}".format(self.measures.x) != "{:.1f}".format(self.positionInitX)):
            self.driveMotors(0.01,0.01)
            self.moving = True
        if("{:.1f}".format(self.measures.x) == "{:.1f}".format(self.positionInitX)):
            self.moving = False

    def moveY(self):
        if("{:.1f}".format(self.measures.y) != "{:.1f}".format(self.positionInitY)):
            self.driveMotors(0.01,0.01)
            self.moving = True
        if("{:.1f}".format(self.measures.y) == "{:.1f}".format(self.positionInitY)):
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
            
        elif self.measures.compass > -80 and self.measures.compass < -100:
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

        return positions 
    
    def write_mapaC2(self):
        file = 'test.txt'
        np.savetxt(file,self.mapaC2.astype(int), fmt='%i',delimiter='')

        fin = open(file, "rt")
        #read file contents to string
        data = fin.read()
        # 'x' -> livre,
        # '|' -> parede vertical, 
        # '-' -> parede horizontal, 
        # ' '-> desconhecido
        # 1 -> 'x' 
        # 2 -> '|' 
        # 3 -> '-'
        data = data.replace('0', ' ')
        data = data.replace('1', 'x')
        data = data.replace('2', '|')
        data = data.replace('3', '-')

        #close the input file
        fin.close()
        #open the input file in write mode
        fin = open(file, "wt")
        #overrite the input file with the resulting data
        fin.write(data)
        #close the file
        fin.close()

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

for i in range(1, len(sys.argv),2):
    if (sys.argv[i] == "--host" or sys.argv[i] == "-h") and i != len(sys.argv) - 1:
        host = sys.argv[i + 1]
    elif (sys.argv[i] == "--pos" or sys.argv[i] == "-p") and i != len(sys.argv) - 1:
        pos = int(sys.argv[i + 1])
    elif (sys.argv[i] == "--robname" or sys.argv[i] == "-p") and i != len(sys.argv) - 1:
        rob_name = sys.argv[i + 1]
    elif (sys.argv[i] == "--map" or sys.argv[i] == "-m") and i != len(sys.argv) - 1:
        mapc = Map(sys.argv[i + 1])
    else:
        print("Unkown argument", sys.argv[i])
        quit()

if __name__ == '__main__':
    rob=MyRob(rob_name,pos,[0.0,90.0,-90.0,180.0],host)
    if mapc != None:
        rob.setMap(mapc.labMap)
        rob.printMap()
    
    rob.run()
