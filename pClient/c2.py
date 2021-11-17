#import numpy as np
import operator
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
    visited_cells = {}         #  (coordx,coordy) : "string de "o" e "c"   <- o = open , c = close
    mapping = {(28,14): 'I'} # 28,14 -> meio do mapa (27.5,13.5)
    x_for_mapping = 0
    y_for_mapping = 0
    came_from = ""

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
                print(self.rob_name + " exiting")
                quit()

            if state == 'stop' and self.measures.start:
                self.positionInitY = self.measures.y
                self.positionInitX = self.measures.x
                self.moving = False
                state = stopped_state

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
        # L = lin + rot/2
        # rot = k(m -2,17) -> parede da direita  

        # bussola: 0 -> direita, 90 -> cima, esquerda -> 180,baixo ->-90  
        walls = self.watch_walls()
        print("x:",self.measures.x,"y:",self.measures.y)
        print("objetivo x:",self.positionInitX,"objetivo y:",self.positionInitY,"\n")
        print("moving",self.moving)
        print(walls,"\n<<<<<<<<<<<<<<<<<<<<<")
        print(self.visited_cells)
        print(self.mapping)

        key2 = ((self.x_for_mapping+2),self.y_for_mapping)
        if key2 in self.visited_cells:
            print(key2)
            print(self.visited_cells.get(key2)[1])

        espace = 0
        for i in walls:
            if i == 0:
                espace = espace+1


        # cima,direita,esquerda,baixo    1 -> parede 0 -> espace
        if not self.moving:
            key = (self.x_for_mapping,self.y_for_mapping)
            key1 = (self.x_for_mapping,(self.y_for_mapping +2)) # cima
            key2 = ((self.x_for_mapping+2),self.y_for_mapping)  # direita
            key3 = ((self.x_for_mapping-2),self.y_for_mapping)  # esquerda
            key4 = (self.x_for_mapping,(self.y_for_mapping-2))  # baixo
            if key in self.visited_cells:
                if self.visited_cells.get(key) == "cccc":
                    prox_cell={}  #dicionario com as celullas disponiveis e a distancia
                    for k in self.visited_cells:                    # ----------------------
                        o_count = 0
                        for i in self.visited_cells.get(k):
                            if i == "o":
                                o_count += 1
                        if o_count > 0:
                            distance = ((key[0] - k[0])**2 + (key[1] - k[1])**2)**0.5
                            prox_cell[k]=distance
                    sorted_prox_cell = sorted(prox_cell.items(), key=operator.itemgetter(1))
                    
                    print("ir para celula mais proxima")
                    destino_x = sorted_prox_cell[0][0][0]
                    destino_y = sorted_prox_cell[0][0][1]
                    
                    #print(key)
                    #print(destino_x,destino_y)
                    caminho = (destino_x-key[0],destino_y-key[1])
                    mod_caminho = (caminho[0]**2+caminho[1]**2)**0.5
                    #print(caminho)
                    #print(mod_caminho)

                    
                    if mod_caminho == 2.0:                     #andar só uma celula
                        if caminho[0] > 0 and caminho[1] == 0:  
                            print("ir para a direita")
                            self.rotateRight()
                            #...
                        elif caminho[0] < 0 and caminho[1] == 0:
                            print("ir para a esquerda")
                            self.rotateLeft()
                            #...
                        elif caminho[0] == 0 and caminho[1] > 0:
                            print("ir para cima")
                            self.rotateUp()
                            #...
                        elif caminho[0] == 0 and caminho[1] < 0:
                            print("ir para baixo")
                            self.rotateDown()
                            #...
                        else:
                            print ("nada")
                    else:
                        print("celula a mais de um quadrado de distancia")       


                elif self.visited_cells.get(key)[0] == 'o' :
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
            else:
                if espace == 1:
                    if walls[0] == 0 and (key1 not in self.visited_cells or
                    self.visited_cells.get(key)[0] == 'o'):
                        if self.rotateUp():
                            self.positionInitY = self.positionInitY + 2
                            self.y_for_mapping = self.y_for_mapping + 2
                            self.came_from = "up"
                            self.moveY()
                        else:
                            self.rotateUp()
                    elif walls[1] == 0 and ( key2 not in self.visited_cells or
                    self.visited_cells.get(key)[1] == 'o'):
                        if self.rotateRight():
                            self.positionInitX = self.positionInitX + 2
                            self.x_for_mapping = self.x_for_mapping + 2
                            self.came_from = "right"
                            self.moveX()
                        else:
                            self.rotateRight()
                    elif walls [2] == 0 and (key3 not in self.visited_cells or
                    self.visited_cells.get(key)[2] == 'o'):
                        if self.rotateLeft():
                            self.positionInitX = self.positionInitX - 2
                            self.x_for_mapping = self.x_for_mapping - 2
                            self.came_from = "left"
                            self.moveX()
                        else:
                            self.rotateLeft()
                    elif walls[3] == 0 and (key4 not in self.visited_cells or
                    self.visited_cells.get(key)[3] == 'o'):
                        if self.rotateDown():
                            self.positionInitY = self.positionInitY - 2
                            self.y_for_mapping = self.y_for_mapping - 2
                            self.came_from = "down"
                            self.moveY()
                        else:
                            self.rotateDown()
                elif espace > 1:
                            # cima,direita,esquerda,baixo    1 -> parede 0 -> espace
                    if walls[0] == 0 and (key1 not in self.visited_cells or
                    self.visited_cells.get(key)[0] == 'o'):
                        if self.rotateUp():
                            self.positionInitY = self.positionInitY + 2
                            self.y_for_mapping = self.y_for_mapping + 2
                            self.came_from = "up"
                            self.moveY()
                        else:
                            self.rotateUp()
                    elif walls[0] == 1 and walls[1] == 0 and (key2 not in self.visited_cells or
                    self.visited_cells.get(key)[1] == 'o'):
                        if self.rotateRight():
                            self.positionInitX = self.positionInitX + 2
                            self.x_for_mapping = self.x_for_mapping + 2
                            self.came_from = "right"
                            self.moveX()
                        else:
                            self.rotateRight()
                    elif walls[0] == 1 and walls[1] == 1 and walls[2] == 0 and (key3 not in self.visited_cells or
                    self.visited_cells.get(key)[2] == 'o'):
                        if self.rotateLeft():
                            self.positionInitX = self.positionInitX - 2
                            self.x_for_mapping = self.x_for_mapping - 2
                            self.came_from = "left"
                            self.moveX()
                        else:
                            self.rotateLeft()
        else:
            # bussola: 0 -> direita, 90 -> cima, esquerda -> 180,baixo ->-90  
            print("esperar q anda")
            if (self.measures.compass > 80 and self.measures.compass < 100) or (self.measures.compass > -100 and self.measures.compass < -80):
                self.moveY()
            else:
                self.moveX()


    # rodar ------------------
    def rotateDown(self):
        # -90 graus
        if self.measures.compass < -95.0 or self.measures.compass > -85.0:
            self.driveMotors(-0.05,+0.05)
            return False
        else:
            return True
    def rotateLeft(self):
        # 180 graus
        if self.measures.compass > 185.0 or self.measures.compass < 175.0:
            self.driveMotors(-0.05,+0.05)
            return False
        else:
            return True
    def rotateUp(self):
        # 90 graus
        if self.measures.compass > 95.0 or self.measures.compass < 85.0:
            self.driveMotors(-0.05,+0.05)
            return False
        else:
            return True
    def rotateRight(self):
        # 0 graus
        if self.measures.compass < -5.0 or self.measures.compass > 5.0:
            self.driveMotors(-0.05,+0.05)
            return False
        else:
            return True


    # andar -----------------
    def moveX(self):
        if(abs(round(self.positionInitX,1)-round(self.measures.x,1)) > 0.2):
            self.driveMotors(0.10,0.10)
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
                self.insert_mapping((28+self.x_for_mapping,14-self.y_for_mapping-1),'-')
            else:
                self.insert_mapping((28+self.x_for_mapping,14-self.y_for_mapping-1),'X')
            if walls[1] == 1:
                self.insert_mapping((28+self.x_for_mapping+1,14-self.y_for_mapping),'|')
            else:
                self.insert_mapping((28+self.x_for_mapping+1,14-self.y_for_mapping),'X')
            if walls[2] == 1:
                self.insert_mapping((28+self.x_for_mapping-1,14-self.y_for_mapping),'|')
            else:
                self.insert_mapping((28+self.x_for_mapping-1,14-self.y_for_mapping),'X')
            if walls[3] == 1:
                self.insert_mapping((28+self.x_for_mapping,14-self.y_for_mapping+1),'-')
            else:
                self.insert_mapping((28+self.x_for_mapping,14-self.y_for_mapping+1),'X')
            self.insert_mapping((28+self.x_for_mapping,14-self.y_for_mapping),'X')
            self.moving = False

    def moveY(self):
        if(abs(round(self.positionInitY,1)-round(self.measures.y,1)) > 0.2):
            self.driveMotors(0.10,0.10)
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
                tmp += "c" +espace[1] + espace[2] + espace[3]
            elif self.came_from == "down":
                tmp += espace[0] + espace[1] + espace[2] +"c" 
            espace = tmp

            if key not in self.visited_cells:
                self.visited_cells[key] = espace
            if walls[0] == 1:
                self.insert_mapping((28+self.x_for_mapping,14-self.y_for_mapping-1),'-')
            else:
                self.insert_mapping((28+self.x_for_mapping,14-self.y_for_mapping-1),'X')
            if walls[1] == 1:
                self.insert_mapping((28+self.x_for_mapping+1,14-self.y_for_mapping),'|')
            else:
                self.insert_mapping((28+self.x_for_mapping+1,14-self.y_for_mapping),'X')
            if walls[2] == 1:
                self.insert_mapping((28+self.x_for_mapping-1,14-self.y_for_mapping),'|')
            else:
                self.insert_mapping((28+self.x_for_mapping-1,14-self.y_for_mapping),'X')
            if walls[3] == 1:
                self.insert_mapping((28+self.x_for_mapping,14-self.y_for_mappin+1),'-')
            else:
                self.insert_mapping((28+self.x_for_mapping,14-self.y_for_mapping+1),'X')
            self.insert_mapping((28+self.x_for_mapping,14-self.y_for_mapping),'X')
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
            
        self.mapping_output()
        return positions 

    def insert_mapping(self, key, symbol):
        if key not in self.mapping or self.mapping.get(key) == 'X':
            print(self.mapping.get(key))
            self.mapping[key] = str(symbol)

    def mapping_output(self):
        f = open("mapping.out",'w')
        for x in range(1,27):
            for y in range(1,55):
                if(y,x) in self.mapping:
                    f.write(self.mapping.get((y,x)))
                else:
                    f.write(' ')
            f.write('\n')
        f.close()

    # def write_mapaC2(self):
    #     file = 'test.txt'
    #     np.savetxt(file,self.mapaC2.astype(int), fmt='%i',delimiter='')

    #     fin = open(file, "rt")
    #     #read file contents to string
    #     data = fin.read()
    #     # 'x' -> livre,
    #     # '|' -> parede vertical, 
    #     # '-' -> parede horizontal, 
    #     # ' '-> desconhecido
    #     # 1 -> 'x' 
    #     # 2 -> '|' 
    #     # 3 -> '-'
    #     data = data.replace('0', ' ')
    #     data = data.replace('1', 'x')
    #     data = data.replace('2', '|')
    #     data = data.replace('3', '-')

    #     #close the input file
    #     fin.close()
    #     #open the input file in write mode
    #     fin = open(file, "wt")
    #     #overrite the input file with the resulting data
    #     fin.write(data)
    #     #close the file
    #     fin.close()

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
