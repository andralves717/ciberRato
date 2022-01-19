
import sys
from croblink import *
from math import *
import xml.etree.ElementTree as ET

CELLROWS=7
CELLCOLS=14

class MyRob(CRobLinkAngs):

    def __init__(self, rob_name, rob_id, angles, host):
        CRobLinkAngs.__init__(self, rob_name, rob_id, angles, host)

    # In this map the center of cell (i,j), (i in 0..6, j in 0..13) is mapped to labMap[i*2][j*2].
    # to know if there is a wall on top of cell(i,j) (i in 0..5), check if the value of labMap[i*2+1][j*2] is space or not
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

        max_center = max_left = max_right = max_back = 0
        min_center = min_left = min_right = min_back = 5

        while True:
            self.readSensors()
            
            if self.measures.endLed:
                print(self.rob_name + " exiting")
                quit()

            max_center = self.measures.irSensor[0] if self.measures.irSensor[0] > max_center else max_center
            max_left = self.measures.irSensor[1] if self.measures.irSensor[1] > max_left else max_left
            max_right = self.measures.irSensor[2] if self.measures.irSensor[2] > max_right else max_right
            max_back = self.measures.irSensor[3] if self.measures.irSensor[3] > max_back else max_back

            min_center = self.measures.irSensor[0] if self.measures.irSensor[0] < min_center else min_center
            min_left = self.measures.irSensor[1] if self.measures.irSensor[1] < min_left else min_left
            min_right = self.measures.irSensor[2] if self.measures.irSensor[2] < min_right else min_right
            min_back = self.measures.irSensor[3] if self.measures.irSensor[3] < min_back else min_back

            print('''
                Center sensor= %0.2f
                Left sensor= %0.2f
                Right sensor= %0.2f
                Back sensor= %0.2f'''%(self.measures.irSensor[0],
                self.measures.irSensor[1],self.measures.irSensor[2],self.measures.irSensor[3]))

            print('''
            Center sensor: Max %0.2f, Min %0.2f
            Left sensor: Max %0.2f, Min %0.2f
            Right sensor: Max %0.2f, Min %0.2f
            Back sensor: Max %0.2f, Min %0.2f'''%(max_center, min_center,
            max_left, min_left, max_right, min_right, max_back, min_back))

            if(self.measures.collision):
                print("\x1B[1;41m COLISÃO \x1B[0m")

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

        if self.measures.irSensor[left_id] < 1.4 or\
            self.measures.irSensor[right_id] < 1.4:
            if self.measures.irSensor[center_id] < 1.4:
                self.driveMotors(0.15, 0.15)
            if self.measures.irSensor[left_id] > self.measures.irSensor[right_id]:
                print("\x1B[37;7m Direita \x1B[0m")
                self.driveMotors(0.15, -0.05)
            else:
                print("\x1B[37;7m Esquerda \x1B[0m")
                self.driveMotors(-0.05, 0.15)
        else:
            print("\x1B[31;1m Direito \x1B[0m")
            self.driveMotors(0.15, 0.15)

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
    elif (sys.argv[i] == "--robname" or sys.argv[i] == "-r") and i != len(sys.argv) - 1:
        rob_name = sys.argv[i + 1]
    elif (sys.argv[i] == "--map" or sys.argv[i] == "-m") and i != len(sys.argv) - 1:
        mapc = Map(sys.argv[i + 1])
    else:
        print("Unkown argument", sys.argv[i])
        quit()

if __name__ == '__main__':
    rob=MyRob(rob_name,pos,[0.0,60.0,-60.0,180.0],host)
    if mapc != None:
        rob.setMap(mapc.labMap)
        rob.printMap()
    
    rob.run()
