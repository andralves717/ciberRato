
import sys
from croblink import *
from math import *
import xml.etree.ElementTree as ET
from astar import *

CELLROWS=7
CELLCOLS=14

class MyRob(CRobLinkAngs):

    Kp = 1
    h = 0.05
    # Ti = sys.float_info.max
    Ti = 0.2
    Td = 0.0

    K0 = Kp*(1+h/Ti+Td/h)
    K1 = -Kp*(1+2*Td/h)
    K2 = Kp*Td/h

    e_m1 = 0
    e_m2 = 0

    u_m1 = 0

    unvisited_coordinates = dict()
    walls = []
    map_p = []

    last_lp = last_rp = 0

    x_tmp = 27
    out_l_tmp = out_r_tmp = out_tmp = 0

    def __init__(self, rob_name, rob_id, angles, host):
        CRobLinkAngs.__init__(self, rob_name, rob_id, angles, host)

    # In this map the center of cell (i,j), (i in 0..6, j in 0..13) is mapped to labMap[i*2][j*2].
    # to know if there is a wall on top of cell(i,j) (i in 0..5), check if the value of labMap[i*2+1][j*2] is space or not
    def setMap(self, labMap):
        self.labMap = labMap

    def printMap(self, f=None):
        for l in reversed(self.labMap):
            print(''.join([str(l) for l in l]), file=f)

    def run(self):
        if self.status != 0:
            print("Connection refused or error")
            quit()

        state = 'stop'
        stopped_state = 'discover'
        comp = 0
        next_comp = 90
        rot_right = False
        ticks = 0

        self.readSensors()
        initial_pos = (27, 13)
        initial_pos_real = (self.measures.x, self.measures.y)
        self.pos = initial_pos
        self.pos_calc = initial_pos

        (x, y) = (initial_pos[0], initial_pos[1])

        if(self.measures.irSensor[3] < 1.2):
            if ((x,y) not in self.unvisited_coordinates.keys()):
                self.unvisited_coordinates[(x,y)] = []
            self.unvisited_coordinates[(x,y)].append((x-2,y))

        

        if(self.measures.irSensor[0] < 1.2):
            next_comp = comp 
            self.next_stop = (round(self.pos_calc[0]+2), round(self.pos_calc[1]))
            state = 'discover'
        elif(self.measures.irSensor[3] < 1.2 and left != back):
            next_comp = 180
            self.next_stop = (round(self.pos_calc[0]-2), round(self.pos_calc[1]))
            state = 'rotate'
        elif(self.measures.irSensor[2] < 1.2 and up != back):
            next_comp = 90
            self.next_stop = (round(self.pos_calc[0]), round(self.pos_calc[1]+2))
            state = 'rotate'
        elif(self.measures.irSensor[1] < 1.2 and down != back):
            next_comp = -90
            self.next_stop = (round(self.pos_calc[0]), round(self.pos_calc[1]-2))
            state = 'rotate'

        rot = True

        while True:
            self.readSensors()
            
            pos_real = (self.measures.x, self.measures.y)
            self.pos = (initial_pos[0] + pos_real[0] - initial_pos_real[0], initial_pos[1] + pos_real[1] - initial_pos_real[1])

            if self.measures.time > 4990:
                self.finish()
                print("\x1B[34;3m TIME FINISHED->%d \x1B[0m"%(self.measures.time))

            if self.measures.endLed:
                for (x, y) in self.map_p:
                    self.labMap[y][x] = 'X'
                self.labMap[initial_pos[1]][initial_pos[0]] = 'I'
                if self.measures.endLed:
                    with open(mapname, 'w') as f:
                        self.printMap(f)
                return

                print(self.rob_name + " exiting")
                quit()

            print('''
                State = %s
                Compass = %0.2f
                GPS: (X,Y) = (%0.2f, %0.2f)
                Calc:(X,Y) = (%0.2f, %0.2f)    

                Center sensor= %0.2f
                Left sensor= %0.2f
                Right sensor= %0.2f
                Back sensor= %0.2f'''%(state ,self.measures.compass, self.pos[0], self.pos[1],
                self.pos_calc[0], self.pos_calc[1], self.measures.irSensor[0], self.measures.irSensor[1],
                self.measures.irSensor[2],self.measures.irSensor[3]))
            

            if(self.measures.collision):
                print("\x1B[1;41m COLISÃO \x1B[0m")
                self.printMap()
                self.finish()

            if state == 'stop' and self.measures.start:
                self.driveMotors(0, 0)
                state = stopped_state

            if state != 'stop' and self.measures.stop:
                self.setMap([[' '] * (CELLCOLS*4-1) for _ in range(CELLROWS*4-1) ])
                self.check_map()
                stopped_state = state
                state = 'stop'

            if(abs(self.pos_calc[0] - round(self.pos_calc[0])) < 0.2 and abs(self.pos_calc[1] - round(self.pos_calc[1])) < 0.2):
                self.check_map()

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
                self.last_lp = self.last_rp = 0
                self.driveMotors(0.0,0.0)
            elif state=='return':
                if self.measures.visitingLed==True:
                    self.setVisitingLed(False)
                if self.measures.returningLed==True:
                    self.setReturningLed(False)
                self.wander()
            elif state=='rotate':
                self.u_m1 = self.e_m1 = self.e_m2 = 0
                self.rotate(next_comp)

                state = stopped_state
            elif state == 'discover':        
                if (self.measures.compass < -170 or self.measures.compass > 170):
                    comp = 180
                    right = 3
                    up = 2
                    down = 1
                    left = 0

                    front = left
                    back = right
                    front_coords = (-2,0)
                elif (-10 < self.measures.compass < 10):
                    comp = 0
                    right = 0
                    up = 1
                    down = 2
                    left = 3

                    front = right
                    back = left
                    front_coords = (2,0)
                elif (78 < self.measures.compass < 100):
                    comp = 90
                    right = 2
                    up = 0
                    down = 3
                    left = 1

                    front = up
                    back = down
                    front_coords = (0,2)
                elif (-100 < self.measures.compass < -80):
                    comp = 270
                    right = 1
                    up = 3
                    down = 0
                    left = 2

                    front = down
                    back = up
                    front_coords = (0,-2)
                else:
                    comp = 0
                    right = 0
                    up = 1
                    down = 2
                    left = 3

                    front = right
                    back = left

                if  (((comp == 0 or comp == 180) and self.next_stop[0] != round(self.pos_calc[0], 1)) or\
                    ((comp == 90 or comp == 270) and self.next_stop[1] != round(self.pos_calc[1], 1))):
                    print(" %0.2f/%0.2f, %0.2f/%0.2f; COMP: %0.2f"%(self.next_stop[0], self.pos_calc[0], self.next_stop[1], self.pos_calc[1],comp))

                    if(comp == 0 or comp == 180):
                        u = self.pid(1, (2-self.next_stop[0]+self.pos_calc[0])/2)
                    else:
                        u = self.pid(1, (2-self.next_stop[1]+self.pos_calc[1])/2)

                    u = -u if (comp == 270 or comp == 180) else u
                    self.move_forward(u)
                else:

                    if self.correct_pos() :
                        stopped_state = 'discover'
                        state = 'correct_pos'
                    else:

                        x = round(self.pos_calc[0])
                        y = round(self.pos_calc[1])

                        if(self.measures.irSensor[right] < 1.2 and right != front and right != back):
                            if ((x,y) not in self.unvisited_coordinates.keys()):
                                self.unvisited_coordinates[(x,y)] = []
                            self.unvisited_coordinates[(x,y)].append((x+2, y))
                        if(self.measures.irSensor[left] < 1.2 and left != front and left != back):
                            if ((x,y) not in self.unvisited_coordinates.keys()):
                                self.unvisited_coordinates[(x,y)] = []
                            self.unvisited_coordinates[(x,y)].append((x-2,y))
                        if(self.measures.irSensor[up] < 1.2 and up != front and up != back):
                            if ((x,y) not in self.unvisited_coordinates.keys()):
                                self.unvisited_coordinates[(x,y)] = []
                            self.unvisited_coordinates[(x,y)].append((x, y+2))
                        if(self.measures.irSensor[down] < 1.2 and down != 0 and down != back):
                            if ((x,y) not in self.unvisited_coordinates.keys()):
                                self.unvisited_coordinates[(x,y)] = []
                            self.unvisited_coordinates[(x,y)].append((x, y-2))

                        if(self.measures.irSensor[front] < 1.2):
                            next_comp = comp 
                            self.next_stop = (round(self.pos_calc[0]+front_coords[0]), round(self.pos_calc[1]+front_coords[1]))
                            state = 'discover'
                        elif(self.measures.irSensor[right] < 0.8 and right != back):
                            next_comp = 0
                            self.next_stop = (round(self.pos_calc[0]+2), round(self.pos_calc[1]))
                            state = 'rotate'
                            stopped_state = 'discover'

                            if self.next_stop in self.unvisited_coordinates[(x,y)]:
                                self.unvisited_coordinates[(x, y)].remove(self.next_stop)

                                if self.unvisited_coordinates[(x, y)] == []:
                                    del self.unvisited_coordinates[(x, y)]
                        elif(self.measures.irSensor[left] < 0.8 and left != back):
                            next_comp = 180
                            self.next_stop = (round(self.pos_calc[0]-2), round(self.pos_calc[1]))
                            state = 'rotate'
                            stopped_state = 'discover'

                            if self.next_stop in self.unvisited_coordinates[(x,y)]:
                                self.unvisited_coordinates[(x, y)].remove(self.next_stop)

                                if self.unvisited_coordinates[(x, y)] == []:
                                    del self.unvisited_coordinates[(x, y)]
                        elif(self.measures.irSensor[up] < 0.8 and up != back):
                            next_comp = 90
                            self.next_stop = (round(self.pos_calc[0]), round(self.pos_calc[1]+2))
                            state = 'rotate'
                            stopped_state = 'discover'

                            if self.next_stop in self.unvisited_coordinates[(x,y)]:
                                self.unvisited_coordinates[(x, y)].remove(self.next_stop)

                                if self.unvisited_coordinates[(x, y)] == []:
                                    del self.unvisited_coordinates[(x, y)]
                        elif(self.measures.irSensor[down] < 0.8 and down != back):
                            next_comp = -90
                            self.next_stop = (round(self.pos_calc[0]), round(self.pos_calc[1]-2))
                            state = 'rotate'
                            stopped_state = 'discover'

                            if self.next_stop in self.unvisited_coordinates[(x,y)]:
                                self.unvisited_coordinates[(x, y)].remove(self.next_stop)

                                if self.unvisited_coordinates[(x, y)] == []:
                                    del self.unvisited_coordinates[(x, y)]

                        
                        if self.next_stop in self.map_p:
                            state = 'start_search'

            elif state == 'start_search':
                self.last_lp = self.last_rp = 0
                self.driveMotors(0, 0)
                dist_min = 1000
                print("Unvisited_coords: "+str(self.unvisited_coordinates.keys()))
                for (uc_x, uc_y) in self.unvisited_coordinates.keys():
                    dist = (uc_x - self.pos_calc[0]) ** 2 + (uc_y - self.pos_calc[1]) ** 2
                    if dist < dist_min:
                        dist_min = dist
                        coord_near = (uc_x, uc_y)

                x = round(self.pos_calc[0])
                y = round(self.pos_calc[1])

                path_to_dest = astar(self.map_p, (x, y), coord_near, self.walls)
                print ("Path to dest: "+str(path_to_dest))
                print ("MAP: "+str(self.map_p))
                print ("Coord near: "+str(coord_near))
                print ("walls: "+str(self.walls))
                if(path_to_dest != [] and path_to_dest != None):
                    self.next_stop = path_to_dest.pop()
                    state = 'move_to_next_stop'
                    rot = True
                elif(path_to_dest == []):
                    state = 'discover_new'

               

            elif state == 'move_to_next_stop':
                
                if self.pos_calc[0] - self.next_stop[0] < -1:
                    comp = 0
                elif self.pos_calc[0] - self.next_stop[0] > 1:
                    comp = 180
                elif self.pos_calc[1] - self.next_stop[1] < -1:
                    comp = 90
                elif self.pos_calc[1] - self.next_stop[1] > 1:
                    comp = -90
                if(rot):
                    if(self.measures.compass < -170 and comp == 0):
                        self.rotate(-90)
                        self.rotate(0)
                    elif(self.measures.compass > 170 and comp == 0):
                        self.rotate(90)
                        self.rotate(0)
                    elif(-10 < self.measures.compass < 10 and comp == 180):
                        self.rotate(90)
                        self.rotate(180)
                    elif(80 < self.measures.compass < 100 and comp == -90):
                        self.rotate(0)
                        self.rotate(-90)
                    elif(-100 < self.measures.compass < -80 and comp == 90):
                        self.rotate(0)
                        self.rotate(90)
                    else:
                        self.rotate(comp)
                    rot = False

                if  (((comp == 0 or comp == 180) and self.next_stop[0] != round(self.pos_calc[0],1)) or\
                    ((comp == 90 or comp == 270 or comp == -90) and self.next_stop[1] != round(self.pos_calc[1],1))):
                    print("%0.2f/%0.2f, %0.2f/%0.2f; COMP: %0.2f"%(self.next_stop[0], self.pos_calc[0], self.next_stop[1], self.pos_calc[1],comp))

                    if(comp == 0 or comp == 180):
                        u = self.pid(1, (2-self.next_stop[0]+self.pos_calc[0])/2)
                    else:
                        u = self.pid(1, (2-self.next_stop[1]+self.pos_calc[1])/2)

                    u = -u if (comp == 270 or comp == 180 or comp == -90) else u
                    self.move_forward(u)
                else:
                    if self.correct_pos() :
                        stopped_state = 'move_to_next_stop'
                        state = 'stop'
                    else:

                        if(path_to_dest != [] and path_to_dest != None):
                            rot = True
                            self.next_stop = path_to_dest.pop()
                        else:
                            state = 'discover_new'

            elif state == 'discover_new':
                x = round(self.pos_calc[0])
                y = round(self.pos_calc[1])

                if(self.unvisited_coordinates == {}):
                    self.finish()
                    print("\x1B[34;3m TIME FINISHED->%d \x1B[0m"%(self.measures.time))
                
                else:
                    if ((x, y) in self.unvisited_coordinates.keys()):
                        self.next_stop = self.unvisited_coordinates[(x, y)].pop()
                        if(len(self.unvisited_coordinates[(x, y)]) == 0):
                            del self.unvisited_coordinates[(x, y)]

                        if x - self.next_stop[0] < -1:
                            comp = 0
                        elif x - self.next_stop[0] > 1:
                            comp = 180
                        elif y - self.next_stop[1] < -1:
                            comp = 90
                        elif y - self.next_stop[1] > 1:
                            comp = -90

                        if((abs(comp - self.measures.compass) > 3 and comp != 180) or (comp == 180 and (self.measures.compass < 177 or self.measures.compass > -177))):
                            self.rotate(comp)
                        state = 'discover'              
                    else:
                        state = 'start_search'

            elif state == 'correct_pos':
                if (self.measures.compass < -170 or self.measures.compass > 170):
                    comp = 180
                elif (-10 < self.measures.compass < 10):
                    comp = 0
                elif (78 < self.measures.compass < 100):
                    comp = 90
                elif (-100 < self.measures.compass < -80):
                    comp = 270
                else:
                    comp = 0

                if  (((comp == 0 or comp == 180) and self.next_stop[0] != round(self.pos_calc[0], 1)) or\
                    ((comp == 90 or comp == 270) and self.next_stop[1] != round(self.pos_calc[1], 1))):
                    print(" %0.2f/%0.2f, %0.2f/%0.2f; COMP: %0.2f"%(self.next_stop[0], self.pos_calc[0], self.next_stop[1], self.pos_calc[1],comp))

                    if(comp == 0 or comp == 180):
                        u = 0.1*self.pid(1, (2-self.next_stop[0]+self.pos_calc[0])/2)
                    else:
                        u = 0.1*self.pid(1, (2-self.next_stop[1]+self.pos_calc[1])/2)

                    if(abs(u) < 0.01 and not self.correct_pos()):
                        state = stopped_state

                    u = -u if (comp == 270 or comp == 180) else u
                    self.move_forward(u)
                else:
                    state = 'stop'


    def move_forward(self, lin):
        fwd = False

        if (self.measures.compass < -145 or self.measures.compass > 145):
            compass_desired = 180
        elif (-35 < self.measures.compass < 35):
            compass_desired = 0
        elif (55 < self.measures.compass < 125):
            compass_desired = 90
        elif (-125 < self.measures.compass < -55):
            compass_desired = -90
        else:
            compass_desired = 0

        if (self.measures.compass < -150):
            self.forward(lin, 0.01, self.measures.compass+360, compass_desired)
        else:
            self.forward(lin, 0.01, self.measures.compass, compass_desired)

    def rotate(self, compass_desired):
        if (self.measures.compass < -170 or self.measures.compass > 170):
            if(compass_desired == 90 or compass_desired == -90):
                compass = self.measures.compass%360
                if compass_desired == -90:
                    comp_d = 270
                else:
                    comp_d = compass_desired
            else:
                compass = self.measures.compass
                comp_d = compass_desired
        else:
            comp_d = compass_desired
            if(compass_desired == 180):
                compass = self.measures.compass%360
            else:
                compass = self.measures.compass

        while(compass != comp_d):
            self.readSensors()
            compass = self.measures.compass%360 if ((comp_d == 270) or (self.measures.compass < -170 and compass_desired == 90) or (compass_desired == 180)) else self.measures.compass
            print("Compass: %0.2f | Compass_desired: %0.2f"%(compass, comp_d))
            l_p = r_p = self.pid(comp_d/360, compass/360)
            if l_p == 0:
                return
                print('Cannot be zero')
                l_p = r_p = 0.01 if comp_d > compass else -0.01
            self.last_lp = -l_p
            self.last_rp = r_p
            self.driveMotors(-l_p, r_p)
        self.driveMotors(0, 0)


    def forward(self, lin, k, m, r):
        rot = k*(m-r)

        if(lin > 0.15):
            lin = 0.15
        elif(lin < -0.15):
            lin = -0.15
        elif(abs(lin) < 0.01):
            lin = 0

        l_power = round(lin + (rot/2),2)
        r_power = round(lin - (rot/2),2)
        
        print("\x1B[1;41m lin: %0.2f; motors %0.2f, %0.2f \x1B[0m"%(lin, l_power, r_power))
        self.last_lp = l_power
        self.last_rp = r_power
        self.driveMotors(l_power, r_power)

        self.out_l_tmp = (l_power+self.out_l_tmp)/2
        self.out_r_tmp = (r_power+self.out_r_tmp)/2
        self.out_tmp = ((l_power+r_power)/2+self.out_tmp)/2
        lin_tmp = (self.out_l_tmp + self.out_r_tmp)/2
        self.x_tmp = self.x_tmp + self.out_tmp

        # if(r == 0):
        #     self.pos_calc = (self.pos_calc[0]+lin_tmp, self.pos_calc[1])
        # elif(r == 180):
        #     self.pos_calc = (self.pos_calc[0]-lin_tmp, self.pos_calc[1])
        # elif(r == 90):
        #     self.pos_calc = (self.pos_calc[0], self.pos_calc[1]+lin_tmp)
        # elif(r == -90 or r == 270):
        #     self.pos_calc = (self.pos_calc[0], self.pos_calc[1]-lin_tmp)

        if(r == 0):
            self.pos_calc = (self.pos_calc[0]+self.out_tmp, self.pos_calc[1])
        elif(r == 180):
            self.pos_calc = (self.pos_calc[0]-self.out_tmp, self.pos_calc[1])
        elif(r == 90):
            self.pos_calc = (self.pos_calc[0], self.pos_calc[1]+self.out_tmp)
        elif(r == -90 or r == 270):
            self.pos_calc = (self.pos_calc[0], self.pos_calc[1]-self.out_tmp)


    def pid(self, setPoint, feedback):
        u = 0

        max_u = 0.15
        delta = 0.01

        
        e = (setPoint - feedback)     

        # Compute control signal
        u = self.u_m1 + self.K0*e + self.K1*self.e_m1 + self.K2*self.e_m2
        print("K0: %0.2f, K1: %0.2f, K2: %0.2f, U: %0.3f, E: %0.3f, E_M1: %0.3f, E_M2: %0.3f, U_M1: %0.3f"%(self.K0, self.K1, self.K2, u, e , self.e_m1, self.e_m2, self.u_m1))


        # Store values for next iterations
        self.e_m2 = self.e_m1
        self.e_m1 = e
        self.u_m1 = u

        # Clip the control signal to avoid saturation

        if (self.u_m1 > max_u):
            self.u_m1 = max_u

        if (self.u_m1 < -max_u):
            self.u_m1 = -max_u

        return u

    def correct_pos(self):
        if (self.measures.irSensor[0] >= 1.2):
            print("\x1B[1;46mVou dar check à posição da frente\x1B[0m")
            print("antes: %4.2f,%4.2f"%(self.pos_calc[0],self.pos_calc[1]))
            if(80 < self.measures.compass < 100):
                self.pos_calc = (self.pos_calc[0], (self.pos_calc[1]+0.5-(1/self.measures.irSensor[0])))
                print("depois: %4.2f,%4.2f"%(self.pos_calc[0],self.pos_calc[1]))
                if abs(round(self.pos_calc[1]) - self.pos_calc[1]) > 0.1:
                    return True

            elif(-100 < self.measures.compass < -80):
                self.pos_calc = (self.pos_calc[0], (self.pos_calc[1]-0.5+(1/self.measures.irSensor[0])))
                print("depois: %4.2f,%4.2f"%(self.pos_calc[0],self.pos_calc[1]))
                if abs(round(self.pos_calc[1]) - self.pos_calc[1]) > 0.1:
                    return True

            elif(self.measures.compass > 170 or self.measures.compass < -170):
                self.pos_calc = ((self.pos_calc[0]-0.5+(1/self.measures.irSensor[0])), self.pos_calc[1])
                print("depois: %4.2f,%4.2f"%(self.pos_calc[0],self.pos_calc[1]))
                if abs(round(self.pos_calc[0]) - self.pos_calc[0]) > 0.1:
                    return True
            else:
                self.pos_calc = ((self.pos_calc[0]+0.5-(1/self.measures.irSensor[0])), self.pos_calc[1])
                print("depois: %4.2f,%4.2f"%(self.pos_calc[0],self.pos_calc[1]))
                if abs(round(self.pos_calc[0]) - self.pos_calc[0]) > 0.1:
                    return True

        return False
            


    def check_map(self):
        front_comp = 0
        right = 0
        up = 1
        down = 2
        left = 3
        if(80 < self.measures.compass < 100):
            front_comp = 90
            right = 2
            up = 0
            down = 3
            left = 1
        elif(-100 < self.measures.compass < -80):
            front_comp = -90
            right = 1
            up = 3
            down = 0
            left = 2
        elif(self.measures.compass > 170 or self.measures.compass < -170):
            front_comp = 180
            right = 3
            up = 2
            down = 1
            left = 0

        x = round(self.pos_calc[0])
        y = round(self.pos_calc[1])

        print("Estou aqui a dar um check no (%4.2f, %4.2f)"%(x,y))

        if (x, y) not in self.map_p and x%2 != 0 and y%2 != 0:
            self.map_p.append((x, y))

        print(self.unvisited_coordinates.__str__())
        for uc in self.unvisited_coordinates.keys():
            cont = True
            for c in self.map_p:
                if c in self.unvisited_coordinates[uc]:
                    self.unvisited_coordinates[uc].remove(c)
                    if self.unvisited_coordinates[uc] == []:
                        del self.unvisited_coordinates[uc]
                        cont = False
                        break
            if not cont:
                break


        if(self.measures.irSensor[right] > 1.5):
            if ((x+1, y) not in self.walls):
                self.walls.append((x+1, y))
                self.labMap[y][x+1] = '|' if (y%2 != 0) else ' '
        elif (self.measures.irSensor[right] < 1):
            if ((x+1, y) not in self.walls):
                self.labMap[y][x+1] = 'X' if (x%2 != 0) else self.labMap[y][x+1]

        if (self.measures.irSensor[left] > 1.5):
            if ((x-1, y) not in self.walls):
                self.walls.append((x-1, y))
                self.labMap[y][x-1] = '|' if (y%2 != 0) else ' '
        elif (self.measures.irSensor[left] < 1):
            if ((x-1, y) not in self.walls):
                self.labMap[y][x-1] = 'X' if (x%2 != 0) else self.labMap[y][x-1]

        if(self.measures.irSensor[up] > 1.5):
            if ((x, y+1) not in self.walls):
                self.walls.append((x, y+1))
                self.labMap[y+1][x] = '-' if (x%2 != 0) else ' '
        elif (self.measures.irSensor[up] < 1):
            if ((x, y+1) not in self.walls):
                self.labMap[y+1][x] = 'X' if (y%2 != 0) else self.labMap[y+1][x]

        if (self.measures.irSensor[down] > 1.5):
            if ((x, y-1) not in self.walls):
                self.walls.append((x, y-1))
                self.labMap[y-1][x] = '-' if (x%2 != 0) else ' '
        elif (self.measures.irSensor[down] < 1):
            if ((x, y-1) not in self.walls):
                self.labMap[y-1][x] = 'X' if (y%2 != 0) else self.labMap[y-1][x]

        for (x, y) in self.map_p:
            self.labMap[y][x] = 'X' if (x%2 != 0 and y%2 != 0) else self.labMap[y][x]
        with open(mapname, 'w') as f:
            self.printMap(f)

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
filename = "mapping.out"

for i in range(1, len(sys.argv),2):
    if (sys.argv[i] == "--host" or sys.argv[i] == "-h") and i != len(sys.argv) - 1:
        host = sys.argv[i + 1]
    elif (sys.argv[i] == "--pos" or sys.argv[i] == "-p") and i != len(sys.argv) - 1:
        pos = int(sys.argv[i + 1])
    elif (sys.argv[i] == "--robname" or sys.argv[i] == "-r") and i != len(sys.argv) - 1:
        rob_name = sys.argv[i + 1]
    elif (sys.argv[i] == "--map" or sys.argv[i] == "-m") and i != len(sys.argv) - 1:
        mapc = Map(sys.argv[i + 1])
    elif (sys.argv[i] == "--file" or sys.argv[i] == "-f") and i != len(sys.argv) -1:
        filename = sys.argv[i + 1]
    else:
        print("Unkown argument", sys.argv[i])
        quit()

mapname = filename+".map"
pathname = filename+".path"

if __name__ == '__main__':
    rob=MyRob(rob_name,pos,[0.0,90.0,-90.0,180.0],host)
    if mapc != None:
        rob.setMap(mapc)
        rob.printMap()
    
    rob.run()
