import random
import math
import sys
import matplotlib.pyplot as plt
import numpy as np
from robot import Robot

robots = []
group_states = []#1-entrando, 2-saindo, 0-esperando
last_configuration = []
highest_priority_group = -1
transit_time = []
feed_time = []
it=0
total_distance=0

random.seed(1111)  #Semente aleatória
VIEW = True #Execução no modo gráfico
GROUP_COLORS =['yo', 'bo', 'go'] #Adicione mais cores para ter mais grupos
NUM_ROBOTS_IN_GROUP = 10 #Numero de robos em cada grupo
SENSOR_RANGE = 2 #Alcance do sensor de obstáculos
TARGET_POS = [5,5] #Posição da área alvo
TARGET_RADIUS = .5 #Raio da área alvo
WAITING_AREA_RADIUS = 1.5 #Raio da área de espera (NAO UTILIZADO)
MAX_INTENSITY = 0.08 #Intensidade máxima da força
K_ATT = 0.1 #Constante de atração
K_REP = 0.01 #Consntante de repulsão
RO = 0.8 #Constante limite da variável aleatória
N = 40  #Número de iterações para atualizar

def init():
    if VIEW:
        plt.show()
        plt.axis([-1, 11, -1, 11])
    for j in range(len(GROUP_COLORS)):
        group_states.insert(j, 0)
        transit_time.insert(j, 0)
        feed_time.insert(j, 0)
        for i in range(NUM_ROBOTS_IN_GROUP):    
            robot = Robot([random.uniform(0, 10),random.uniform(0, 10)], 'e', 'g' + str(j+1))
            robots.insert(i,robot)
            if VIEW:
                plt.plot(robot.pos[0], robot.pos[1], GROUP_COLORS[j])

    if VIEW:
        target = plt.Circle((TARGET_POS[0], TARGET_POS[1]), TARGET_RADIUS, color='r', fill=False)
        waiting_area = plt.Circle((TARGET_POS[0], TARGET_POS[1]), WAITING_AREA_RADIUS, color='b', fill=False)
        fig = plt.gcf()
        ax = fig.gca()
        ax.add_artist(target)
        ax.add_artist(waiting_area)

def update_view():
    plt.clf()
    plt.axis([-1, 11, -1, 11])
    for i in range(len(robots)):   
        plt.plot(robots[i].pos[0], robots[i].pos[1], GROUP_COLORS[int(robots[i].group[1])-1])

    target = plt.Circle((TARGET_POS[0], TARGET_POS[1]), .5, color='r', fill=False)
    waiting_area = plt.Circle((TARGET_POS[0], TARGET_POS[1]), 1.5, color='b', fill=False)
    fig = plt.gcf()
    ax = fig.gca()
    ax.add_artist(target)
    ax.add_artist(waiting_area)

def save_state():
    global last_configuration
    last_configuration = robots

def get_distance(p1,p2):
    return math.sqrt(pow(p1[0]-p2[0],2)+pow(p1[1]-p2[1],2))

def read_sensor(robot):
    all_robots = last_configuration
    read = []
    for i in range(len(robots)):  
        if(robot.pos[0] != all_robots[i].pos[0] or robot.pos[1] != all_robots[i].pos[1]):
            distance = get_distance(robot.pos, all_robots[i].pos)
            if(distance < SENSOR_RANGE):
                read.append(all_robots[i])           
    return read

def get_att_force(robot):
    angle = math.atan2(robot.pos[1]-TARGET_POS[1], robot.pos[0]-TARGET_POS[0])
    intensity = K_ATT*get_distance(robot.pos, TARGET_POS)
    x_component = intensity*math.cos(angle)
    y_component = intensity*math.sin(angle)
    return x_component, y_component 

def get_rep_force(robot):
    robots_in_range = read_sensor(robot)
    x_component = 0;
    y_component = 0;
    for next_robot in robots_in_range:
        distance = get_distance(robot.pos, next_robot.pos)
        force = K_REP*((1/distance)-(1/SENSOR_RANGE))*(1/pow(distance,2))
        angle = math.atan2(robot.pos[1]-next_robot.pos[1], robot.pos[0]-next_robot.pos[0])
        
        #Componente aleatória
        angle += math.pi/random.uniform(2, 8)
        
        x_component = x_component + force*math.cos(angle)
        y_component = y_component + force*math.sin(angle)
    return x_component, y_component

def get_group_distance(group):
    distance = 0
    for robot in robots:
        if robot.group == group:
            distance += get_distance(robot.pos, TARGET_POS)
    return distance
        

def change_group_states():
    global it
    for i in range(len(GROUP_COLORS)):
        if group_states[i] != 1:
            dice = random.uniform(0,1)
            if dice <= RO and math.fmod(it,N) == 0:
                group_states[i] = 1
    it += 1
    
def move_all():
    global total_distance
    save_state()
    change_group_states()
    for i in range(len(GROUP_COLORS)):
        feed = False
        ended = True
        if group_states[i] == 1 or group_states[i] == 2:
            for robot in robots:
                if robot.group == 'g' + str(i+1) :
                    att_force_x, att_force_y = get_att_force(robot)
                    rep_force_x, rep_force_y = get_rep_force(robot)
                    robot_oldx = robot.pos[0]
                    robot_oldy = robot.pos[1]
                    if get_distance(robot.pos, TARGET_POS) <= TARGET_RADIUS:
                        robot.state = 'l'
                    if robot.state == 'e':
                        ended = False
                        robot.pos[0] = robot.pos[0] + check_force(- att_force_x + rep_force_x)
                        robot.pos[1] = robot.pos[1] + check_force(- att_force_y + rep_force_y)
                    elif robot.state == 'l':
                        feed = True
                        robot.pos[0] = robot.pos[0] + check_force(att_force_x + rep_force_x)
                        robot.pos[1] = robot.pos[1] + check_force(att_force_y + rep_force_y)
                    total_distance += get_distance([robot_oldx, robot_oldy], robot.pos)
        else:
            ended = False
            
        if not ended:        
            if feed == True:
                feed_time[i] += 1
            else:
                transit_time[i] += 1
        else:
            group_states[i] = 2

def check_force(f):
    if f > MAX_INTENSITY:
        return MAX_INTENSITY
    if f < -MAX_INTENSITY:
        return -MAX_INTENSITY
    return f
    
def completed():
    for i in range(len(GROUP_COLORS)):
        if group_states[i] != 2:
            return False
    return True

init()  
while(not completed()):
    plt.pause(0.001)
    move_all()
    if VIEW:
        update_view()

print(str(sum(transit_time)/len(transit_time)) +";" + str(sum(feed_time)/len(feed_time)) + ";" + str(total_distance))