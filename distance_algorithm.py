"""
Distance algorithm shell

Usage:
    distance_algorithm.py <number_of_robots_in_group> <group_colors> [--view]

Options:
    --view
        GUI Mode
        
"""

import random
import math
import sys
import docopt
import matplotlib.pyplot as plt
from PIL import ImageGrab
from robot import Robot

robots = []
group_states = []#1-entrando, 2-saindo, 0-esperando
last_configuration = []
highest_priority_group = -1
transit_time = []
feed_time = []
total_distance=0
total_it=0

random.seed(1)  #Semente aleatória
VIEW = True #Execução no modo gráfico
GROUP_COLORS = ['yo', 'bo', 'go'] #Adicione mais cores para ter mais grupos
NUM_ROBOTS_IN_GROUP = 10 #Numero de robos em cada grupo
SENSOR_RANGE = 1 #Alcance do sensor de obstáculos
TARGET_POS = [5,5] #Posição da área alvo
TARGET_RADIUS = .5 #Raio da área alvo
WAITING_AREA_RADIUS = 1.5 #Raio da área de espera (NAO UTILIZADO)
MAX_INTENSITY = 0.08 #Intensidade máxima da força
K_ATT = 0.1 #Constante de atração
K_REP = 0.01 #Consntante de repulsão


def init():
    if VIEW:
        plt.show()
        plt.axis([-1, 11, -1, 11])
        
    #Inicializa os robôs
    for j in range(len(GROUP_COLORS)):
        group_states.insert(j, 0)
        transit_time.insert(j, 0)
        feed_time.insert(j, 0)
        for i in range(NUM_ROBOTS_IN_GROUP): 
            robot_pos = [random.uniform(0, 10),random.uniform(0, 10)]
            #Evita que robôs apareçam dentro do alvo
            while get_distance(robot_pos, TARGET_POS) <= TARGET_RADIUS:
                robot_pos = [random.uniform(0, 10),random.uniform(0, 10)]
            robot = Robot(robot_pos, 'e', 'g' + str(j+1))
            robots.insert(i,robot)
            if VIEW:
                plt.plot(robot.pos[0], robot.pos[1], GROUP_COLORS[j])

    if VIEW:
        target = plt.Circle((TARGET_POS[0], TARGET_POS[1]), TARGET_RADIUS, color='r', fill=False)
        waiting_area = plt.Circle((TARGET_POS[0], TARGET_POS[1]), WAITING_AREA_RADIUS, color='b', fill=False)
        fig = plt.gcf()
        ax = fig.gca()
        ax.add_artist(target)
        #ax.add_artist(waiting_area)


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
    #ax.add_artist(waiting_area)


def save_state():
    global last_configuration
    last_configuration = robots


def get_distance(p1,p2):
    return math.sqrt(pow(p1[0]-p2[0],2)+pow(p1[1]-p2[1],2))


def read_sensor(robot): #Lê o sensor
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


def get_group_distance(group):#Retorna a média da distância
    distance = 0
    for robot in robots:
        if robot.group == group:
            distance += get_distance(robot.pos, TARGET_POS)
    return distance/NUM_ROBOTS_IN_GROUP


def set_highest_priority_group():#Define o grupo com permissão de se mover
    global highest_priority_group
    min_dist = sys.maxsize;
    min_group = -1
    distances = []
    for i in range(len(GROUP_COLORS)):
        distances.insert(i, get_group_distance('g' + str(i+1)))
        if distances[i] < min_dist and group_states[i] == 0:
            min_dist = distances[i]
            min_group = i            
            
    highest_priority_group = min_group
    group_states[min_group] = 1


def move_all():
    global total_distance
    save_state()
    for i in range(len(GROUP_COLORS)):#Para cada grupo
        feed = False
        ended = True
        if group_states[i] == 1 or group_states[i] == 2:#Se o grupo está no estado entrando ou saindo do alvo
            pass_priority = 1
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
                    if highest_priority_group == i and robot.state == 'e':
                        pass_priority = 0 
            if pass_priority == 1 and highest_priority_group == i:
                set_highest_priority_group()
        else:
            ended = False
            for robot in robots:
                if robot.group == 'g' + str(i+1) :
                    rep_force_x, rep_force_y = get_rep_force(robot)
                    robot_oldx = robot.pos[0]
                    robot_oldy = robot.pos[1]
                    robot.pos[0] = robot.pos[0] + check_force(rep_force_x)
                    robot.pos[1] = robot.pos[1] + check_force(rep_force_y)
                    total_distance += get_distance([robot_oldx, robot_oldy], robot.pos)
        
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


def begin_execution():
    init()
    global total_it
    set_highest_priority_group()
    while not completed():
        total_it+=1
        move_all()
        if VIEW:
            plt.pause(0.001)
            update_view()
            img = ImageGrab.grab()
            img.save("prints/dist/{}.png".format(total_it),"PNG")
            update_view()

    #print(str(sum(transit_time)/len(transit_time)) + ";" + str(sum(feed_time)/len(feed_time)) + ";" + str(total_distance))
    return str(sum(transit_time)/len(transit_time)), str(sum(feed_time)/len(feed_time)), str(total_distance)


def main(args):
    arguments = docopt.docopt(__doc__)
    global VIEW
    global GROUP_COLORS
    global NUM_ROBOTS_IN_GROUP
    global robots
    global group_states
    global last_configuration
    global highest_priority_group
    global transit_time
    global feed_time
    global total_distance
    global total_it

    if arguments["--view"]:
        VIEW = True
    else:
        VIEW = False
    NUM_ROBOTS_IN_GROUP = int(arguments["<number_of_robots_in_group>"])
    GROUP_COLORS = arguments["<group_colors>"].split(',')


    for i in range(1):
        robots = []
        group_states = []  # 1-entrando, 2-saindo, 0-esperando
        last_configuration = []
        highest_priority_group = -1
        transit_time = []
        feed_time = []
        total_distance = 0
        total_it=0
        random.seed(i)
        print("Beginning execution number {}...".format(i))
        try:
            mean_tt, mean_ft, total_dist = begin_execution()
            f = open('testes/dist/time_dist_{}_{}'.format(NUM_ROBOTS_IN_GROUP, len(GROUP_COLORS)), 'a')
            f.write("{}\n".format(total_it))
            f.close()
            print("Execution number {} finished!".format(i))
        except Exception as e:
            print("Execution failed due to {}".format(e))
    f.close()


if __name__ == '__main__':
    main(sys.argv)