from pygame.locals import *
import pygame.transform
import pygame.image
import sys
import rclpy
from rclpy.node import Node
import numpy as np
from std_msgs.msg import Int16MultiArray
import math
from math import cos, pow, atan2, sqrt, sin
from pygame.math import Vector2
import DStarLite
import time


worldx = 960
worldy = 720
pixel_per_meter = 120

Ts = 0.05 #sampling period

#real size of map in meters
screen_width = float(worldx)/pixel_per_meter 
screen_height = float(worldy)/pixel_per_meter


def pos_to_pixels(pos):
    return (int(pos[0] * pixel_per_meter),worldy - int(pos[1] * pixel_per_meter))

def pixels_to_pos(pixels):
    return round(float(pixels[0])/pixel_per_meter, 5), round(-float(pixels[1]-worldy)/pixel_per_meter, 5)


# Wrapping angle to interval < -pi , pi >
def wrapToPi(angle):

    while angle > np.pi:
        angle -= 2.0 * np.pi

    while angle < -np.pi:
        angle += 2.0 * np.pi

    return angle 

# Odometry
def getPos(v,w,L,x,y,theta,ts):
        
        d_r = (v + 1/2*L*w) * ts
        d_l = (v - 1/2*L*w) * ts

        d_c = (d_l + d_r)/2
        phi = (d_r - d_l)/L

        v = d_c / ts
        w = phi / ts

        x = x + d_c * math.cos(theta)
        y = y + d_c * math.sin(theta)
        theta = theta + phi

        return x, y, theta, phi 




    
def euclidean_distance(x,y, goal_posex, goal_posey):
    return sqrt(pow((goal_posex - x), 2) + pow((goal_posey - y), 2))

def linear_vel(x,y, goal_posex, goal_posey, constant=10):
    return constant * euclidean_distance(x,y,goal_posex,goal_posey)

def steering_angle(x,y, goal_posex, goal_posey):
    return atan2(goal_posey - y, goal_posex -x)

def angular_vel(x,y,theta,goal_posex, goal_posey, constant=5.8):
    return constant * wrapToPi(steering_angle(x,y,goal_posex, goal_posey) - theta)

def uprav(x):
    for i in range(len(x)-2):

        pr = x[i]
        pr1 = x[i+1]
        pr2 = x[i+2]
        if((pr[0]==pr1[0] or pr[1]==pr1[1]) and (pr[0]==pr2[0] or pr[1]==pr2[1])):
            x[i+1] = x[i]
        
    for i in range(len(x)-2):
        pr = x[i]
        pr1 = x[i+1]
        pr2 = x[i+2]
        if abs(atan2(pr[1] - pr1[1], pr[0] - pr1[0])) == abs(atan2(pr[1] - pr2[1], pr[0] - pr2[0])):
            x[i+1] = x[i]
            

    res = []
    for i in x:
        if i not in res:
            res.append(i)

    return res



class Robot(pygame.sprite.Sprite):

    def __init__(self,pos):

        pygame.sprite.Sprite.__init__(self)
        # img = pygame.image.load('/home/asus/Desktop/vozidlo.png').convert()  
        # self.image = pygame.transform.smoothscale(img, (30,30))
        
        self.radius = 12
        self.radius_s = 14
        self.image = pygame.Surface((24, 24), pygame.SRCALPHA)
        pygame.draw.circle(self.image, pygame.Color('red'),(self.radius, self.radius), self.radius)
        self.sensor = pygame.Surface((30, 30), pygame.SRCALPHA)
        pygame.draw.circle(self.sensor,(30,224,33,50),(self.radius_s, self.radius_s), self.radius_s)
        
        self.direction = 0
        self.x = pos[0]
        self.y = pos[1]
        self.v = 0
        self.w = 0
        self.distance_tolerance = 0.001


        self.pos = pos_to_pixels(pos) 
        self.rect = self.image.get_rect(center = self.pos)
        self.rect_s = self.sensor.get_rect(center = self.pos)

        self.front = Vector2(0, -self.radius).rotate(90)
        
    def draw_vectors(self,screen):
        pygame.draw.circle(screen, (0,0,0), self.rect.center+self.front, 2)
        pygame.draw.line(screen, (0,0,0), self.rect.center, self.rect.center+self.front, 2)

    
    def control(self,x,y,theta,xd,yd):
            running = 0

            if abs(wrapToPi(angular_vel(x,y,theta,xd,yd,1))) >= self.distance_tolerance:
                w = angular_vel(x,y,theta,xd,yd)
                v = 0        
            elif abs(euclidean_distance(x,y,xd,yd)) >= self.distance_tolerance:
                v = linear_vel(x,y,xd,yd)
                w = 0
            else:
                v = 0
                w = 0
                running = 1

            # if abs(euclidean_distance(x,y,xd,yd)) >= self.distance_tolerance:
            #     v = linear_vel(x,y,xd,yd)
            #     w = angular_vel(x,y,theta,xd,yd)
            # else:
            #     v = 0
            #     w = 0

            if abs(v) > 1.5:
                v = np.sign(v) * 1.5
            if abs(w) > 20:
                w = np.sign(w) * 20

            return v, w, running

    def update(self,x,y,phi):

        position = pos_to_pixels((x,y))
        self.pos = position
        self.rect.center = position
        self.rect_s.center = position
        self.front.rotate_ip_rad(-phi)
        

    
   






class SimulatorNode(Node):

    def __init__(self):

        super().__init__('Simulator')

        # Handling Simulator settings
        pygame.init()
        pygame.display.set_caption('Simulator')
        self.screen = pygame.display.set_mode([worldx, worldy]) 
        self.background = pygame.image.load('/home/asus/Desktop/task2/images/bg.png')
        self.background = pygame.transform.smoothscale(self.background , (worldx,worldy))
        self.bck_rect = self.screen.get_rect()


        # Handling map receiving and publishing nodes
        self.subscription = self.create_subscription(Int16MultiArray,'pub_map',self.receive_map,10)
        self.subscription
        self.publish = self.create_publisher(Int16MultiArray,'rew_map',10)


        # Handling mobile robot instance settings
        self.robot = Robot((0.43,0.45))  
        self.start = []
        self.goal = []
        # self.robot.rect.x = 500
        # self.robot.rect.y = 500


        self.robot_list = pygame.sprite.Group()
        self.robot_list.add(self.robot)
        self.obst = []
        self.mapa = np.array([])

        # Handling simulation
        self.timer = self.create_timer(1/60, self.run)

        self.goal_posex = self.robot.x 
        self.goal_posey = self.robot.y 

        self.lastx = 0
        self.lasty = 0

        self.lx = (0,0)

        self.dstar = None
        self.new = None
        self.start1 = None


        self.robot_path = []
        self.old_robot_path = []    
        self.running = 0   


    def run(self):


        for event in pygame.event.get():

            if event.type == pygame.QUIT:
                pygame.quit()
                sys.exit() 

            if event.type == pygame.MOUSEBUTTONDOWN:

                pos = pygame.mouse.get_pos()
                pressed = pygame.mouse.get_pressed()
                msg = Int16MultiArray()
                

                for ob in self.obst:


                    if ob[0].collidepoint(pos) and pressed[2]:
                        
                        if ob[1] == 0:
                            self.start =  tuple(map(int, [self.robot.pos[1]/ (worldx / self.columns), self.robot.pos[0]/ (worldy/ self.lines)])) 
                            self.goal = (ob[2],ob[3])
                            self.dstar = DStarLite.DStar(self.start,self.goal,self.mapa)
                            self.dstar.ComputeShortestPath()
                            print(self.dstar.GetStart(self.start))
                            
                            
                            self.old_robot_path = self.robot_path.copy()
                            self.robot_path = list(reversed(self.dstar.ExtractPath()))
                            print(self.robot_path)

     
                            
                            if not self.old_robot_path:

                                self.old_robot_path = self.robot_path.copy()

           

                    if ob[0].collidepoint(pos) and pressed[0]:
                        
                        if self.new and self.running == 0:
                            self.dstar.start = self.start
                            self.dstar.NewObst(self.start,self.new,ob[2],ob[3])
                            self.dstar.ComputeShortestPath()
                            self.old_robot_path = self.robot_path.copy()
                            self.robot_path = list(reversed(self.dstar.ExtractPath()))
                            

                        if ob[1] == 1:
                            ob[1] = 0
                        elif ob[1] == 0 and not ob[0].colliderect(self.robot.rect):
                            ob[1] = 1

                        self.mapa[ob[2], ob[3]] = ob[1]
                        a,b = self.mapa.shape
                        msg.data = self.mapa.flatten().astype(int).tolist()
                        msg.data.append(a)
                        msg.data.append(b)
                        self.publish.publish(msg)    
 

        

        if self.running == 1 and self.mapa.size != 0 and self.goal:

            if self.goal != self.start:
                new = self.dstar.GetStart(self.start)

                self.new = pixels_to_pos((new[1]* worldx / self.columns + (worldx / self.columns)/2, new[0]* worldy/ self.lines + (worldy / self.lines)/2))

                self.goal_posex = self.new[0]
                self.goal_posey = self.new[1]
                self.start = new

        self.robot.v,self.robot.w, self.running = self.robot.control(self.robot.x,self.robot.y,self.robot.direction,self.goal_posex, self.goal_posey)
        self.robot.x,self.robot.y,self.robot.direction, phi = getPos(self.robot.v,self.robot.w,0.066,self.robot.x,self.robot.y,self.robot.direction,Ts)
        


        self.screen.blit(self.background, self.bck_rect)


        

        for rect in self.obst:
                    if rect[1] == 1:
                        pygame.draw.rect(self.screen, (0,0,0), rect[0],2,2)
                    
                
                   

        for i in range(len(self.old_robot_path) - 1):
            pygame.draw.line(self.screen,(20,255,0),(self.old_robot_path[i][1]* worldx / self.columns + (worldx / self.columns)/2, self.old_robot_path[i][0]* worldy/ self.lines + (worldy / self.lines)/2), (self.old_robot_path[i+1][1]* worldx / self.columns + (worldx / self.columns)/2, self.old_robot_path[i+1][0]* worldy/ self.lines + (worldy / self.lines)/2),2)                        
        for i in range(len(self.robot_path) - 1):
            pygame.draw.line(self.screen,(255,0,0),(self.robot_path[i][1]* worldx / self.columns + (worldx / self.columns)/2, self.robot_path[i][0]* worldy/ self.lines + (worldy / self.lines)/2), (self.robot_path[i+1][1]* worldx / self.columns + (worldx / self.columns)/2, self.robot_path[i+1][0]* worldy/ self.lines + (worldy / self.lines)/2),2)                        


        self.robot.update(self.robot.x,self.robot.y,phi)

        self.robot_list.draw(self.screen)
        self.robot.draw_vectors(self.screen)
        

        pygame.display.flip()





    def receive_map(self, msg):

        received_map = np.array(msg.data[:-2]).reshape(msg.data[-2],msg.data[-1])

        self.lines = msg.data[-2]
        self.columns = msg.data[-1]

        width = worldx / self.columns
        height = worldy / self.lines

        self.mapa = np.zeros((self.lines,self.columns))
        self.obst = []

        self.start =  list(map(int, [self.robot.pos[1]/ (worldx / self.columns), self.robot.pos[0]/ (worldy/ self.lines)])) 

        print("Sape of Map: ", self.lines, "x", self.columns)


        for x in range(self.lines):
            for y in range(self.columns):

                if received_map[x][y] == 1:
                    self.obst.append([pygame.Rect((width * y, height * x), (width, height)), 1 ,x, y])
                    self.mapa[x][y] = 1
                else:
                    self.obst.append([pygame.Rect((width * y, height * x), (width, height)), 0, x, y])
                    self.mapa[x][y] = 0

        

def main(args=None):

   rclpy.init(args=args)
   simulator_node = SimulatorNode()
   rclpy.spin(simulator_node)
   simulator_node.destroy_node()
   rclpy.shutdown()


if __name__ == "__main__":
    main()
