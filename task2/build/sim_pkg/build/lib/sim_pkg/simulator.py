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

def linear_vel(x,y, goal_posex, goal_posey, constant=1.8):
    return constant * euclidean_distance(x,y,goal_posex,goal_posey)

def steering_angle(x,y, goal_posex, goal_posey):
    return atan2(goal_posey - y, goal_posex -x)

def angular_vel(x,y,theta,goal_posex, goal_posey, constant=5.8):
    return constant * wrapToPi(steering_angle(x,y,goal_posex, goal_posey) - theta)





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

            if abs(wrapToPi(angular_vel(x,y,theta,xd,yd,1))) >= self.distance_tolerance:
                w = angular_vel(x,y,theta,xd,yd)
                v = 0        
            elif abs(euclidean_distance(x,y,xd,yd)) >= self.distance_tolerance:
                v = linear_vel(x,y,xd,yd)
                w = 0
            else:
                v = 0
                w = 0

            if abs(v) > 1.5:
                v = np.sign(v) * 1.5
            if abs(w) > 20:
                w = np.sign(w) * 20

            return v, w

    def update(self,x,y,phi):

        position = pos_to_pixels((x,y))
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
        self.background = pygame.image.load('/home/asus/Desktop/GIT_tasks/NMVR/task2/src/sim_pkg/images/bg.png')
        self.background = pygame.transform.smoothscale(self.background , (worldx,worldy))
        self.bck_rect = self.screen.get_rect()


        # Handling map receiving and publishing nodes
        self.subscription = self.create_subscription(Int16MultiArray,'pub_map',self.receive_map,10)
        self.subscription
        self.publish = self.create_publisher(Int16MultiArray,'rew_map',10)


        # Handling mobile robot instance settings
        self.robot = Robot((1,1))  
        # self.robot.rect.x = 500
        # self.robot.rect.y = 500


        self.robot_list = pygame.sprite.Group()
        self.robot_list.add(self.robot)
        self.obst = []
        self.mapa = []

        # Handling simulation
        self.timer = self.create_timer(1/60, self.run)

        self.goal_posex = self.robot.x 
        self.goal_posey = self.robot.y 

        self.lastx = 0
        self.lasty = 0

        self.lx = (0,0)



        

       


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
                            new = pixels_to_pos(ob[0].center)
                            self.goal_posex = new[0]
                            self.goal_posey = new[1]
                            self.lx = ob[0].center

                    if ob[0].collidepoint(pos) and pressed[0]:
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
 
        for ob in self.obst:

            if ob[1] == 0 and ob[0].colliderect(self.robot.rect):
                new = pixels_to_pos(ob[0].center)
                self.lastx = new[0]
                self.lasty = new[1]
                self.ly = self.robot.y
               
            if ob[1] == 1 and ob[0].colliderect(self.robot.rect_s):
                
                self.goal_posex = self.lastx
                self.goal_posey = self.lasty

                self.robot.v,self.robot.w = -self.robot.v,-self.robot.w
           
        self.robot.v,self.robot.w = self.robot.control(self.robot.x,self.robot.y,self.robot.direction,self.goal_posex, self.goal_posey)
        self.robot.x,self.robot.y,self.robot.direction, phi = getPos(self.robot.v,self.robot.w,0.066,self.robot.x,self.robot.y,self.robot.direction,Ts)

        self.screen.blit(self.background, self.bck_rect)


        self.robot.update(self.robot.x,self.robot.y,phi)

        self.robot_list.draw(self.screen)
        self.robot.draw_vectors(self.screen)

        for rect in self.obst:
                    if rect[1] == 1:
                        pygame.draw.rect(self.screen, (0,0,0), rect[0],2,2)



        pygame.display.flip()





    def receive_map(self, msg):

        received_map = np.array(msg.data[:-2]).reshape(msg.data[-2],msg.data[-1])

        self.lines = msg.data[-2]
        self.columns = msg.data[-1]

        width = worldx / self.columns
        height = worldy / self.lines

        self.mapa = np.zeros((self.lines,self.columns))
        self.obst = []

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
