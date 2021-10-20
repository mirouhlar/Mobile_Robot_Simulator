from pygame.locals import *
import pygame.transform
import pygame.image
import sys
import rclpy
from rclpy.node import Node
import numpy as np
from std_msgs.msg import Int16MultiArray

worldx = 960
worldy = 720

class Robot(pygame.sprite.Sprite):

    def __init__(self):

        pygame.sprite.Sprite.__init__(self)




        img = pygame.image.load('/home/asus/Desktop/task1/src/sim_pkg/images/vehicle.png').convert()
        
        self.image = pygame.transform.smoothscale(img, (30,30))
        self.rect = self.image.get_rect()



        


class SimulatorNode(Node):

    def __init__(self):

        super().__init__('simulator')
        pygame.init()
        pygame.display.set_caption('Simulator')

        self.subscription = self.create_subscription(Int16MultiArray,'pub_map',self.receive_map,10)
        self.subscription

        self.publish = self.create_publisher(Int16MultiArray,'rew_map',10)


        self.screen = pygame.display.set_mode([worldx, worldy]) 
        self.background = pygame.image.load('/home/asus/Desktop/task1/src/sim_pkg/images/bg.png')
        self.background = pygame.transform.smoothscale(self.background , (worldx,worldy))
        self.bck_rect = self.screen.get_rect()

        self.timer = self.create_timer(1 / 100, self.run)


        self.robot = Robot()  
        self.robot.rect.x = 50  
        self.robot.rect.y = 500 

        self.robot_list = pygame.sprite.Group()
        self.robot_list.add(self.robot)
        self.obst = []
        self.mapa = []



    def run(self):

        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                pygame.quit()
                sys.exit() 


            if event.type == pygame.MOUSEBUTTONDOWN:
                pos = pygame.mouse.get_pos()
                pressed = pygame.mouse.get_pressed()[0]

                msg = Int16MultiArray()

                
                for ob in self.obst:
                    if ob[0].collidepoint(pos) and pressed:
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

        self.screen.blit(self.background, self.bck_rect)
        self.robot_list.draw(self.screen)

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
