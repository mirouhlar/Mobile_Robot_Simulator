import rclpy
from rclpy.node import Node
from std_msgs.msg import Int16MultiArray
import numpy as np
import csv

class MapReceivingNode(Node):

    def __init__(self):
        super().__init__('map_receiver')
        self.subscription = self.create_subscription(Int16MultiArray,'rew_map',self.receive_map,10)
        self.subscription  

    def receive_map(self, msg):
        received_map = np.array(msg.data[:-2]).reshape(msg.data[-2],msg.data[-1])

        with open('/home/asus/Desktop/GIT_tasks/NMVR/task2/src/sim_pkg/maps/map.csv',"w+") as my_csv:
            csvWriter = csv.writer(my_csv,delimiter = ' ')
            csvWriter.writerows(received_map)
            my_csv.close()
            print("done")    


def main(args=None):

    rclpy.init(args=args)
    receiver_node = MapReceivingNode()
    rclpy.spin(receiver_node)
    receiver_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
