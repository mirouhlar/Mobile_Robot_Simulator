import rclpy
from rclpy.node import Node
from std_msgs.msg import Int16MultiArray
import numpy as np


class MapReadingNode(Node):
    def __init__(self):
        super().__init__("map_publisher")
        self.map_publisher_ = self.create_publisher(Int16MultiArray, 'pub_map', 10)
        self.map_timer_ = self.create_timer(1, self.publish_map)

    def publish_map(self):

        map_array = np.genfromtxt('/home/asus/Desktop/task2/maps/map.csv', delimiter=' ').astype(int)
        a,b = map_array.shape
        pole = map_array.flatten().tolist()
        pole.append(a)
        pole.append(b)

        msg = Int16MultiArray()
        
        msg.data = pole
        self.map_publisher_.publish(msg)
        print("Done!")

def main(args=None):

    rclpy.init(args=args)
    node = MapReadingNode()
    rclpy.spin_once(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
