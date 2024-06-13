import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_msgs.msg import Int16
from geometry_msgs.msg import Twist
from rclpy.qos import QoSProfile
import serial
import time

########## This python file is the one uploaded into the Raspberry Pi 3B+ ##########
########## Main loop process time = 0.01790475845336914 [sec]  ######################

VERBOSE = False
QOS_DEPTH = 10
BAUD_RATE = 9600

class RPI_Node(Node):

    def __init__(self):

        super().__init__('rpi_com_node')

        self.linear_x = 0.0
        self.angular_z = 0.0
        self.left_tick_counts = Int16()
        self.left_tick_counts.data = 0
        self.right_tick_counts = Int16()
        self.right_tick_counts.data = 0
        self.last_received_ticks = None

        qos_profile = QoSProfile(depth=QOS_DEPTH )

        # Open a serial connection to /dev/ttyACM0
        self.ser = serial.Serial('/dev/ttyACM0', BAUD_RATE, timeout=0.1)
        
        # Publish encoder ticks counts
        self.left_ticks_pub = self.create_publisher(Int16, 'left_ticks_counts', qos_profile)
        self.right_ticks_pub = self.create_publisher(Int16, 'right_ticks_counts', qos_profile)
        self.subscription = self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, qos_profile)
        
        self.subscription   # prevent warning unused variable

        timer_frequency = 10.0
        self.timer = self.create_timer(1.0 / timer_frequency, self.publish_data)

    def publish_data(self):
        start_time = time.time()
        # Convert linear_x and angular_z to bytes and send over serial
        data_to_send = f"{self.linear_x},{self.angular_z}\n"
        self.ser.write(data_to_send.encode())

        ser_data = self.ser.readline()
        received_ticks_counts = ser_data.decode('utf-8').split(',')

        if received_ticks_counts:
            try:
                self.left_tick_counts.data, self.right_tick_counts.data =  int(received_ticks_counts[0]), int(received_ticks_counts[1])
                self.last_received_ticks = (self.left_tick_counts.data, self.right_tick_counts.data)
            except:
                self.get_logger().debug("Wrong data type received for ticks counts: {0}".format(type(received_ticks_counts)))
        else:
            self.get_logger().debug("No ticks counts received.")

        elapsed_time = time.time() - start_time
        if VERBOSE:
            self.get_logger().info("Main loop time: {0} [sec]".format(elapsed_time))

        if self.last_received_ticks is not None:
            self.get_logger().debug("Publishing ticks counts: {0} and {1}".format(self.left_tick_counts.data, self.right_tick_counts.data))
            self.left_ticks_pub.publish(self.left_tick_counts)
            self.right_ticks_pub.publish(self.right_tick_counts)

    def cmd_vel_callback(self, msg):
        self.get_logger().info("Received cmd_vel: {0} and {1}".format(msg.linear.x, msg.angular.z))

        self.linear_x = msg.linear.x
        self.angular_z = msg.angular.z


def main(args=None):
    rclpy.init(args=args)
    vel_pub_node = RPI_Node()
    vel_pub_node.get_logger().info("Initiates node")
    rclpy.spin(vel_pub_node)
    vel_pub_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
