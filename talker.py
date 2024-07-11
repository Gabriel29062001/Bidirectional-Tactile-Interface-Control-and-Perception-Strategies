# This class enables comunication across Multiple Computers on ROS2 using the library sockets.
# It will connect to an ip port and send message throught to this ip port.

import rclpy
from rclpy.node import Node
import re
from std_msgs.msg import String
from serial.tools import list_ports
from .arduino_helper import Arduino
import argparse
import sys

import socket

class MySubscriber(Node):
    def __init__(self, mapping_direction):

        # Initialize the Node
        super().__init__('my_subscriber')
        
        # Subscribe to module_x/current_state topic with x the corresponding module number as user_command_pub
        subscripiton_name = "module_" + str(mapping_direction) + "/current_state"
        self.subscription = self.create_subscription(String, subscripiton_name, self.listener_callback, 10)
        self.subscription  # Prevent unused variable warning

        # Address and port of the destination
        self.address_ip_destinataire = '128.179.186.185'  # Replace this with the IP address of the destination
        self.port_destinataire = 1030# Replace this with the port on which the destination is listening

        # Create a socket object
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

        try:
            # Connect to the destination
            self.socket.connect((self.address_ip_destinataire, self.port_destinataire))
            print("Connection established successfully!")

        except socket.error as e:
            print(f"Error connecting to the destination: {e}")

    def listener_callback(self, msg):
        new_msg = String()
        desired_state = self.edit_data(msg)
        new_msg.data = 'Desired state mapping:%s' % desired_state

        # Send the current state over the socket connection
        try:
            self.socket.sendall(new_msg.data.encode())
            print("Message sent successfully!")

        except socket.error as e:
            print(f"Error sending message: {e}")
        
    def edit_data(self, msg):
        pattern = r"[-+]?\d*\.\d+|[-+]?\d+"
        # Extract numerical values using regex
        numbers = re.findall(pattern, msg.data)
        default_position = [0.03, 0, 0]
        
        desired_state = []

        limit_pos_sup = [40, 40,40]
        limit_pos_inf = [0, 0,0]
        
        for i in range(len(numbers)):
            if float(numbers[i]) > limit_pos_sup[i] :
                desired_state.append(limit_pos_inf[i])
            elif  float(numbers[i]) < limit_pos_inf[i]:
                desired_state.append(limit_pos_sup[i])
            else :
                  
                desired_state.append(abs(limit_pos_sup[i]-float(numbers[i])))

        

        return desired_state


def main(args=None):
    rclpy.init(args=args)

    # Take as argument an integer to choose on which topic he should subscribe to
    parser = argparse.ArgumentParser(description='Haptic Joystick User Command')
    parser.add_argument('mapping_direction', type=int, help='')
    args = parser.parse_args(args)

    my_subscriber = MySubscriber(args.mapping_direction)

    try:
        rclpy.spin(my_subscriber)

    except KeyboardInterrupt:
        print("Keyboard Interrupt!")

    finally:
        my_subscriber.socket.close()
        my_subscriber.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
