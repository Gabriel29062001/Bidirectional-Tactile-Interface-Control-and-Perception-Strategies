# This class enables comunication across Multiple Computers on ROS2 using the library sockets.
# It will connect to an ip port and read value send it to it.

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
        
        #Publish to the second module 
        inverse_mapping_direction = int(not mapping_direction)
        #inverse_mapping_direction = int(mapping_direction)
        publisher_name = "mapping_desired_state_" + str(inverse_mapping_direction)
        self.publisher_ = self.create_publisher(String, publisher_name, 10)


        # Address and port of the destination
        self.address_ip_destinataire = '128.179.186.185'  # Replace this with the IP address of the destination
        self.port_destinataire = 1026# Replace this with the port on which the destination is listening

        # Create a socket object
        self.socket =socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.socket.bind((self.address_ip_destinataire, self.port_destinataire))

        self.socket.listen(1)

        print("En attente de connexion")

        while True:
            conn, addr = self.socket.accept()
            print(f"Connexion établie avec {addr}")
            while True:
                message = String()
                data_receive = conn.recv(1024).decode()
                message.data = data_receive
                self.publisher_.publish(message)
                if not data_receive:
                    break

                print(f"Message reçu : {data_receive}")

        conn.close()




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




