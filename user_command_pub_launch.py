# This class connect to a x Teensy/Arduino board and read current values availbale then send it formated inside
# a topic called module_x/current_state, then subscribe to a topic called /mapping_desired_state_x
# and send the desired mapping command to the same Teensy/Arduino board.
# The only difference with user_command_pub is that it takes some parameters that can be launched with a launch file.

import rclpy
from rclpy.node import Node
import re
from std_msgs.msg import String
from serial.tools import list_ports
from .arduino_helper import Arduino
import argparse
import sys


class HapticJoystickUserCommand(Node):
    def __init__(self):
        super().__init__('haptic_joystick_user_command')

        self.declare_parameter('module_number', '0')
        self.module_param = self.get_parameter('module_number').get_parameter_value().string_value
        module_number = int(self.module_param)

        teensy_serial_number = ["11149580","11156490"]
        port_list = list(list_ports.comports(include_links=False))
        teensy_port = find_teensy_port(teensy_serial_number[module_number], port_list)
        # Publish current and desired state 
       
        topic_name = "module_" + str(module_number)
        current_state_topic = topic_name + "/current_state"

        self.publisher_current_state = self.create_publisher(String, current_state_topic, 1)
  
        timer_period = 0.01  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.arduino = Arduino(teensy_port, 115200, "Haptic Joystick")

        # Subscrible to topic mapping_desired_state_x with x the number corresponding to the module
        subscribe_module =  'mapping_desired_state_' + str(module_number)
        self.subscription_ = self.create_subscription(String, subscribe_module, self.callback, 1)
        self.subscription_

    def callback(self, msg):
        self.get_logger().info("%s" % msg.data)
        pattern = r"[-+]?\d*\.\d+|[-+]?\d+"
        numbers = re.findall(pattern, msg.data)
        desired_arduino_command = "<" + numbers[0] + ";" + numbers[1] + ";" + numbers[2]  +">"
        print(desired_arduino_command)
        self.arduino.send_command(desired_arduino_command)
        

        # Process the received message and do something

    def timer_callback(self):
        # Get data from arduino
        data = self.arduino.receive_data()
        
        # Extract sensor data from raw arduino message
        ard_data = self.edit_data(data)
        current_state = ard_data[0:3]
        desired_pos_arduino = ard_data[3:6]
        current_force = ard_data[6:9]

        msg_current = String()
        msg_current.data = "Current state: " + str(current_state) + ","  + str(current_force)

        print("Arduino1 Desired state: " + str(desired_pos_arduino) )
   
 
        self.publisher_current_state.publish(msg_current)
        self.get_logger().info(msg_current.data)


    def edit_data(self, data):

        ard_data = []
        split_data = data.split(";")

        for i in range(len(split_data)):
            if i == 0:
                data = float(split_data[i][1:])
            elif i == len(split_data)-1:
                data = float(split_data[i][:-1])
            else:
                data = float(split_data[i])

            ard_data.append(data)
        
        return ard_data
    

def main(args=None):
    rclpy.init(args=args)

    # Launch haptic_joystick_user_command node with corresponding to the serial number chosen
    haptic_joystick_user_command = HapticJoystickUserCommand()
    rclpy.spin(haptic_joystick_user_command)
    # Destroy the node explicitly
    haptic_joystick_user_command.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main(sys.argv[1:]) 



def find_teensy_port(serial_number, port_list):

    for port in port_list:
        if port.serial_number == serial_number:
            return port.device
    return None