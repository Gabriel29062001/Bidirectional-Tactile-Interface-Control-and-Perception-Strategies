## This class enable the communication between ROS and an Arduino.
## It enable encodage of a message for an Arduino board.

import serial
import serial.tools.list_ports


class Arduino:
    def __init__(self, port_name, baud_rate, description=None):
        self.port_name = port_name
        self.baud_rate = baud_rate
        self.description = description

        # Check on which port the arduino is connected (currently necessary because of usage in virtual machine)
        #ports = serial.tools.list_ports.comports()
        #for port, desc, hwid in sorted(ports):
        #    print("{}: {} [{}]".format(port, desc, hwid))

        self.ser = serial.Serial(self.port_name, self.baud_rate)

    def send_command(self, command):
        encoded_command = command.encode()
        print("Encoded command:", encoded_command)
        self.ser.write(command.encode())

    def receive_data(self):
        msg_complete = False
        output_msg = ""

        # Flush the input buffer to get the most recent data
        self.ser.reset_input_buffer()

        while not msg_complete:
            data_recv = self.ser.readline().decode("utf-8")

            # Get data from TCP server
            output_msg = output_msg + data_recv

            # Search for unique first element of message "<" (find returns -1 if element is not found)
            position_first_element = output_msg.find("<")
            # Check if the unique first element is included in the data and cut everything before if so
            if position_first_element == -1:
                continue
            else:
                output_msg = output_msg[position_first_element:]

            # Search for the unique last element of message ">"
            position_last_element = output_msg.find(">")
            # Check if the unique last element is included in the data and cut everything after if so
            if position_last_element == -1:
                continue
            else:
                output_msg = output_msg[:position_last_element + 1]

            # Check for complete message
            if output_msg[0] == "<" and output_msg[-1] == ">":
                msg_complete = True
            else:
                continue

        return output_msg

