## This code enable to read serial_number of Arduino board in order to connect to it in other nodes.

from serial.tools.list_ports import comports
port_list = list(comports(include_links=False))
for port in port_list:
    print(port)
    print(port.serial_number)
